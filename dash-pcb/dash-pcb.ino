//Teensy 4.1 i2c libraries
#include <Arduino.h>
#include <i2c_device.h>

//CAN libraries
#include <circular_buffer.h>
#include <FlexCAN_T4.h>
#include <imxrt_flexcan.h>
#include <isotp.h>
#include <isotp_server.h>
#include <kinetis_flexcan.h>

//SD Library
#include <SD.h>

#define LOG_DATA_TO_SERIAL


// The I2C device. Determines which pins we use on the Teensy.
I2CMaster& master = Master;

//I2C addresses
const uint8_t ACC_MAG_ADDR = 0x1F;
const uint8_t GYR_ADDR = 0x21;

//objects for the i2c sensors (adafruit 3463)
I2CDevice acc_mag = I2CDevice(master, ACC_MAG_ADDR, _BIG_ENDIAN);
I2CDevice gyr = I2CDevice(master, GYR_ADDR, _BIG_ENDIAN);

//addresses for relevant registers
const uint8_t ACC_MAG_WHOAMI_REG = 0x0D;
const uint8_t ACC_MAG_CTRL_REG_1 = 0x2A;
const uint8_t ACC_MAG_XYZ_DATA_CFG_REG = 0x0El;
const uint8_t GYR_WHOAMI_REG =  0x0C;
const uint8_t GYR_CTRL_REG0 = 0x0D;
const uint8_t GYR_CTRL_REG1 = 0x13;

int tele_data_1B[10]; // intList: ACCX, ACCY, ACCZ, GYRX, GYRY, GYRZ, exTemp(C), intakeTemp(C), coolantTemp(C), lambda1
int tele_data_2B[4][2]; // list for 2-byte data: engineSpeed(RPM), engineLoad(%), throttle(%), MAP(kPa)
bool tele_data_fan1; // fan 1 status
// changed all to int, send only buffer values; further conversion done by html code

#define LOG_VERSION 3

FlexCAN_T4<CAN1, RX_SIZE_16, TX_SIZE_16> can1;
CAN_message_t msg;

//microseconds between each reading
#define IMU_MICROS_INCR             1400
#define IMU_CHECK_MICROS_INCR     100000
#define SD_MICROS_INCR          10000000
#define ANALOG_READ_MICROS_INCR   200000
#define REALTIME_TELE_MICROS_INCR  90000

//next time a reading should be taken
unsigned long next_imu_micros;
unsigned long next_sd_write_micros;
unsigned long next_analog_read_micros;
unsigned long next_realtime_tele_micros;

//path to log file on sd card
char log_name[] = {'X','X','X','X','/','l','o','g','.','c','s','v','\0'};
char directory_name[] = {'X','X','X','X','\0'};
File log_file;

void setup(void) {
  //Set up communication with plugged in laptop if there is one
  Serial.begin(115200);
  //Turn on on-board LED, wait for Serial
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(400);
  
  Serial.println("---BEGIN---");
  Serial.println("");
  
  prep_SD();
  
  //Initialize the CAN bus at 500kbps
  Serial.println("Initializing CAN bus...");
  can1.begin();
  can1.setBaudRate(500000);
  Serial.println("CAN bus initialized.");
  log_pair("MSG", "CAN init done");
  log_file.flush();
  
  // Initialise the I2C bus at 400kbps
  master.begin(400 * 1000U);
  Serial.println("I2C initialized.");
  log_pair("MSG", "I2C init done");
  log_file.flush();
  
  // Initialize accelerometer and gyroscope
  prep_3463();
  log_file.flush();
  
  Serial.println("Sending CAN message");
  log_pair("MSG", "Sending CAN test message");
  msg.id = 0x12;
  msg.buf[0] = 12;
  msg.buf[1] = 13;
  can1.write(msg);
  
  Serial1.begin(9600); // to ESP8266 Telemetry
}

void loop(void) {
  
  read_3463();
  read_CAN();
  read_A1();
  wireless_tele();
  log_file.flush();
  
}

//Initialize the adafruit 3463
void prep_3463(){
  
  next_imu_micros = micros() + IMU_MICROS_INCR;
  uint8_t result_byte;
  
  Serial.println("Attempting communication with ACC_MAG");
  
  acc_mag.read(ACC_MAG_WHOAMI_REG, &result_byte, false);
  
  if(result_byte != 0xC7){
    Serial.print("ERROR: accel mag wrong who_am_i, reported 0x");
    Serial.println(result_byte, HEX);
    log_file.print("ERR,am whoami,");
    log_file.println(result_byte,HEX);
  }else{
    Serial.println("ACC_MAG sensor found");
    log_file.println("MSG,acc_mag found");
  }
  
  Serial.println("Attempting communication with GYR");
  
  acc_mag.read(GYR_WHOAMI_REG, &result_byte, false);
  
  if(result_byte != 0b11010111){
    Serial.print("ERROR: gyr wrong who_am_i, reported 0x");
    Serial.println(result_byte, HEX);
    log_file.print("ERR,am whoami,");
    log_file.println(result_byte,HEX);
  }else{
    Serial.println("GYR sensor found");
    log_file.println("MSG,gyr found");
  }
  
  uint8_t msg;
  
  //accelerometer in +/- 4g max mode
  msg = 0b00000001;
  acc_mag.write(ACC_MAG_XYZ_DATA_CFG_REG, msg, false);
  //accelerometer in active mode, max data rate
  msg = 0b00000001;
  acc_mag.write(ACC_MAG_CTRL_REG_1, msg, false);
  
  // +/- 250dps
  msg = 0b00000011;
  gyr.write(GYR_CTRL_REG0, msg, false);
  
  //800Hz, active
  msg = 0b00000011;
  gyr.write(GYR_CTRL_REG1, msg, false);
}

void prep_SD(){
  
  Serial.println("Initializing SD card...");
  if(SD.begin(BUILTIN_SDCARD)){
    Serial.println("SD card initialization complete.");
  }else{
    Serial.println("ERROR: Failed SD card initialization!");
  }
  
  Serial.println("Finding unused directory...");
  // Set a, b, c, d to the first available 4 digits for a directory name
  for(int a = 0; a < 10; a++){
    for(int b = 0; b < 10; b++){
      for(int c = 0; c < 10; c++){
        for(int d = 0; d < 10; d++){
          directory_name[0] = (char) (a + 48);
          directory_name[1] = (char) (b + 48);
          directory_name[2] = (char) (c + 48);
          directory_name[3] = (char) (d + 48);
          if(!SD.exists(directory_name)){
            goto dirfound;
          }
        }
      }
    }
  }
  Serial.println("ERROR: All possible directory names taken");
  
  dirfound:
  //Set the name of the log file to include the chosen directory
  log_name[0] = directory_name[0];
  log_name[1] = directory_name[1];
  log_name[2] = directory_name[2];
  log_name[3] = directory_name[3];
  SD.mkdir(directory_name);
  
  log_file = SD.open(log_name, FILE_WRITE);
  Serial.print("Starting log file at ");
  Serial.println(directory_name);
  if(log_file){
    Serial.println("Log file started.");
    log_pair("MSG", "DIRNAME:"+String(directory_name));
  }else{
    Serial.println("Could not create log file.");
  }
  
  log_pair("VER", String(LOG_VERSION));
  log_file.flush();
}

//Read data for the adafruit 3463 if enough time has passed
void read_3463(){
  
  //If current time is past time for next reading, do reading
  if(micros() < next_imu_micros) return;
  
  //Storage for the 7 bytes that will be sent back by each communication
  uint8_t response[7];
  //Sum of shifted MSB and LSB
  short total;
  //Data scaled to a more understandable unit
  int converted_total_x;
  int converted_total_y;
  int converted_total_z;
  //Message that will be logged
  char log_message[128];
  
  //Get data from accelerometer output registers
  acc_mag.read(0x00, response, 7, false);
  
  if(!(response[0] & 0b111)){
    log_pair("MSG", "Accelerometer not ready");
    tele_data_1B[0]=0;tele_data_1B[1]=0;tele_data_1B[2]=0;
  } else{
    //combine 6 bits MSB in response[1] with 8 bits LSB in [2]
    total = (int16_t)(((response[1] << 8) | response[2])) >> 2;
    //convert from arbitrary bit value to ug and log
    converted_total_x = ((int)total) * 488;
    
    total = (int16_t)(((response[3] << 8) | response[4])) >> 2;
    converted_total_y = ((int)total) * 488;
    
    total = (int16_t)(((response[5] << 8) | response[6])) >> 2;
    converted_total_z = ((int)total) * 488;
    
    //unit is ug, or one millionth of the acceleration due to gravity
    sprintf(log_message, "%i,%i,%i", converted_total_x, converted_total_y, converted_total_z);
    log_pair("ACC",log_message);
    tele_data_1B[0]=converted_total_x;tele_data_1B[1]=converted_total_y;tele_data_1B[2]=converted_total_z;
  }
  
  //Get data from gyroscope output registers
  gyr.read(0x00, response, 7, false);
  
  if(!(response[0] & 0b111)){
    log_pair("MSG", "Gyroscope not ready");
    prep_3463();
    tele_data_1B[3]=0;tele_data_1B[4]=0;tele_data_1B[5]=0;
  } else{
    
    //combine 8 bits MSB in response[1] with 8 bits LSB in [2]
    total = (short)(((response[1] << 8) | response[2]));
    //convert from arbitrary bit value to mdps and log
    converted_total_x = ((int)total) * 125 / 16;
    
    total = (short)(((response[3] << 8) | response[4]));
    converted_total_y = ((int)total) * 125 / 16;
    
    total = (short)(((response[5] << 8) | response[6]));
    converted_total_z = ((int)total) * 125 / 16;
    
    //unit is mdps, or thousandths of a degree per second
    sprintf(log_message, "%i,%i,%i", converted_total_x, converted_total_y, converted_total_z);
    log_pair("GYR",log_message);
    tele_data_1B[3]=converted_total_x;tele_data_1B[4]=converted_total_y;tele_data_1B[5]=converted_total_z;
  
  }
  
  //Set time for next reading
  next_imu_micros = max(micros(), next_imu_micros + IMU_MICROS_INCR);
  
}

void read_CAN(){

  while ( can1.read(msg) ) {
    
    //create an empty string
    char data_string[100] = {0};
    //fill the string with the formatted data
    sprintf(data_string,"%i,%08lX,%02X%02X%02X%02X%02X%02X%02X%02X",
    msg.flags.extended,msg.id,msg.buf[0],msg.buf[1],msg.buf[2],
    msg.buf[3],msg.buf[4],msg.buf[5],msg.buf[6],msg.buf[7]);

    if(msg.id == 0x01F0A000){
      // 01F0A000: EngSpd,EngLoad,Thro,inTemp,coTemp
      /* 
        msg.buf[0:5]: 0-255 (uint8_t)
        tele: need to convert to actual data
      */
      tele_data_2B[0][0] = msg.buf[0]; tele_data_2B[0][1] = msg.buf[1]; // engine_speed
      tele_data_2B[1][0] = msg.buf[2]; tele_data_2B[1][1] = msg.buf[3]; // engine_load
      tele_data_2B[2][0] = msg.buf[4]; tele_data_2B[2][1] = msg.buf[5]; // throttle
      tele_data_1B[7] = msg.buf[6]; // intake_air_temp
      tele_data_1B[8] = msg.buf[7]; // coolant_temp
    }
    else if(msg.id == 0x01F0A003){
      tele_data_1B[9] = msg.buf[0];// lambda1
    }
    else if(msg.id == 0x01F0A004){
      tele_data_2B[3][0] = msg.buf[0]; tele_data_2B[3][1] = msg.buf[1]; // MAP
      tele_data_fan1 = (msg.buf[6] & 0b00000010);
    }
    
    //log that with CAN message name and timestamp attached
    log_pair("CAN",data_string);
    //Serial1.println(data_string);
  }
}

void read_A1(){
  
  if(micros() > next_analog_read_micros){
    tele_data_1B[6]=analogRead(A1);
    log_pair("A1",String(tele_data_1B[6]));
    next_analog_read_micros = micros() + ANALOG_READ_MICROS_INCR;
  }
  
}

//log message like "s1,micros,s2\n"
void log_pair(String s1, String s2){
  log_file.print(s1);
  log_file.print(',');
  log_file.print(micros());
  log_file.print(',');
  log_file.println(s2);
  #ifdef LOG_DATA_TO_SERIAL
  Serial.print(s1);
  Serial.print(',');
  Serial.print(micros());
  Serial.print(',');
  Serial.println(s2);
  #endif
}

//send string for wireless telementry
void wireless_tele(){
  
  if(micros() > next_realtime_tele_micros){
    
    char data_string[150] = {0};
    // ACCX,ACCY,ACCZ|GYRX,GYRY,GYRZ|A1|
    // engineSpeed(RPM)|engineLoad(%)|throttle(%)|
    // intakeTemp(C)|coolantTemp(C)|
    // lambda1|manifold_pressure(kPa)|fan1(bool)|
    sprintf(data_string,"%d,%d,%d|%d,%d,%d|%d|%d,%d|%d,%d|%d,%d|%d|%d|%d|%d,%d|%d|\n",
    tele_data_1B[0],tele_data_1B[1],tele_data_1B[2],tele_data_1B[3],tele_data_1B[4],tele_data_1B[5],tele_data_1B[6],
    tele_data_2B[0][0],tele_data_2B[0][1],tele_data_2B[1][0],tele_data_2B[1][1],tele_data_2B[2][0],tele_data_2B[2][1],
    tele_data_1B[7],tele_data_1B[8],
    tele_data_1B[9],tele_data_2B[3][0],tele_data_2B[3][1],tele_data_fan1);
    Serial1.print(data_string);
    
    next_realtime_tele_micros = micros() + REALTIME_TELE_MICROS_INCR;
  }
}
