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

//objects for the i2c sensors (adafruit 3463)
const uint8_t acc_mag_addr = 0x1F;
I2CDevice acc_mag = I2CDevice(master, acc_mag_addr, _BIG_ENDIAN);
const uint8_t gyr_addr = 0x20;
I2CDevice gyr = I2CDevice(master, gyr_addr, _BIG_ENDIAN);

//addresses for relevant registers
const uint8_t ACC_MAG_WHOAMI_REG = 0x0D;
const uint8_t ACC_MAG_CTRL_REG_1 = 0x2A;
const uint8_t ACC_MAG_XYZ_DATA_CFG_REG = 0x0El;
const uint8_t GYR_WHOAMI_REG =  0x0C;
const uint8_t GYR_CTRL_REG0 = 0x0D;
const uint8_t GYR_CTRL_REG1 = 0x13;

#define LOG_VERSION 2

FlexCAN_T4<CAN1, RX_SIZE_16, TX_SIZE_16> can1;
CAN_message_t msg;

//microseconds between each reading
#define IMU_MICROS_INCR 1400
#define SD_MICROS_INCR 10000000
#define ANALOG_READ_MICROS_INCR 200000

//next time a reading should be taken
unsigned long next_imu_micros;
unsigned long next_sd_write_micros;
unsigned long next_analog_read_micros;

//path to log file on sd card
char log_name[] = {'X','X','X','X','/','0','0','0','0','.','c','s','v','\0'};
char directory_name[] = {'X','X','X','X','\0'};
File log_file;

void setup(void) {

  Serial.begin(115200);
  //Blink LED, wait for Serial
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(400);
  
  Serial.println("---BEGIN---");
  Serial.println("");

  prep_SD();
  log_file.flush();

  Serial.println("Initializing CAN bus...");
  can1.begin();
  can1.setBaudRate(500000);
  Serial.println("CAN bus initialized.");
  log_pair("MSG", "CAN init done");
  log_file.flush();

  // Initialise the I2C bus
  master.begin(400 * 1000U);
  Serial.println("I2C initialized.");
  log_pair("MSG", "I2C init done");
  log_file.flush(); 

  prep_3463();
  log_file.flush();

  Serial.println("Sending CAN message");
  log_pair("MSG", "Sending CAN test message");
  msg.id = 0x12;
  msg.buf[0] = 12;
  msg.buf[1] = 13;
  can1.write(msg);

}

void loop(void) {
  
  read_3463();

  read_CAN();

  if(micros() > next_analog_read_micros){
    log_pair("A1",String(analogRead(A1)));
    next_analog_read_micros = micros() + ANALOG_READ_MICROS_INCR;
  }

  log_file.flush();

  if(micros() > next_sd_write_micros){
    new_log_file();
    next_sd_write_micros = micros() + SD_MICROS_INCR;
  }
  
}

//Initialize the adafruit 3463
void prep_3463(){
  
  next_imu_micros = micros();
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

void new_log_file(){

  log_pair("MSG", "closing file");
  log_file.close();
  Serial.println("Making new log file...");
  Serial.println("Finding unused log_name...");
  for(int a = 0; a < 10; a++){
    for(int b = 0; b < 10; b++){
      for(int c = 0; c < 10; c++){
        for(int d = 0; d < 10; d++){
          log_name[5] = (char) (a + 48);
          log_name[6] = (char) (b + 48);
          log_name[7] = (char) (c + 48);
          log_name[8] = (char) (d + 48);
          if(!SD.exists(log_name)){
            goto filefound;
          }
        }
      }
    }
  }
  Serial.println("ERROR: All possible file names taken");
  
  filefound:
  log_file = SD.open(log_name, FILE_WRITE);
  Serial.print("Starting log file ");
  Serial.println(log_name);
  if(log_file){
    Serial.println("Log file started.");
    log_pair("LOGNAME", String(log_name));
  }else{
    Serial.println("Could not create log file.");
  }
}

void prep_SD(){
  
  Serial.println("Initializing SD card...");
  if(SD.begin(BUILTIN_SDCARD)){
    Serial.println("SD card initialization complete.");
  }else{
    Serial.println("ERROR: Failed SD card initialization!");
  }
  
  Serial.println("Finding unused directory...");
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
  log_name[0] = directory_name[0];
  log_name[1] = directory_name[1];
  log_name[2] = directory_name[2];
  log_name[3] = directory_name[3];
  SD.mkdir(directory_name);
  log_file = SD.open(log_name, FILE_WRITE);
  Serial.print("Starting log files at ");
  Serial.println(directory_name);
  if(log_file){
    Serial.println("Log file started.");
    log_pair("MSG", "DIRNAME:"+String(directory_name));
  }else{
    Serial.println("Could not create log file.");
  }

  log_pair("VER", String(LOG_VERSION));
}

//Read data for the adafruit 3463 if enough time has passed
void read_3463(){

  //If current time is past time for next reading, do reading
  if(micros() < next_imu_micros) return;
  
  //Storage for the 7 bytes that will be sent back by each communication
  uint8_t response[7];
  //Sum of shifted MSB and LSB
  int16_t total;
  //Data scaled to a more understandable unit
  int32_t converted_total_x;
  int32_t converted_total_y;
  int32_t converted_total_z;
  int32_t converted_total;

  //Get data from accelerometer output registers
  acc_mag.read(0x00, response, 7, false);
  
  if(!(response[0] & 0b111)){
    log_pair("MSG", "Accelerometer not ready");
    log_pair("ACCS", String(response[0] + 0x100, BIN));
  } else{
    //combine 6 bits MSB in response[1] with 8 bits LSB in [2]
    total = (int16_t)(((response[1] << 8) | response[2])) >> 2;
    //convert from arbitrary bit value to ug and log
    converted_total_x = total;
    converted_total_x *= 488;

    total = (int16_t)(((response[3] << 8) | response[4])) >> 2;
    converted_total_y = total;
    converted_total_y *= 488;

    total = (int16_t)(((response[5] << 8) | response[6])) >> 2;
    converted_total_z = total;
    converted_total_z *= 488;

    log_file.print("ACC,");
    log_file.print(micros());
    log_file.print(',');
    log_file.print(converted_total_x);
    log_file.print(',');
    log_file.print(converted_total_y);
    log_file.print(',');
    log_file.println(converted_total_z);

    #ifdef LOG_DATA_TO_SERIAL
    Serial.print("ACC,");
    Serial.print(micros());
    Serial.print(',');
    Serial.print(converted_total_x);
    Serial.print(',');
    Serial.print(converted_total_y);
    Serial.print(',');
    Serial.println(converted_total_z);
    #endif
  }

  //Get data from gyroscope output registers
  gyr.read(0x00, response, 7, false);
  
  if(!(response[0] & 0b111)){
    log_pair("MSG", "Gyroscope not ready");
    log_pair("GYRS", String(response[0] + 0x100, BIN));
  } else{

    //combine 8 bits MSB in response[1] with 8 bits LSB in [2]
    total = (int16_t)(((response[1] << 8) | response[2]));
    //convert from arbitrary bit value to mdps and log
    converted_total = total;
    converted_total = converted_total * 125 / 16;
    log_pair("GYRX", String(converted_total));
  
    total = (int16_t)(((response[3] << 8) | response[4]));
    converted_total = total;
    converted_total = converted_total * 125 / 16;
    log_pair("GYRY", String(converted_total));
  
    total = (int16_t)(((response[5] << 8) | response[6]));
    converted_total = total;
    converted_total = converted_total * 125 / 16;
    log_pair("GYRZ", String(converted_total));

  }

  //Set time for next reading
  next_imu_micros = max(micros(), next_imu_micros + IMU_MICROS_INCR);
    
}

void read_CAN(){

  while ( can1.read(msg) ) {

    unsigned long can_micros = micros();
    char data_string[100] = {0};
    sprintf(data_string,"CAN,%d,%d,%08X,%02X%02X%02X%02X%02X%02X%02X%02X",can_micros,msg.flags.extended,msg.id,msg.buf[0],msg.buf[1],msg.buf[2],msg.buf[3],msg.buf[4],msg.buf[5],msg.buf[6],msg.buf[7]);

    #ifdef LOG_DATA_TO_SERIAL
    Serial.println(data_string);
    #endif
    
    log_file.println(data_string);

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
