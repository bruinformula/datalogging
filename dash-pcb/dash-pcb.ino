#include "constants.h"

//Standard libraries
#include <stdint.h>
#include <Arduino.h>

//I2C communication library
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

// shift light strip library
#include <Adafruit_NeoPixel.h>

#define LOG_DATA_TO_SERIAL
#define LOG_VERSION 3

// The I2C device. Determines which pins we use on the Teensy.
I2CMaster& master = Master;
//objects for the i2c sensors (adafruit 3463)
I2CDevice acc_mag = I2CDevice(master, ACC_MAG_ADDR, _BIG_ENDIAN);
I2CDevice gyr = I2CDevice(master, GYR_ADDR, _BIG_ENDIAN);
I2CDevice egt_amp = I2CDevice(master, EGT_AMP_ADDR, _BIG_ENDIAN);

int tele_data_1B[15]; // intList: ACCX, ACCY, ACCZ, GYRX, GYRY, GYRZ, exTemp(C), intakeTemp(C), coolantTemp(C), lambda1, dbtdc, fpr, to2, injduty
int tele_data_2B[5][2]; // list for 2-byte data: engineSpeed(RPM), engineLoad(%), throttle(%), MAP(kPa), V_BAT
bool tele_data_fan1; // fan 1 status
bool tele_data_fpump; // fuel pump status

FlexCAN_T4<CAN2, RX_SIZE_16, TX_SIZE_16> can2;
CAN_message_t msg;

//0: working, 1: uninitialized, 2: initialized no file, 3: initialization failed, 4: file creation failed
uint8_t SD_status = 1;
//time of last ECU message
uint32_t last_ecu_can_micros;

//path to log file on sd card
char log_name[] = {'X','X','X','X','/','l','o','g','.','c','s','v','\0'};
char directory_name[] = {'X','X','X','X','\0'};
File log_file;

//next time an action should be taken
uint32_t next_imu_micros;
uint32_t next_sd_write_micros;
uint32_t next_analog_read_micros;
uint32_t next_realtime_tele_micros;
uint32_t next_shift_light_frame_micros;
uint32_t next_status_micros;
uint32_t next_egt_micros;

// timings & state vars for shifting
uint32_t next_shift_update_time_micros;
uint32_t pneumatic_state_change_micros;
uint32_t next_allowed_shift_micros;
uint32_t upshiftLastStates = 0x0; // revert to uint_64
uint32_t downshiftLastStates = 0x0; // revert to uint_64
uint8_t shiftDirection;

// for shift light color control
uint32_t color_red, color_green, color_blue, color_yellow, color_white, color_purple;

uint8_t gear = 0;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(16, SHIFT_LIGHTS_PIN, NEO_GRB + NEO_KHZ800);

bool egt_warn = false;

void setup(void) {

  // shift light color and object initiation
  pinMode(SHIFT_LIGHTS_PIN, OUTPUT);
  strip.begin();
  strip.show(); 
  color_red = strip.Color(100,  0,  0);
  color_green = strip.Color(  0, 40,  0);
  color_blue = strip.Color(  0,  0, 100);
  color_yellow = strip.Color( 100, 50,  0);
  color_white = strip.Color( 57, 57, 57);
  color_purple = strip.Color(100, 0, 60);

  strip.clear();
  strip.setPixelColor(0, color_white);
  strip.setPixelColor(1, color_white);
  strip.show();

  //Set up communication with plugged in laptop if there is one
  Serial.begin(115200);
  strip.setPixelColor(2, color_white);
  strip.show();

  //Turn on on-board LED, wait for Serial
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(200);
  strip.setPixelColor(3, color_white);
  strip.setPixelColor(4, color_white);
  
  Serial.println("---BEGIN---");
  Serial.println("");

  strip.setPixelColor(5, color_white);
  strip.show();
  
  prep_SD();
  
  strip.setPixelColor(6, SD_status>0 ? color_white : color_red);
  strip.setPixelColor(7, SD_status>0 ? color_white : color_red);
  strip.show();
  
  //Initialize the CAN bus at 500kbps
  Serial.println("Initializing CAN bus...");
  can2.begin();

  strip.setPixelColor(8, color_white);
  strip.setPixelColor(9, color_white);
  strip.show();

  can2.setBaudRate(500000);
  Serial.println("CAN bus initialized.");
  log_pair("MSG", "CAN init done");
  log_file.flush();
  
  strip.setPixelColor(10, color_white);
  strip.setPixelColor(11, color_white);
  strip.show();
  
  // Initialise the I2C bus at 100kbps
  master.begin(100 * 1000U);
  Serial.println("I2C initialized.");
  log_pair("MSG", "I2C init done");
  log_file.flush();
  
  strip.setPixelColor(12, color_white);
  strip.setPixelColor(13, color_white);
  strip.show();
  
  // Initialize accelerometer and gyroscope
  prep_3463();
  log_file.flush();
  
  strip.setPixelColor(14, color_white);
  strip.show();
  
  Serial.println("Sending CAN message");
  log_pair("MSG", "Sending CAN test message");
  msg.id = 0x12;
  msg.buf[0] = 12;
  msg.buf[1] = 13;
  can2.write(msg);
  
  strip.setPixelColor(15, color_white);
  strip.show();
  
  Serial8.begin(9600); // to ESP8266 Telemetry

  // setup shifting and set default state
  pinMode(FWD_PIN, OUTPUT);
  pinMode(RVS_PIN, OUTPUT);
  pinMode(FLATSHIFT_PIN, OUTPUT);
  pinMode(UPSHIFT_PIN, INPUT_PULLUP);
  pinMode(DOWNSHIFT_PIN, INPUT_PULLUP);
  pinMode(WHEEL_SPARE_PIN, OUTPUT);
  digitalWrite(WHEEL_SPARE_PIN, LOW);
  
  delay(200);

}

void loop(void) {
  
  read_3463();
  read_CAN();
  read_A1();
  read_EGT();
  wireless_tele();
  log_file.flush();
  update_shift_lights();
  shift();
  
}

//Initialize the SD card and make a new folder
void prep_SD(){

  //Initialize the SD card reader built into the Teensy
  Serial.println("Initializing SD card...");
  if(SD.begin(BUILTIN_SDCARD)){
    SD_status = 2;
    Serial.println("SD card initialization complete.");
  }else{
    SD_status = 3;
    Serial.println("ERROR: Failed SD card initialization!");
  }

  SD_status = 4;
  
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
    if(SD_status == 2) SD_status = 0;
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

void read_EGT(){
  
  //If current time is past time for next reading, do reading
  if(micros() < next_egt_micros) return;
  
  uint8_t response[2];
  //Get data from accelerometer output registers
  acc_mag.read(0x00, response, 2, false);
  //Scale according to datasheet of MCP9000
  short egt = response[0] << 8 + response[1];

  //prepare CAN message
  msg.id = 0x00DA5401;
  
  msg.buf[0] = response[0];
  msg.buf[1] = response[1];

  can2.write(msg);

  log_pair("EGT", egt);

  egt_warn = egt > EGT_THRES;

  next_egt_micros = micros() + EGT_MICROS_INCR;

}

//Initialize the adafruit 3463
void prep_3463(){
  
  next_imu_micros = micros() + IMU_MICROS_INCR;
  uint8_t result_byte;

  //This register is the id of the device, which should always be 0xC7
  acc_mag.read(ACC_MAG_WHOAMI_REG, &result_byte, false);
  
  if(result_byte != 0xC7){
    //wrong or no id, print error
    log_file.print("ERR,am whoami,");
    log_file.println(result_byte,HEX);
  }else{
    log_file.println("MSG,acc_mag found");
  }
  
  //This register is the id of the device, which should always be 0b11010111
  acc_mag.read(GYR_WHOAMI_REG, &result_byte, false);
  
  if(result_byte != 0b11010111){
    log_file.print("ERR,am whoami,");
    log_file.println(result_byte,HEX);
  }else{
    Serial.println("GYR sensor found");
    log_file.println("MSG,gyr found");
  }

  //byte to be written to device
  uint8_t msg;
  
  //set accelerometer range to +/- 4g
  //see datasheet for explanation of registers
  msg = 0b00000001;
  acc_mag.write(ACC_MAG_XYZ_DATA_CFG_REG, msg, false);
  //set accelerometer to active mode, max data rate
  msg = 0b00000001;
  acc_mag.write(ACC_MAG_CTRL_REG_1, msg, false);
  
  //set gyro range to +/- 250dps
  msg = 0b00000011;
  gyr.write(GYR_CTRL_REG0, msg, false);
  
  //set to 800Hz, active
  msg = 0b00000011;
  gyr.write(GYR_CTRL_REG1, msg, false);
}

//Check all CAN messages, log them, and send some to telemetry
void read_CAN(){

  while ( can2.read(msg) ) {
    
    //create an empty string
    char data_string[100] = {0};
    //fill the string with the formatted data
    sprintf(data_string,"%i,%08lX,%02X%02X%02X%02X%02X%02X%02X%02X",
    msg.flags.extended, msg.id, msg.buf[0], msg.buf[1], msg.buf[2],
    msg.buf[3], msg.buf[4], msg.buf[5], msg.buf[6], msg.buf[7]);
    
    //log that with CAN message name and timestamp attached
    log_pair("CAN",data_string);
    //Serial8.println(data_string);

    //check if message is from ecu by seeing if first 3 bytes match
    bool message_is_from_ecu = (msg.id & 0x001F0A000) == 0x01F0A000;
    if(message_is_from_ecu){
      last_ecu_can_micros = micros();
    }

    //update values to be sent over serial for certain messages
    if(msg.id == 0x01F0A000){
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
      tele_data_1B[10] = msg.buf[5]; //DBTDC
      tele_data_2B[4][0] = msg.buf[6]; tele_data_2B[3][1] = msg.buf[7]; //vbat
    }
    else if(msg.id == 0x01F0A004){
      tele_data_2B[3][0] = msg.buf[0]; tele_data_2B[3][1] = msg.buf[1]; // MAP
      tele_data_fan1 = (msg.buf[6] & 0b00000010); //fan
      tele_data_fpump = (msg.buf[6] & 0b00000001); //fuel pump
      tele_data_1B[11] = msg.buf[3]; //fpr
      tele_data_1B[12] = msg.buf[5]; //to2
    }
    else if(msg.id == 0x01F0A006){
      tele_data_1B[13] = msg.buf[2]; //injduty
    }
  }
}

//Read pin A1, connected to EGT, log it, and prepare it to be sent with telemetry
void read_A1(){
  
  if(micros() > next_analog_read_micros){
    tele_data_1B[6] = analogRead(A1);
    log_pair("A1", String(tele_data_1B[6]));
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

//send which log we're on over CAN
void send_log_status(){

  if(micros () < next_status_micros) return;
  next_status_micros = micros() + STATUS_MICROS_INCR;

  msg.id = 0x00DA5400;
  //first 4 bytes are the first 4 bytes of the log name (ie the directory, eg 0143)
  msg.buf[0] = log_name[0];
  msg.buf[1] = log_name[1];
  msg.buf[2] = log_name[2];
  msg.buf[3] = log_name[3];
  //next byte is SD status, 0 if all is well
  msg.buf[4] = SD_status;
  
  uint32_t tenths_since_last_ecu_msg = (micros() - last_ecu_can_micros) / 100000;
  if(tenths_since_last_ecu_msg > 255) tenths_since_last_ecu_msg = 255;
  uint8_t tenths_byte = tenths_since_last_ecu_msg;

  msg.buf[5] = tenths_byte;
  msg.buf[6] = 0;
  msg.buf[7] = 0;

  can2.write(msg);

}

//send string for wireless telementry
void wireless_tele(){
  
  if(micros() > next_realtime_tele_micros){
    
    char data_string[150] = {0};
    // ACCX,ACCY,ACCZ|GYRX,GYRY,GYRZ|A1|
    // engineSpeed(RPM)|engineLoad(%)|throttle(%)|
    // intakeTemp(C)|coolantTemp(C)|
    // lambda1|manifold_pressure(kPa)|fan1(bool)|
    // logFileName|targetO2|DBTDC|INJDUTY|VBAT|FPUMP|FPR
     // 0: ACC, 1: GYR, 2: EGT, 3: ENGSPD, 4: ENGLD, 5: TPS, 
      // 6: IAT, 7: CLT, 8: O2, 9: MAP, 10: FAN, 11: LOGNAME, 12: TO2, 13: DBTDC, 14: INJDUTY, 15: VBAT, 16: FPUMP, 17: FPR, 18: EGT
    sprintf(data_string,"%d,%d,%d|%d,%d,%d|%d|%d,%d|%d,%d|%d,%d|%d|%d|%d|%d,%d|%d|%s|%d|%d|%d|%d,%d|%d|%d|\n",
    tele_data_1B[0],    tele_data_1B[1],    tele_data_1B[2],    tele_data_1B[3],    tele_data_1B[4],
    tele_data_1B[5],    tele_data_1B[6],    tele_data_2B[0][0], tele_data_2B[0][1], tele_data_2B[1][0],
    tele_data_2B[1][1], tele_data_2B[2][0], tele_data_2B[2][1], tele_data_1B[7],    tele_data_1B[8],
    tele_data_1B[9],    tele_data_2B[3][0], tele_data_2B[3][1], tele_data_fan1,     log_name, 
    tele_data_1B[12],   tele_data_1B[10],   tele_data_1B[13],   tele_data_2B[4][0], tele_data_2B[4][1], 
    tele_data_fpump,    tele_data_1B[11]);
    Serial8.print(data_string);
    
    next_realtime_tele_micros = micros() + REALTIME_TELE_MICROS_INCR;
  }
}

void update_shift_lights(){

  if(micros() > next_shift_light_frame_micros){

    //Get RPM from most recent CAN data
    uint32_t RPM = (tele_data_2B[0][0] * 256 + tele_data_2B[0][1]) * 0.39063f;

    //At low RPM (ie car stopped), show gear in blue instead of showing RPM.
    if(RPM < 600){
      //If gear is neutral, light up all lights white
      if(gear == 0){
        for(uint8_t i=0; i<16; i++){
          //set to yellow if cranking, purple otherwise
          strip.setPixelColor(i,(RPM > 50) ? color_yellow : color_purple);
        }
      }

      //If in gear, light up a number of lights to indicate which
      else{
        for(uint8_t i = 0; i < (gear*3); i += 3){
          strip.setPixelColor(i, color_blue);
          strip.setPixelColor(i + 1, 0);
          strip.setPixelColor(i + 2, 0);
        }
        for(uint8_t i = (gear*3); i<16; i++){
          strip.setPixelColor(i, 0);
        }
      }
    }
    else {

      int number_of_lights;
      if(RPM < SL_RPM_MIN){
        number_of_lights = 1;        
      }else{
        number_of_lights = (RPM - SL_RPM_MIN) * 7 / (SL_RPM_MAX - SL_RPM_MIN) + 1;
      }
      
      if(number_of_lights > 8)
        number_of_lights = 8;

      //Light up the specified number of lights, color depending
      //on which light it is
      for(uint8_t i=0; i < number_of_lights; i++) {
        if(i<4){
          strip.setPixelColor(i, color_green);
          strip.setPixelColor(15 - i, color_green);
        }
        else if(i<7){
          strip.setPixelColor(i, color_yellow);
          strip.setPixelColor(15 - i, color_yellow);
        }
        else{
          strip.setPixelColor(i, color_red);
          strip.setPixelColor(15 - i, color_red);
        }
      }

      //Turn off the other lights
      for(uint8_t i=number_of_lights; i<8; i++) {
        strip.setPixelColor(i,0);
        strip.setPixelColor(15 - i,0);
      }

      //if rpm is high enough, turn off for some      
      if(RPM > SL_RPM_BLINK && micros() % SL_BLINK_LENGTH > SL_BLINK_OFF) {
        strip.clear();
      }

    }
    strip.show();
    
    next_shift_light_frame_micros = micros() + SHIFT_LIGHT_MICROS_INCR;
  }
}

void shift() {
  if(micros() > next_shift_update_time_micros) {
    bool upshiftPaddle = !digitalRead(UPSHIFT_PIN);
    bool downshiftPaddle = !digitalRead(DOWNSHIFT_PIN);
    
    // update state by shifting over bit array and adding latest state
    upshiftLastStates = (upshiftLastStates << 1) + upshiftPaddle;
    downshiftLastStates = (downshiftLastStates << 1) + downshiftPaddle;

    // require at least 59 cycles (59ms off paddle to shift again) & allowed shift time
    if (micros() > next_allowed_shift_micros && !(upshiftLastStates >> 5)  && !(downshiftLastStates >> 5)) {
      // currently do nothing if both pressed (will change for neutral behavior once switch implemented)
      // debounce by 5 latest values (5ms)
      if (((upshiftLastStates & 0x1F) == 0x1F) ^ ((downshiftLastStates & 0x1F) == 0x1F)) {
        if (upshiftPaddle) {
          digitalWrite(FWD_PIN, HIGH);
          digitalWrite(FLATSHIFT_PIN, HIGH);
          log_pair("SHIFTER", "UPSHIFT");
        } 
        else if (downshiftPaddle) {
          digitalWrite(RVS_PIN, HIGH);
          log_pair("SHIFTER", "DOWNSHIFT");
        }
        pneumatic_state_change_micros = micros() + SHIFT_PNEUMATIC_TIME;
        next_allowed_shift_micros = micros() + SHIFT_PAUSE_TIME;
      }
    }

    // reset all pins to low after shifting executed
    if (micros() > pneumatic_state_change_micros) {
      digitalWrite(FWD_PIN, LOW);
      digitalWrite(RVS_PIN, LOW);
      digitalWrite(FLATSHIFT_PIN, LOW);
    }

    next_shift_update_time_micros = micros() + SHIFT_UPDATE_INCR;
  }
}
