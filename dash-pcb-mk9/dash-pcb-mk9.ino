#include "constants.h"

//Standard libraries
#include <stdint.h>
#include <Arduino.h>

//I2C communication library
#include <i2c_device.h>

//r2d system library
#include <Wire.h>
#include "Adafruit_TPA2016.h"

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

// 7 segment library
#include <Adafruit_LEDBackpack.h>

// interrupt and avr libs
#include <avr/io.h>
#include <avr/interrupt.h>

//GPS libraries
// #include <TinyGPS.h>
// #include <SoftwareSerial.h>
// #include <TimeLib.h>

#define LOG_DATA_TO_SERIAL
#define LOG_VERSION 3

// The I2C device. Determines which pins we use on the Teensy.
I2CMaster& master = Master2;
//objects for the i2c sensors (adafruit 3463)
I2CDevice acc_mag = I2CDevice(master, ACC_MAG_ADDR, _BIG_ENDIAN);
I2CDevice gyr = I2CDevice(master, GYR_ADDR, _BIG_ENDIAN);
I2CDevice egt_amp = I2CDevice(master, EGT_AMP_ADDR, _BIG_ENDIAN);

I2CDevice speaker = I2CDevice(master, SPEAKER_ADDR, _BIG_ENDIAN);

//GPS objects
// TinyGPS gps;
// String newNMEA = "";
// String GPGGANMEA = "";
// String GPRMCNMEA = "";

// tele
byte teleData[32];
#define Serial_TELE Serial4
#include <Base64.h> // Base64 by Arturo Guadalupi

FlexCAN_T4<CAN2, RX_SIZE_16, TX_SIZE_16> can2;
CAN_message_t msg;

//0: working, 1: uninitialized, 2: initialized no file, 3: initialization failed, 4: file creation failed
uint8_t SD_status = 1;
//time of last ECU message
uint32_t last_ecu_can_micros;

//path to log file on sd card
char log_name[] = { 'X', 'X', 'X', 'X', '.', 'c', 's', 'v', '\0' };
File log_file;

//next time an action should be taken
uint32_t next_imu_micros;
uint32_t next_sd_write_micros;
uint32_t next_analog_read_micros;
uint32_t next_realtime_tele_micros;
uint32_t next_shift_light_frame_micros;
uint32_t next_status_micros;
uint32_t next_egt_micros;
uint32_t next_flush_micros;
uint32_t next_shift_update_time_micros;
// uint32_t next_gps_micros;


// timings & state vars for shifting
// uint32_t pneumatic_state_change_micros;  // when we turn off pneumatic/flatshift after starting
uint32_t next_allowed_shift_micros;      // when we allow next shift to be executed after shift
uint32_t upshift_delay_micros;           // when we allow upshift to physically begin
int8_t shiftState;                       // track state of shifter system
int8_t desiredGear;                      // desired gear to shift to
uint8_t upshiftPrev;                     // previous upshift paddle state
uint8_t downshiftPrev;                   // previous downshift paddle state

// for shift light color control
uint32_t color_red, color_green, color_blue, color_yellow, color_white, color_purple;

uint8_t gear = 0;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(8, SHIFT_LIGHTS_PIN, NEO_GRB + NEO_KHZ800);


Adafruit_NeoPixel charge_strip = Adafruit_NeoPixel(8, CHARGE_STATUS_PIN, NEO_GRB + NEO_KHZ800);


bool egt_warn = false;

// speaker amp control
Adafruit_TPA2016 audioAmp = Adafruit_TPA2016();



void setup(void) {

  // shift light color and object initiation
  pinMode(SHIFT_LIGHTS_PIN, OUTPUT);
  strip.begin();
  strip.show();
  color_red = strip.Color(100, 0, 0);
  color_green = strip.Color(0, 40, 0);
  color_blue = strip.Color(0, 0, 100);
  color_yellow = strip.Color(100, 50, 0);
  color_white = strip.Color(57, 57, 57);
  color_purple = strip.Color(70, 0, 12);

  strip.clear();
  strip.setPixelColor(0, color_blue);
  strip.setPixelColor(1, color_blue);
  strip.show();

  //Set up communication with plugged in laptop if there is one
  
  Serial.begin(115200);
  strip.setPixelColor(2, color_blue);
  strip.show();

  //Turn on on-board LED, wait for Serial
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(200);
  strip.setPixelColor(3, color_blue);
  strip.setPixelColor(4, color_blue);

  Serial.println("---BEGIN---");
  Serial.println("");

  strip.setPixelColor(5, color_blue);
  strip.show();

  prep_SD();

  strip.setPixelColor(6, SD_status > 0 ? color_blue : color_red);
  strip.setPixelColor(7, SD_status > 0 ? color_blue : color_red);
  strip.show();

  //Initialize the CAN bus at 500kbps
  Serial.println("Initializing CAN bus...");
  can2.begin();

  strip.setPixelColor(8, color_blue);
  strip.setPixelColor(9, color_blue);
  strip.show();

  can2.setBaudRate(500000);
  Serial.println("CAN bus initialized.");
  log_pair("MSG", "CAN init done");
  log_file.flush();

  strip.setPixelColor(10, color_blue);
  strip.setPixelColor(11, color_blue);
  strip.show();

  // Initialise the I2C bus at 100kbps
  master.begin(100 * 1000U);
  Serial.println("I2C initialized.");
  log_pair("MSG", "I2C init done");
  log_file.flush();

  strip.setPixelColor(12, color_blue);
  strip.setPixelColor(13, color_blue);
  strip.show();

  // Initialize accelerometer and gyroscope
  prep_3463();
  log_file.flush();

  strip.setPixelColor(14, color_blue);
  strip.show();

  Serial.println("Sending CAN message");
  log_pair("MSG", "Sending CAN test message");
  msg.id = 0x12;
  msg.buf[0] = 12;
  msg.buf[1] = 13;
  can2.write(msg);

  strip.setPixelColor(15, color_blue);
  strip.show();

  //Set up R2D system
  Serial.begin(9600);
  pinMode(R2D_PIN, INPUT_PULLUP);
  pinMode(dacPin, OUTPUT);

  // Telemetry (serial port)
  Serial_TELE.begin(9600);  

  // setup shifting and set default state
  pinMode(UPSHIFT_PIN, INPUT_PULLUP);    // upshift paddle
  pinMode(DOWNSHIFT_PIN, INPUT_PULLUP);  // downshift paddle
  pinMode(FWD_PIN, OUTPUT);              // solenoid FWD pin
  pinMode(RVS_PIN, OUTPUT);              // solenoid RVSS pin
  pinMode(FLATSHIFT_PIN, OUTPUT);        // ecu "Shift Switch" pin
  pinMode(WHEEL_SPARE_PIN, OUTPUT);      // spare input pin on wheel
  digitalWrite(WHEEL_SPARE_PIN, LOW);    // use spare input pin as a ground

  // setup charge indicator strip
  pinMode(CHARGE_STATUS_PIN, INPUT_PULLUP);
  charge_strip.begin();

  charge_strip.show();

  //Set up GPS
  // gpsSerial.begin(9600);

  delay(200);
}


void loop(void) {
  read_3463();
  read_CAN();
  resetPneumatics();
  read_EGT();
  // read_GPS();
  wireless_tele();
  send_log_status();
  read_CAN();
  flush();
  read_CAN();
  update_shift_lights();

  if(digitalRead(R2D_PIN) == LOW) {
    sendSpeakerSignal();
  }

  update_charge_lights();


  shift();
}

//Initialize the SD card and make a new folder
void prep_SD() {

  //Initialize the SD card reader built into the Teensy
  Serial.println("Initializing SD card...");
  if (SD.begin(BUILTIN_SDCARD)) {
    SD_status = 2;
    Serial.println("SD card initialization complete.");
  } else {
    SD_status = 3;
    Serial.println("ERROR: Failed SD card initialization!");
  }

  SD_status = 4;

  Serial.println("Finding unused file...");
  // Set a, b, c, d to the first available 4 digits for a directory name
  for (int a = 0; a < 10; a++) {
    for (int b = 0; b < 10; b++) {
      for (int c = 0; c < 10; c++) {
        for (int d = 0; d < 10; d++) {
          log_name[0] = (char)(a + 48);
          log_name[1] = (char)(b + 48);
          log_name[2] = (char)(c + 48);
          log_name[3] = (char)(d + 48);
          if (!SD.exists(log_name)) {
            goto filefound;
          }
        }
      }
    }
  }
  Serial.println("ERROR: All possible names taken");

filefound:

  log_file = SD.open(log_name, FILE_WRITE);
  Serial.print("Starting log file at ");
  Serial.println(log_name);
  if (log_file) {
    Serial.println("Log file started.");
    if (SD_status == 2) SD_status = 0;
    log_pair("LOGNAME", log_name);
  } else {
    Serial.println("Could not create log file.");
  }

  log_pair("VER", String(LOG_VERSION));
  log_file.flush();
}

//Read data for the adafruit 3463 if enough time has passed
void read_3463() {

  //If current time is past time for next reading, do reading
  if (micros() < next_imu_micros) return;

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

  if (!(response[0] & 0b111)) {
    log_pair("MSG", "ANR");
    prep_3463();
    next_imu_micros = micros() + 1000000;
    return;
  } else {
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
    log_pair("ACC", log_message);

    for (uint8_t i=0;i<6;i++){
      teleData[i] = response[i+1];
    }
  }

  //Get data from gyroscope output registers
  gyr.read(0x00, response, 7, false);

  if (!(response[0] & 0b111)) {
    log_pair("MSG", "GNR");
    prep_3463();
    next_imu_micros = micros() + 1000000;
    return;
  } else {

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
    log_pair("GYR", log_message);
    for (uint8_t i=0;i<6;i++){
      teleData[i+6] = response[i+1];
    }
  }

  //Set time for next reading
  next_imu_micros = max(micros(), next_imu_micros + IMU_MICROS_INCR);
}

void read_EGT() {

  //If current time is past time for next reading, do reading
  if (micros() < next_egt_micros) return;

  uint8_t response[2];

  egt_amp.read(0b00100000, response, 2, false);
  if (response[0] != 64) {
    log_pair("MSG", "No EGT AMP");
  } else {
    //Get data from accelerometer output registers
    egt_amp.read(0x00, response, 2, false);

    //prepare CAN message
    msg.id = 0x00DA5401;

    msg.buf[0] = response[0];
    msg.buf[1] = response[1];

    can2.write(msg);

    //Scale according to datasheet of MCP9000
    short egt_bits = (response[0] << 8) + response[1];
    float egt = egt_bits / 16.0;

    log_pair("EGT", egt);
    egt_warn = egt > EGT_THRES;
  }
  next_egt_micros = micros() + EGT_MICROS_INCR;
}

//Initialize the adafruit 3463
void prep_3463() {

  next_imu_micros = micros() + IMU_MICROS_INCR;
  uint8_t result_byte;

  //This register is the id of the device, which should always be 0xC7
  acc_mag.read(ACC_MAG_WHOAMI_REG, &result_byte, false);

  if (result_byte != 0xC7) {
    //wrong or no id, print error
    log_file.print("ERR,am whoami,");
    log_file.println(result_byte, HEX);
  } else {
    log_file.println("MSG,acc_mag found");
  }

  //This register is the id of the device, which should always be 0b11010111
  acc_mag.read(GYR_WHOAMI_REG, &result_byte, false);

  if (result_byte != 0b11010111) {
    log_file.print("ERR,am whoami,");
    log_file.println(result_byte, HEX);
  } else {
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
void read_CAN() {

  //  Serial.println("looply can scan");
  while (can2.read(msg)) {
    //    Serial.print("MSGID: ");
    //    Serial.print(msg.id, HEX);
    //    Serial.print(";\tMSG: ");
    //    for (int i = 0; i < msg.len; i++) {
    //      Serial.print(msg.buf[i]);
    //    }
    //    Serial.println();

    //create an empty string
    char data_string[100] = { 0 };
    //fill the string with the formatted data
    sprintf(data_string, "%i,%08lX,%02X%02X%02X%02X%02X%02X%02X%02X",
            msg.flags.extended, msg.id, msg.buf[0], msg.buf[1], msg.buf[2],
            msg.buf[3], msg.buf[4], msg.buf[5], msg.buf[6], msg.buf[7]);

    //log that with CAN message name and timestamp attached
    log_pair("CAN", data_string);
    //Serial8.println(data_string);

    //check if message is from ecu by seeing if first 3 bytes match
    bool message_is_from_ecu = (msg.id & 0x001F0A000) == 0x01F0A000;
    if (message_is_from_ecu) {
      last_ecu_can_micros = micros();
    }

    //update values to be sent over serial for certain messages
    //tele: need to convert to actual data
    if (msg.id == 0x01F0A000) {
      /*
        msg.buf[i]:
        0-1 engine speed (RPM)
        2-3 engine load
        4-5 throttle
        6   intake temp
        7   coolant temp
      */ 
      for (uint8_t i=0;i<8;i++){
        teleData[i+12] = msg.buf[i];
      }

    } else if (msg.id == 0x01F0A003) {
      /*
        msg.buf[i]:
        0   lambda
        5   DBTDC
        6-7 VBATT
      */ 
      teleData[20] = msg.buf[0];
      teleData[21] = msg.buf[5];
      teleData[22] = msg.buf[6];
      teleData[23] = msg.buf[7];
      gear = msg.buf[4];
    } else if (msg.id == 0x01F0A004) {
      /*
        msg.buf[i]:
        0-1 MAP
        3   FPR
        5   TO2
        6   1LSB:fuelPump, 2LSB:FAN
      */ 
      teleData[24] = msg.buf[0];
      teleData[25] = msg.buf[1];
      teleData[26] = msg.buf[3];
      teleData[27] = msg.buf[5];
      teleData[28] = (msg.buf[6] & 0b00000011);
    } else if (msg.id == 0x01F0A006) {
      teleData[29] = msg.buf[2];  //injduty
    }
  }
}

//log message like "s1,micros,s2\n"
void log_pair(String s1, String s2) {
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
void send_log_status() {

  if (micros() < next_status_micros) return;
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
  if (tenths_since_last_ecu_msg > 255) tenths_since_last_ecu_msg = 255;
  uint8_t tenths_byte = tenths_since_last_ecu_msg;

  msg.buf[5] = tenths_byte;
  msg.buf[6] = 0;
  msg.buf[7] = 0;

  can2.write(msg);
}

void flush() {
  if (micros() > next_flush_micros) {
    log_file.flush();

    next_flush_micros = micros() + SD_MICROS_INCR;
  }
}

//send string for wireless telementry
void wireless_tele() {
  if (micros() > next_realtime_tele_micros) {
    int inputStringLength = sizeof(teleData);
    int encodedLength = Base64.encodedLength(inputStringLength);
    char encodedString[encodedLength];
    Base64.encode(encodedString, teleData, inputStringLength);
    for(uint8_t i=0;i<encodedLength;i++)
      Serial_TELE.print(encodedString[i]);
    Serial_TELE.println();
    next_realtime_tele_micros = micros() + REALTIME_TELE_MICROS_INCR;
  }
}
void update_shift_lights() {

  if (micros() > next_shift_light_frame_micros) {

    //Get RPM from most recent CAN data
    uint32_t RPM = (teleData[0] * 256 + teleData[1]) * 0.39063f;

    //At low RPM (ie car stopped), show gear in blue instead of showing RPM.
    if (RPM < 600) {
      //If gear is neutral, light up all lights white
      if (gear == 0) {
        for (uint8_t i = 0; i < 16; i++) {
          //set to yellow if cranking, blue otherwise
          strip.setPixelColor(i, (RPM > 50) ? color_yellow : color_white);
        }
      }

      //If in gear, light up a number of lights to indicate which
      else {
        for (uint8_t i = 0; i < (gear * 3); i += 3) {
          strip.setPixelColor(i, color_blue);
          strip.setPixelColor(i + 1, 0);
          strip.setPixelColor(i + 2, 0);
        }
        for (uint8_t i = (gear * 3); i < 16; i++) {
          strip.setPixelColor(i, 0);
        }
      }
    } else {

      int number_of_lights;
      if (RPM < SL_RPM_MIN) {
        number_of_lights = 1;
      } else {
        number_of_lights = (RPM - SL_RPM_MIN) * 7 / (SL_RPM_MAX - SL_RPM_MIN) + 1;
      }

      if (number_of_lights > 8)
        number_of_lights = 8;

      //Light up the specified number of lights, color depending
      //on which light it is
      for (uint8_t i = 0; i < number_of_lights; i++) {
        if (i < 4) {
          strip.setPixelColor(i, color_green);
          strip.setPixelColor(15 - i, color_green);
        } else if (i < 7) {
          strip.setPixelColor(i, color_yellow);
          strip.setPixelColor(15 - i, color_yellow);
        } else {
          strip.setPixelColor(i, color_red);
          strip.setPixelColor(15 - i, color_red);
        }
      }

      //Turn off the other lights
      for (uint8_t i = number_of_lights; i < 8; i++) {
        strip.setPixelColor(i, 0);
        strip.setPixelColor(15 - i, 0);
      }

      //if rpm is high enough, turn off for some
      if (RPM > SL_RPM_BLINK && micros() % SL_BLINK_LENGTH > SL_BLINK_OFF) {
        strip.clear();
      }
    }
    strip.show();

    next_shift_light_frame_micros = micros() + SHIFT_LIGHT_MICROS_INCR;
  }
}

void shift() {
  if (micros() > next_shift_update_time_micros) {
    uint8_t upshiftCurr = digitalRead(UPSHIFT_PIN);
    uint8_t downshiftCurr = digitalRead(DOWNSHIFT_PIN);

    if (!upshiftCurr && upshiftPrev) upshiftStart();             // if upshift paddle pressed & debounce timer inactive
    else if (!downshiftCurr && downshiftPrev) downshiftStart();  // if downshift paddle pressed & debounce timer inactive

    switch (shiftState) {
      case SHF_IDLE:
        //log_pair("SHF", 0);
        break;
      case DOWNSHIFTING:
        log_pair("SHF", -desiredGear);
        break;
      case UPSHIFTSTART:
        if (micros() > upshift_delay_micros) {
          digitalWrite(FWD_PIN, HIGH);
          shiftState = UPSHIFTING;
        }
        log_pair("SHF", desiredGear);
        break;
      case UPSHIFTING:
        log_pair("SHF", desiredGear);
        break;
      case SHF_NEUTRAL:
        log_pair("SHF", 6);
        break;
    }

    next_shift_update_time_micros = micros() + SHIFT_UPDATE_INCR;

    upshiftPrev = upshiftCurr;
    downshiftPrev = downshiftCurr;
  }
}

void upshiftStart() {
  if (micros() > next_allowed_shift_micros) {
    desiredGear = gear + 1;

    // block shifting past 5th
    if (desiredGear > 5) {
      return;  // do nothing (prevents damage to gearbox)
      // may need to change depending off the shift activates while gear state is counted as 7 (between gears)
    }

    // shift N->1
    else if (desiredGear == 1) {
      digitalWrite(RVS_PIN, HIGH);
      shiftState = UPSHIFTING;
      log_pair("SHF", desiredGear);
    }

    // upshift & signal flatshift
    else {
      digitalWrite(FLATSHIFT_PIN, HIGH);
      shiftState = UPSHIFTSTART;
      log_pair("SHF", desiredGear);
    }

    pneumatic_state_change_micros = micros() + UPSHIFT_DELAY_TIME[gear] + UPSHIFT_PNEUMATIC_TIME[gear];  // time when we deactivate both flatshift signal & solenoid
    upshift_delay_micros = micros() + UPSHIFT_DELAY_TIME[gear];                                          // when we actually actuate the upshift
    next_allowed_shift_micros = micros() + UPSHIFT_PAUSE_TIME;                                           // delay time until next shift allowed
  }
}

void downshiftStart() {
  if (micros() > next_allowed_shift_micros) {
    desiredGear = gear - 1;

    // block downshifting into 1st (handled by upshifting N->1 see above)
    if (desiredGear < 0) {
      return;  // do nothing (this would just shift us into gear, when not desired)
    }

    // shift 1->N
    if (desiredGear == 0) {
      digitalWrite(FWD_PIN, HIGH);
      shiftState = SHF_NEUTRAL;
      log_pair("SHF", 6);  // neutral shows up in logs as 6, to distinguish it from idle state
    }

    // downshift
    else {
      digitalWrite(RVS_PIN, HIGH);
      shiftState = DOWNSHIFTING;
      log_pair("SHF", -desiredGear);
    }

    pneumatic_state_change_micros = micros() + DOWNSHIFT_PNEUMATIC_TIME[desiredGear];  // when we turn off the pins after shifting
    next_allowed_shift_micros = micros() + DOWNSHIFT_PAUSE_TIME;                       // delay time until next shift allowed
  }
}

void resetPneumatics() {
  // reset all pins to low after shifting executed
  if ((micros() > pneumatic_state_change_micros && shiftState != SHF_IDLE) || (gear == desiredGear)) {
    digitalWrite(FWD_PIN, LOW);
    digitalWrite(RVS_PIN, LOW);
    digitalWrite(FLATSHIFT_PIN, LOW);
    shiftState = SHF_IDLE;
  }
}

void update_charge_lights() {
  // Read battery voltage
  float batteryVoltage = analogRead(CHARGE_STATUS_PIN) * (5.0 / 1023.0);

  // Determine battery charge level
  uint8_t chargeLevel = 0;
  if (batteryVoltage >= FULL_CHARGE) {
    chargeLevel = 3; // Full charge
  } else if (batteryVoltage >= MED_CHARGE) {
    chargeLevel = 2; // Medium charge
  } else if (batteryVoltage >= LOW_CHARGE) {
    chargeLevel = 1; // Low charge
  } else {
    chargeLevel = 0; // Very low charge
  }

  display_battery_indicator(chargeLevel);
}

void display_battery_indicator(uint8_t chargeLevel) {
  charge_strip.clear();
  
  switch (chargeLevel) {
    case 3:
      charge_strip.setPixelColor(0, charge_strip.Color(255, 0, 0));
      charge_strip.setPixelColor(1, charge_strip.Color(255, 0, 0));
      charge_strip.setPixelColor(2, charge_strip.Color(255, 0, 0));
      charge_strip.setPixelColor(3, charge_strip.Color(100, 50, 0));
      charge_strip.setPixelColor(4, charge_strip.Color(100, 50, 0));
      charge_strip.setPixelColor(5, charge_strip.Color(0, 40, 0));
      charge_strip.setPixelColor(6, charge_strip.Color(0, 40, 0));
      charge_strip.setPixelColor(7, charge_strip.Color(0, 40, 0));
    case 2:
      charge_strip.setPixelColor(5, charge_strip.Color(0, 0, 0));
      charge_strip.setPixelColor(6, charge_strip.Color(0, 0, 0));
      charge_strip.setPixelColor(7, charge_strip.Color(0, 0, 0));

      break;
    case 1:
      charge_strip.setPixelColor(3, charge_strip.Color(0, 0, 0));
      charge_strip.setPixelColor(4, charge_strip.Color(0, 0, 0));
      
      break;
    case 0:
      if (millis() % 1000 < 500) {
          strip.setPixelColor(0, charge_strip.Color(255, 0, 0));
          strip.setPixelColor(1, charge_strip.Color(255, 0, 0));
      }

      break;
  }

  charge_strip.show();
}
