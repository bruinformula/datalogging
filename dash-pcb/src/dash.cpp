#include "constants.h"

// Standard libraries
#include <Arduino.h>
#include <stdint.h>

// I2C communication library
#include <i2c_device.h>
#include <SPI.h>

// RTD sound
#include "Adafruit_TPA2016.h"
#include "AudioSamples.h"
#include <Audio.h>

// CAN libraries
#include <FlexCAN_T4.h>
#include <circular_buffer.h>
#include <imxrt_flexcan.h>
#include <isotp.h>
#include <isotp_server.h>

// SD Library
#include <SD.h>

// shift light strip library
#include <Adafruit_NeoPixel.h>

// interrupt and avr libs
#include <avr/interrupt.h>
#include <avr/io.h>

#define LOG_DATA_TO_SERIAL
#define LOG_VERSION 4

// The I2C device. Determines which pins we use on the Teensy.
I2CMaster& master = Master2;
// objects for the i2c sensors (adafruit 3463)
// I2CDevice acc_mag = I2CDevice(master, ACC_MAG_ADDR, _BIG_ENDIAN);
// I2CDevice gyr = I2CDevice(master, GYR_ADDR, _BIG_ENDIAN);
I2CDevice speaker = I2CDevice(master, SPEAKER_ADDR, _BIG_ENDIAN);

FlexCAN_T4<CAN3, RX_SIZE_16, TX_SIZE_16> can3;
CAN_message_t msg;

// 0: working, 1: uninitialized, 2: initialized no file, 3: initialization
// failed, 4: file creation failed
uint8_t SD_status = 1;
// time of last ECU message
uint32_t last_ecu_can_micros;

// path to log file on sd card
char log_name[] = "XXXX.csv";
File log_file;

// next time an action should be taken
uint32_t next_sd_write_micros;
uint32_t next_analog_read_micros;
uint32_t next_realtime_tele_micros;
uint32_t next_shift_light_frame_micros;
uint32_t next_status_micros;
uint32_t next_egt_micros;
uint32_t next_flush_micros;
uint32_t next_rtd_micros;

// for shift light color control
uint32_t color_red, color_green, color_blue, color_yellow, color_white,
    color_purple;

Adafruit_NeoPixel strip =
    Adafruit_NeoPixel(8, SHIFT_LIGHTS_PIN, NEO_GRB + NEO_KHZ800);

// function instantiation
void prep_SD();
void read_CAN();
void log_pair(String, String);
void send_log_status();
void flush();
void check_RTD();

// speaker amp control
Adafruit_TPA2016 audioAmp = Adafruit_TPA2016();
AudioPlayMemory    sound0;
AudioPlayMemory    sound1;  // six memory players, so we can play
AudioPlayMemory    sound2;  // all six sounds simultaneously
AudioPlayMemory    sound3;
AudioPlayMemory    sound4;
AudioPlayMemory    sound5;
AudioMixer4        mix1;    // two 4-channel mixers are needed in
AudioMixer4        mix2;    // tandem to combine 6 audio sources
AudioOutputI2S     headphones;
AudioOutputAnalog  dac;     // play to both I2S audio board and on-chip
AudioOutputAnalogStereo  dacs1;          //xy=361,179


// Create Audio connections between the components
AudioConnection c1(sound0, 0, mix1, 0);
AudioConnection c2(sound1, 0, mix1, 1);
AudioConnection c3(sound2, 0, mix1, 2);
AudioConnection c4(sound3, 0, mix1, 3);
AudioConnection c5(mix1, 0, mix2, 0);   // output of mix1 into 1st input on mix2
AudioConnection c6(sound4, 0, mix2, 1);
AudioConnection c7(sound5, 0, mix2, 2);
AudioConnection c8(mix2, 0, headphones, 0);
AudioConnection c9(mix2, 0, headphones, 1);
AudioConnection c10(mix2, 0, dac, 0);

AudioControlSGTL5000 audioShield;

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

  // Set up communication with plugged in laptop if there is one
  Serial.begin(115200);
  strip.setPixelColor(2, color_blue);
  strip.show();

  // Turn on on-board LED, wait for Serial
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

  // Initialize the CAN bus at 500kbps
  Serial.println("Initializing CAN bus...");
  can3.begin();

  strip.setPixelColor(8, color_blue);
  strip.setPixelColor(9, color_blue);
  strip.show();

  can3.setBaudRate(500000);
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

  // Audio connections require memory to work.  For more
  // detailed information, see the MemoryAndCpuUsage example
  AudioMemory(5);

  // turn on the output
  audioShield.enable();
  audioShield.volume(10);

  // reduce the gain on mixer channels, so more than 1
  // sound can play simultaneously without clipping
  mix1.gain(0, 20);
  mix1.gain(1, 20);
  mix1.gain(2, 20);
  mix1.gain(3, 20);
  mix2.gain(1, 20);
  mix2.gain(2, 20);
  
  audioAmp.begin();
  audioAmp.setReleaseControl(0);
  audioAmp.setAGCCompression(TPA2016_AGC_2);
  audioAmp.setLimitLevelOff();
  audioAmp.setAttackControl(5);
  audioAmp.setHoldControl(0);
  audioAmp.setReleaseControl(11);


  strip.setPixelColor(14, color_blue);
  strip.show();

  Serial.println("Sending CAN message");
  log_pair("MSG", "Sending CAN test message");
  msg.id = 0x12;
  msg.buf[0] = 12;
  msg.buf[1] = 13;
  can3.write(msg);

  strip.setPixelColor(15, color_blue);
  strip.show();

  // setup RTD pins
  pinMode(RTD_PIN, INPUT_PULLUP);
  pinMode(DACPin, OUTPUT);

  delay(200);
}

void loop(void) {
  read_CAN();
  send_log_status();
  read_CAN();
  flush();
  read_CAN();
  check_RTD();
}

// Initialize the SD card and make a new folder
void prep_SD() {
  // Initialize the SD card reader built into the Teensy
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

// Check all CAN messages, log them, and send some to telemetry
void read_CAN() {
  //  Serial.println("looply can scan");
  while (can3.read(msg)) {
    //    Serial.print("MSGID: ");
    //    Serial.print(msg.id, HEX);
    //    Serial.print(";\tMSG: ");
    //    for (int i = 0; i < msg.len; i++) {
    //      Serial.print(msg.buf[i]);
    //    }
    //    Serial.println();

    // create an empty string
    char data_string[100] = {0};
    // fill the string with the formatted data
    sprintf(data_string, "%i,%08lX,%02X%02X%02X%02X%02X%02X%02X%02X",
            msg.flags.extended, msg.id, msg.buf[0], msg.buf[1], msg.buf[2],
            msg.buf[3], msg.buf[4], msg.buf[5], msg.buf[6], msg.buf[7]);

    // log that with CAN message name and timestamp attached
    log_pair("CAN", data_string);
    // Serial8.println(data_string);

    // check if message is from ecu by seeing if first 3 bytes match
    bool message_is_from_ecu = (msg.id & 0x001F0A000) == 0x01F0A000;
    if (message_is_from_ecu) {
      last_ecu_can_micros = micros();
    }

    
  }
}

// log message like "s1,micros,s2\n"
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

// send which log we're on over CAN
void send_log_status() {
  if (micros() < next_status_micros) return;
  next_status_micros = micros() + STATUS_MICROS_INCR;

  msg.id = 0x00DA5400;
  // first 4 bytes are the first 4 bytes of the log name (ie the directory, eg
  // 0143)
  msg.buf[0] = log_name[0];
  msg.buf[1] = log_name[1];
  msg.buf[2] = log_name[2];
  msg.buf[3] = log_name[3];
  // next byte is SD status, 0 if all is well
  msg.buf[4] = SD_status;

  uint32_t tenths_since_last_ecu_msg =
      (micros() - last_ecu_can_micros) / 100000;
  if (tenths_since_last_ecu_msg > 255) tenths_since_last_ecu_msg = 255;
  uint8_t tenths_byte = tenths_since_last_ecu_msg;

  msg.buf[5] = tenths_byte;
  msg.buf[6] = 0;
  msg.buf[7] = 0;

  can3.write(msg);
}

void flush() {
  if (micros() > next_flush_micros) {
    log_file.flush();

    next_flush_micros = micros() + SD_MICROS_INCR;
  }
}

void check_RTD() {
  if(micros() < next_rtd_micros) return;
  next_rtd_micros = micros() + RTD_MICROS_INCR; 
  if(digitalRead(RTD_PIN) == LOW) {
    sound0.play(AudioSampleMetalpipe);
  }
}