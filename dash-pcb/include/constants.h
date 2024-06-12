#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <SoftwareSerial.h>

// Adddresses for various registers
#define ACC_MAG_WHOAMI_REG        0x0D
#define ACC_MAG_CTRL_REG_1        0x2A
#define ACC_MAG_XYZ_DATA_CFG_REG  0x0El

#define SPK_EN_REG 0x01

#define GYR_WHOAMI_REG            0x0C
#define GYR_CTRL_REG0             0x0D
#define GYR_CTRL_REG1             0x13

#define EGT_AMP_TEMP_REG          0x00
#define EGT_AMP_STATUS_REG        0x04

// I2C addresses
#define ACC_MAG_ADDR              0x1F
#define GYR_ADDR                  0x21
#define EGT_AMP_ADDR              0x67
#define SPEAKER_ADDR              0xB0

// Pin Definitions
#define FLATSHIFT_PIN     4       // flatshift pin
#define RVS_PIN           5       // reverse solenoid
#define FWD_PIN           6       // forward solenoid
#define LED_PIN           13      // dash LEDs
#define SHIFT_LIGHTS_PIN  4       // level translator for shift lights
#define RPM_LIGHTS_PIN    2       // level translator for motor RPM lights
#define SOC_LIGHTS_PIN    3       // level translator for accumulator SoC lights
#define SOC_AIN_PIN       4       // analog signal from BMS to dash giving accumulator SoC
#define DOWNSHIFT_PIN     30      // downshift paddle
#define UPSHIFT_PIN       31      // upshift paddle
#define WHEEL_SPARE_PIN   32      // spare wheel pin
#define RTD_PIN           19      // RTD Button
#define SPEAKERS_PIN      0

// Microseconds between each periodic function
#define IMU_MICROS_INCR            50000  // how often IMU samples
#define IMU_CHECK_MICROS_INCR     100000  // not used
#define EGT_MICROS_INCR           100000  // how often the EGT samples
#define SD_MICROS_INCR          10000000  // how often we force a flush to the SD card
#define ANALOG_READ_MICROS_INCR   200000  // how often read_A1 samples (no longer used)
#define REALTIME_TELE_MICROS_INCR  90000  // how often we send packets to real time telemetry transmitter
#define SHIFT_LIGHT_MICROS_INCR    30000  // how often the shift light updates
#define STATUS_MICROS_INCR       1000000  // how often send dash PCB status to CANbus
#define SHIFT_UPDATE_INCR           1000  // how often shifter checks if it needs to change state
#define GPS_MICROS_INCR          1000000  // how often GPS data is logged
#define RTD_MICROS_INCR            50000  // 20Hz update rate for RTD

//GPS serial port
#define gpsSerial Serial4

// timing for shifting
const uint32_t UPSHIFT_PNEUMATIC_TIME[5] = {75000, 75000, 75000, 75000, 75000};         // per gear timing {N->1, 1->2, 2->3, 3->4, 4->5}
const uint32_t DOWNSHIFT_PNEUMATIC_TIME[7] = {8420, 100000, 100000, 100000, 100000, 0, 0};    // per gear timing {1->N, 2->1, 3->2, 4->3, 5->4}
const uint32_t UPSHIFT_DELAY_TIME[5] = {0, 0, 0, 0, 0};                                 // per gear delay {N->1, 1->2, 2->3, 3->4, 4->5}
const uint32_t SHIFT_DEBOUNCE_TIME = 2000;      // debounce for button
const uint32_t UPSHIFT_PAUSE_TIME = 200000;     // timing between paddle presses to avoid saturating the piston
const uint32_t DOWNSHIFT_PAUSE_TIME = 200000;   // timing between paddle presses to avoid saturating the piston


// shifting state variables
#define SHF_IDLE 0          // shifter is doing nothing
#define DOWNSHIFTING -1     // shifter is downshifting
#define UPSHIFTING 1        // shifter is upshifting
#define UPSHIFTSTART 2      // shifter signals flatshift, but solenoid is idle
#define SHF_NEUTRAL 3       // shifter is trying to find neutral

// shift light configuration (TODO: RFG)
// where it starts lighting up beyond 1 light
const uint32_t SL_RPM_MIN = 5000;
// where it is fully lit up
const uint32_t SL_RPM_MAX = 9500;
// where it starts blinking
const uint32_t SL_RPM_BLINK = 10500;
// length between each blink, us
const uint32_t SL_BLINK_LENGTH = 250000;
// how much of the blink is off, us
const uint32_t SL_BLINK_OFF = 160000;

// when to blink shift lights red
const int32_t EGT_THRES = 900;

// DAC Pin
const int DACPin = A0;

#define DAC_RESOLUTION (8)