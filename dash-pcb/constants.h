#pragma once

#include <Arduino.h>
#include <stdint.h>

//adddresses for various registers
const uint8_t ACC_MAG_WHOAMI_REG = 0x0D;
const uint8_t ACC_MAG_CTRL_REG_1 = 0x2A;
const uint8_t ACC_MAG_XYZ_DATA_CFG_REG = 0x0El;

const uint8_t GYR_WHOAMI_REG =  0x0C;
const uint8_t GYR_CTRL_REG0 = 0x0D;
const uint8_t GYR_CTRL_REG1 = 0x13;

const uint8_t EGT_AMP_TEMP_REG = 0x00;
const uint8_t EGT_AMP_STATUS_REG = 0x04;

//I2C addresses
const uint8_t ACC_MAG_ADDR = 0x1F;
const uint8_t GYR_ADDR = 0x21;
const uint8_t EGT_AMP_ADDR = 0x67;

// pins
const uint8_t FWD_PIN = 4;              // forward solenoid
const uint8_t RVS_PIN = 5;              // reverse solenoi
const uint8_t FLATSHIFT_PIN = 6;        // flatshift pin
const uint8_t LED_PIN = 13;             // dash LEDs
const uint8_t SHIFT_LIGHTS_PIN = 14;    // level translator for shift lights
const uint8_t DOWNSHIFT_PIN = 30;       // downshift paddle
const uint8_t UPSHIFT_PIN = 31;         // upshift paddle
const uint8_t WHEEL_SPARE_PIN = 32;     // spare wheel pin

//microseconds between each function
#define IMU_MICROS_INCR             1400
#define IMU_CHECK_MICROS_INCR     100000
#define EGT_MICROS_INCR           100000
#define SD_MICROS_INCR          10000000
#define ANALOG_READ_MICROS_INCR   200000
#define REALTIME_TELE_MICROS_INCR  90000
#define SHIFT_LIGHT_MICROS_INCR    30000
#define STATUS_MICROS_INCR       1000000
#define SHIFT_UPDATE_INCR           1000
#define SHIFT_PNEUMATIC_TIME       75000
#define SHIFT_PAUSE_TIME          250000

// shifting state definition
#define UPSHIFTING 1
#define DOWNSHIFTING 2
#define NEUTRAL 3

//shift light configuration
//where it starts lighting up beyond 1 light
const uint32_t SL_RPM_MIN = 5000;
//where it is fully lit up
const uint32_t SL_RPM_MAX = 9500;
//where it starts blinking
const uint32_t SL_RPM_BLINK = 10500;
//length between each blink, us
const uint32_t SL_BLINK_LENGTH = 250000;
//how much of the blink is off, us
const uint32_t SL_BLINK_OFF = 160000;

//when to blink shift lights red
const int32_t EGT_THRES = 900;
