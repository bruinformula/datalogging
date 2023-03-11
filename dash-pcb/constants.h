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

//I2C addresses
const uint8_t ACC_MAG_ADDR = 0x1F;
const uint8_t GYR_ADDR = 0x21;

// pins
const uint8_t FWD_PIN = 4;              // forward solenoid
const uint8_t RVS_PIN = 5;              // reverse solenoi
const uint8_t FLATSHIFT_PIN = 6;        // flatshift pin
const uint8_t LED_PIN = 13;             // dash LEDs
const uint8_t SHIFT_LIGHTS_PIN = 14;    // level translator for shift lights
const uint8_t DOWNSHIFT_PIN = 30;       // downshift paddle
const uint8_t UPSHIFT_PIN = 31;         // upshift paddle
const uint8_t WHEEL_SPARE_PIN = 32;     // spare wheel pin

//microseconds between each reading
#define IMU_MICROS_INCR             1400
#define IMU_CHECK_MICROS_INCR     100000
#define SD_MICROS_INCR          10000000
#define ANALOG_READ_MICROS_INCR   200000
#define REALTIME_TELE_MICROS_INCR  90000
#define SHIFT_LIGHT_MICROS_INCR    30000
#define STATUS_MICROS_INCR       1000000
#define SHIFT_UPDATE_INCR           1000
#define SHIFT_PNEUMATIC_TIME       50000
#define SHIFT_PAUSE_TIME          250000

// shifting state definition
#define UPSHIFTING 1
#define DOWNSHIFTING 2
#define NEUTRAL 3
