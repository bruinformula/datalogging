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

//LED on teensy
const uint8_t LED_PIN = 13;
//forward solenoid
const uint8_t FWDS_PIN = 4;
//reverse solenoid
const uint8_t RVSS_PIN = 5;
//level translator for shift lights
const uint8_t SHIFT_LIGHTS_PIN = 6;


//microseconds between each reading
#define IMU_MICROS_INCR             1400
#define IMU_CHECK_MICROS_INCR     100000
#define SD_MICROS_INCR          10000000
#define ANALOG_READ_MICROS_INCR   200000
#define REALTIME_TELE_MICROS_INCR  90000
#define SHIFT_LIGHT_MICROS_INCR    30000
