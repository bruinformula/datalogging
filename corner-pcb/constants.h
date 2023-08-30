#pragma once

//stadard shit
#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>

//pin shit
#define LIN_POT_PIN A0
#define BLINK_LED_PIN 2
#define WHS_PIN 3

union CAN_msg_data{
  uint64_t integer;
  uint8_t bytes[8];
};


//sensor read interval shit
#define LIN_POT_READ_INT    10000
#define SG_READ_INT         42069
#define BRK_TEMP_READ_INT   50000
#define TIRE_TEMP_READ_INT  2000000
#define BLINK_LED_INT       200000
#define WHS_SEND_INT        50000

#define ADCA_ADDR 0x48
#define ADCB_ADDR 0x49

/* CAN ID CONSTANTS INFO
 * id of pcb: 0xC[#]0[XXXX]
 * C0: IDs board as a corner PCB
 * [#]: 1/2/3/4 for which # corner PCB it is
 * [XXXX]: 4 digits to ID what kind of sensor readout it is
 * C0 is front left C1 is front right C2 is rear right C3 is rear left
 */

#if BOARD_INDEX == 0
  #define CAN_ID_FRAME      0xC000000
#elif BOARD_INDEX == 1
  #define CAN_ID_FRAME      0xC100000
#elif BOARD_INDEX == 2
  #define CAN_ID_FRAME      0xC200000
#else
  #define CAN_ID_FRAME      0xC300000
#endif

#define CAN_ID_LIN_POT    0x0001000
#define CAN_ID_ADCA       0x0002000
#define CAN_ID_ADCB       0x0003000
#define CAN_ID_BRK_TEMP   0x0004000
#define CAN_ID_TIRE_START 0x0005000
#define CAN_ID_TIRE_DATA  0x0006000
#define CAN_ID_TIRE_END   0x0007000
#define CAN_ID_TIRE_ERR   0x0008000
#define CAN_ID_WHS        0x0009000
