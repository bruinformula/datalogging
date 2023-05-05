#pragma once

//pin shit
#define LIN_POT_PIN A0
#define BLINK_LED_PIN 2


//sensor read interval shit
#define LIN_POT_READ_INT    50000
#define SG_READ_INT         42069
#define BRK_TEMP_READ_INT   50000
#define TIRE_TEMP_READ_INT  50000
#define BLINK_LED_INT       1000000

#define ADCA_ADDR 0x48
#define ADCB_ADDR 0x49

/*                 ***CAN ID CONSTNATS INFO***
 * id of pcb: 0x[XXXX][#]C0
 * C0: IDs board as a corner PCB
 * [#]: 1/2/3/4 for which # corner PCB it is
 * [XXXX]: 4 digits to ID what kind of sensor readout it is
 * C0 is front left C1 is front right C2 is rear right C3 is rear left
 */
#define CAN_ID_FRAME      0xC000000
#define CAN_ID_LIN_POT    0x0001000
#define CAN_ID_ADCA       0x0002000
#define CAN_ID_ADCB       0x0003000
#define CAN_ID_BRK_TEMP   0x0004000
#define CAN_ID_TIRE_TEMP  0x0005000
