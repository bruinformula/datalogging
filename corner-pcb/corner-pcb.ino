
//costnats r probalby inportant
#include "constants.h"

//stadrad shit
#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>
#include <wiring.h>

//CAN shit
#include <mcp_can.h>
#include <SPI.h>

//device specific shit
#include <Adafruit_ADS1X15.h>

//sylers i2c lib cause hes cool like that
//#include whatever the new i2c library is
#include 

/*                 ***CAN ID CONSTNATS INFO***
 * id of pcb: 0x[XXXX][#]C0
 * C0: IDs board as a corner PCB
 * [#]: 1/2/3/4 for which # corner PCB it is
 * [XXXX]: 4 digits to ID what kind of sensor readout it is
 * C0 is front left C1 is front right C2 is rear right C3 is rear left
 */
#define CAN_ID_FRAME    = 0xC000000
#define CAN_ID_LIN_POT  = 0x0001000
#define CAN_ID_SG_01  = 0x0002000
#define CAN_ID_SG_02  = 0x0003000
#define CAN_ID_SG_03  = 0x0004000
#define CAN_ID_BRK_TEMP  = 0x0005000
#define CAN_ID_TIRE_TEMP  = 0x0006000

//next time reads should be read (in microseconds)
uint32_t nextLPMicros;
uint32_t nextSGMicros;
uint32_t nextBrkTempMicros;
uint32_t nextTireTempMicros;

//init adcs
Adafruit_ADS1115 ADCA;
Adafruit_ADS1115 ADCB;

//send data array
uint8_t canMsg[8];

void setup() {
  //light that says "hi im working + initializing + booting (silly LED on the right of mcu)
  digitalWrite(BLINK_LED_PIN, HIGH);
  pinMode(BLINK_LED_PIN, OUTPUT);
  Serial.begin(9600);

  //init lin pot
  pinMode(LIN_POT_PIN, INPUT);

  //init i2c shit
  ADCA.begin(0x48);  // Initialize ADC A at address 0x48
  ADCB.begin(0x49);  // Initialize ADC B at address 0x49
  
  //init can shit (500kbps, 16 MHZ)
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");
  CAN0.setMode(MCP_NORMAL);

  //init i2c here

  //ok done initializing w
  digitalWrite(BLINK_LED_PIN, LOW);
}

void loop() {
  readLP();
  readADCs();
}

//when its time has come: read linpot
void readLP() {
  if (micros() < nextLPMicros) return;
  
  //get data
  int anaOut = analogRead(LIN_POT_PIN);
  sendMsgBuf(CAN_ID_FRAME + CAN_ID_LIN_POT, 1, 8, {(byte)(anaOut), (byte)(anaOut >> 8), (byte)(anaOut >> 16), (byte)(anaOut >> 24), 0, 0, 0, 0}}

  nextLPMicros += LIN_POT_READ_INT;
}

void readADCs() {
  readADCA();
  readADCB();
}

void readADCA() {
  int16_t out;
  
  out = ADCA.readADC_Differential_0_3();
  
}

void readADCB() {

}
