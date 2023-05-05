/*
 * to add:::
 * 
 * sht pants case for each read method
 */


//costnats r probalby inportant
#include "constants.h"

//stadrad shit
#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>

//CAN shit
#include <mcp_can.h>
#include <SPI.h>

//device specific shit
#include <Adafruit_ADS1X15.h>

<<<<<<< HEAD
//randall's i2c lib cause hes cool like that
#include "mlx90640.hpp"

/*                 ***CAN ID CONSTNATS INFO***
 * id of pcb: 0x[XXXX][#]C0
 * C0: IDs board as a corner PCB
 * [#]: 1/2/3/4 for which # corner PCB it is
 * [XXXX]: 4 digits to ID what kind of sensor readout it is
 * C0 is front left C1 is front right C2 is rear right C3 is rear left
 */
#define CAN_ID_FRAME    = 0xC000000
#define CAN_ID_LIN_POT  = 0x0001000
#define CAN_ID_ADCA  = 0x0002000
#define CAN_ID_ADCB  = 0x0003000
#define CAN_ID_BRK_TEMP  = 0x0004000
#define CAN_ID_TIRE_TEMP  = 0x0005000

=======
>>>>>>> 4e4eaffc11a029984fc7920f14be7a0f94fc2327
//next time reads should be read (in microseconds)
uint32_t nextLPMicros;
uint32_t nextSGMicros;
uint32_t nextBrkTempMicros;
uint32_t nextTireTempMicros;
uint32_t nextToggMicros;

bool ledState = false;

//init adcs
Adafruit_ADS1115 ADCA;
Adafruit_ADS1115 ADCB;

//init can object
MCP_CAN CAN(10);

//send data array
uint8_t canMsg[8];

<<<<<<< HEAD
void setup() {
  Serial.begin(9600);
  Serial.println("Good morning, folks. Quick preflight checks:");
  //light that says "hi im working + initializing + booting (silly LED on the right of mcu)
  digitalWrite(BLINK_LED_PIN, HIGH);
  pinMode(BLINK_LED_PIN, OUTPUT);
  
=======
MCP_CAN CAN0(10);     // Declare CAN controller
>>>>>>> 4e4eaffc11a029984fc7920f14be7a0f94fc2327

void setup() {

  //light that says "hi im working + initializing + booting (silly LED on the right of mcu)
  pinMode(BLINK_LED_PIN, OUTPUT);
  //init lin pot
  pinMode(LIN_POT_PIN, INPUT);

<<<<<<< HEAD
  //init ADC shit
  Serial.println("\n Initializing ADCA: ... ");
  Serial.println(ADCA.begin(0x48) ? "Complete." : "FAILED");  // Initialize ADC A at address 0x48
  ADCA.setGain(GAIN_SIXTEEN);
  Serial.println("Initializing ADCB: ... ");
  Serial.println(ADCB.begin(0x49) ? "Complete." : "FAILED");  // Initialize ADC B at address 0x49
=======
  Serial.begin(9600);


  //init i2c shit
  ADCA.begin(ADCA_ADDR);  // Initialize ADC A at address 0x48
  ADCA.setGain(GAIN_SIXTEEN);
  ADCB.begin(ADCB_ADDR);  // Initialize ADC B at address 0x49
>>>>>>> 4e4eaffc11a029984fc7920f14be7a0f94fc2327
  ADCB.setGain(GAIN_SIXTEEN);
  Serial.println("ADC Initialization complete.");

  //init ir sensor shit
  Serial.println("Starting up the MLX...");
  Serial.println(MLX90640_init() ? "...Succeeded!" : "...Failed!");
  
  //init can shit (500kbps, 16 MHZ)
<<<<<<< HEAD
  Serial.println("Booting up CAN bus...");
  if(CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) Serial.println("Booted successfully!");
  else Serial.println("Error Initializing MCP2515...");
  CAN.setMode(MCP_NORMAL);
=======
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else 
    Serial.println("Error Initializing MCP2515...");
  CAN0.setMode(MCP_NORMAL);
>>>>>>> 4e4eaffc11a029984fc7920f14be7a0f94fc2327


  //ok done initializing w
<<<<<<< HEAD
  digitalWrite(BLINK_LED_PIN, LOW);
  Serial.println("This concludes our preflight checks. Welcome aboard!");
=======
  digitalWrite(BLINK_LED_PIN, HIGH);
>>>>>>> 4e4eaffc11a029984fc7920f14be7a0f94fc2327
}

void loop() {
  toggLED();
  readLP();
  //readSGs();
  readTireTemp();
  readBrakeTemp();
}

//when its time has come: read linpot
void readLP() {
  if (micros() < nextLPMicros) return;
  
  //get data
  uint16_t anaOut = analogRead(LIN_POT_PIN);
  Serial.print(F("Reading Linear Potentiometer: "));
  Serial.println(anaOut);
  
  Serial.print(F("Sending over CAN: "));
  uint8_t msg[8] = {(byte)(anaOut), (byte)(anaOut >> 8), (byte)(anaOut >> 16), 0, 0, 0, 0, 0};
  uint8_t canResult = CAN0.sendMsgBuf(CAN_ID_FRAME + CAN_ID_LIN_POT, 1, 8, msg);
  Serial.println(canResult);

  nextLPMicros += LIN_POT_READ_INT;
}

//read from the 2 ADCs which have 3 strain gauges each
void readSGs() {
  if (micros() < nextSGMicros) return;
  int16_t SGA0 = ADCA.readADC_Differential_0_3();
  int16_t SGA1 = ADCA.readADC_Differential_1_3();
  int16_t SGA2 = ADCA.readADC_Differential_2_3();
  
  int8_t msgA[8] = {(byte)(SGA0), (byte)(SGA0 >> 8), (byte)(SGA1), (byte)(SGA1 >> 8), 
                  (byte)(SGA2), (byte)(SGA2 >> 8), 0, 0};
  
  CAN0.sendMsgBuf(CAN_ID_FRAME + CAN_ID_ADCA, 1, 8, msgA);

  int16_t SGB0 = ADCB.readADC_Differential_0_3();
  int16_t SGB1 = ADCB.readADC_Differential_1_3();
  int16_t SGB2 = ADCB.readADC_Differential_2_3();
  
  int8_t msgB[8] = {(byte)(SGB0), (byte)(SGB0 >> 8), (byte)(SGB1), (byte)(SGB1 >> 8), 
                  (byte)(SGB2), (byte)(SGB2 >> 8), 0, 0};
  
  CAN0.sendMsgBuf(CAN_ID_FRAME + CAN_ID_ADCB, 1, 8, msgB);

  nextSGMicros = micros() + SG_READ_INT;
}

void toggLED(){
  if (micros() < nextToggMicros) 
    return;
  
  if (ledState) 
    digitalWrite(BLINK_LED_PIN, LOW);
  else 
    digitalWrite(BLINK_LED_PIN, HIGH);

  ledState = !ledState;

  nextToggMicros = micros() + BLINK_LED_INT;
}
<<<<<<< HEAD
void readTireTemp() {
  
}

void readBrakeTemp() {
  
}

//helper method for readTireTemp and readBrakeTemp
void printEightBytes(uint8_t* block) {
  for (int i = 0; i < 8; i++) {
    if (block[i] < 0x10) {
      Serial.print("0");
    }
    Serial.print(block[i], HEX);
  }
  Serial.println();
}
=======

 void readTireTemp() {
 }
 void readBrakeTemp() {
 }
 void readADCA(){}
 void readADCB(){}
>>>>>>> 4e4eaffc11a029984fc7920f14be7a0f94fc2327
