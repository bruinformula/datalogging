// 0xC000000, 0xC100000, 0xC200000, or 0xC300000 depending on which PCB it is
// 0 is front right, 1 is front left, 2 is back left, 3 is back right
#define BOARD_INDEX 0

#if BOARD_INDEX == 0
  #define CAN_ID_FRAME      0xC000000
#elif BOARD_INDEX == 1
  #define CAN_ID_FRAME      0xC100000
#elif BOARD_INDEX == 2
  #define CAN_ID_FRAME      0xC200000
#else
  #define CAN_ID_FRAME      0xC300000
#endif

//costnats r probalby inportant
#include "constants.h"

//stadrad shit
#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>

//CAN shit
#include <mcp_can.h>
#include <SPI.h>

//adcs for strain gauges
#include <Adafruit_ADS1X15.h>

//next time reads should be read (in microseconds)
uint32_t nextLPMicros;
uint32_t nextSGMicros;
uint32_t nextBrkTempMicros;
uint32_t nextTireTempMicros;
uint32_t nextToggMicros;

bool ledState;

//init adcs
Adafruit_ADS1115 ADCA;
Adafruit_ADS1115 ADCB;

//send data array
uint8_t canMsg[8];

MCP_CAN CAN0(9);     // Declare CAN controller

void setup() {

  //light that says "hi im working + initializing + booting (silly LED on the right of mcu)
  pinMode(BLINK_LED_PIN, OUTPUT);
  //init lin pot
  pinMode(LIN_POT_PIN, INPUT);

  Serial.begin(115200);


  //init i2c shit
  ADCA.begin(ADCA_ADDR);  // Initialize ADC A at address 0x48 
  ADCA.setGain(GAIN_SIXTEEN);
  ADCB.begin(ADCB_ADDR);  // Initialize ADC B at address 0x49
  ADCB.setGain(GAIN_SIXTEEN);
  
  //init can shit (500kbps, 16 MHZ)
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else 
    Serial.println("Error Initializing MCP2515...");
  CAN0.setMode(MCP_NORMAL);

  //init i2c here

  //ok done initializing w
  digitalWrite(BLINK_LED_PIN, HIGH);
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
  uint8_t msg[8] = {(uint8_t)(anaOut >> 8), (uint8_t)(anaOut), 0, 0, 0, 0, 0, 0};
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
  
  uint8_t msgA[8] = {(uint8_t)(SGA0 >> 8), (uint8_t)(SGA0),(uint8_t)(SGA1 >> 8),  (uint8_t)(SGA1), 
                  (uint8_t)(SGA2 >> 8), (uint8_t)(SGA2), 0, 0};
  
  CAN0.sendMsgBuf(CAN_ID_FRAME + CAN_ID_ADCA, 1, 8, msgA);

  int16_t SGB0 = ADCB.readADC_Differential_0_3();
  int16_t SGB1 = ADCB.readADC_Differential_1_3();
  int16_t SGB2 = ADCB.readADC_Differential_2_3();
  
  uint8_t msgB[8] = {(uint8_t)(SGB0 >> 8), (uint8_t)(SGB0),(uint8_t)(SGB1 >> 8),  (uint8_t)(SGB1), 
                  (uint8_t)(SGB2 >> 8), (uint8_t)(SGB2), 0, 0};
  
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

 void readTireTemp() {
 }
 void readBrakeTemp() {
 }
