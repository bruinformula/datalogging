// 0xC000000, 0xC100000, 0xC200000, or 0xC300000 depending on which PCB it is
// 0 is front right, 1 is front left, 2 is back left, 3 is back right
#define BOARD_INDEX 3
//#define LOG_CAN
//#define LOG_LIN_POT
//#define LOG_TIRE_TEMP

//costnats r probalby inportant
#include "constants.h"

//stadard shit
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
uint32_t nextBrkTempMicros;
uint32_t nextToggMicros;
uint32_t nextWhsMicros;

volatile uint32_t lastWhsWidth = 0xFFFFFFFF;
volatile uint32_t lastWhsStart = 0;
volatile bool whsState;

bool ledState;

//send data array
uint8_t canMsg[8];

MCP_CAN CAN0(9);     // Declare CAN controller

void setup() {
  Serial.begin(115200);
  Serial.println("----------");
  Serial.print(F("Starting Corner PCB "));
  Serial.println(BOARD_INDEX);
  
  Wire.setClock(400000UL);
  
  pinMode(BLINK_LED_PIN, OUTPUT);
  pinMode(LIN_POT_PIN, INPUT);
  pinMode(WHS_PIN, INPUT);

  //init can shit (500kbps, 16 MHZ)
  Serial.println(F("Initializing MCP2515"));
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println(F("MCP2515 Initialized Successfully!"));
  else 
    Serial.println(F("Error Initializing MCP2515..."));
  CAN0.setMode(MCP_NORMAL);

  //ok done initializing w
  digitalWrite(BLINK_LED_PIN, HIGH);
  Serial.println("This concludes our preflight checks. Welcome aboard!");

  attachInterrupt(digitalPinToInterrupt(3), whsInt, RISING);
}

void loop() {
  toggLED();
  readLP();
  readBrakeTemp();
  readWhs();
}

void whsInt(){
  uint32_t currentTime = micros();
  lastWhsWidth = currentTime - lastWhsStart;
  lastWhsStart = currentTime;
}

void readWhs(){
  
  if (micros() < nextWhsMicros) return;
  
  uint8_t msg[8] = {
      (uint8_t)((lastWhsWidth & 0xFF000000) >> 24), 
      (uint8_t)((lastWhsWidth & 0x00FF0000) >> 16), 
      (uint8_t)((lastWhsWidth & 0x0000FF00) >> 8), 
      (uint8_t)((lastWhsWidth & 0x000000FF) >> 0), 
      0, 0, 0, 0};
  CAN0.sendMsgBuf(CAN_ID_FRAME + CAN_ID_WHS, 1, 8, msg);
  
  nextWhsMicros = micros() + WHS_SEND_INT;
  
}

//when its time has come: read linpot
void readLP() {
  if (micros() < nextLPMicros) return;
  
  //get data
  uint16_t anaOut = analogRead(LIN_POT_PIN);
#ifdef LOG_LIN_POT
  Serial.print(F("Reading Linear Potentiometer: "));
  Serial.println(anaOut);
#endif

  uint8_t msg[8] = {(uint8_t)(anaOut >> 8), (uint8_t)(anaOut), 0, 0, 0, 0, 0, 0};
  CAN0.sendMsgBuf(CAN_ID_FRAME + CAN_ID_LIN_POT, 1, 8, msg);

  nextLPMicros = micros() + LIN_POT_READ_INT;
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

void readBrakeTemp() {
  if(micros() < nextBrkTempMicros)return;

  // insert code here
  nextBrkTempMicros = micros() + BRK_TEMP_READ_INT;
 }
