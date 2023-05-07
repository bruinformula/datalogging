// 0xC000000, 0xC100000, 0xC200000, or 0xC300000 depending on which PCB it is
// 0 is front right, 1 is front left, 2 is back left, 3 is back right
#define BOARD_INDEX 3
//#define LOG_CAN
//#define LOG_LIN_POT
//#define LOG_TIRE_TEMP

//costnats r probalby inportant
#include "constants.h"

// custom mlx90640 low-memory code
#include "mlx90640.hpp"

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
uint32_t nextSGMicros;
uint32_t nextBrkTempMicros;
uint32_t nextTireTempMicros;
uint32_t nextToggMicros;

bool ledState;

//init adc objects
Adafruit_ADS1115 ADCA;
Adafruit_ADS1115 ADCB;


//send data array
uint8_t canMsg[8];

CAN_msg_data tireTempIndex;

MCP_CAN CAN0(9);     // Declare CAN controller

void setup() {
  Serial.begin(115200);
  Serial.println("----------");
  Serial.print(F("Starting Corner PCB "));
  Serial.println(BOARD_INDEX);
  
  Wire.setClock( 400000UL);
  
  pinMode(BLINK_LED_PIN, OUTPUT);
  //init lin pot
  pinMode(LIN_POT_PIN, INPUT);
  
  Serial.println("Starting ADCs");
  //init ADCs shit
  ADCA.begin(ADCA_ADDR);  // Initialize ADC A at address 0x48 
  ADCA.setGain(GAIN_SIXTEEN);
  ADCB.begin(ADCB_ADDR);  // Initialize ADC B at address 0x49
  ADCB.setGain(GAIN_SIXTEEN);
  Serial.println("ADC Initialization complete.");

  //init can shit (500kbps, 16 MHZ)
  Serial.println(F("Initializing MCP2515"));
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println(F("MCP2515 Initialized Successfully!"));
  else 
    Serial.println(F("Error Initializing MCP2515..."));
  CAN0.setMode(MCP_NORMAL);

  //init tiretemp
  Serial.println(F("Starting up the MLX..."));
  Serial.println(MLX90640_init() ? "...Succeeded!" : "...Failed!");

  //ok done initializing w
  digitalWrite(BLINK_LED_PIN, HIGH);
  Serial.println("This concludes our preflight checks. Welcome aboard!");
}

// calls (most) tasks, each task that is called handles its own frame rate to avoid doing stuff too often
void loopWithoutTireTemp() {
  toggLED();
  readLP();
  //readSGs();
  readBrakeTemp();
}

// calls tasks, each task that is called handles its own frame rate to avoid doing stuff too often
void loop() {
  loopWithoutTireTemp();
  readTireTemp();
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

//helper method for readTireTemp and readBrakeTemp
void printBlock(uint8_t* block) {
  for (int i = 0; i < BLOCK_SIZE; i++) {
    if (block[i] < 0x10) {
      Serial.print("0");
    }
    Serial.print(block[i], HEX);
  }
  Serial.println();
}

int ID_FOR_TIRETEMP_BLOCKS = 0;

// warning, this function does not check length, so there must be exactly 8 bytes
// (the length of a CAN message body) in the block, no more and no less
void tireTempOverCAN(uint8_t* block) {
  loopWithoutTireTemp(); // in case we've been stuck on this task for a long time, give the other tasks a chance
#ifdef LOG_CAN
  Serial.println("Sending tire temp block over CAN...");
#endif
  for(int block_offset = 0; block_offset < BLOCK_SIZE; block_offset += 8){
    int res;
    do {
      res = CAN0.sendMsgBuf(CAN_ID_FRAME + CAN_ID_TIRE_DATA + (++ID_FOR_TIRETEMP_BLOCKS % 0x100), 1, 8, block + block_offset);
      if (res) {
        delay(5);
        loopWithoutTireTemp(); // in case we've been stuck on this task for a long time, give the other tasks a chance
      }
    } while (res);
//    if (res) Serial.println("Got non-zero result for attempt to send tire temp block!");
//    Serial.println(res);
//    delay(10);
#ifdef LOG_CAN
    if (res) Serial.println("Got non-zero result for attempt to send tire temp block!");
#endif
  }
}

 void readTireTemp() {
  ID_FOR_TIRETEMP_BLOCKS = 0;

  if(micros() < nextTireTempMicros)return;
#ifdef LOG_TIRE_TEMP
  Serial.print(F("Sending CAN delim for start of tire temp "));
  Serial.println((long) tireTempIndex.integer, 16);
#endif
  CAN0.sendMsgBuf(CAN_ID_FRAME + CAN_ID_TIRE_START, 1, 8, tireTempIndex.bytes);

  int result;
  uint8_t msg[8];
#ifdef LOG_TIRE_TEMP
  Serial.println("Reading first tire temp half-frame");
#endif
  result = MLX90640_dumpFrameData(/*printBlock*/ tireTempOverCAN);
  msg[0] = (uint32_t) result;
#ifdef LOG_TIRE_TEMP
  Serial.println(result);
  Serial.println("Reading second tire temp half-frame");
#endif
  result = MLX90640_dumpFrameData(/*printBlock*/ tireTempOverCAN);
  msg[1] = (uint32_t) result;
#ifdef LOG_TIRE_TEMP
  Serial.println(result);
#endif
  
  CAN0.sendMsgBuf(CAN_ID_FRAME + CAN_ID_TIRE_END, 1, 8, tireTempIndex.bytes);

  CAN0.sendMsgBuf(CAN_ID_FRAME + CAN_ID_TIRE_ERR, 1, 8, msg);

  tireTempIndex.integer++;

  nextTireTempMicros = micros() + TIRE_TEMP_READ_INT;

 }

 void readBrakeTemp() {
  if(micros() < nextBrkTempMicros)return;

  // insert code here
  nextBrkTempMicros = micros() + BRK_TEMP_READ_INT;
 }
