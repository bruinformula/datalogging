#include "mlx90640.hpp"

// warning, this function does not check length, so there must be exactly
// 8 bytes in the block, no more and no less
void printEightBytes(uint8_t* block) {
  for (int i = 0; i < 8; i++) {
    if (block[i] < 0x10) {
      Serial.print("0");
    }
    Serial.print(block[i], HEX);
  }
  Serial.println();
}

// warning, this function does not check length, so there must be exactly 8 bytes
// (the length of a CAN message body) in the block, no more and no less
void transmitOverCAN(uint8_t* block) {
  // uhh help
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting up the MLX...");
  Serial.println(MLX90640_init() ? "...Succeeded!" : "...Failed!");
  Serial.println("Look, it's the EEPROM");
  MLX90640_dumpEeprom(printEightBytes);
}

void loop() {
  Serial.println("Look, it's the first half-frame");
  MLX90640_dumpFrameData(printEightBytes);
  Serial.println("Look again, it's the second half-frame");
  MLX90640_dumpFrameData(printEightBytes);
  delay(5000);
}
