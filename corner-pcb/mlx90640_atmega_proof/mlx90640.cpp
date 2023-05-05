#include "mlx90640.hpp"

#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>

#include <Wire.h>

Adafruit_I2CDevice mlx90640(0x33, &Wire);

bool MLX90640_init() {
    return mlx90640.begin();
}

void MLX90640_I2CRead(uint16_t addr, uint8_t* data, int size) {
  /* the arduino and mlx90640 disagree on endianness, so we swap */ \
  /* the command bytes before sending them */ \
  addr = (addr << 8) | (addr >> 8); \
  mlx90640.write_then_read((uint8_t*) &addr, 2, data, size, false); \
}

// size must not exceed BLOCK_SIZE
void MLX90640_I2CWrite(uint16_t addr, uint8_t* data, int size) {
  if (size > BLOCK_SIZE) return;
  uint8_t cmd[2 + BLOCK_SIZE];
  cmd[0] = addr >> 8;
  cmd[1] = addr & 0x00FF;
  for (int i = 0; i < size; i++) {
    cmd[i+2] = data[i];
  }
  mlx90640.write(cmd, 2 + size, true);
}

// the block_stream supplied here must be able to handle 8-byte blocks
// exactly 104 blocks will be streamed, and they form what would go into
// the (uint16_t* eeData) if `MLX90640_DumpEE` was called normally
void MLX90640_dumpEeprom(block_stream dump) {
  uint8_t ee_line_buffer[BLOCK_SIZE];
  uint16_t addr = 0x2400;
  while (addr != 0x2740) {
    MLX90640_I2CRead(addr, ee_line_buffer, BLOCK_SIZE);
    // divide by two needed because the ATMega does addresses
    // in bytes but the MLX does addresses in 16-bit words
    addr += BLOCK_SIZE / 2;
    dump(ee_line_buffer);
  }
}

uint16_t last_halfFrame_mask = 0x00ff;
// the block_stream supplied here must be able to handle 8-byte blocks
// exactly 209 blocks will be streamed, and they form what would go into
// the (uint16_t* frameData) if `MLX90640_GetFrameData` was called normally
// (with four garbage bytes of padding to fill up the last block)
int MLX90640_dumpFrameData(block_stream dump) {
  uint8_t frame_buffer[BLOCK_SIZE];
  uint16_t statusRegister = 0;
  uint16_t addr;
  uint16_t reg;

  // wait for data to be ready
  MLX90640_I2CRead(0x8000, (uint8_t*) &reg, 2);
  if (!(reg & 0x0800)) return -1; // indicator that no data is ready
  if (!((reg & 0x0100) ^ last_halfFrame_mask))
    return -2; // indicator that the current data is the wrong half-frame
  last_halfFrame_mask = reg & 0x0100;

  // stream the frame data out from sensor RAM
  addr = 0x0400;
  while (addr != 0x0740) {
    MLX90640_I2CRead(addr, frame_buffer, BLOCK_SIZE); 
    // divide by two needed because the ATMega does addresses
    // in bytes but the MLX does addresses in 16-bit words
    addr += BLOCK_SIZE / 2;
    dump(frame_buffer);
  }
  MLX90640_I2CRead(0x800D, frame_buffer, 2);
  MLX90640_I2CRead(0x8000, frame_buffer + 2, 2);

  // reset data-ready bit
  reg = reg & 0xf7ff;
  __builtin_bswap16(reg);
  MLX90640_I2CWrite(0x8000, (uint8_t*) &reg, 2);
}
