#pragma once
#include <stdint.h>

// look, a function pointer! consumes the blocks of bytes pointed to
typedef void (*block_stream)(uint8_t*);
#define BLOCK_SIZE 0x40

bool MLX90640_init();
void MLX90640_I2CRead(uint16_t addr, uint8_t* data, int size);
void MLX90640_I2CWrite(uint16_t addr, uint8_t* data, int size);
void MLX90640_dumpEeprom(block_stream dump);
int MLX90640_dumpFrameData(block_stream dump);
