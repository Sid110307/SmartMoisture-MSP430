#pragma once

#include "./config.h"

void maxInit(void);
float maxReadRtdTemp(void);
uint8_t maxReadReg(uint8_t regAddr);
void maxWriteReg(uint8_t regAddr, uint8_t value);
