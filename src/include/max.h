#pragma once

#include "./config.h"

#define MAX_TEMP_INVALID (-32768) /* converter flagged the reading */
#define MAX_TEMP_RANGE (-32767)   /* outside -50..150 C: probe open or shorted */

void maxInit(void);
int maxReadRtdTemp(void); /* 0.01 C, or MAX_TEMP_INVALID / MAX_TEMP_RANGE */
uint8_t maxReadReg(uint8_t regAddr);
void maxWriteReg(uint8_t regAddr, uint8_t value);
