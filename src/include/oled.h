#pragma once

#include "./config.h"

int oledInit(void);
void oledRetry(void);
void oledClear(void);
void oledDrawChar(uint8_t col, uint8_t page, char c);
void oledDrawString(uint8_t col, uint8_t page, const char* s);
