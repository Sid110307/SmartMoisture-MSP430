#pragma once

#include "./config.h"

void oledInit(void);
void oledClearLine(uint8_t page);
void oledClear(void);
void oledDrawString(uint8_t col, uint8_t page, const char* s);
