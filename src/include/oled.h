#pragma once

#include "./config.h"

void oledInit(void);
void oledClear(void);
void oledDrawString(uint8_t col, uint8_t page, const char* s);
