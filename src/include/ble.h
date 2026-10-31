#pragma once

#include "./config.h"

typedef struct
{
	int tempX100;
	uint16_t adcRaw;
	uint8_t maxFault;
	uint8_t tempState; /* TEMP_OK, TEMP_INVALID, TEMP_RANGE or TEMP_FAULT */
} SensorSnapshot;

enum { TEMP_OK = 0, TEMP_INVALID = 1, TEMP_RANGE = 2, TEMP_FAULT = 3 };

void bleInit(uint16_t tickHz);
void bleGpioInit(void);

void bleSetResetCause(uint16_t cause);
void bleProcessRx(const SensorSnapshot* s);
void bleOnTick(const SensorSnapshot* s);

uint8_t bleIsConnected(void);
