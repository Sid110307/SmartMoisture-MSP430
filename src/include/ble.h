#pragma once

#include "./config.h"

typedef struct
{
	int tempX100;
	uint16_t adcRaw;
	uint8_t maxFault;
} SensorSnapshot;

void bleInit(uint16_t tickHz);
void bleGpioInit(void);

void bleHardwareReset(void);
void bleProcessRx(const SensorSnapshot* s);
void bleOnTick(const SensorSnapshot* s);

uint8_t bleIsConnected(void);
