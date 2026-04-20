#pragma once

#include <msp430.h>
#include "./types.h"

// #define ENABLE_OLED // Comment to disable I2C OLED display
#define ENABLE_BLE // Comment to disable BLE device
#define ENABLE_ADC // Comment to disable ADC moisture reading
#define ENABLE_MAX // Comment to disable MAX31865 temperature reading

#define MAX_CS_PORT P2OUT
#define MAX_CS_DIR P2DIR
#define MAX_CS_PIN BIT3

#define MAX_REG_CONF 0x00
#define MAX_REG_RTD_MSB 0x01
#define MAX_REG_RTD_LSB 0x02
#define MAX_REG_FAULT 0x07

#define MAX_CFG_BIAS 0x80
#define MAX_CFG_AUTO 0x40
#define MAX_CFG_1SHOT 0x20
#define MAX_CFG_3WIRE 0x10
#define MAX_CFG_FAULT 0x02
#define MAX_CFG_50HZ 0x01

#define OLED_ADDR 0x3C
#define OLED_WAIT 20000

#define LED_PORT P1OUT
#define LED_DIR P1DIR
#define LED_PIN BIT7

#define BLE_WAKE_PORT P2OUT
#define BLE_WAKE_DIR P2DIR
#define BLE_WAKE_PIN BIT2

#define BLE_RESET_PORT P3OUT
#define BLE_RESET_DIR P3DIR
#define BLE_RESET_PIN BIT0

#define BLE_PWR_PORT P2OUT
#define BLE_PWR_DIR P2DIR
#define BLE_PWR_PIN BIT7

#define BLE_TXSEL_BIT BIT4
#define BLE_RXSEL_BIT BIT5

#define COMMAND_DELAY 500
#define FAULT_BLINK_DELAY 250

static void delayA0(const uint16_t ticks)
{
	const uint16_t start = TA0R;
	while (TA0R - start < ticks);
}

static void delayMs(const uint16_t ms)
{
	uint32_t ticks = ((uint32_t)ms * 32768UL + 500UL) / 1000UL;
	while (ticks)
	{
		const uint16_t chunk = ticks > 60000UL ? 60000U : (uint16_t)ticks;

		delayA0(chunk);
		ticks -= chunk;
	}
}
