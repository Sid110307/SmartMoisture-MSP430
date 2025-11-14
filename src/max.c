#include <stdio.h>

#include "./include/max.h"
#include "./include/oled.h"

static void spiDelay(void) { delayCyclesUl(10); }

static void maxSpiInitPins(void)
{
	MAX_CS_DIR |= MAX_CS_PIN;
	MAX_SCLK_DIR |= MAX_SCLK_PIN;
	MAX_MOSI_DIR |= MAX_MOSI_PIN;

	MAX_MISO_DIR &= ~MAX_MISO_PIN;

	MAX_DRDY_DIR &= ~MAX_DRDY_PIN;
	MAX_DRDY_REN |= MAX_DRDY_PIN;
	MAX_DRDY_OUT |= MAX_DRDY_PIN;

	MAX_CS_PORT |= MAX_CS_PIN;
	MAX_SCLK_PORT &= ~MAX_SCLK_PIN;
	MAX_MOSI_PORT &= ~MAX_MOSI_PIN;
}

static uint8_t maxSpiTransfer(const uint8_t outByte)
{
	uint8_t inByte = 0;

	for (int i = 7; i >= 0; i--)
	{
		const uint8_t outBit = outByte >> i & 0x01;

		if (outBit)
			MAX_MOSI_PORT |= MAX_MOSI_PIN;
		else
			MAX_MOSI_PORT &= ~MAX_MOSI_PIN;

		spiDelay();
		MAX_SCLK_PORT |= MAX_SCLK_PIN;
		spiDelay();
		MAX_SCLK_PORT &= ~MAX_SCLK_PIN;

		inByte <<= 1;
		if (MAX_MISO_PORT & MAX_MISO_PIN) inByte |= 1;

		spiDelay();
	}

	return inByte;
}

static void maxCsLow(void)
{
	MAX_CS_PORT &= ~MAX_CS_PIN;
	spiDelay();
}

static void maxCsHigh(void)
{
	spiDelay();
	MAX_CS_PORT |= MAX_CS_PIN;
	spiDelay();
}

static void maxReadMulti(const uint8_t startAddr, uint8_t* buf, const uint8_t len)
{
	maxCsLow();

	(void)maxSpiTransfer(startAddr & 0x7F);
	for (uint8_t i = 0; i < len; ++i) buf[i] = maxSpiTransfer(0x00);

	maxCsHigh();
}

static uint8_t maxWaitDrdy(void)
{
	unsigned long count = 4000000UL;

	while (MAX_DRDY_PORT & MAX_DRDY_PIN && count--);
	return count != 0;
}

void maxInit(void)
{
	maxSpiInitPins();
	delayCyclesUl(100000UL);

	maxWriteReg(MAX_REG_CONF, 0xD1);
	maxWriteReg(MAX_REG_CONF, 0xD3);
	maxWriteReg(MAX_REG_CONF, 0xD1);

	(void)maxReadReg(MAX_REG_FAULT);
}

float maxReadRtdTemp(void)
{
	uint8_t buf[2];
	if (!maxWaitDrdy())
		while (1)
		{
			char line1[16];
			snprintf(line1, sizeof(line1), "MAX: No DRDY");
			oledDrawString(0, 2, line1);

			LED_PORT ^= LED_PIN;
			delayCyclesUl(BLE_FAULT_BLINK_DELAY);
		}

	maxReadMulti(MAX_REG_RTD_MSB, buf, 2);
	uint16_t raw = (uint16_t)buf[0] << 8 | buf[1];
	raw >>= 1;

	return (float)raw / 32.0f - 256.0f;;
}

uint8_t maxReadReg(const uint8_t regAddr)
{
	maxCsLow();

	(void)maxSpiTransfer(regAddr & 0x7F);
	const uint8_t value = maxSpiTransfer(0x00);
	maxCsHigh();

	return value;
}

void maxWriteReg(const uint8_t regAddr, const uint8_t value)
{
	maxCsLow();

	(void)maxSpiTransfer(0x80 | (regAddr & 0x7F));
	(void)maxSpiTransfer(value);

	maxCsHigh();
}
