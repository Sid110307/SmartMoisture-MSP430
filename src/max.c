#include "./include/max.h"

static void spiInit(void)
{
	P2SEL0 |= BIT4 | BIT5 | BIT6;
	P2SEL1 &= ~(BIT4 | BIT5 | BIT6);

	MAX_CS_DIR |= MAX_CS_PIN;
	MAX_CS_PORT |= MAX_CS_PIN;

	UCA1CTLW0 = UCSWRST;
	UCA1CTLW0 |= UCSYNC | UCMST | UCMSB | UCCKPH | UCSSEL__SMCLK;
	UCA1BRW = 2;
	UCA1CTLW0 &= ~UCSWRST;
}

static void csLow(void)
{
	MAX_CS_PORT &= ~MAX_CS_PIN;
}

static void csHigh(void)
{
	MAX_CS_PORT |= MAX_CS_PIN;
}

static uint8_t spiTransfer(const uint8_t data)
{
	while (!(UCA1IFG & UCTXIFG));
	UCA1TXBUF = data;

	while (!(UCA1IFG & UCRXIFG));
	return UCA1RXBUF;
}

static void maxReadMulti(const uint8_t start, uint8_t* buf, const uint8_t len)
{
	csLow();
	spiTransfer(start & 0x7F);
	for (uint8_t i = 0; i < len; ++i) buf[i] = spiTransfer(0x00);
	csHigh();
}

void maxInit(void)
{
	spiInit();

	MAX_DRDY_DIR &= ~MAX_DRDY_PIN;
	MAX_DRDY_REN |= MAX_DRDY_PIN;
	MAX_DRDY_OUT |= MAX_DRDY_PIN;

	delayCyclesUl(100000UL);
	maxWriteReg(MAX_REG_CONF, 0xD1);
	maxWriteReg(MAX_REG_CONF, 0xD3);
	maxWriteReg(MAX_REG_CONF, 0xD1);

	(void)maxReadReg(MAX_REG_FAULT);
}

uint8_t maxIsReady(void) { return (uint8_t)!(MAX_DRDY_PORT & MAX_DRDY_PIN); }

float maxReadRtdTemp(void)
{
	uint8_t buf[2];
	maxReadMulti(MAX_REG_RTD_MSB, buf, 2);

	uint16_t raw = (uint16_t)buf[0] << 8 | buf[1];
	raw >>= 1;

	return (float)raw / 32.0f - 256.0f;
}

uint8_t maxReadReg(const uint8_t regAddr)
{
	csLow();
	spiTransfer(regAddr & 0x7F);
	const uint8_t value = spiTransfer(0x00);
	csHigh();

	return value;
}

void maxWriteReg(const uint8_t regAddr, const uint8_t value)
{
	csLow();
	spiTransfer(0x80 | (regAddr & 0x7F));
	spiTransfer(value);
	csHigh();
}
