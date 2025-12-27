#include "./include/max.h"

static void spiInit(void)
{
	P2SEL0 |= BIT4 | BIT5 | BIT6;
	P2SEL1 &= ~(BIT4 | BIT5 | BIT6);

	MAX_CS_DIR |= MAX_CS_PIN;
	MAX_CS_PORT |= MAX_CS_PIN;

	UCA1CTLW0 = UCSWRST;
	UCA1CTLW0 |= UCSYNC | UCMST | UCMSB | UCSSEL__SMCLK;
	UCA1BRW = 4;
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

	const uint8_t r = UCA1RXBUF;
	while (UCA1STATW & UCBUSY);

	return r;
}

static void maxReadMulti(const uint8_t start, uint8_t* buf, const uint8_t len)
{
	csLow();
	spiTransfer(start & 0x7F);
	for (uint8_t i = 0; i < len; ++i) buf[i] = spiTransfer(0x00);

	while (UCA1STATW & UCBUSY);
	csHigh();
}

static void maxClearFault(void)
{
	uint8_t t = maxReadReg(MAX_REG_CONF);

	t &= ~0x2C;
	t |= MAX_CFG_FAULT;
	maxWriteReg(MAX_REG_CONF, t);
}

static float maxTempFromResistance(const float Rt)
{
	const float R0 = 100.0f;

	const float T = (Rt / R0 - 1.0f) / CVD_A;
	const float f = R0 * (1.0f + CVD_A * T + CVD_B * T * T) - Rt;
	const float fp = R0 * (CVD_A + 2.0f * CVD_B * T);

	return T - f / fp;
}

void maxInit(void)
{
	spiInit();
	delayCyclesUl(BLE_COMMAND_DELAY);

	uint8_t cfg = 0;
	cfg |= MAX_CFG_3WIRE;
	cfg |= MAX_CFG_50HZ;
	maxWriteReg(MAX_REG_CONF, cfg);

	maxClearFault();
	(void)maxReadReg(MAX_REG_FAULT);
}

float maxReadRtdTemp(void)
{
	maxClearFault();

	uint8_t t = maxReadReg(MAX_REG_CONF);
	t |= MAX_CFG_BIAS;
	t &= ~MAX_CFG_AUTO;
	maxWriteReg(MAX_REG_CONF, t);

	delayCyclesUl(2500UL);
	t |= MAX_CFG_1SHOT;
	maxWriteReg(MAX_REG_CONF, t);
	delayCyclesUl(20000UL);

	uint8_t buf[2];
	maxReadMulti(MAX_REG_RTD_MSB, buf, 2);

	t = maxReadReg(MAX_REG_CONF);
	t &= ~MAX_CFG_BIAS;
	maxWriteReg(MAX_REG_CONF, t);

	uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
	raw >>= 1;
	const float Rt = (float)raw * 430.0f / 32768.0f;

	return maxTempFromResistance(Rt);
}

uint8_t maxReadReg(const uint8_t regAddr)
{
	csLow();
	spiTransfer(regAddr & 0x7F);
	const uint8_t value = spiTransfer(0x00);

	while (UCA1STATW & UCBUSY);
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
