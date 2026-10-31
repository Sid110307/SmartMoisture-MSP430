#include "./include/max.h"

#define PT100_POINTS 21
#define PT100_FIRST_C (-50)
static const uint16_t PT100_TABLE[PT100_POINTS] = {
	8031, 8427, 8822, 9216, 9609, 10000, 10390, 10779, 11167, 11554, 11940,
	12324, 12708, 13090, 13471, 13851, 14229, 14607, 14983, 15358, 15733
};

static void spiInit(void)
{
	P2SEL0 |= BIT4 | BIT5 | BIT6;
	P2SEL1 &= ~(BIT4 | BIT5 | BIT6);

	MAX_CS_DIR |= MAX_CS_PIN;
	MAX_CS_PORT |= MAX_CS_PIN;

	UCA1CTLW0 = UCSWRST;
	UCA1CTLW0 |= UCSYNC | UCMST | UCMSB | UCCKPH | UCSSEL__SMCLK;
	UCA1CTLW0 &= ~UCCKPL;
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

#define SPI_POLL_TIMEOUT 2000U

static uint8_t spiTransfer(const uint8_t data)
{
	uint16_t t = SPI_POLL_TIMEOUT;
	while (!(UCA1IFG & UCTXIFG)) if (--t == 0) return 0xFF;
	UCA1TXBUF = data;

	t = SPI_POLL_TIMEOUT;
	while (!(UCA1IFG & UCRXIFG)) if (--t == 0) return 0xFF;

	return UCA1RXBUF;
}

static void spiWaitIdle(void)
{
	uint16_t t = SPI_POLL_TIMEOUT;
	while ((UCA1STATW & UCBUSY) && --t);
}

static void maxReadMulti(const uint8_t start, uint8_t* buf, const uint8_t len)
{
	csLow();
	spiTransfer(start & 0x7F);
	for (uint8_t i = 0; i < len; ++i) buf[i] = spiTransfer(0x00);

	spiWaitIdle();
	csHigh();
}

static void maxClearFault(void)
{
	uint8_t t = maxReadReg(MAX_REG_CONF);

	t &= ~(MAX_CFG_1SHOT | 0x0C);
	t |= MAX_CFG_FAULT;
	maxWriteReg(MAX_REG_CONF, t);
}

void maxInit(void)
{
	spiInit();
	delayMs(COMMAND_DELAY);

	uint8_t cfg = 0;
	cfg |= MAX_CFG_3WIRE;
	cfg |= MAX_CFG_50HZ;
	maxWriteReg(MAX_REG_CONF, cfg);

	maxClearFault();
	(void)maxReadReg(MAX_REG_FAULT);
}

int maxReadRtdTemp(void)
{
	maxClearFault();

	uint8_t t = maxReadReg(MAX_REG_CONF);
	t |= MAX_CFG_BIAS;
	t &= ~MAX_CFG_AUTO;
	maxWriteReg(MAX_REG_CONF, t);

	delayMs(10);
	t |= MAX_CFG_1SHOT;
	maxWriteReg(MAX_REG_CONF, t);
	delayMs(65);

	uint8_t buf[2];
	maxReadMulti(MAX_REG_RTD_MSB, buf, 2);

	t = maxReadReg(MAX_REG_CONF);
	t &= ~MAX_CFG_BIAS;
	maxWriteReg(MAX_REG_CONF, t);

	if (buf[1] & 0x01) return MAX_TEMP_INVALID;
	uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
	raw >>= 1;

	/* R in 0.01 ohm for a 430 ohm reference: raw * 43000 / 32768. */
	const uint32_t r = ((uint32_t)raw * 43000UL) >> 15;
	if (r < PT100_TABLE[0] || r > PT100_TABLE[PT100_POINTS - 1]) return MAX_TEMP_RANGE;

	/* IEC 60751 table, linear between 10 C points (error below 0.01 C). */
	uint8_t i = 0;
	while (i < PT100_POINTS - 2 && r >= PT100_TABLE[i + 1]) ++i;
	const uint32_t span = PT100_TABLE[i + 1] - PT100_TABLE[i];
	const int base = (int)(PT100_FIRST_C + 10 * (int)i) * 100;
	return base + (int)(((r - PT100_TABLE[i]) * 1000UL) / span);
}

uint8_t maxReadReg(const uint8_t regAddr)
{
	csLow();
	spiTransfer(regAddr & 0x7F);
	const uint8_t value = spiTransfer(0x00);

	spiWaitIdle();
	csHigh();

	return value;
}

void maxWriteReg(const uint8_t regAddr, const uint8_t value)
{
	csLow();
	spiTransfer(0x80 | (regAddr & 0x7F));
	spiTransfer(value);

	spiWaitIdle();
	csHigh();
}
