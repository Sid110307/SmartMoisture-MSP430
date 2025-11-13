#include <msp430.h>
#include <stdint.h>

#define MAX_CS_PORT P2OUT
#define MAX_CS_DIR P2DIR
#define MAX_CS_PIN BIT3

#define MAX_SCLK_PORT P2OUT
#define MAX_SCLK_DIR P2DIR
#define MAX_SCLK_PIN BIT4

#define MAX_MISO_PORT P2IN
#define MAX_MISO_DIR P2DIR
#define MAX_MISO_PIN BIT5

#define MAX_MOSI_PORT P2OUT
#define MAX_MOSI_DIR P2DIR
#define MAX_MOSI_PIN BIT6

#define MAX_DRDY_PORT P3IN
#define MAX_DRDY_DIR P3DIR
#define MAX_DRDY_REN P3REN
#define MAX_DRDY_OUT P3OUT
#define MAX_DRDY_PIN BIT2

#define MAX_REG_CONF 0x00
#define MAX_REG_RTD_MSB 0x01
#define MAX_REG_RTD_LSB 0x02
#define MAX_REG_FAULT 0x07

#define OLED_SCL_PORT P1OUT
#define OLED_SCL_DIR P1DIR
#define OLED_SCL_PIN BIT2

#define OLED_SDA_PORT P1OUT
#define OLED_SDA_DIR P1DIR
#define OLED_SDA_PIN BIT3
#define OLED_SDA_IN P1IN

#define LED_PORT P1OUT
#define LED_DIR P1DIR
#define LED_PIN BIT7

static void delayCyclesUl(unsigned long n)
{
	__asm__ __volatile__ (
		"1: \n"
		" dec %[n] \n"
		" jnz 1b \n"
		: [n] "+r"(n)
	);
}

static void spiDelay(void) { delayCyclesUl(10); }
static void i2cDelay(void) { delayCyclesUl(20); }

static void i2cSclLow(void)
{
	OLED_SCL_DIR |= OLED_SCL_PIN;
	OLED_SCL_PORT &= ~OLED_SCL_PIN;
}

static void i2cSclHigh(void)
{
	OLED_SCL_DIR &= ~OLED_SCL_PIN;
}

static void i2cSdaLow(void)
{
	OLED_SDA_DIR |= OLED_SDA_PIN;
	OLED_SDA_PORT &= ~OLED_SDA_PIN;
}

static void i2cSdaHigh(void)
{
	OLED_SDA_DIR &= ~OLED_SDA_PIN;
}

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

static uint8_t maxReadReg(const uint8_t regAddr)
{
	maxCsLow();

	(void)maxSpiTransfer(regAddr & 0x7F);
	const uint8_t value = maxSpiTransfer(0x00);
	maxCsHigh();

	return value;
}

static void maxWriteReg(const uint8_t regAddr, const uint8_t value)
{
	maxCsLow();

	(void)maxSpiTransfer(0x80 | (regAddr & 0x7F));
	(void)maxSpiTransfer(value);

	maxCsHigh();
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
	unsigned long count = 1000000UL;

	while (MAX_DRDY_PORT & MAX_DRDY_PIN && count--);
	return count != 0;
}

static void maxInit(void)
{
	maxSpiInitPins();
	delayCyclesUl(100000);

	maxWriteReg(MAX_REG_CONF, 0xD1);
	maxWriteReg(MAX_REG_CONF, 0xD3);
	maxWriteReg(MAX_REG_CONF, 0xD1);

	(void)maxReadReg(MAX_REG_FAULT);
}

static uint16_t maxReadRtdRaw(void)
{
	uint8_t buf[2];
	if (!maxWaitDrdy())
		while (1)
		{
			LED_PORT ^= LED_PIN;
			delayCyclesUl(200000);
		}

	maxReadMulti(MAX_REG_RTD_MSB, buf, 2);
	uint16_t raw = (uint16_t)buf[0] << 8 | buf[1];
	raw >>= 1;

	return raw;
}

static float maxRtdCodeToResistance(const uint16_t code) { return (float)code * 430.0f / 32768.0f; }
static float maxRtdCodeToTempApprox(const uint16_t code) { return (float)code / 32.0f - 256.0f; }

static void adcInit(void)
{
	SYSCFG2 |= ADCPCTL6;
	ADCCTL0 = ADCSHT_2 | ADCON;
	ADCCTL1 = ADCSHP;
	ADCCTL2 = ADCRES_2;
	ADCMCTL0 = ADCINCH_6 | ADCSREF_0;
}

static uint16_t adcReadRaw(void)
{
	ADCCTL0 &= ~ADCENC;
	ADCMCTL0 = ADCINCH_6 | ADCSREF_0;
	ADCCTL0 |= ADCENC | ADCSC;

	while (ADCCTL1 & ADCBUSY);
	return ADCMEM0;
}

static void i2cInit(void)
{
	i2cSclHigh();
	i2cSdaHigh();
}

static void i2cStart(void)
{
	i2cSdaHigh();
	i2cSclHigh();
	i2cDelay();
	i2cSdaLow();
	i2cDelay();
	i2cSclLow();
	i2cDelay();
}

static void i2cStop(void)
{
	i2cSdaLow();
	i2cDelay();
	i2cSclHigh();
	i2cDelay();
	i2cSdaHigh();
	i2cDelay();
}

static uint8_t i2cWriteByte(const uint8_t byte)
{
	for (int i = 7; i >= 0; i--)
	{
		if (byte & (1u << i)) i2cSdaHigh();
		else i2cSdaLow();

		i2cDelay();
		i2cSclHigh();
		i2cDelay();
		i2cSclLow();
		i2cDelay();
	}

	i2cSdaHigh();
	i2cDelay();
	i2cSclHigh();
	i2cDelay();

	const uint8_t ack = !(OLED_SDA_IN & OLED_SDA_PIN);
	i2cSclLow();
	i2cDelay();

	return ack;
}

static void oledSendCommand(const uint8_t cmd)
{
	i2cStart();
	i2cWriteByte((0x3C << 1) | 0);
	i2cWriteByte(0x00);
	i2cWriteByte(cmd);
	i2cStop();
}

static void oledSendData(const uint8_t data)
{
	i2cStart();
	i2cWriteByte((0x3C << 1) | 0);
	i2cWriteByte(0x40);
	i2cWriteByte(data);
	i2cStop();
}

static void oledInit(void)
{
	i2cInit();
	delayCyclesUl(50000);

	oledSendCommand(0xAE);
	oledSendCommand(0xD5);
	oledSendCommand(0x80);
	oledSendCommand(0xA8);
	oledSendCommand(0x3F);
	oledSendCommand(0xD3);
	oledSendCommand(0x00);
	oledSendCommand(0x40);
	oledSendCommand(0x8D);
	oledSendCommand(0x14);
	oledSendCommand(0x20);
	oledSendCommand(0x00);
	oledSendCommand(0xA1);
	oledSendCommand(0xC8);
	oledSendCommand(0xDA);
	oledSendCommand(0x12);
	oledSendCommand(0x81);
	oledSendCommand(0xCF);
	oledSendCommand(0xD9);
	oledSendCommand(0xF1);
	oledSendCommand(0xDB);
	oledSendCommand(0x40);
	oledSendCommand(0xA4);
	oledSendCommand(0xA6);
	oledSendCommand(0x2E);
	oledSendCommand(0xAF);
}

static void oledClear(void)
{
	oledSendCommand(0x21);
	oledSendCommand(0x00);
	oledSendCommand(0x7F);
	oledSendCommand(0x22);
	oledSendCommand(0x00);
	oledSendCommand(0x07);

	for (uint16_t i = 0; i < 1024; ++i) oledSendData(0x00);
}

static void gpioInit(void)
{
	LED_DIR |= LED_PIN;
	LED_PORT &= ~LED_PIN;

	P1SEL0 &= ~(BIT2 | BIT3 | BIT7);
	P1SEL1 &= ~(BIT2 | BIT3 | BIT7);
}

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;
	PM5CTL0 &= ~LOCKLPM5;

	gpioInit();
	maxInit();
	adcInit();
	oledInit();
	oledClear();

	while (1)
	{
		const uint8_t fault = maxReadReg(MAX_REG_FAULT);
		if (fault != 0)
		{
			LED_PORT ^= LED_PIN;
			delayCyclesUl(50000);

			maxWriteReg(MAX_REG_CONF, 0xD1 | (1u << 1));
			maxWriteReg(MAX_REG_CONF, 0xD1);

			continue;
		}

		const uint16_t code = maxReadRtdRaw();
		const float rOhms = maxRtdCodeToResistance(code);
		const float tDegC = maxRtdCodeToTempApprox(code);
		const uint16_t adcRaw = adcReadRaw();

		(void)rOhms;
		(void)tDegC;
		(void)code;
		(void)adcRaw;

		LED_PORT ^= LED_PIN;
		delayCyclesUl(200000);
	}
}
