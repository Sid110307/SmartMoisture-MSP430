#include <stdio.h>
#include <stdint.h>
#include <msp430.h>

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

#define ADC_INPUT_CTL ADCPCTL6
#define ADC_INPUT_CHANNEL ADCINCH_6
#define ADC_REF_SELECT ADCSREF_0
#define ADC_SHT_SETTING ADCSHT_2
#define ADC_RES_SETTING ADCRES_2

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

#define BLE_WAKE_PORT P2OUT
#define BLE_WAKE_DIR P2DIR
#define BLE_WAKE_PIN BIT2

#define BLE_RESET_PORT P3OUT
#define BLE_RESET_DIR P3DIR
#define BLE_RESET_PIN BIT0

#define BLE_IRQ_PORT P3IN
#define BLE_IRQ_DIR P3DIR
#define BLE_IRQ_REN P3REN
#define BLE_IRQ_OUT P3OUT
#define BLE_IRQ_PIN BIT1

#define BLE_PWR_PORT P2OUT
#define BLE_PWR_DIR P2DIR
#define BLE_PWR_PIN BIT7

#define BLE_TXSEL_BIT BIT5
#define BLE_RXSEL_BIT BIT4
#define BLE_RX_BUFFER_SIZE 64

static volatile char bleRxBuffer[BLE_RX_BUFFER_SIZE];
static volatile uint8_t bleRxHead = 0;
static uint8_t bleIrqPrevLevel = 0, bleConnected = 0;

static const uint8_t font5x7[][5] = {
	{0x00, 0x00, 0x00, 0x00, 0x00}, // 0x20 ' '
	{0x00, 0x00, 0x5F, 0x00, 0x00}, // 0x21 '!'
	{0x00, 0x07, 0x00, 0x07, 0x00}, // 0x22 '"'
	{0x14, 0x7F, 0x14, 0x7F, 0x14}, // 0x23 '#'
	{0x24, 0x2A, 0x7F, 0x2A, 0x12}, // 0x24 '$'
	{0x23, 0x13, 0x08, 0x64, 0x62}, // 0x25 '%'
	{0x36, 0x49, 0x55, 0x22, 0x50}, // 0x26 '&'
	{0x00, 0x05, 0x03, 0x00, 0x00}, // 0x27 '''
	{0x00, 0x1C, 0x22, 0x41, 0x00}, // 0x28 '('
	{0x00, 0x41, 0x22, 0x1C, 0x00}, // 0x29 ')'
	{0x14, 0x08, 0x3E, 0x08, 0x14}, // 0x2A '*'
	{0x08, 0x08, 0x3E, 0x08, 0x08}, // 0x2B '+'
	{0x00, 0x50, 0x30, 0x00, 0x00}, // 0x2C ','
	{0x08, 0x08, 0x08, 0x08, 0x08}, // 0x2D '-'
	{0x00, 0x60, 0x60, 0x00, 0x00}, // 0x2E '.'
	{0x20, 0x10, 0x08, 0x04, 0x02}, // 0x2F '/'

	{0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0x30 '0'
	{0x00, 0x42, 0x7F, 0x40, 0x00}, // 0x31 '1'
	{0x42, 0x61, 0x51, 0x49, 0x46}, // 0x32 '2'
	{0x21, 0x41, 0x45, 0x4B, 0x31}, // 0x33 '3'
	{0x18, 0x14, 0x12, 0x7F, 0x10}, // 0x34 '4'
	{0x27, 0x45, 0x45, 0x45, 0x39}, // 0x35 '5'
	{0x3C, 0x4A, 0x49, 0x49, 0x30}, // 0x36 '6'
	{0x01, 0x71, 0x09, 0x05, 0x03}, // 0x37 '7'
	{0x36, 0x49, 0x49, 0x49, 0x36}, // 0x38 '8'
	{0x06, 0x49, 0x49, 0x29, 0x1E}, // 0x39 '9'

	{0x00, 0x36, 0x36, 0x00, 0x00}, // 0x3A ':'
	{0x00, 0x56, 0x36, 0x00, 0x00}, // 0x3B ';'
	{0x08, 0x14, 0x22, 0x41, 0x00}, // 0x3C '<'
	{0x14, 0x14, 0x14, 0x14, 0x14}, // 0x3D '='
	{0x00, 0x41, 0x22, 0x14, 0x08}, // 0x3E '>'
	{0x02, 0x01, 0x51, 0x09, 0x06}, // 0x3F '?'
	{0x32, 0x49, 0x79, 0x41, 0x3E}, // 0x40 '@'

	{0x7E, 0x11, 0x11, 0x11, 0x7E}, // 0x41 'A'
	{0x7F, 0x49, 0x49, 0x49, 0x36}, // 0x42 'B'
	{0x3E, 0x41, 0x41, 0x41, 0x22}, // 0x43 'C'
	{0x7F, 0x41, 0x41, 0x22, 0x1C}, // 0x44 'D'
	{0x7F, 0x49, 0x49, 0x49, 0x41}, // 0x45 'E'
	{0x7F, 0x09, 0x09, 0x09, 0x01}, // 0x46 'F'
	{0x3E, 0x41, 0x49, 0x49, 0x7A}, // 0x47 'G'
	{0x7F, 0x08, 0x08, 0x08, 0x7F}, // 0x48 'H'
	{0x00, 0x41, 0x7F, 0x41, 0x00}, // 0x49 'I'
	{0x20, 0x40, 0x41, 0x3F, 0x01}, // 0x4A 'J'
	{0x7F, 0x08, 0x14, 0x22, 0x41}, // 0x4B 'K'
	{0x7F, 0x40, 0x40, 0x40, 0x40}, // 0x4C 'L'
	{0x7F, 0x02, 0x0C, 0x02, 0x7F}, // 0x4D 'M'
	{0x7F, 0x04, 0x08, 0x10, 0x7F}, // 0x4E 'N'
	{0x3E, 0x41, 0x41, 0x41, 0x3E}, // 0x4F 'O'
	{0x7F, 0x09, 0x09, 0x09, 0x06}, // 0x50 'P'
	{0x3E, 0x41, 0x51, 0x21, 0x5E}, // 0x51 'Q'
	{0x7F, 0x09, 0x19, 0x29, 0x46}, // 0x52 'R'
	{0x46, 0x49, 0x49, 0x49, 0x31}, // 0x53 'S'
	{0x01, 0x01, 0x7F, 0x01, 0x01}, // 0x54 'T'
	{0x3F, 0x40, 0x40, 0x40, 0x3F}, // 0x55 'U'
	{0x1F, 0x20, 0x40, 0x20, 0x1F}, // 0x56 'V'
	{0x7F, 0x20, 0x18, 0x20, 0x7F}, // 0x57 'W'
	{0x63, 0x14, 0x08, 0x14, 0x63}, // 0x58 'X'
	{0x07, 0x08, 0x70, 0x08, 0x07}, // 0x59 'Y'
	{0x61, 0x51, 0x49, 0x45, 0x43}, // 0x5A 'Z'

	{0x00, 0x7F, 0x41, 0x41, 0x00}, // 0x5B '['
	{0x02, 0x04, 0x08, 0x10, 0x20}, // 0x5C '\'
	{0x00, 0x41, 0x41, 0x7F, 0x00}, // 0x5D ']'
	{0x04, 0x02, 0x01, 0x02, 0x04}, // 0x5E '^'
	{0x40, 0x40, 0x40, 0x40, 0x40}, // 0x5F '_'
	{0x00, 0x01, 0x02, 0x04, 0x00}, // 0x60 '`'

	{0x20, 0x54, 0x54, 0x54, 0x78}, // 0x61 'a'
	{0x7F, 0x48, 0x44, 0x44, 0x38}, // 0x62 'b'
	{0x38, 0x44, 0x44, 0x44, 0x20}, // 0x63 'c'
	{0x38, 0x44, 0x44, 0x48, 0x7F}, // 0x64 'd'
	{0x38, 0x54, 0x54, 0x54, 0x18}, // 0x65 'e'
	{0x08, 0x7E, 0x09, 0x01, 0x02}, // 0x66 'f'
	{0x0C, 0x52, 0x52, 0x52, 0x3E}, // 0x67 'g'
	{0x7F, 0x08, 0x04, 0x04, 0x78}, // 0x68 'h'
	{0x00, 0x44, 0x7D, 0x40, 0x00}, // 0x69 'i'
	{0x20, 0x40, 0x44, 0x3D, 0x00}, // 0x6A 'j'
	{0x7F, 0x10, 0x28, 0x44, 0x00}, // 0x6B 'k'
	{0x00, 0x41, 0x7F, 0x40, 0x00}, // 0x6C 'l'
	{0x7C, 0x04, 0x18, 0x04, 0x7C}, // 0x6D 'm'
	{0x7C, 0x08, 0x04, 0x04, 0x78}, // 0x6E 'n'
	{0x38, 0x44, 0x44, 0x44, 0x38}, // 0x6F 'o'
	{0x7C, 0x14, 0x14, 0x14, 0x08}, // 0x70 'p'
	{0x08, 0x14, 0x14, 0x14, 0x7C}, // 0x71 'q'
	{0x7C, 0x08, 0x04, 0x04, 0x08}, // 0x72 'r'
	{0x48, 0x54, 0x54, 0x54, 0x20}, // 0x73 's'
	{0x04, 0x3F, 0x44, 0x40, 0x20}, // 0x74 't'
	{0x3C, 0x40, 0x40, 0x20, 0x7C}, // 0x75 'u'
	{0x1C, 0x20, 0x40, 0x20, 0x1C}, // 0x76 'v'
	{0x3C, 0x40, 0x30, 0x40, 0x3C}, // 0x77 'w'
	{0x44, 0x28, 0x10, 0x28, 0x44}, // 0x78 'x'
	{0x0C, 0x50, 0x50, 0x50, 0x3C}, // 0x79 'y'
	{0x44, 0x64, 0x54, 0x4C, 0x44}, // 0x7A 'z'

	{0x00, 0x08, 0x36, 0x41, 0x00}, // 0x7B '{'
	{0x00, 0x00, 0x7F, 0x00, 0x00}, // 0x7C '|'
	{0x00, 0x41, 0x36, 0x08, 0x00}, // 0x7D '}'
	{0x08, 0x04, 0x08, 0x10, 0x08}, // 0x7E '~'
};

static uint8_t fontIndexForChar(const char c)
{
	if (c < 0x20 || c > 0x7E) return (0x7E - 0x20 + 1);
	return (uint8_t)(c - 0x20);
}

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

static void oledSetCursor(const uint8_t col, const uint8_t page)
{
	oledSendCommand(0x21);
	oledSendCommand(col);
	oledSendCommand(0x7F);
	oledSendCommand(0x22);
	oledSendCommand(page);
	oledSendCommand(0x07);
}

static void oledClear(void)
{
	oledSetCursor(0, 0);
	for (uint16_t i = 0; i < 1024; ++i) oledSendData(0x00);
}

static void oledDrawChar(const uint8_t col, const uint8_t page, const char c)
{
	const uint8_t idx = fontIndexForChar(c);
	const uint8_t* glyph = font5x7[idx];

	oledSetCursor(col, page);
	for (uint8_t i = 0; i < 5; ++i) oledSendData(glyph[i]);
	oledSendData(0x00);
}

static void oledDrawString(const uint8_t col, const uint8_t page, const char* s)
{
	uint8_t x = col;

	while (*s && x < 128)
	{
		oledDrawChar(x, page, *s++);
		x += 6;
	}
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
	unsigned long count = 4000000UL;

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

static float maxReadRtdTemp(void)
{
	uint8_t buf[2];
	if (!maxWaitDrdy())
		while (1)
		{
			char line1[16];
			snprintf(line1, sizeof(line1), "MAX: No DRDY");
			oledDrawString(0, 2, line1);

			LED_PORT ^= LED_PIN;
			delayCyclesUl(200000);
		}

	maxReadMulti(MAX_REG_RTD_MSB, buf, 2);
	uint16_t raw = (uint16_t)buf[0] << 8 | buf[1];
	raw >>= 1;

	return (float)raw / 32.0f - 256.0f;;
}

static void adcInit(void)
{
	SYSCFG2 |= ADC_INPUT_CTL;
	ADCCTL0 = ADC_SHT_SETTING | ADCON;
	ADCCTL1 = ADCSHP;
	ADCCTL2 = ADC_RES_SETTING;
	ADCMCTL0 = ADC_INPUT_CHANNEL | ADC_REF_SELECT;
}

static uint16_t adcReadRaw(void)
{
	ADCCTL0 &= ~ADCENC;
	ADCMCTL0 = ADC_INPUT_CHANNEL | ADC_REF_SELECT;
	ADCCTL0 |= ADCENC | ADCSC;

	while (ADCCTL1 & ADCBUSY);
	return ADCMEM0;
}

static void gpioInit(void)
{
	LED_DIR |= LED_PIN;
	LED_PORT &= ~LED_PIN;

	P1SEL0 &= ~(OLED_SCL_PIN | OLED_SDA_PIN | LED_PIN);
	P1SEL1 &= ~(OLED_SCL_PIN | OLED_SDA_PIN | LED_PIN);

	BLE_PWR_DIR |= BLE_PWR_PIN;
	BLE_PWR_PORT |= BLE_PWR_PIN;
	BLE_WAKE_DIR |= BLE_WAKE_PIN;
	BLE_WAKE_PORT &= ~BLE_WAKE_PIN;
	BLE_RESET_DIR |= BLE_RESET_PIN;
	BLE_RESET_PORT |= BLE_RESET_PIN;
	BLE_IRQ_DIR &= ~BLE_IRQ_PIN;
	BLE_IRQ_REN |= BLE_IRQ_PIN;
	BLE_IRQ_OUT &= ~BLE_IRQ_PIN;

	bleIrqPrevLevel = (BLE_IRQ_PORT & BLE_IRQ_PIN) ? 1u : 0u;
	bleConnected = bleIrqPrevLevel;
}

static void bleUartInit(void)
{
	P1SEL0 |= (BLE_RXSEL_BIT | BLE_TXSEL_BIT);
	P1SEL1 &= ~(BLE_RXSEL_BIT | BLE_TXSEL_BIT);

	UCA0CTLW0 = UCSWRST;
	UCA0CTLW0 |= UCSSEL__SMCLK;

	UCA0BRW = 6;
	UCA0MCTLW = UCOS16 | UCBRF_8 | 0x20;
	UCA0CTLW0 &= ~UCSWRST;
}

static void blePrintChar(const char c)
{
	while (!(UCA0IFG & UCTXIFG));
	UCA0TXBUF = (uint8_t)c;
}

static void blePrintString(const char* str) { while (*str) blePrintChar(*str++); }

static void bleSendMeasurement(const float tempC, const float moisturePercent)
{
	char buf[32];
	const int n = snprintf(buf, sizeof(buf), "Temp:%.2f;Moisture:%.2f\r\n", tempC, moisturePercent);
	if (n <= 0) return;

	blePrintString(buf);
}

static uint8_t bleRxContains(const char* pattern)
{
	const uint8_t len = bleRxHead;
	uint8_t patLen = 0;

	while (pattern[patLen] != '\0') patLen++;
	if (len < patLen) return 0;

	for (uint8_t i = 0; i + patLen <= len; ++i)
	{
		uint8_t j = 0;

		while (j < patLen && bleRxBuffer[i + j] == pattern[j]) j++;
		if (j == patLen) return 1;
	}
	return 0;
}

static uint8_t bleSendCommand(const char* cmd, const char* expect, const unsigned long timeoutCycles)
{
	bleRxHead = 0;
	blePrintString(cmd);

	unsigned long count = timeoutCycles;
	while (count--) if (bleRxContains(expect)) return 1;

	return 0;
}

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;
	PM5CTL0 &= ~LOCKLPM5;

	gpioInit();
	maxInit();
	adcInit();
	bleUartInit();
	oledInit();
	oledClear();

	UCA0IE |= UCRXIE;
	__enable_interrupt();

	BLE_WAKE_PORT |= BLE_WAKE_PIN;
	if (!bleSendCommand("AT\r\n", "OK", 200000UL))
		while (1)
		{
			char line1[16];
			snprintf(line1, sizeof(line1), "BLE: No OK");
			oledDrawString(0, 0, line1);

			LED_PORT ^= LED_PIN;
			delayCyclesUl(50000);
		}

	bleSendCommand("AT+NAME=SmartMoisture\r\n", "OK", 200000UL);
	bleSendCommand("AT+BAUD\r\n", "OK", 200000UL);

	while (1)
	{
		const uint8_t irqLevel = (BLE_IRQ_PORT & BLE_IRQ_PIN) ? 1u : 0u;

		if (irqLevel != bleIrqPrevLevel)
		{
			bleIrqPrevLevel = irqLevel;
			bleConnected = irqLevel;

			char line1[16], line2[16];
			if (bleConnected)
			{
				snprintf(line1, sizeof(line1), "BLE: Connected");
				oledDrawString(0, 0, line1);
			}
			else
			{
				snprintf(line1, sizeof(line1), "BLE: Disconnected");
				oledDrawString(0, 0, line1);
			}
			oledDrawString(0, 1, "                ");

			LED_PORT ^= LED_PIN;
			delayCyclesUl(50000);
		}

		const uint8_t fault = maxReadReg(MAX_REG_FAULT);
		if (fault != 0)
		{
			LED_PORT ^= LED_PIN;
			delayCyclesUl(50000);

			maxWriteReg(MAX_REG_CONF, 0xD1 | (1u << 1));
			maxWriteReg(MAX_REG_CONF, 0xD1);

			continue;
		}

		const uint16_t adcRaw = adcReadRaw();
		const float tDegC = maxReadRtdTemp();
		const float moisturePercent = (float)adcRaw * 100.0f / 1023.0f;

		char line1[16], line2[16];
		bleSendMeasurement(tDegC, moisturePercent);

		snprintf(line1, sizeof(line1), "Temp: %.2f C", tDegC);
		snprintf(line2, sizeof(line2), "Moist: %.2f %%", moisturePercent);
		oledDrawString(0, 2, line1);
		oledDrawString(0, 3, line2);

		delayCyclesUl(200000);
	}
}

#pragma vector = USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
	if (UCA0IFG & UCRXIFG)
	{
		const char c = (char)UCA0RXBUF;
		const uint8_t next = (uint8_t)((bleRxHead + 1u) % BLE_RX_BUFFER_SIZE);

		if (next != 0)
		{
			bleRxBuffer[bleRxHead] = c;
			bleRxHead = next;
		}
	}
}
