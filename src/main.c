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
	{0x00, 0x00, 0x00, 0x00, 0x00}, // ' '
	{0x3E, 0x51, 0x49, 0x45, 0x3E}, // '0'
	{0x00, 0x42, 0x7F, 0x40, 0x00}, // '1'
	{0x42, 0x61, 0x51, 0x49, 0x46}, // '2'
	{0x21, 0x41, 0x45, 0x4B, 0x31}, // '3'
	{0x18, 0x14, 0x12, 0x7F, 0x10}, // '4'
	{0x27, 0x45, 0x45, 0x45, 0x39}, // '5'
	{0x3C, 0x4A, 0x49, 0x49, 0x30}, // '6'
	{0x01, 0x71, 0x09, 0x05, 0x03}, // '7'
	{0x36, 0x49, 0x49, 0x49, 0x36}, // '8'
	{0x06, 0x49, 0x49, 0x29, 0x1E}, // '9'
	{0x00, 0x60, 0x60, 0x00, 0x00}, // '.'
	{0x00, 0x36, 0x36, 0x00, 0x00}, // ':'
	{0x04, 0x04, 0x7F, 0x04, 0x04}, // 'T'
	{0x7F, 0x02, 0x0C, 0x02, 0x7F}, // 'M'
	{0x3E, 0x41, 0x41, 0x41, 0x22}, // 'C'
	{0x61, 0x13, 0x08, 0x64, 0x13}, // '%'
};

static uint8_t fontIndexForChar(const char c)
{
	if (c == ' ') return 0;
	if (c >= '0' && c <= '9') return (uint8_t)(1 + (c - '0'));
	if (c == '.') return 11;
	if (c == ':') return 12;
	if (c == 'T') return 13;
	if (c == 'M') return 14;
	if (c == 'C') return 15;
	if (c == '%') return 16;
	return 0;
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

		snprintf(line1, sizeof(line1), "T:%.1fC", tDegC);
		snprintf(line2, sizeof(line2), "M:%.0f%%", moisturePercent);
		oledDrawString(0, 0, line1);
		oledDrawString(0, 1, line2);

		LED_PORT ^= LED_PIN;
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
