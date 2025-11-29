#include <stdio.h>
#include <msp430.h>

#include "./include/config.h"
#include "./include/max.h"
#include "./include/oled.h"

#define BLE_RX_BUFFER_SIZE 64

static volatile char bleRxBuffer[BLE_RX_BUFFER_SIZE];
static volatile uint8_t bleRxHead = 0, bleRxCount = 0;
static uint8_t bleIrqPrevLevel = 0, bleConnected = 0;

static void adcInit(void)
{
	SYSCFG2 |= ADC_INPUT_CTL;
	ADCCTL0 = ADC_SHT_SETTING;
	ADCCTL1 = ADCSHP;
	ADCCTL2 = ADC_RES_SETTING;
	ADCMCTL0 = ADC_INPUT_CHANNEL | ADC_REF_SELECT;
}

static uint16_t adcReadRaw(void)
{
	ADCCTL0 &= ~ADCENC;
	ADCCTL0 |= ADCON;
	ADCCTL0 |= ADCENC | ADCSC;

	while (ADCCTL1 & ADCBUSY);
	const uint16_t result = ADCMEM0;

	ADCCTL0 &= ~ADCON;
	return result;
}

static void timerInit(void)
{
	TA0CTL = TASSEL__ACLK | MC__UP | TACLR;
	TA0CCR0 = 32768 / 5;
	TA0CCTL0 = CCIE;
}

static void gpioInit(void)
{
	LED_DIR |= LED_PIN;
	LED_PORT &= ~LED_PIN;

	P1SEL0 &= ~LED_PIN;
	P1SEL1 &= ~LED_PIN;

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
	uint8_t patLen = 0, len, head;
	while (pattern[patLen] != '\0') patLen++;

	__disable_interrupt();
	len = bleRxCount;
	head = bleRxHead;
	__enable_interrupt();

	if (len < patLen) return 0;
	const uint8_t start = (uint8_t)((head + BLE_RX_BUFFER_SIZE - len) % BLE_RX_BUFFER_SIZE);

	for (uint8_t i = 0; i <= (uint8_t)(len - patLen); ++i)
	{
		uint8_t j = 0;

		while (j < patLen)
		{
			const uint8_t idx = (uint8_t)((start + i + j) % BLE_RX_BUFFER_SIZE);
			if (bleRxBuffer[idx] != pattern[j]) break;

			j++;
		}
		if (j == patLen) return 1;
	}
	return 0;
}

static uint8_t bleSendCommand(const char* cmd, const char* expect, const unsigned long timeoutCycles)
{
	__disable_interrupt();
	bleRxHead = 0;
	bleRxCount = 0;
	__enable_interrupt();

	blePrintString(cmd);
	unsigned long count = timeoutCycles;
	while (count--) if (bleRxContains(expect)) return 1;

	return 0;
}

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;
	PM5CTL0 &= ~LOCKLPM5;

	timerInit();
	gpioInit();
	maxInit();
	adcInit();
	bleUartInit();
	oledInit();
	oledClear();

	UCA0IE |= UCRXIE;
	__enable_interrupt();
	oledDrawString(0, 0, "Initializing...");

	BLE_WAKE_PORT |= BLE_WAKE_PIN;
	if (!bleSendCommand("AT\r\n", "OK", BLE_AT_TIMEOUT))
		while (1)
		{
			oledDrawString(0, 0, "BLE: No Response");
			LED_PORT ^= LED_PIN;
			delayCyclesUl(BLE_FAULT_BLINK_DELAY);
		}

	bleSendCommand("AT+NAME=SmartMoisture\r\n", "OK", BLE_AT_TIMEOUT);
	bleSendCommand("AT+BAUD=9600\r\n", "OK", BLE_AT_TIMEOUT);

	while (1)
	{
		_bis_SR_register(LPM3_bits | GIE);
		const uint8_t irqLevel = (BLE_IRQ_PORT & BLE_IRQ_PIN) ? 1u : 0u;

		if (irqLevel != bleIrqPrevLevel)
		{
			bleIrqPrevLevel = irqLevel;
			bleConnected = irqLevel;

			if (bleConnected)
			{
				BLE_WAKE_PORT |= BLE_WAKE_PIN;
				oledDrawString(0, 0, "BLE: Connected");
			}
			else
			{
				BLE_WAKE_PORT &= ~BLE_WAKE_PIN;
				oledDrawString(0, 0, "BLE: Disconnected");
			}
			oledDrawString(0, 1, "                ");

			LED_PORT ^= LED_PIN;
			delayCyclesUl(BLE_FAULT_BLINK_DELAY);
		}

		const uint8_t fault = maxReadReg(MAX_REG_FAULT);
		if (fault != 0)
		{
			oledDrawString(0, 2, "MAX: Fault Detected");
			LED_PORT ^= LED_PIN;
			delayCyclesUl(BLE_FAULT_BLINK_DELAY);

			maxWriteReg(MAX_REG_CONF, 0xD1 | (1u << 1));
			maxWriteReg(MAX_REG_CONF, 0xD1);

			continue;
		}

		const uint16_t adcRaw = adcReadRaw();
		const float tDegC = maxReadRtdTemp();
		const float moisturePercent = (float)adcRaw * 100.0f / 1023.0f;

		char line1[16], line2[16];
		bleSendMeasurement(tDegC, moisturePercent);

		snprintf(line1, sizeof(line1), "Temp:  %.2f C", tDegC);
		snprintf(line2, sizeof(line2), "Moist: %.2f %%", moisturePercent);
		oledDrawString(0, 2, line1);
		oledDrawString(0, 3, line2);
	}
}

__attribute__((interrupt(USCI_A0_VECTOR))) void USCI_A0_ISR(void)
{
	if (UCA0IFG & UCRXIFG)
	{
		const char c = (char)UCA0RXBUF;
		bleRxBuffer[bleRxHead] = c;
		bleRxHead = (uint8_t)((bleRxHead + 1u) % BLE_RX_BUFFER_SIZE);

		if (bleRxCount < BLE_RX_BUFFER_SIZE) bleRxCount++;
	}
}

__attribute__((interrupt(TIMER0_A0_VECTOR))) void TIMER0_A0_ISR(void) { _bic_SR_register_on_exit(LPM3_bits); }
