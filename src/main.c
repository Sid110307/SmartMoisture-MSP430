#include <stdio.h>
#include <string.h>
#include <msp430.h>

#include "./include/config.h"
#include "./include/max.h"
#include "./include/oled.h"

#define BLE_BUFFER_SIZE 64

static volatile uint8_t bleRxBuffer[BLE_BUFFER_SIZE];
static volatile char bleLine[BLE_BUFFER_SIZE];
static volatile uint8_t bleRxHead = 0, bleRxCount = 0, bleLineLen = 0, bleLineReady = 0;

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
}

static void bleUartInit(void)
{
	P1SEL0 |= BLE_RXSEL_BIT | BLE_TXSEL_BIT;
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

static void bleSendMeasurement(const float tempC, const int adcRaw)
{
	const int tempX100 = (int)(tempC * 100.0f);
	int tempFrac = tempX100 % 100;
	if (tempFrac < 0) tempFrac = -tempFrac;

	char buf[48];
	const int n = snprintf(buf, sizeof(buf), "CMD+DATA=0,Temp:%d.%02d;Moisture:%d\r\n", tempX100 / 100, tempFrac,
	                       adcRaw);
	if (n > 0) blePrintString(buf);
}

static void oledShowLastRx8(void)
{
	uint8_t head, cnt;
	__disable_interrupt();
	head = bleRxHead;
	cnt = bleRxCount;
	__enable_interrupt();

	char line[32];
	uint8_t b[8] = {0};

	uint8_t n = (cnt < 8) ? cnt : 8;
	for (uint8_t k = 0; k < n; k++)
	{
		uint8_t idx = (uint8_t)((head + BLE_BUFFER_SIZE - n + k) % BLE_BUFFER_SIZE);
		b[k] = (uint8_t)bleRxBuffer[idx];
	}

	snprintf(line, sizeof(line), "C%u%02X%02X%02X%02X%02X%02X%02X%02X", cnt, b[0], b[1], b[2], b[3], b[4], b[5], b[6],
	         b[7]);
	oledDrawString(0, 2, line);
}

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;
	PM5CTL0 &= ~LOCKLPM5;

	timerInit();
	gpioInit();

	oledInit();
	oledClear();
	oledDrawString(0, 3, "  SmartMoisture v2.0");
	oledDrawString(0, 4, "  Indriya Sensotech");

	maxInit();
	adcInit();
	bleUartInit();

	UCA0IE |= UCRXIE;
	__enable_interrupt();

	BLE_RESET_PORT &= ~BLE_RESET_PIN;
	delayCyclesUl(BLE_COMMAND_DELAY);
	BLE_RESET_PORT |= BLE_RESET_PIN;
	delayCyclesUl(BLE_COMMAND_DELAY);

	BLE_WAKE_PORT |= BLE_WAKE_PIN;
	blePrintString("CMD+RESET=0\r\n");
	delayCyclesUl(BLE_COMMAND_DELAY);
	blePrintString("CMD+NAME=SmartMoisture\r\n");
	delayCyclesUl(BLE_COMMAND_DELAY);
	blePrintString("CMD+ADV=1\r\n");
	delayCyclesUl(BLE_COMMAND_DELAY);
	blePrintString("CMD+NOTIFY=1\r\n");
	delayCyclesUl(BLE_COMMAND_DELAY);

	oledClear();
	oledDrawString(0, 0, "BLE: Ready");

	while (1)
	{
		_bis_SR_register(LPM0_bits | GIE);
		oledShowLastRx8();

		if (bleLineReady)
		{
			char local[BLE_BUFFER_SIZE];

			__disable_interrupt();
			uint8_t n = bleLineLen;
			for (uint8_t i = 0; i <= n; ++i) local[i] = bleLine[i];
			bleLineLen = 0;
			bleLineReady = 0;
			__enable_interrupt();

			oledDrawString(0, 2, local);
			if (strncmp(local, "EVT+CON", 7) == 0) oledDrawString(0, 0, "BLE: Connected");
			else if (strncmp(local, "EVT+DISCON", 10) == 0) oledDrawString(0, 0, "BLE: Disconnected");
		} // TODO: check why this aint showing

		const uint8_t fault = maxReadReg(MAX_REG_FAULT);
		if (fault != 0)
		{
			char faultMsg[16];
			snprintf(faultMsg, sizeof(faultMsg), "MAX: Error (%02X)", fault);

			oledDrawString(0, 1, faultMsg);
			LED_PORT ^= LED_PIN;
			delayCyclesUl(FAULT_BLINK_DELAY);

			maxWriteReg(MAX_REG_CONF, 0xD3);
			maxWriteReg(MAX_REG_CONF, 0xD1);

			continue;
		}

		const uint16_t adcRaw = adcReadRaw();
		const float tDegC = maxReadRtdTemp();

		int tempX100 = (int)(tDegC * 100.0f);
		int tempFrac = tempX100 % 100;
		if (tempFrac < 0) tempFrac = -tempFrac;

		char line1[20], line2[20];
		bleSendMeasurement(tDegC, adcRaw);

		snprintf(line1, sizeof(line1), "Temp: %d.%02d C", tempX100 / 100, tempFrac);
		snprintf(line2, sizeof(line2), "ADC:  %u", adcRaw);
		oledDrawString(0, 3, line1);
		oledDrawString(0, 4, line2);
	}
}

__attribute__((interrupt(USCI_A0_VECTOR))) void USCI_A0_ISR(void)
{
	if (UCA0IFG & UCRXIFG)
	{
		const uint8_t c = UCA0RXBUF;
		if (!bleLineReady)
		{
			if (c == '\r')
			{
				bleLine[bleLineLen < BLE_BUFFER_SIZE ? bleLineLen : BLE_BUFFER_SIZE - 1] = '\0';
				bleLineReady = 1;
			}
			else if (c != '\n')
			{
				if (bleLineLen < BLE_BUFFER_SIZE - 1) bleLine[bleLineLen++] = (char)c;
				else bleLineLen = 0;
			}
		}

		bleRxBuffer[bleRxHead] = c;
		bleRxHead = (uint8_t)((bleRxHead + 1u) % BLE_BUFFER_SIZE);
		if (bleRxCount < BLE_BUFFER_SIZE) bleRxCount++;
	}
}

__attribute__((interrupt(TIMER0_A0_VECTOR))) void TIMER0_A0_ISR(void) { _bic_SR_register_on_exit(LPM0_bits); }
