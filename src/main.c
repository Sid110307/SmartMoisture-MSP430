#include <stdio.h>
#include <string.h>
#include <msp430.h>

#include "./include/config.h"
#include "./include/max.h"
#include "./include/oled.h"

#define BLE_BUFFER_SIZE 64

static volatile char bleLine[BLE_BUFFER_SIZE];
static volatile uint8_t bleInitPending = 1, bleLineLen = 0, bleLineReady = 0, bleOverflow = 0, tick = 0, ack = 0;
static volatile uint16_t resetCount = 25;

static uint8_t bleConnected = 0;
static uint16_t sampleCountdown = 0;

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

static void clockInit(void)
{
	__bis_SR_register(SCG0);

	CSCTL3 = SELREF__REFOCLK;
	CSCTL1 = DCOFTRIMEN | DCOFTRIM0 | DCOFTRIM1 | DCORSEL_3;
	CSCTL2 = FLLD_0 | 243;

	__bic_SR_register(SCG0);
	CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK;
}

static void timerInit(void)
{
	TA0CTL = TASSEL__ACLK | MC__CONTINUOUS | TACLR;
	TA0CCR0 = TA0R + (32768 / 5);
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

	UCA0BRW = 4;
	UCA0MCTLW = UCOS16 | (5 << 4) | (0x55 << 8);
	UCA0IFG &= ~UCRXIFG;
	UCA0CTLW0 &= ~UCSWRST;

	delayMs(10);
	UCA0IE |= UCRXIE;
}

static uint8_t bleGetLine(char* out, uint8_t outSize)
{
	if (!bleLineReady) return 0;
	__disable_interrupt();

	uint8_t n = bleLineLen;
	if (n >= outSize) n = outSize - 1;

	for (uint8_t i = 0; i < n; ++i) out[i] = bleLine[i];
	out[n] = '\0';

	bleLineLen = 0;
	bleLineReady = 0;
	bleOverflow = 0;

	__enable_interrupt();
	return 1;
}

static void blePrintChar(const char c)
{
	while (!(UCA0IFG & UCTXIFG));
	UCA0TXBUF = (uint8_t)c;
}

static void blePrintString(const char* str) { while (*str) blePrintChar(*str++); }

static void bleInitSequence(void)
{
	BLE_WAKE_PORT |= BLE_WAKE_PIN;
	ack = 0;

	blePrintString("CMD+RESET=0\r\n");
	delayMs(BLE_COMMAND_DELAY);
	blePrintString("CMD+NAME=SmartMoisture\r\n");
	delayMs(BLE_COMMAND_DELAY);
	blePrintString("CMD+RESET=0\r\n");
	delayMs(BLE_COMMAND_DELAY);
	blePrintString("CMD+ADV=1\r\n");
	delayMs(BLE_COMMAND_DELAY);
	blePrintString("CMD+NOTIFY=1\r\n");
	delayMs(BLE_COMMAND_DELAY);
}

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

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;
	PM5CTL0 &= ~LOCKLPM5;

	clockInit();
	timerInit();
	gpioInit();

	oledInit();
	oledClear();
	oledDrawString(0, 3, "  SmartMoisture v2.0");
	oledDrawString(0, 4, "  Indriya Sensotech");
	delayMs(1000);

	maxInit();
	adcInit();
	bleUartInit();
	__enable_interrupt();

	BLE_RESET_PORT &= ~BLE_RESET_PIN;
	delayMs(BLE_COMMAND_DELAY);
	BLE_RESET_PORT |= BLE_RESET_PIN;
	delayMs(BLE_COMMAND_DELAY);

	oledClear();
	oledDrawString(0, 0, "BLE: Ready");

	while (1)
	{
		__bis_SR_register(LPM0_bits | GIE);
		char line[BLE_BUFFER_SIZE];

		if (bleGetLine(line, sizeof(line)))
		{
			oledDrawString(0, 2, line);

			if (strncmp(line, "EVT+READY", 9) == 0)
			{
				bleConnected = 0;
				ack = 0;
				bleInitPending = 1;
				resetCount = 0;
			}
			else if (strncmp(line, "RSP", 3) == 0)
			{
				ack++;
				if (ack > 4) ack = 0;
			}
			else if (strncmp(line, "EVT+CON", 7) == 0)
			{
				bleConnected = 1;
				oledDrawString(0, 0, "BLE: Connected");
			}
			else if (strncmp(line, "EVT+DISCON", 10) == 0)
			{
				bleConnected = 0;
				ack = 0;
				bleInitPending = 1;
				resetCount = 25;
				oledDrawString(0, 0, "BLE: Disconnected");
			}
		}

		if (!tick) continue;
		tick = 0;

		if (sampleCountdown) sampleCountdown--;
		if (!bleConnected)
		{
			if (resetCount == 0)
			{
				resetCount = 25;
				if (bleInitPending) bleInitSequence();
			}
			else resetCount--;
		}
		else
		{
			resetCount = 25;
			bleInitPending = 0;
		}

		const uint16_t adcRaw = adcReadRaw();
		float tDegC = maxReadRtdTemp();

		int tempX100 = (int)(tDegC * 100.0f);
		int tempFrac = tempX100 % 100;
		if (tempFrac < 0) tempFrac = -tempFrac;

		const uint8_t fault = maxReadReg(MAX_REG_FAULT);
		if (fault != 0)
		{
			char faultMsg[16];
			snprintf(faultMsg, sizeof(faultMsg), "MAX: Error (%02X)", fault);

			oledDrawString(0, 1, faultMsg);
			LED_PORT ^= LED_PIN;
			delayMs(FAULT_BLINK_DELAY);

			maxInit();
			delayMs(BLE_SAMPLE_DELAY);

			continue;
		}

		char line1[20], line2[20];
		if (bleConnected && sampleCountdown == 0)
		{
			bleSendMeasurement(tDegC, adcRaw);
			sampleCountdown = 5;
		}

		snprintf(line1, sizeof(line1), "Temp: %d.%02d C", tempX100 / 100, tempFrac);
		snprintf(line2, sizeof(line2), "ADC:  %u", adcRaw);
		oledDrawString(0, 3, line1);
		oledDrawString(0, 4, line2);
	}
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_A0_VECTOR __interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(USCI_A0_VECTOR)))
#else
#error Compiler not supported!
#endif
void USCI_A0_ISR(void)
{
	switch (__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG))
	{
		case USCI_NONE:
			break;
		case USCI_UART_UCRXIFG:
		{
			const uint8_t c = (uint8_t)UCA0RXBUF;
			LED_PORT ^= LED_PIN; // TODO: Remove

			if (c == 0)
			{
				__bic_SR_register_on_exit(LPM0_bits);
				return;
			}

			if (bleLineReady)
			{
				bleLineReady = 0;
				bleLineLen = 0;
				bleOverflow = 0;
			}

			if (c == '\r' || c == '\n')
			{
				if (bleLineLen > 0)
				{
					if (bleLineLen >= BLE_BUFFER_SIZE) bleLineLen = BLE_BUFFER_SIZE - 1;

					bleLine[bleLineLen] = '\0';
					bleLineReady = 1;
				}
				else bleLineLen = 0;

				bleOverflow = 0;
				__bic_SR_register_on_exit(LPM0_bits);

				return;
			}

			if (!bleOverflow)
			{
				if (bleLineLen < BLE_BUFFER_SIZE - 1) bleLine[bleLineLen++] = (char)c;
				else bleOverflow = 1;
			}

			__bic_SR_register_on_exit(LPM0_bits);
			break;
		}
		case USCI_UART_UCTXIFG:
		case USCI_UART_UCSTTIFG:
		case USCI_UART_UCTXCPTIFG: default:
			break;
	}
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR __interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(TIMER0_A0_VECTOR)))
#else
#error Compiler not supported!
#endif
void TIMER0_A0_ISR(void)
{
	TA0CCR0 += (32768 / 5);
	tick = 1;

	__bic_SR_register_on_exit(LPM0_bits);
}
