#include <stdio.h>
#include <string.h>
#include <msp430.h>

#define OLED

#include "./include/config.h"
#include "./include/max.h"
#include "./include/oled.h"

#define BLE_BUFFER_SIZE 64
#define SAMPLE_TICK 5

static volatile char bleLine[BLE_BUFFER_SIZE];
static volatile uint8_t bleInitPending = 1, bleLineLen = 0, bleLineReady = 0, bleOverflow = 0, tick = 0, ack = 0, resetCount = 25;

static uint8_t bleConnected = 0;
static uint16_t sampleCountdown = 0;

static void adcInit(void)
{
	SYSCFG2 |= ADCPCTL6;
	ADCCTL0 = ADCSHT_2;
	ADCCTL1 = ADCSHP;
	ADCCTL2 = ADCRES_1;
	ADCMCTL0 = ADCINCH_6 | ADCSREF_0;
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
	TA0CCR0 = TA0R + (32768 / SAMPLE_TICK);
	TA0CCTL0 = CCIE;
	TA0CCTL0 &= ~CCIFG;
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
	switch (ack)
	{
		case 0:
			blePrintString("CMD+RESET=0\r\n");
			delayMs(BLE_COMMAND_DELAY);
		case 1:
			blePrintString("CMD+NAME=SmartMoisture\r\n");
			delayMs(BLE_COMMAND_DELAY);
		case 2:
			blePrintString("CMD+RESET=0\r\n");
			delayMs(BLE_COMMAND_DELAY);
		case 3:
			blePrintString("CMD+ADV=1\r\n");
			delayMs(BLE_COMMAND_DELAY);
			break;
		case 4:
			blePrintString("CMD+NOTIFY=1\r\n");
			delayMs(BLE_COMMAND_DELAY);
			break;
		default:
			break;
	}
}

static uint8_t checksum(const char* s)
{
	uint8_t x = 0;

	while (*s) x ^= *s++;
	return x;
}

static void bleSendMeasurement(const float tempC, const int adcRaw)
{
	const int tempX100 = (int)(tempC * 100.0f);
	int tempFrac = tempX100 % 100;
	if (tempFrac < 0) tempFrac = -tempFrac;

	char payload[48];
	int n = snprintf(payload, sizeof(payload), "{\"t\":%d.%02d,\"m\":%d}", tempX100 / 100, tempFrac, adcRaw);
	if (n <= 0) return;

	uint8_t cs = checksum(payload);
	char frame[80];
	int m = snprintf(frame, sizeof(frame), "CMD+DATA=0,%s*%02X\r\n", payload, cs);
	if (m > 0) blePrintString(frame);
}

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;
	PM5CTL0 &= ~LOCKLPM5;

	clockInit();
	timerInit();
	gpioInit();

#ifdef OLED
	oledInit();
	oledClear();
	oledDrawString(0, 3, "  SmartMoisture v2.0");
	oledDrawString(0, 4, "  Indriya Sensotech");
	delayMs(600);
#endif

	maxInit();
	adcInit();
	bleUartInit();
	__enable_interrupt();

	BLE_RESET_PORT &= ~BLE_RESET_PIN;
	delayMs(BLE_COMMAND_DELAY);
	BLE_RESET_PORT |= BLE_RESET_PIN;
	delayMs(BLE_COMMAND_DELAY);

#ifdef OLED
	oledClear();
	oledDrawString(0, 0, "BLE: Ready");
#endif

	while (1)
	{
		__bis_SR_register(LPM0_bits | GIE);
		char line[BLE_BUFFER_SIZE];

		if (bleGetLine(line, sizeof(line)))
		{
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
				#ifdef OLED
				oledDrawString(0, 0, "BLE: Connected");
				#endif
			}
			else if (strncmp(line, "EVT+DISCON", 10) == 0)
			{
				bleConnected = 0;
				ack = 0;
				bleInitPending = 1;
				resetCount = 25;
				#ifdef OLED
				oledDrawString(0, 0, "BLE: Disconnected");
				#endif
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

			#ifdef OLED
			oledDrawString(0, 1, faultMsg);
			#endif
			LED_PORT ^= LED_PIN;
			delayMs(FAULT_BLINK_DELAY);

			maxInit();
			delayMs(BLE_SAMPLE_DELAY);

			continue;
		}

		if (bleConnected && sampleCountdown == 0)
		{
			bleSendMeasurement(tDegC, adcRaw);
			sampleCountdown = SAMPLE_TICK;
		}

		#ifdef OLED
		char line1[20], line2[20];
		snprintf(line1, sizeof(line1), "Temp: %d.%02d C", tempX100 / 100, tempFrac);
		snprintf(line2, sizeof(line2), "ADC:  %u", adcRaw);

		oledDrawString(0, 3, line1);
		oledDrawString(0, 4, line2);
		#endif
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

			if (c == 0)
			{
				__bic_SR_register_on_exit(LPM0_bits);
				return;
			}

			if (bleLineReady)
			{
				__bic_SR_register_on_exit(LPM0_bits);
				return;
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
	TA0CCR0 += (32768 / SAMPLE_TICK);
	tick = 1;
	__bic_SR_register_on_exit(LPM0_bits);
}
