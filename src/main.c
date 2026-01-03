#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <msp430.h>

#include "./include/config.h"
#include "./include/max.h"
#include "./include/oled.h"

#define BLE_BUFFER_SIZE 64
#define SAMPLE_TICK 5
#define BLE_RETRY_TICKS (SAMPLE_TICK * 5)

#if defined(ENABLE_BLE)
static volatile char bleLine[BLE_BUFFER_SIZE];
static volatile uint8_t bleInitPending = 1, bleLineLen = 0, bleLineReady = 0, bleOverflow = 0, tick = 0, ack = 0,
                        samplingEnabled = 1, resetCount = BLE_RETRY_TICKS;
static volatile uint16_t sampleEveryTicks = SAMPLE_TICK;

static uint8_t bleConnected = 0;
static uint16_t sampleCountdown = 0, seq = 0;
#else
static volatile uint8_t tick = 0;
#endif

typedef struct
{
	int tempX100;
	uint16_t adcRaw;
	uint8_t maxFault;
} SensorSnapshot;

#if defined(ENABLE_ADC)
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
#endif

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

#if defined(ENABLE_BLE)
	BLE_PWR_DIR |= BLE_PWR_PIN;
	BLE_PWR_PORT |= BLE_PWR_PIN;
	BLE_WAKE_DIR |= BLE_WAKE_PIN;
	BLE_WAKE_PORT &= ~BLE_WAKE_PIN;
	BLE_RESET_DIR |= BLE_RESET_PIN;
	BLE_RESET_PORT |= BLE_RESET_PIN;
#endif
}

#if defined(ENABLE_BLE)

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
	uint16_t counter = 60000;
	while (!(UCA0IFG & UCTXIFG)) if (--counter == 0) break;
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

static void tempStrFromX100(int tempX100, char* out, unsigned outSize)
{
	int w = tempX100 / 100;
	int f = tempX100 % 100;

	if (f < 0) f = -f;
	snprintf(out, outSize, "%d.%02d", w, f);
}

static void readSensors(SensorSnapshot* s)
{
	s->tempX100 = 0;
	s->adcRaw = 0;
	s->maxFault = 0;

#if defined(ENABLE_ADC)
	s->adcRaw = adcReadRaw();
#endif

#if defined(ENABLE_MAX)
	s->maxFault = maxReadReg(MAX_REG_FAULT);
	if (s->maxFault == 0) s->tempX100 = maxReadRtdTemp();
#endif
}

static uint8_t checksum(const char* s)
{
	uint8_t x = 0;

	while (*s) x ^= (uint8_t)(*s++);
	return x;
}

static int hexNibble(char c)
{
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
	if (c >= 'a' && c <= 'f') return 10 + (c - 'a');

	return -1;
}

static uint8_t verifyChecksum(char* cmd)
{
	char* star = strchr(cmd, '*');
	if (!star) return 0;

	if (star[1] == '\0' || star[2] == '\0' || star[3] != '\0') return 0;

	const int hi = hexNibble(star[1]);
	const int lo = hexNibble(star[2]);
	if (hi < 0 || lo < 0) return 0;
	const uint8_t got = (uint8_t)((hi << 4) | lo);

	char saved = *star;
	*star = '\0';
	const uint8_t calc = checksum(cmd);
	*star = saved;

	if (calc != got) return 0;
	*star = '\0';

	return 1;
}

static void bleSendMeasurement(int tempX100, uint16_t adcRaw)
{
	static char tBuf[12];
	tempStrFromX100(tempX100, tBuf, sizeof(tBuf));

	static char payload[48];
	int n = snprintf(payload, sizeof(payload), "{\"s\":%u,\"t\":%s,\"m\":%u}", seq++, tBuf, adcRaw);
	if (n <= 0) return;

	uint8_t cs = checksum(payload);
	static char frame[80];
	int m = snprintf(frame, sizeof(frame), "CMD+DATA=0,%s*%02X\r\n", payload, cs);
	if (m > 0) blePrintString(frame);
}

static void handleCommand(char* cmd, const SensorSnapshot* s)
{
	if (!verifyChecksum(cmd))
	{
		blePrintString("CMD+DATA=0,ERR CS\r\n");
		return;
	}

	if (strncmp(cmd, "START", 5) == 0)
	{
		samplingEnabled = 1;
		sampleCountdown = 0;

		blePrintString("CMD+DATA=0,OK START\r\n");
	}
	else if (strncmp(cmd, "STOP", 4) == 0)
	{
		samplingEnabled = 0;
		blePrintString("CMD+DATA=0,OK STOP\r\n");
	}
	else if (strncmp(cmd, "RATE ", 5) == 0)
	{
		int rate = atoi(cmd + 5);
		if (rate < 1) rate = 1;
		if (rate > 3600) rate = 3600;

		if (rate > 0)
		{
			sampleEveryTicks = (uint16_t)(rate * SAMPLE_TICK);
			if (sampleCountdown > sampleEveryTicks) sampleCountdown = sampleEveryTicks;
		}
		blePrintString("CMD+DATA=0,OK RATE\r\n");
	}
	else if (strncmp(cmd, "SEQ ", 4) == 0)
	{
		const int newSeq = atoi(cmd + 4);
		if (newSeq >= 0) seq = (uint16_t)newSeq;

		blePrintString("CMD+DATA=0,OK SEQ\r\n");
	}
	else if (strncmp(cmd, "GET", 3) == 0)
	{
		char tbuf[12];
		tempStrFromX100(s->tempX100, tbuf, sizeof(tbuf));

		char response[64];
		int n = snprintf(response, sizeof(response), "CMD+DATA=0,OK s:%u,t:%s,m:%u,r:%u\r\n", seq, tbuf, s->adcRaw,
		                 sampleEveryTicks / SAMPLE_TICK);
		if (n > 0) blePrintString(response);
	}
	else if (strncmp(cmd, "RESET", 5) == 0)
	{
		bleInitPending = 1;
		resetCount = 0;
	}
	else blePrintString("CMD+DATA=0,ERR CMD\r\n");
}

#endif

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;
	PM5CTL0 &= ~LOCKLPM5;

	clockInit();
	timerInit();
	gpioInit();

#if defined(ENABLE_OLED)
	oledInit();
	oledClear();
	oledDrawString(0, 3, "  SmartMoisture v2.0");
	oledDrawString(0, 4, "  Indriya Sensotech");
	delayMs(600);
#endif

#if defined(ENABLE_MAX)
	maxInit();
#endif

#if defined(ENABLE_ADC)
	adcInit();
#endif

#if defined(ENABLE_BLE)
	bleUartInit();
#endif

	__enable_interrupt();

#if defined(ENABLE_BLE)
	BLE_RESET_PORT &= ~BLE_RESET_PIN;
	delayMs(BLE_COMMAND_DELAY);
	BLE_RESET_PORT |= BLE_RESET_PIN;
	delayMs(BLE_COMMAND_DELAY);
#endif

#if defined(ENABLE_OLED)
	oledClear();
	oledDrawString(0, 0, "BLE: Ready");
#endif

	while (1)
	{
		__bis_SR_register(LPM0_bits | GIE);

		SensorSnapshot s;
		readSensors(&s);

#if defined(ENABLE_BLE)
		char line[BLE_BUFFER_SIZE];

		if (bleGetLine(line, sizeof(line)))
		{
			if (strncmp(line, "EVT+DATA=0,", 11) == 0) handleCommand(line + 11, &s);
			else if (strncmp(line, "EVT+READY", 9) == 0)
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
#if defined(ENABLE_OLED)
				oledDrawString(0, 0, "BLE: Connected");
#endif
			}
			else if (strncmp(line, "EVT+DISCON", 10) == 0)
			{
				bleConnected = 0;
				ack = 0;
				bleInitPending = 1;
				resetCount = BLE_RETRY_TICKS;
#if defined(ENABLE_OLED)
				oledDrawString(0, 0, "BLE: Disconnected");
#endif
			}
		}
#endif

		if (!tick) continue;
		tick--;

#if defined(ENABLE_BLE)
		if (sampleCountdown) sampleCountdown--;

		if (!bleConnected)
		{
			samplingEnabled = 0;

			if (resetCount == 0)
			{
				resetCount = BLE_RETRY_TICKS;
				if (bleInitPending) bleInitSequence();
			}
			else resetCount--;
		}
		else
		{
			resetCount = BLE_RETRY_TICKS;
			bleInitPending = 0;
		}
#endif

		if (s.maxFault != 0)
		{
			char faultMsg[16];
			snprintf(faultMsg, sizeof(faultMsg), "MAX: Error (%02X)", s.maxFault);

#if defined(ENABLE_OLED)
			oledDrawString(0, 1, faultMsg);
#endif
			LED_PORT ^= LED_PIN;
			delayMs(FAULT_BLINK_DELAY);

			maxInit();
			delayMs(BLE_SAMPLE_DELAY);
			continue;
		}

#if defined(ENABLE_BLE)
		if (bleConnected && samplingEnabled && sampleCountdown == 0)
		{
			bleSendMeasurement(s.tempX100, s.adcRaw);
			sampleCountdown = sampleEveryTicks;
		}
#endif

#if defined(ENABLE_OLED)
		char line1[20], line2[20];
		char tbuf[12];
		tempStrFromX100(s.tempX100, tbuf, sizeof(tbuf));

		snprintf(line1, sizeof(line1), "Temp: %s C", tbuf);
		snprintf(line2, sizeof(line2), "ADC:  %u", s.adcRaw);

		oledDrawString(0, 3, line1);
		oledDrawString(0, 4, line2);
#endif
	}
}

#if defined(ENABLE_BLE)
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
#endif

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
	if (tick < 255) tick++;
	__bic_SR_register_on_exit(LPM0_bits);
}
