#include "./include/ble.h"

#include <msp430.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BLE_BUFFER_SIZE 64
#define BLE_LINE_SLOTS 2

static volatile char bleLines[BLE_LINE_SLOTS][BLE_BUFFER_SIZE], txBuffer[BLE_BUFFER_SIZE];
static volatile uint8_t bleLengths[BLE_LINE_SLOTS] = {0, 0}, bleReady[BLE_LINE_SLOTS] = {0, 0},
                        bleOverflow[BLE_LINE_SLOTS] = {0, 0}, bleW = 0, bleR = 0, bootSent = 0, named = 0, txpwrSet = 0,
                        samplingEnabled = 1, bleConnected = 0, txHead = 0, txTail = 0;
static uint16_t tickHzLocal = 1, bootDelayTicks = 5, resetCount = 0, sampleEveryTicks = 1, sampleCountdown = 0, seq = 0;
static uint16_t resetCauseVal = 0, uptimeTicks = 0;
static uint32_t uptimeS = 0;
static uint8_t lastTempState = 0, lastFault = 0;

static void blePrintChar(const char c)
{
	uint8_t next = (txHead + 1) % BLE_BUFFER_SIZE;

	uint16_t guard = 60000U;
	while (next == txTail) if (--guard == 0) return;

	__disable_interrupt();
	txBuffer[txHead] = c;
	txHead = next;

	if (!(UCA0IE & UCTXIE))
	{
		UCA0TXBUF = txBuffer[txTail];
		txTail = (txTail + 1) % BLE_BUFFER_SIZE;
		UCA0IE |= UCTXIE;
	}
	__enable_interrupt();
}

static void blePrintString(const char* str) { while (*str) blePrintChar(*str++); }
#include "./include/oled.h"
static uint8_t bleGetLine(char* out, uint8_t outSize, uint8_t* overflow)
{
	__disable_interrupt();
	if (!bleReady[bleR])
	{
		__enable_interrupt();
		return 0;
	}

	uint8_t n = bleLengths[bleR];
	if (n >= outSize) n = outSize - 1;

	for (uint8_t i = 0; i < n; ++i) out[i] = bleLines[bleR][i];
	out[n] = '\0';

	*overflow = bleOverflow[bleR];
	bleLengths[bleR] = 0;
	bleReady[bleR] = 0;
	bleOverflow[bleR] = 0;
	bleR = (bleR + 1) % BLE_LINE_SLOTS;

	__enable_interrupt();
	return 1;
}

static uint8_t checksum(const char* s)
{
	uint8_t x = 0;
	while (*s) x ^= (uint8_t)*s++;

	return x;
}

static int hexNibble(const char c)
{
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
	if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
	return -1;
}

static uint8_t verifyChecksum(char* cmd)
{
	char* star = strchr(cmd, '*');
	if (!star || star[1] == '\0' || star[2] == '\0' || star[3] != '\0') return 0;

	const int high = hexNibble(star[1]), low = hexNibble(star[2]);
	if (high < 0 || low < 0) return 0;
	const uint8_t got = (uint8_t)(high << 4 | low);

	const char saved = *star;
	*star = '\0';
	const uint8_t calc = checksum(cmd);
	*star = saved;

	if (calc != got) return 0;
	*star = '\0';

	return 1;
}

static int tokenIs(const char* cmd, const char* word)
{
	const size_t n = strlen(word);
	return strncmp(cmd, word, n) == 0 && (cmd[n] == '\0' || cmd[n] == ' ');
}

static void fmtTemp(char* buf, const size_t size, const int x100)
{
	const int a = x100 < 0 ? -x100 : x100;
	snprintf(buf, size, "%s%d.%02d", x100 < 0 ? "-" : "", a / 100, a % 100);
}

static void bleSendFault(const SensorSnapshot* s)
{
	char what[12];
	if (s->tempState == TEMP_FAULT) snprintf(what, sizeof(what), "RTD %02X", s->maxFault);
	else if (s->tempState == TEMP_INVALID) snprintf(what, sizeof(what), "RTD INVALID");
	else snprintf(what, sizeof(what), "TEMP RANGE");

	char frame[48];
	const int m = snprintf(frame, sizeof(frame), "CMD+DATA=0,ERR %s;M1:%04u\r\n", what, s->adcRaw);
	if (m > 0) blePrintString(frame);
}

static void bleSendMeasurement(const int tempX100, const uint16_t adcRaw)
{
	const uint16_t s = seq % 1000;
	seq++;

	static char payload[24];
	const int n = snprintf(payload, sizeof(payload), "S%03uT%+05dM%04u", s, tempX100, adcRaw);
	if (n <= 0) return;

	const uint8_t cs = checksum(payload);
	static char frame[48];
	const int m = snprintf(frame, sizeof(frame), "CMD+DATA=0,%s*%02X\r\n", payload, cs);
	if (m > 0) blePrintString(frame);
}

static void handleCommand(char* cmd, const SensorSnapshot* s)
{
	if (!verifyChecksum(cmd))
	{
		blePrintString("CMD+DATA=0,ERR CS\r\n");
		return;
	}

	if (tokenIs(cmd, "START"))
	{
		samplingEnabled = 1;
		sampleCountdown = 0;
		blePrintString("CMD+DATA=0,OK START\r\n");
	}
	else if (tokenIs(cmd, "STOP"))
	{
		samplingEnabled = 0;
		blePrintString("CMD+DATA=0,OK STOP\r\n");
	}
	else if (tokenIs(cmd, "RATE"))
	{
		char* end;
		long rate = strtol(cmd + 4, &end, 10);
		if (end == cmd + 4 || *end != '\0')
		{
			blePrintString("CMD+DATA=0,ERR RATE\r\n");
			return;
		}

		if (rate < 1) rate = 1;
		if (rate > 3600) rate = 3600;

		sampleEveryTicks = (uint16_t)(rate * tickHzLocal);
		if (sampleEveryTicks == 0) sampleEveryTicks = 1;

		if (sampleCountdown > sampleEveryTicks) sampleCountdown = sampleEveryTicks;
		blePrintString("CMD+DATA=0,OK RATE\r\n");
	}
	else if (tokenIs(cmd, "SEQ"))
	{
		char* end;
		const long newSeq = strtol(cmd + 4, &end, 10);
		if (end == cmd + 4 || *end != '\0' || newSeq < 0 || newSeq > 65535)
		{
			blePrintString("CMD+DATA=0,ERR SEQ\r\n");
			return;
		}

		seq = (uint16_t)newSeq;
		blePrintString("CMD+DATA=0,OK SEQ\r\n");
	}
	else if (tokenIs(cmd, "GET"))
	{
		char temp[10], frame[64];
		fmtTemp(temp, sizeof(temp), s->tempX100);
		if (s->tempState != TEMP_OK) blePrintString("CMD+DATA=0,ERR RTD INVALID\r\n");
		else
		{
			const int m = snprintf(frame, sizeof(frame), "CMD+DATA=0,OK s:%u t:%s m:%u r:%u\r\n", seq % 1000, temp,
			                       s->adcRaw, sampleEveryTicks / tickHzLocal);
			if (m > 0) blePrintString(frame);
		}
	}
	else if (tokenIs(cmd, "INFO"))
	{
		char frame[72];
		const int m = snprintf(frame, sizeof(frame), "CMD+DATA=0,OK fw:%s up:%lu rst:%04X flt:%02X smp:%u\r\n", FW_VERSION,
		                       (unsigned long)uptimeS, resetCauseVal, lastFault, samplingEnabled);
		if (m > 0) blePrintString(frame);
	}
	else if (tokenIs(cmd, "PING")) blePrintString("CMD+DATA=0,OK PONG\r\n");
	else if (tokenIs(cmd, "RESET"))
	{
		named = 0;
		txpwrSet = 0;
		blePrintString("CMD+RESET=0\r\n");
	}
	else blePrintString("CMD+DATA=0,ERR CMD\r\n");
}

void bleGpioInit(void)
{
	BLE_PWR_DIR |= BLE_PWR_PIN;
	BLE_PWR_PORT &= ~BLE_PWR_PIN;
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
	UCA0BRW = 4; // 115200
	UCA0MCTLW = UCOS16 | (5 << 4) | (0x55 << 8);
	UCA0IFG &= ~UCRXIFG;
	UCA0CTLW0 &= ~UCSWRST;

	delayMs(10);
	UCA0IE |= UCRXIE;
}

void bleSetResetCause(const uint16_t cause) { resetCauseVal = cause; }

void bleInit(uint16_t tickHz)
{
	if (tickHz == 0) tickHz = 1;
	tickHzLocal = tickHz;
	bootDelayTicks = tickHzLocal * 2;
	if (bootDelayTicks == 0) bootDelayTicks = 2;

	resetCount = bootDelayTicks;
	sampleEveryTicks = tickHzLocal;
	sampleCountdown = 0;

	bleConnected = 0;
	bootSent = 0;
	named = 0;
	txpwrSet = 0;
	samplingEnabled = 1;

	bleUartInit();
}

uint8_t bleIsConnected(void) { return bleConnected; }

void bleProcessRx(const SensorSnapshot* s)
{
	char line[BLE_BUFFER_SIZE];
	uint8_t overflow = 0;
	while (bleGetLine(line, sizeof(line), &overflow))
	{
		if (overflow && strncmp(line, "EVT+DATA=0,", 11) == 0) blePrintString("CMD+DATA=0,ERR LEN\r\n");
		else if (strncmp(line, "EVT+DATA=0,", 11) == 0) handleCommand(line + 11, s);
		else if (strncmp(line, "EVT+READY", 9) == 0)
		{
			bleConnected = 0;
			if (!named)
			{
				named = 1;
				blePrintString("CMD+NAME=SmartMoisture\r\n");
				blePrintString("CMD+RESET=0\r\n");
			}
			else if (!txpwrSet)
			{
				txpwrSet = 1;
				blePrintString("CMD+TXPWR=-8\r\n");
			}
		}
		else if (strncmp(line, "EVT+CON", 7) == 0) bleConnected = 1;
		else if (strncmp(line, "EVT+DISCON", 10) == 0) bleConnected = 0;
	}
}

void bleOnTick(const SensorSnapshot* s)
{
	lastTempState = s->tempState;
	lastFault = s->maxFault;
	if (++uptimeTicks >= tickHzLocal)
	{
		uptimeTicks = 0;
		uptimeS++;
	}
	if (sampleCountdown) sampleCountdown--;
	if (!bootSent)
	{
		if (resetCount == 0)
		{
			bootSent = 1;
			blePrintString("CMD+RESET=0\r\n");
		}
		else resetCount--;
	}

	if (bleConnected && samplingEnabled && sampleCountdown == 0)
	{
		if (s->tempState == TEMP_OK) bleSendMeasurement(s->tempX100, s->adcRaw);
		else bleSendFault(s);  /* the app shows what is wrong instead of silently freezing */
		sampleCountdown = sampleEveryTicks;
	}
}

#if defined(ENABLE_BLE)
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_A0_VECTOR
__interrupt
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
					LPM4_EXIT;
					return;
				}

				if (bleReady[bleW])
				{
					uint8_t next = (bleW + 1) % BLE_LINE_SLOTS;
					if (!bleReady[next])
					{
						bleW = next;
						bleLengths[bleW] = 0;
						bleOverflow[bleW] = 0;
					}
					else
					{
						LPM4_EXIT;
						return;
					}
				}

				if (c == '\r' || c == '\n')
				{
					if (bleLengths[bleW] > 0)
					{
						if (bleLengths[bleW] >= BLE_BUFFER_SIZE) bleLengths[bleW] = BLE_BUFFER_SIZE - 1;
						bleLines[bleW][bleLengths[bleW]] = '\0';
						bleReady[bleW] = 1;

						uint8_t next = (bleW + 1) % BLE_LINE_SLOTS;
						if (!bleReady[next])
						{
							bleW = next;
							bleLengths[bleW] = 0;
							bleOverflow[bleW] = 0;
						}
					}
					else bleLengths[bleW] = 0;

					LPM4_EXIT;
					return;
				}

				if (!bleOverflow[bleW])
				{
					if (bleLengths[bleW] < BLE_BUFFER_SIZE - 1) bleLines[bleW][bleLengths[bleW]++] = (char)c;
					else bleOverflow[bleW] = 1;
				}

				LPM4_EXIT;
				break;
			}
		case USCI_UART_UCTXIFG:
			{
				if (txHead != txTail)
				{
					UCA0TXBUF = txBuffer[txTail];
					txTail = (txTail + 1) % BLE_BUFFER_SIZE;
				}
				else UCA0IE &= ~UCTXIE;

				LPM4_EXIT;
				break;
			}
		case USCI_UART_UCSTTIFG:
		case USCI_UART_UCTXCPTIFG:
		default:
			break;
	}
}
#endif
