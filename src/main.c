#include <stdio.h>
#include <string.h>
#include <msp430.h>

#include "./include/config.h"
#include "./include/max.h"
#include "./include/oled.h"

#define BLE_BUFFER_SIZE 64
#define BLE_INIT_TICKS 25

static volatile char bleLine[BLE_BUFFER_SIZE];
static volatile uint8_t bleInitPending = 1, bleLineLen = 0, bleLineReady = 0, bleOverflow = 0, tick = 0;
static uint8_t bleConnected = 0;
static uint16_t bleInitCountdown = 0;

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
	CSCTL1 = DCOFTRIMEN | DCORSEL_0;
	CSCTL2 = FLLD_0 | 30;

	delayCyclesUl(3UL);
	__bic_SR_register(SCG0);
	CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK;
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

	UCA0BRW = 104;
	UCA0MCTLW = 0xD600;
	UCA0CTLW0 &= ~UCSWRST;
	UCA0IE |= UCRXIE;
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

	blePrintString("CMD+RESET=0\r\n");
	delayCyclesUl(BLE_COMMAND_DELAY);
	blePrintString("CMD+NAME=SmartMoisture\r\n");
	delayCyclesUl(BLE_COMMAND_DELAY);
	blePrintString("CMD+ADV=1\r\n");
	delayCyclesUl(BLE_COMMAND_DELAY);
	blePrintString("CMD+NOTIFY=1\r\n");
	delayCyclesUl(BLE_COMMAND_DELAY);
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

	maxInit();
	adcInit();
	bleUartInit();
	__enable_interrupt();

	BLE_RESET_PORT &= ~BLE_RESET_PIN;
	delayCyclesUl(BLE_COMMAND_DELAY);
	BLE_RESET_PORT |= BLE_RESET_PIN;
	delayCyclesUl(BLE_COMMAND_DELAY);

	bleInitPending = 1;
	oledClear();
	oledDrawString(0, 0, "BLE: Ready");

	while (1)
	{
		__bis_SR_register(LPM0_bits | GIE);
		if (bleInitPending)
		{
			bleInitSequence();
			bleInitPending = 0;
		}

		if (bleLineReady)
		{
			char local[BLE_BUFFER_SIZE];

			__disable_interrupt();
			uint8_t n = bleLineLen;
			if (n >= BLE_BUFFER_SIZE) n = BLE_BUFFER_SIZE - 1;

			memcpy(local, (const void*)bleLine, n);
			local[n] = '\0';

			bleLineLen = 0;
			bleLineReady = 0;
			__enable_interrupt();

			oledDrawString(0, 5, local);
			if (strncmp(local, "EVT+READY", 9) == 0) bleInitPending = 1;
			else if (strncmp(local, "EVT+CON", 7) == 0)
			{
				bleConnected = 1;
				bleInitCountdown = 0;

				oledDrawString(0, 0, "BLE: Connected");
			}
			else if (strncmp(local, "EVT+DISCON", 10) == 0)
			{
				bleConnected = 0;
				bleInitCountdown = 0;

				oledDrawString(0, 0, "BLE: Disconnected");
			}
		}

		if (!tick) continue;
		tick = 0;

		if (!bleConnected && ++bleInitCountdown >= BLE_INIT_TICKS)
		{
			bleInitCountdown = 0;
			bleInitPending = 1;
		}
		else bleInitCountdown = 0;

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

		if (!maxIsReady())
		{
			oledDrawString(0, 1, "MAX: Not ready");
			continue;
		}

		const uint16_t adcRaw = adcReadRaw();
		float tDegC = maxReadRtdTemp();

		int tempX100 = (int)(tDegC * 100.0f);
		int tempFrac = tempX100 % 100;
		if (tempFrac < 0) tempFrac = -tempFrac;

		char line1[20], line2[20];
		if (bleConnected) bleSendMeasurement(tDegC, adcRaw);

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
			if (!bleLineReady)
			{
				if (c == '\r')
				{
					if (bleOverflow)
					{
						bleLineLen = 0;
						bleOverflow = 0;
					}
					else
					{
						bleLine[bleLineLen] = '\0';
						bleLineReady = 1;
					}
				}
				else if (c != '\n' && !bleOverflow)
				{
					if (bleLineLen < BLE_BUFFER_SIZE - 1) bleLine[bleLineLen++] = (char)c;
					else bleOverflow = 1;
				}
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
	tick = 1;
	__bic_SR_register_on_exit(LPM0_bits);
}
