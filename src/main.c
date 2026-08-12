#include <msp430.h>
#include <stdio.h>

#include "./include/ble.h"
#include "./include/config.h"
#include "./include/max.h"
#include "./include/oled.h"

#define SAMPLE_TICK_PERIOD 5
#define UI_UPDATE_TICKS 2

#if defined(ENABLE_OLED)
static uint8_t uiTicks = 0;
#endif

static volatile uint8_t mainLoopAlive = 0, tick = 0;
static SensorSnapshot s;

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

	do CSCTL7 &= ~(FLLUNLOCK0 | FLLUNLOCK1);
	while (CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));
	CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK;
}

static void timerInit(void)
{
	TA0CTL = TASSEL__ACLK | MC__CONTINUOUS | TACLR;
	TA0CCR0 = TA0R + (32768U / SAMPLE_TICK_PERIOD);
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
	bleGpioInit();
#endif
}

static void tempStrFromX100(const int tempX100, char* out, const unsigned outSize)
{
	const int w = tempX100 / 100;
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
	s->tempX100 = maxReadRtdTemp();
	s->maxFault = maxReadReg(MAX_REG_FAULT);
#endif
}

static void enableWatchdog(void) { WDTCTL = WDTPW | WDTCNTCL | WDTSSEL__ACLK | WDTIS__512K; }

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;
	PM5CTL0 &= ~LOCKLPM5;
	const uint16_t resetCause = SYSRSTIV;

	clockInit();
	timerInit();
	gpioInit();

#if defined(ENABLE_OLED)
	oledInit();
	oledClear();
	oledDrawString(0, 3, "  SmartMoisture v2.0");
	oledDrawString(0, 4, "  Indriya Sensotech");

	char rstMsg[20];
	snprintf(rstMsg, sizeof(rstMsg), "  Last reset: %04X", resetCause);
	oledDrawString(0, 6, rstMsg);

	delayMs(600);
#else
	(void)resetCause;
#endif

#if defined(ENABLE_MAX)
	maxInit();
#endif

#if defined(ENABLE_ADC)
	adcInit();
#endif

#if defined(ENABLE_BLE)
	bleInit(SAMPLE_TICK_PERIOD);
#endif

	enableWatchdog();
	__enable_interrupt();

#if defined(ENABLE_BLE)
	BLE_PWR_PORT |= BLE_PWR_PIN;
	BLE_WAKE_PORT |= BLE_WAKE_PIN;
#endif

#if defined(ENABLE_OLED)
	oledClear();
	oledDrawString(0, 0, "BLE: Ready");
#endif

	while (1)
	{
		mainLoopAlive = 1;

#if defined(ENABLE_BLE)
		__bis_SR_register(LPM0_bits | GIE);
#else
		__bis_SR_register(LPM3_bits | GIE);
#endif

#if defined(ENABLE_BLE)
		bleProcessRx(&s);
#if defined(ENABLE_OLED)
		{
			static uint8_t lastConn = 0xFF;
			const uint8_t nowConn = bleIsConnected() ? 1 : 0;
			if (nowConn != lastConn)
			{
				lastConn = nowConn;
				oledDrawString(0, 0, nowConn ? "BLE: Connected" : "BLE: Ready    ");
			}
		}
#endif
#endif

		if (!tick) continue;
		tick--;

#if defined(ENABLE_OLED)
		oledRetry();
#endif
		readSensors(&s);

		if (s.maxFault != 0)
		{
			char faultMsg[20];
			snprintf(faultMsg, sizeof(faultMsg), "MAX: Error (%02X)", s.maxFault);

#if defined(ENABLE_OLED)
			oledDrawString(0, 1, faultMsg);
#endif
			LED_PORT |= LED_PIN;
			delayMs(FAULT_BLINK_DELAY);
			LED_PORT &= ~LED_PIN;
			delayMs(FAULT_BLINK_DELAY);

			continue;
		}

#if defined(ENABLE_BLE)
		bleOnTick(&s);
#endif

#if defined(ENABLE_OLED)
		if (uiTicks < 255) uiTicks++;
		if (uiTicks >= UI_UPDATE_TICKS)
		{
			uiTicks = 0;

			char line1[20], line2[20], tBuf[12];
			tempStrFromX100(s.tempX100, tBuf, sizeof(tBuf));

			snprintf(line1, sizeof(line1), "Temp: %5s C", tBuf);
			snprintf(line2, sizeof(line2), "ADC:  %4u", s.adcRaw);

			oledDrawString(0, 3, line1);
			oledDrawString(0, 4, line2);
		}
#endif
	}
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(TIMER0_A0_VECTOR)))
#else
#error Compiler not supported!
#endif
void TIMER0_A0_ISR(void)
{
	TA0CCR0 += 32768U / SAMPLE_TICK_PERIOD;
	if (tick < 255) tick++;

	if (mainLoopAlive)
	{
		mainLoopAlive = 0;
		enableWatchdog();
	}

	LPM4_EXIT;
}
