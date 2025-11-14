#pragma once

#include <msp430.h>
#include <stdint.h>

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

#define BLE_AT_TIMEOUT 200000UL
#define BLE_SAMPLE_DELAY 200000UL
#define BLE_FAULT_BLINK_DELAY 50000UL

static void delayCyclesUl(unsigned long n)
{
	__asm__ __volatile__ (
		"1:\n\t"
		"dec %[count]\n\t"
		"jnz 1b"
		: [count] "+r" (n)
	);
}
