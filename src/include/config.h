#pragma once

#include <msp430.h>
#include <stdint.h>

#define MAX_CS_PORT P2OUT
#define MAX_CS_DIR P2DIR
#define MAX_CS_PIN BIT3

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
#define ADC_RES_SETTING ADCRES_1

#define OLED_ADDR 0x3C

#define LED_PORT P1OUT
#define LED_DIR P1DIR
#define LED_PIN BIT7

#define BLE_WAKE_PORT P2OUT
#define BLE_WAKE_DIR P2DIR
#define BLE_WAKE_PIN BIT2

#define BLE_RESET_PORT P3OUT
#define BLE_RESET_DIR P3DIR
#define BLE_RESET_PIN BIT0

#define BLE_PWR_PORT P2OUT
#define BLE_PWR_DIR P2DIR
#define BLE_PWR_PIN BIT7

#define BLE_TXSEL_BIT BIT4
#define BLE_RXSEL_BIT BIT5

#define BLE_COMMAND_DELAY 20000UL
#define BLE_SAMPLE_DELAY 200000UL
#define FAULT_BLINK_DELAY 30000UL

static void delayCyclesUl(uint32_t n)
{
	while (n--)
		__no_operation();
}
