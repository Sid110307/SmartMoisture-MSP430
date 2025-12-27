#pragma once

#include <msp430.h>
#include <stdint.h>

#define MAX_CS_PORT P2OUT
#define MAX_CS_DIR P2DIR
#define MAX_CS_PIN BIT3

#define MAX_REG_CONF 0x00
#define MAX_REG_RTD_MSB 0x01
#define MAX_REG_RTD_LSB 0x02
#define MAX_REG_FAULT 0x07

#define MAX_CFG_BIAS 0x80
#define MAX_CFG_AUTO 0x40
#define MAX_CFG_1SHOT 0x20
#define MAX_CFG_3WIRE 0x10
#define MAX_CFG_FAULT 0x02
#define MAX_CFG_50HZ 0x01

#define CVD_A (3.9083e-3f)
#define CVD_B (-5.775e-7f)

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

#define BLE_COMMAND_DELAY 10000UL
#define BLE_SAMPLE_DELAY 250000UL
#define FAULT_BLINK_DELAY 125000UL

static void delayCyclesUl(uint32_t n)
{
	while (n--)
		__no_operation();
}
