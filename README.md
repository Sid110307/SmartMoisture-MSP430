# SmartMoisture-MSP430

> BLE soil moisture sensor + temperature sensor project using TI MSP430 MCU.

This project implements a soil moisture and temperature sensor using the TI MSP430 microcontroller. The device
communicates via Bluetooth Low Energy (BLE) to transmit sensor data to a mobile application or other BLE-enabled
devices.

## Components

- **MCU**: Texas Instruments MSP430FR2433
- **Moisture Sensor**: Capacitive soil moisture sensor (HW-101)
- **Temperature Sensor**: MAX31865 RTD-to-Digital Converter with PT100/PT1000
- **Display**: SSD1306 OLED Display (128x64)
- **BLE Module**: nRF52810-based custom BLE Module
- **Status LED**: Standard LED

## Wiring

### Moisture Sensor:

- A0 - P1.6
- GND - GND
- VCC - 3.3V

### Temperature Sensor (MAX31865):

- CS - P2.3
- SCLK - P2.4
- MISO - P2.5
- MOSI - P2.6
- DRDY - P3.2
- BLE_PWR - P2.7
- VCC - 3.3V
- GND - GND

### OLED Display (SSD1306):

- SCL - P1.2
- SDA - P1.3
- VCC - 3.3V
- GND - GND

### BLE Module (nRF52810):

- BLE_WAKE - P2.2
- BLE_IRQ - P3.1
- BLE_RESET - P3.0
- BLE_TX - P1.5
- BLE_RX - P1.4
- VCC - 3.3V (via BLE_PWR, controlled by P2.7)
- GND - GND

### Status LED:

- LED - P1.7
- GND - GND
