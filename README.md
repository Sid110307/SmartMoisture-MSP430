# SmartMoisture-MSP430

> BLE soil moisture sensor + temperature sensor project using TI MSP430 MCU.

This project implements a soil moisture and temperature sensor using the TI MSP430 microcontroller. The device
communicates via Bluetooth Low Energy (BLE) to transmit sensor data to a mobile application or other BLE-enabled
devices.

## Components

- **MCU**: TI MSP430FR2433
- **Moisture Sensor**: Capacitive soil moisture sensor (HW-101)
- **Temperature Sensor**: MAX31865 RTD-to-Digital Converter with PT100
- **Display**: SSD1306 OLED Display (128x64)
- **BLE Module**: nRF52810-based custom BLE Module
- **Status LED**: Standard LED

## Wiring

### Moisture Sensor

- A0 - P1.6
- GND - GND
- VCC - 3.3V

### Temperature Sensor (MAX31865)

- CS - P2.3
- SCLK - P2.4
- MISO - P2.5
- MOSI - P2.6
- DRDY - P3.2 (unused)
- BLE_PWR - P2.7
- VCC - 3.3V
- GND - GND

### OLED Display (SSD1306)

- SCL - P1.2
- SDA - P1.3
- VCC - 3.3V
- GND - GND

### BLE Module (nRF52810-based, UART @ 115200 bps)

- BLE_WAKE - P2.2
- BLE_IRQ - P3.1
- BLE_RESET - P3.0
- BLE_TX - P1.5
- BLE_RX - P1.4
- VCC - 3.3V (via BLE_PWR, controlled by P2.7)
- GND - GND

### Status LED

- LED - P1.7
- GND - GND

## Data

BLE data (from nRF52810) comes in the form:

```
SsssT+ttttMmmmm*HH\r\n
```

where `Ssss` is the sequence number, `T+tttt` is the temperature read in Celsius * 100 (signed), `Mmmmm` is the raw ADC value of
the soil moisture sensor, and `HH` is the data's XORed checksum.

Example: `S0001T+0234M1023*5A\r\n` indicates sequence 1, temperature 23.4°C, and soil moisture reading of 1023.

Default sampling rate is every 1 second.
The BLE device's pairing pin is `123456`.

### Commands

| Command | Description                       | Example          | Response                 |
|---------|-----------------------------------|------------------|--------------------------|
| START   | Start periodic sampling           | `START*HH\r\n`   | `OK START\r\n`           |
| STOP    | Stop periodic sampling            | `STOP*HH\r\n`    | `OK STOP\r\n`            |
| RATE n  | Set sampling rate to n seconds    | `RATE 30*HH\r\n` | `OK RATE\r\n`            |
| SEQ n   | Set sequence number to n          | `SEQ 42*HH\r\n`  | `OK SEQ\r\n`             |
| GET     | Get most recent sample and config | `GET*HH\r\n`     | `S123T+2450M1023*5A\r\n` |
| RESET   | Reset the BLE module              | `RESET*HH\r\n`   | No response              |

Include `*HH\r\n` at the end of each command, where `HH` is the XORed checksum of the command string.

## Feature Flags

In [`config.h`](src/include/config.h), you can enable/disable features:

- `ENABLE_OLED`: Enable the OLED display for showing sensor data and status.
- `ENABLE_BLE`: Enable the BLE module for wireless communication.
- `ENABLE_ADC`: Enable the ADC reading of the soil moisture sensor.
- `ENABLE_MAX`: Enable the MAX31865 temperature sensor reading.

## MAX31865 Fault Codes

| Mask | Description                                                             |
|------|-------------------------------------------------------------------------|
| 0x80 | RTD High Threshold: Measured resistance &geq; programmed high threshold |
| 0x40 | RTD Low Threshold: Measured resistance &leq; programmed low threshold   |
| 0x20 | REFIN- &gt; 0.85 &times; V<sub>BIAS</sub>: Reference input high fault   |
| 0x10 | REFIN- &lt; 0.85 &times; V<sub>BIAS</sub>: Reference input low fault    |
| 0x08 | RTDIN- &lt; 0.85 &times; V<sub>BIAS</sub>: RTD input low fault          |
| 0x04 | Overvoltage/Undervoltage fault: Measured voltage exceeds normal range   |
| 0x02 | Reserved                                                                |
| 0x01 | Reserved                                                                |
