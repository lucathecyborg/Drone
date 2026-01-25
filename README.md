
# Esp32 based quadrocopter

This is an open source quadrocopter/drone using an esp32 as its flight controller, created for my final year project at school.  
For complete functionality, use with my [custom RF controller](https://github.com/lucathecyborg/Controller) made speficically for this drone.  
All 3d files are avalible on my github as well, so you can print the drone body yourself.


## Features

- PID based stabilization using an Adafruit TDK InvenSense ICM-20948 9-DoF IMU
- Communication using an e01 ml01dp5 antenna
- Return to home function using a Neo-8M GPS module

## BOM

**Controller:** 30pin esp32

**Motors:** 4x KV2700 brushless motors

**Batteries:** 2x 3s 4500mAh Lipo

**ESC:** 4x 40A ESC

**Antenna:** e01 ml01dp5

**IMU:** Adafruit TDK InvenSense ICM-20948 9-DoF

**GPS:** NEO-8M with antenna

**Frame:** F330 drone frame arms + 3d printed body

**Other sensors:** ADS1115 for battery monitoring, BMP-280 barometer

**Various hardware:** Step-down buck converter (5v-3.3v), 4pin female jst XH2.54 XH 2.54mm cable, XT60 2-way splitter cable (x2), 4x propellers (3 or 2 blade it depending on what you want)

**Other supplies:** Protoboard, wires, screw terminals, female pin headers, power switch

#
# Drone Pinout Reference

## ESP32 Main Controller Pinout

| Pin Number | Pin Name | Function | Connected To | Notes |
|------------|----------|----------|--------------|-------|
| GPIO 4 | CE | nRF24L01 Chip Enable | E01-ML01DP5 CE | Radio module control |
| GPIO 5 | CSN | nRF24L01 Chip Select | E01-ML01DP5 CSN | SPI slave select (active low) |
| GPIO 14 | PWM Out | ESC Control | Top Left ESC | PWM Channel 0, 50Hz |
| GPIO 18 | SCK | SPI Clock | E01-ML01DP5 SCK | 1MHz SPI clock |
| GPIO 19 | MISO | SPI Data In | E01-ML01DP5 MISO | SPI receive |
| GPIO 21 | SDA | I²C Data | ICM-20948, BMP280, ADS1115 | I²C communication (shared bus) |
| GPIO 22 | SCL | I²C Clock | ICM-20948, BMP280, ADS1115 | I²C communication (shared bus) |
| GPIO 23 | MOSI | SPI Data Out | E01-ML01DP5 MOSI | SPI transmit |
| GPIO 25 | PWM Out | ESC Control | Bottom Right ESC | PWM Channel 3, 50Hz |
| GPIO 26 | PWM Out | ESC Control | Bottom Left ESC | PWM Channel 2, 50Hz |
| GPIO 27 | PWM Out | ESC Control | Top Right ESC | PWM Channel 1, 50Hz |
| GPIO 16 | RX2 | UART Receive | NEO-8M GPS TX | GPS communication (planned) |
| GPIO 17 | TX2 | UART Transmit | NEO-8M GPS RX | GPS communication (planned) |
| 3.3V | Power | Power Supply | All sensor modules | 3.3V regulated output |
| GND | Ground | Ground | All modules | Common ground |

## IMU Sensor (ICM-20948 9-DoF)

| IMU Pin | Function | ESP32 Pin | I²C Address | Notes |
|---------|----------|-----------|-------------|-------|
| VCC | Power | 3.3V | - | 3.3V power supply |
| GND | Ground | GND | - | Ground connection |
| SDA | I²C Data | GPIO 21 | 0x68 or 0x69 | I²C data line (shared) |
| SCL | I²C Clock | GPIO 22 | 0x68 or 0x69 | I²C clock line (shared) |
| INT | Interrupt | Not connected | - | Optional interrupt pin |
| AD0 | Address Select | GND or 3.3V | - | I²C address selection |

## Barometric Pressure Sensor (BMP280)

| BMP280 Pin | Function | ESP32 Pin | I²C Address | Notes |
|------------|----------|-----------|-------------|-------|
| VCC | Power | 3.3V | - | 3.3V power supply |
| GND | Ground | GND | - | Ground connection |
| SDA | I²C Data | GPIO 21 | 0x76 or 0x77 | I²C data line (shared) |
| SCL | I²C Clock | GPIO 22 | 0x76 or 0x77 | I²C clock line (shared) |
| CSB | Chip Select | 3.3V | - | Pull high for I²C mode |
| SDO | Address Select | GND or 3.3V | - | 0x76 (GND) or 0x77 (3.3V) |

## ADC Module (ADS1115) - Battery Monitoring

| ADS1115 Pin | Function | ESP32 Pin | I²C Address | Notes |
|-------------|----------|-----------|-------------|-------|
| VCC | Power | 3.3V | - | 3.3V power supply |
| GND | Ground | GND | - | Ground connection |
| SDA | I²C Data | GPIO 21 | 0x48-0x4B | I²C data line (shared) |
| SCL | I²C Clock | GPIO 22 | 0x48-0x4B | I²C clock line (shared) |
| ADDR | Address Select | GND/VCC/SDA/SCL | - | Sets I²C address (0x48 default) |
| A0 | Analog In 0 | Battery 1 via divider | - | 3S LiPo #1 voltage |
| A1 | Analog In 1 | Battery 2 via divider | - | 3S LiPo #2 voltage (optional) |
| A2 | Analog In 2 | Battery 3 via divider | - | 3S LiPo #3 voltage (optional) |
| A3 | Analog In 3 | Battery 4 via divider | - | 3S LiPo #4 voltage (optional) |

### Battery Voltage Divider Circuit (per channel)
- **R1:** 10kΩ (to battery positive)
- **R2:** 3.3kΩ (to ground)
- **Divider Ratio:** Scales 12.6V (3S fully charged) to ~3.13V
- **ADS1115 Input:** Connect between R1 and R2

## GPS Module (NEO-8M)

| GPS Pin | Function | ESP32 Pin | Notes |
|---------|----------|-----------|-------|
| VCC | Power | 3.3V or 5V | Check module voltage rating |
| GND | Ground | GND | Ground connection |
| TX | UART Transmit | GPIO 16 (RX2) | GPS data output |
| RX | UART Receive | GPIO 17 (TX2) | GPS commands input |
| PPS | Pulse Per Second | Not connected | Optional timing signal |

## nRF24L01 Radio Module (E01-ML01DP5)

| Radio Pin | Function | ESP32 Pin | Notes |
|-----------|----------|-----------|-------|
| VCC | Power | 3.3V | Requires 100µF capacitor nearby |
| GND | Ground | GND | Ground connection |
| CE | Chip Enable | GPIO 4 | TX/RX mode control |
| CSN | Chip Select | GPIO 5 | SPI slave select (active low) |
| SCK | SPI Clock | GPIO 18 | SPI clock |
| MOSI | SPI Data Out | GPIO 23 | Master Out Slave In |
| MISO | SPI Data In | GPIO 19 | Master In Slave Out |
| IRQ | Interrupt | Not connected | Optional interrupt output |

## ESC Connections (All ESCs)

| ESC Position | ESP32 Pin | PWM Channel | Motor Position | Wire Color (typical) |
|--------------|-----------|-------------|----------------|---------------------|
| Top Left | GPIO 14 | Channel 0 | Front Left | Signal: White/Yellow |
| Top Right | GPIO 27 | Channel 1 | Front Right | Signal: White/Yellow |
| Bottom Left | GPIO 26 | Channel 2 | Rear Left | Signal: White/Yellow |
| Bottom Right | GPIO 25 | Channel 3 | Rear Right | Signal: White/Yellow |

### ESC Signal Specifications
- **PWM Frequency:** 50Hz
- **PWM Resolution:** 16-bit
- **Signal Range:** 1000µs - 2000µs (mapped to 3276-6553 duty cycle)
- **Arm Signal:** 1000µs (duty cycle 3276)

## I²C Device Address Summary

| Device | Default Address | Alternative Address | Address Pin |
|--------|----------------|---------------------|-------------|
| ICM-20948 | 0x68 | 0x69 | AD0 |
| BMP280 | 0x76 | 0x77 | SDO |
| ADS1115 | 0x48 | 0x49, 0x4A, 0x4B | ADDR |



## Important Notes

1. **I²C Bus:** Shared between IMU (ICM-20948), BMP280, and ADS1115
   - Use different addresses to avoid conflicts
   - Maximum bus length: ~10-15cm for reliability
   - Pull-up resistors (4.7kΩ) typically on-board modules
   
2. **SPI Bus:** Dedicated to nRF24L01 radio module (VSPI)
   - SCK: GPIO 18, MISO: GPIO 19, MOSI: GPIO 23
   
3. **UART:** Serial2 reserved for GPS communication
   - Baud rate: 9600 (default for NEO-8M)
   
4. **nRF24L01 Power:** Must have 100µF capacitor close to VCC/GND pins
   
5. **Radio Configuration:**
   - Channel: 125 (avoids WiFi interference - changed from 108)
   - Data rate: 1Mbps
   - PA Level: Low
   - Update rate: 50Hz
   
6. **IMU Orientation:** Mounted FLAT with chip and LED facing UP
   - X-axis = Roll
   - Y-axis = Pitch
   - Z-axis = Yaw
   
7. **Motor Configuration:** X-configuration quadcopter layout
   
8. **Battery Monitoring:** Voltage dividers scale 12.6V down to 3.13V for safe ADC input

## Wiring Recommendations

- **Signal Integrity:** 
  - I²C wires: Keep under 15cm, use twisted pair if possible
  - SPI wires: Keep under 20cm, avoid parallel runs with power wires
  
- **Power Filtering:** 
  - 100µF capacitor on nRF24L01 VCC (critical!)
  - 0.1µF ceramic capacitor on ESP32 VDD
  - 10µF on each sensor VCC (recommended)
  
- **ESC Power:** 
  - Connect ESCs directly to battery, not through ESP32
  - Use appropriate gauge wire for motor current
  
- **Ground Loop:** 
  - Star grounding topology recommended
  - Single common ground point at battery

