# Matek F405-HDTE Technical Specification

## Hardware Configuration

### Microcontroller
- STM32F405RGT6 (ARM Cortex-M4F)
- Clock: 168MHz
- Flash: 1MB (894.6KB firmware usage, 88.07% utilization)
- RAM: 192KB (128KB SRAM + 64KB CCM)

### Sensors
- IMU: ICM42688-P on SPI1 (CS: PA4)
- Barometer: SPL06-001 on I2C1 (address 0x76/0x77)
- OSD: AT7456E on SPI3 (CS: PA15)

### Pin Assignments

| Function | Pin | Bus/Timer | Notes |
|----------|-----|-----------|-------|
| Motor 1 | PC6 | TIM3_CH1 | PWM output |
| Motor 2 | PC7 | TIM3_CH2 | PWM output |
| Motor 3 | PC8 | TIM3_CH3 | PWM output |
| Motor 4 | PC9 | TIM3_CH4 | PWM output, DSHOT DMA conflict |
| RC Input | PA10 | UART1_RX | SBUS, PPM, Spektrum |
| RC Output | PA9 | UART1_TX | Serial passthrough |
| GPS RX | PA3 | UART2_RX | GPS module input |
| GPS TX | PA2 | UART2_TX | GPS module output |
| IMU CS | PA4 | SPI1 | ICM42688P chip select |
| OSD CS | PA15 | SPI3 | AT7456E chip select |
| I2C SCL | PB6 | I2C1_SCL | External sensors |
| I2C SDA | PB7 | I2C1_SDA | External sensors |
| Battery V | PC5 | ADC12_IN15 | Voltage sensing |
| Battery I | PC4 | ADC12_IN14 | Current sensing |
| RSSI | PB1 | ADC12_IN9 | Signal strength |
| Status LED | PB5 | GPIO | Blue LED |
| Tone Alarm | PA0 | TIM5_CH1 | Buzzer output |

### Bus Configuration

#### SPI Buses
- SPI1: ICM42688P IMU (PA4 CS, PA5 CLK, PA6 MISO, PA7 MOSI)
- SPI3: AT7456E OSD (PA15 CS, PC10 CLK, PC11 MISO, PC12 MOSI)

#### I2C Buses
- I2C1: External sensors (PB6 SCL, PB7 SDA)

#### UART Interfaces
- UART1: RC receiver (PA9 TX, PA10 RX)
- UART2: GPS module (PA2 TX, PA3 RX)
- UART4: Additional telemetry (PA0 TX, PA1 RX)
- UART6: High-speed interface (PC6 TX, PC7 RX)

### Power Management
- Battery voltage scale: 21.0 (1K:20K voltage divider, 60V max)
- Current sensor scale: 17.0 A/V
- Input voltage range: 9-60V (3-12S LiPo)

## Software Configuration

### Build Target
```bash
make matek_f405-hdte_default
```

### Memory Usage
- Flash: 881452 bytes / 992KB (86.77%)
- SRAM: 24412 bytes / 128KB (18.62%)
- CCSRAM: 0 bytes / 64KB (0.00%)

### Driver Configuration
- ICM42688P IMU driver enabled
- SPL06 barometer driver enabled
- AT7456E OSD driver enabled
- DSHOT, PWM output drivers enabled
- Battery monitoring enabled
- RC input processing enabled

### Key Parameters
```
BAT1_V_DIV 21.0         # Battery voltage divider
BAT1_A_PER_V 17.0       # Current sensor scale
PWM_MAIN_DIS1-4 1-4     # Motor channel assignments
PWM_MAIN_FREQ 1000      # PWM frequency (Hz)
SYS_HAS_MAG 0           # No onboard magnetometer
CBRK_SUPPLY_CHK 894281  # Disable power supply check
```

## Known Issues

### DSHOT DMA Conflict
- Timer 3 Channel 4 (motor 4, PC9) shares DMA1 Stream 2 Channel 5 with Timer 3 Update
- Affects DSHOT protocol reliability on motor 4
- Workaround: Use PWM, OneShot125, or MultiShot protocols
- Root cause: STM32F405 hardware limitation

### Memory Constraints
- 1MB flash requires constrained build configuration
- Some advanced features disabled for memory optimization
- CONFIG_BOARD_CONSTRAINED_FLASH=y
- CONFIG_BOARD_CONSTRAINED_MEMORY=y

## File Structure
```
boards/matek/f405-hdte/
├── default.px4board                    # PX4 board configuration
├── src/
│   ├── board_config.h                  # Hardware definitions
│   ├── timer_config.cpp                # PWM timer configuration
│   ├── spi.cpp                         # SPI bus setup
│   ├── i2c.cpp                         # I2C bus setup
│   ├── init.c                          # Board initialization
│   ├── led.c                           # LED control
│   ├── usb.c                           # USB configuration
│   └── CMakeLists.txt                  # Build configuration
├── nuttx-config/
│   ├── include/
│   │   ├── board.h                     # NuttX board definitions
│   │   └── board_dma_map.h            # DMA channel assignments
│   └── nsh/defconfig                   # NuttX configuration
└── init/
    ├── rc.board_defaults               # Default parameters
    ├── rc.board_sensors                # Sensor initialization
    └── rc.board_extras                 # Additional configuration
```

## Validation Status
- Build: Successful compilation
- Pin mapping: No conflicts detected
- Driver integration: All sensors configured
- Memory usage: Within STM32F405 constraints
- Hardware compatibility: Verified against Matek F405-HDTE specifications