/****************************************************************************
 *
 *   Copyright (c) 2018, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * Matek F405-HDTE internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/

/* Matek F405-HDTE GPIOs ****************************************************************************/
/* LEDs */
// Status LED - PB5 - blue
#define GPIO_LED1       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN5)
#define GPIO_LED_BLUE   GPIO_LED1

#define BOARD_OVERLOAD_LED     LED_BLUE

#define  FLASH_BASED_PARAMS

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver
 * ADC channels for Matek F405-HDTE:
 * PC5 - ADC12_IN15 - Battery voltage with 21.0 scale factor (60V max)
 * PC4 - ADC12_IN14 - Battery current
 * PB1 - ADC12_IN9  - RSSI
 * PC1 - ADC123_IN11 - External ADC
 */
#define ADC_CHANNELS (1 << 9) | (1 << 11) | (1 << 14) | (1 << 15)

#define ADC_BATTERY_VOLTAGE_CHANNEL  15  // PC5 - ADC12_IN15
#define ADC_BATTERY_CURRENT_CHANNEL  14  // PC4 - ADC12_IN14
#define ADC_RC_RSSI_CHANNEL          9   // PB1 - ADC12_IN9
#define ADC_EXTERNAL_1_CHANNEL       11  // PC1 - ADC123_IN11

/* Battery voltage scale factor - 21.0 for 60V max */
#define BOARD_BATTERY_V_DIV         21.0f
#define BOARD_BATTERY_A_PER_V       17.0f

/* User GPIOs
 *
 * GPIO0-7 are the PWM motor/servo outputs for Matek F405-HDTE:
 * Motor 1: PC6 (TIM3_CH1)
 * Motor 2: PC7 (TIM3_CH2)
 * Motor 3: PC8 (TIM3_CH3)
 * Motor 4: PC9 (TIM3_CH4)
 * Motor 5: PA15 (TIM2_CH1)
 * Motor 6: PA8 (TIM1_CH1)
 * Motor 7: PB8 (TIM4_CH3)
 * Motor 8: PB9 (TIM4_CH4)
 */

#define _MK_GPIO_INPUT(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLUP))

#define GPIO_GPIO0_INPUT             _MK_GPIO_INPUT(GPIO_TIM3_CH3IN)
#define GPIO_GPIO1_INPUT             _MK_GPIO_INPUT(GPIO_TIM3_CH4IN)
#define GPIO_GPIO2_INPUT             _MK_GPIO_INPUT(GPIO_TIM2_CH4IN)
#define GPIO_GPIO3_INPUT             _MK_GPIO_INPUT(GPIO_TIM2_CH3IN)
//#define GPIO_GPIO4_INPUT             _MK_GPIO_INPUT(GPIO_TIM5_CH2IN)
//#define GPIO_GPIO5_INPUT             _MK_GPIO_INPUT(GPIO_TIM1_CH1IN)

#define _MK_GPIO_OUTPUT(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR))

#define GPIO_GPIO0_OUTPUT            _MK_GPIO_OUTPUT(GPIO_TIM3_CH3OUT)
#define GPIO_GPIO1_OUTPUT            _MK_GPIO_OUTPUT(GPIO_TIM3_CH4OUT)
#define GPIO_GPIO2_OUTPUT            _MK_GPIO_OUTPUT(GPIO_TIM2_CH4OUT)
#define GPIO_GPIO3_OUTPUT            _MK_GPIO_OUTPUT(GPIO_TIM2_CH3OUT)
//#define GPIO_GPIO4_OUTPUT            _MK_GPIO_OUTPUT(GPIO_TIM5_CH2OUT)
//#define GPIO_GPIO5_OUTPUT            _MK_GPIO_OUTPUT(GPIO_TIM1_CH1OUT)

/* USB OTG FS
 *
 * PC5 used for battery voltage sensing, USB VBUS detection method needs verification
 */
#define GPIO_OTGFS_VBUS		(GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)

/* PWM
 *
 * 4 PWM outputs for Matek F405-HDTE (Motors 1-4 on TIM3)
 * Additional outputs available but not configured for PX4
 */
#define DIRECT_PWM_OUTPUT_CHANNELS      4
#define BOARD_NUM_IO_TIMERS            1

/* High-resolution timer */
#define HRT_TIMER                    4 // T4C1
#define HRT_TIMER_CHANNEL            1 // use capture/compare channel 1

#define HRT_PPM_CHANNEL              3 // capture/compare channel 3
#define GPIO_PPM_IN                  (GPIO_ALT|GPIO_AF2|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN8)

#define RC_SERIAL_PORT               "/dev/ttyS0"
#define BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT

/*
 * One RC_IN
 *
 * GPIO PPM_IN on PB8 T4CH3
 * SPEKTRUM_RX (it's TX or RX in Bind) on PA10 UART1
 * The FMU can drive GPIO PPM_IN as an output
 */
// TODO?
//#define GPIO_PPM_IN_AS_OUT            (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN6)
//#define SPEKTRUM_RX_AS_GPIO_OUTPUT()  px4_arch_configgpio(GPIO_PPM_IN_AS_OUT)
//#define SPEKTRUM_RX_AS_UART()         px4_arch_configgpio(GPIO_USART1_RX)
//#define SPEKTRUM_OUT(_one_true)       px4_arch_gpiowrite(GPIO_PPM_IN_AS_OUT, (_one_true))


/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

#define BOARD_HAS_ON_RESET 1

#define BOARD_ENABLE_CONSOLE_BUFFER
#define BOARD_CONSOLE_BUFFER_SIZE (1024*3)

#define TONE_ALARM_TIMER        5
#define TONE_ALARM_CHANNEL      1
#define GPIO_TONE_ALARM_IDLE    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN0)
#define GPIO_TONE_ALARM         (GPIO_ALT|GPIO_AF2|GPIO_SPEED_2MHz|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN0)

/* SPI bus assignments for Matek F405-HDTE */

/* SPI1 - ICM42688-P IMU */
#define PX4_SPI_BUS_SENSORS     1
#define PX4_SPIDEV_ICM_42688P   PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS, 1)
#define GPIO_SPI1_ICM42688P_CS  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)

/* Note: SPI2 is available but not configured (no physical SD card slot on Matek F405-HDTE) */
/* The board uses 16M built-in flash for logging instead */

/* SPI3 - AT7456E OSD */
#define PX4_SPI_BUS_OSD         3
#define PX4_SPIDEV_OSD          PX4_MK_SPI_SEL(PX4_SPI_BUS_OSD, 1)
#define GPIO_SPI3_OSD_CS        (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN15)

/* I2C bus assignments for Matek F405-HDTE */

/* I2C1 - External sensors (SPL06-001 barometer) */
#define PX4_I2C_BUS_EXPANSION   1

/* Define supported sensors */


__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);


/****************************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to configure USB IO.
 *
 ****************************************************************************************************/

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
