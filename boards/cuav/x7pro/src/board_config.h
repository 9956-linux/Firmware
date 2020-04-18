/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * CUAV X7Pro internal definitions
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/* GPIOs ************************************************************************/

/* LEDs */
#define GPIO_nLED_RED        /* PI5 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN5)
#define GPIO_nLED_GREEN      /* PI6 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN6)
#define GPIO_nLED_BLUE       /* PI7 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN7)

#define BOARD_HAS_CONTROL_STATUS_LEDS      1
#define BOARD_OVERLOAD_LED     LED_RED
#define BOARD_ARMED_STATE_LED  LED_BLUE

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */
#define PX4_ADC_GPIO  \
	/* PA0 */  GPIO_ADC1_INP16,   \
	/* PA1 */  GPIO_ADC1_INP17,   \
	/* PA2 */  GPIO_ADC12_INP14,  \
	/* PA3 */  GPIO_ADC12_INP15,  \
	/* PA4 */  GPIO_ADC12_INP18,  \
	/* PB0 */  GPIO_ADC12_INP9,   \
	/* PC0 */  GPIO_ADC123_INP10, \
	/* PC1 */  GPIO_ADC123_INP11, \
	/* PC2 */  GPIO_ADC123_INP12, \
	/* PC3 */  GPIO_ADC12_INP13,  \
	/* PC4 */  GPIO_ADC12_INP4

/* Define Channel numbers must match above GPIO pin IN(n)*/
#define ADC_BATTERY1_VOLTAGE_CHANNEL        16 /* PA0 */
#define ADC_BATTERY1_CURRENT_CHANNEL        17 /* PA1 */
#define ADC_BATTERY2_VOLTAGE_CHANNEL        14 /* PA2 */
#define ADC_BATTERY2_CURRENT_CHANNEL        15 /* PF11 */
#define ADC1_6V6_IN_CHANNEL                  4 /* PC4 */
#define ADC_RSSI_IN_CHANNEL                  9 /* PF12 */
#define ADC_SCALED_V5_CHANNEL               10 /* PC5 */
#define ADC_SCALED_VDD_3V3_SENSORS_CHANNEL  11 /* PC1 */
#define ADC_HW_VER_SENSE_CHANNEL            12 /* PC2 */
#define ADC_HW_REV_SENSE_CHANNEL            13 /* PC3 */
#define ADC1_3V3_IN_CHANNEL                 18 /* PA4 */

#define ADC_CHANNELS \
	((1 << ADC_BATTERY1_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY1_CURRENT_CHANNEL)       | \
	 (1 << ADC_BATTERY2_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY2_CURRENT_CHANNEL)       | \
	 (1 << ADC1_6V6_IN_CHANNEL)                | \
	 (1 << ADC_RSSI_IN_CHANNEL)                | \
	 (1 << ADC_SCALED_V5_CHANNEL)              | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS_CHANNEL) | \
	 (1 << ADC_HW_VER_SENSE_CHANNEL)           | \
	 (1 << ADC_HW_REV_SENSE_CHANNEL)           | \
	 (1 << ADC1_3V3_IN_CHANNEL))

/* HW has to large of R termination on ADC todo:change when HW value is chosen */
#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

#define GPIO_HW_REV_DRIVE    /* PH14  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTH|GPIO_PIN14)
#define GPIO_HW_REV_SENSE    /* PC3   */ GPIO_ADC12_INP13
#define GPIO_HW_VER_DRIVE    /* PH14  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN3)
#define GPIO_HW_VER_SENSE    /* PC2   */ GPIO_ADC123_INP12

/* CAN Silence Silent mode control */
#define GPIO_CAN1_SILENT_S0  /* PH2  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN2)
#define GPIO_CAN2_SILENT_S1  /* PH3  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN3)

/* HEATER */
#define GPIO_HEATER_OUTPUT   /* PA8 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN8)

/* PWM */
#define DIRECT_PWM_OUTPUT_CHANNELS   8

/* Power supply control and monitoring GPIOs */
#define GPIO_nPOWER_IN_A                /* PG1  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN1)
#define GPIO_nPOWER_IN_B                /* PG2  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN2)
#define GPIO_nPOWER_IN_C                /* PG0  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN0)

#define GPIO_nVDD_BRICK1_VALID          GPIO_nPOWER_IN_A /* Brick 1 Is Chosen */
#define GPIO_nVDD_BRICK2_VALID          GPIO_nPOWER_IN_B /* Brick 2 Is Chosen  */
#define BOARD_NUMBER_BRICKS             2
#define GPIO_nVDD_USB_VALID             GPIO_nPOWER_IN_C /* USB     Is Chosen */

#define GPIO_nVDD_5V_PERIPH_EN          /* PG4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN4)
#define GPIO_nVDD_5V_HIPOWER_EN         /* PD11 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN11)
#define GPIO_VDD_3V3_SPEKTRUM_POWER_EN  /* PE4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN4)
#define GPIO_VDD_3V3_SD_CARD_EN         /* PG7  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN7)

#define GPIO_nVDD_5V_HIPOWER_OC         /* PJ3 */  (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTJ|GPIO_PIN3)
#define GPIO_nVDD_5V_PERIPH_OC          /* PJ3 */  (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTJ|GPIO_PIN4)

/* Define True logic Power Control in arch agnostic form */
#define VDD_5V_PERIPH_EN(on_true)          px4_arch_gpiowrite(GPIO_nVDD_5V_PERIPH_EN, !(on_true))
#define VDD_5V_HIPOWER_EN(on_true)         px4_arch_gpiowrite(GPIO_nVDD_5V_HIPOWER_EN, !(on_true))
#define VDD_3V3_SPEKTRUM_POWER_EN(on_true) px4_arch_gpiowrite(GPIO_VDD_3V3_SPEKTRUM_POWER_EN, (on_true))
#define READ_VDD_3V3_SPEKTRUM_POWER_EN()   px4_arch_gpioread(GPIO_VDD_3V3_SPEKTRUM_POWER_EN)
#define VDD_3V3_SD_CARD_EN(on_true)        px4_arch_gpiowrite(GPIO_VDD_3V3_SD_CARD_EN, (on_true))

/* Tone alarm output */
#define TONE_ALARM_TIMER        15  /* timer 15 */
#define TONE_ALARM_CHANNEL      1  /* PE5 TIM15_CH1 */

#define GPIO_BUZZER_1           /* PE5 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN5)

#define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1
#define GPIO_TONE_ALARM         GPIO_TIM15_CH1OUT_2

/* USB OTG FS: PA9  OTG_FS_VBUS VBUS sensing */
#define GPIO_OTGFS_VBUS         /* PA9 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER               8  /* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL       3  /* use capture/compare channel 3 */

/* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 2 */
#define PWMIN_TIMER                       4
#define PWMIN_TIMER_CHANNEL    /* T4C2 */ 2
#define GPIO_PWM_IN            /* PD13 */ GPIO_TIM4_CH2IN

/* RSSI_IN is grounded via a 10K */
#define GPIO_RSSI_IN                  /* PB0 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTB|GPIO_PIN0)

/* Safety Switch is only on PX4IO */
#define GPIO_nSAFETY_SWITCH_LED_OUT   /* PE12 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTE|GPIO_PIN12)
#define GPIO_SAFETY_SWITCH_IN         /* PE10 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTE|GPIO_PIN10)

/* Power switch controls ******************************************************/
#define SPEKTRUM_POWER(_on_true)           VDD_3V3_SPEKTRUM_POWER_EN(_on_true)

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))
#define BOARD_ADC_USB_VALID     (!px4_arch_gpioread(GPIO_nVDD_USB_VALID))
#define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#define BOARD_ADC_BRICK2_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK2_VALID))
#define BOARD_ADC_PERIPH_5V_OC  (!px4_arch_gpioread(GPIO_nVDD_5V_PERIPH_OC))
#define BOARD_ADC_HIPOWER_5V_OC (!px4_arch_gpioread(GPIO_nVDD_5V_HIPOWER_OC))


/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1

#define BOARD_HAS_PWM  DIRECT_PWM_OUTPUT_CHANNELS

#define BOARD_DSHOT_MOTOR_ASSIGNMENT {3, 2, 1, 0, 4};

#define BOARD_ENABLE_CONSOLE_BUFFER

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_HW_REV_DRIVE,                \
		GPIO_HW_VER_DRIVE,                \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_CAN2_TX,                     \
		GPIO_CAN2_RX,                     \
		GPIO_CAN1_SILENT_S0,              \
		GPIO_CAN2_SILENT_S1,              \
		GPIO_HEATER_OUTPUT,               \
		GPIO_nPOWER_IN_A,                 \
		GPIO_nPOWER_IN_B,                 \
		GPIO_nPOWER_IN_C,                 \
		GPIO_nVDD_5V_PERIPH_EN,           \
		GPIO_nVDD_5V_PERIPH_OC,           \
		GPIO_nVDD_5V_HIPOWER_EN,          \
		GPIO_nVDD_5V_HIPOWER_OC,          \
		GPIO_VDD_3V3_SPEKTRUM_POWER_EN,   \
		PX4_GPIO_PIN_OFF(GPIO_SDMMC1_D0), \
		PX4_GPIO_PIN_OFF(GPIO_SDMMC1_D1), \
		PX4_GPIO_PIN_OFF(GPIO_SDMMC1_D2), \
		PX4_GPIO_PIN_OFF(GPIO_SDMMC1_D3), \
		PX4_GPIO_PIN_OFF(GPIO_SDMMC1_CMD),\
		GPIO_VDD_3V3_SD_CARD_EN,          \
		GPIO_TONE_ALARM_IDLE,             \
		GPIO_OTGFS_VBUS,                  \
		GPIO_RSSI_IN,                     \
		GPIO_nSAFETY_SWITCH_LED_OUT,      \
		GPIO_SAFETY_SWITCH_IN,            \
	}

__BEGIN_DECLS
#ifndef __ASSEMBLY__

extern void stm32_spiinitialize(void);
extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>
#endif /* __ASSEMBLY__ */
__END_DECLS
