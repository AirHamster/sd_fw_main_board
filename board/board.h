/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 * This file has been automatically generated using ChibiStudio board
 * generator plugin. Do not edit manually.
 */

#ifndef BOARD_H
#define BOARD_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*
 * Setup for STMicroelectronics STM32 Nucleo144-F767ZI board.
 */

/*
 * Board identifier.
 */
#define BOARD_ST_NUCLEO144_F767ZI
#define BOARD_NAME                  "SailData main CPU board"


/*
 * Board oscillators-related settings.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                32768U
#endif

#define STM32_LSEDRV                (3U << 3U)

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                8000000U
#endif

#define STM32_HSE_BYPASS

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   300U

/*
 * MCU type as defined in the ST header.
 */
#define STM32F767xx

/*
 * IO pins assignments.
 */
#define GPIO_USART1_TX				9U
#define GPIO_USART1_RX				10U

#define GPIO_USART2_TX				2U
#define GPIO_USART2_RX				3U

#define GPIO_USART3_TX				8U
#define GPIO_USART3_RX				9U

#define GPIO_USART4_TX				0U
#define GPIO_USART4_RX				1U

#define GPIO_USART5_TX				12U
#define GPIO_USART5_RX				2U

#define GPIO_USART7_TX				15U
#define GPIO_USART7_RX				3U

#define GPIO_SPI1_MOSI				7U
#define GPIO_SPI1_MISO				6U
#define GPIO_SPI1_CLK				5U

#define GPIO_SPI2_MOSI				1U
#define GPIO_SPI2_MISO				2U
#define GPIO_SPI2_CLK				10U

#define GPIO_SPI3_MOSI				6U
#define GPIO_SPI3_MISO				11U
#define GPIO_SPI3_CLK				10U

#define GPIO_I2C1_SCL				6U
#define GPIO_I2C1_SDA				7U

#define GPIO_I2C3_SCL				8U
#define GPIO_I2C3_SDA				9U

#define GPIO_I2C4_SCL				12U
#define GPIO_I2C4_SDA				13U

#define GPIO_CAN_RX					0U
#define GPIO_CAN_TX					1U

#define GPIO_868_CS					4U
#define GPIO_USB_DM					11U
#define GPIO_USB_DP					12U

#define GPIO_CAN1_RX				0U
#define GPIO_CAN1_TX				1U

#define GPIO_LED_ORANGE				0U
#define GPIO_LED_GREEN				1U
#define GPIO_LED_RED				2U
#define GPIO_BUZZER					4U
#define GPIO_RTK_STAT				5U
#define GPIO_2_4_CS					8U
#define GPIO_M8T_RST				9U
#define GPIO_MPU_CS					0U
#define GPIO_MCU_CS					4U
#define GPIO_868_SPI_ATTN			5U
#define GPIO_SD_CS					8U
#define GPIO_SAT_NTW_AV				0U
#define GPIO_SAT_RING_SGN			1U
#define GPIO_RAM_CS1				2U
#define GPIO_RAM_CS2				3U
#define GPIO_M8T_CS					5U
#define GPIO_KEY1					13U
#define GPIO_KEY2					14U
#define GPIO_KEY3					15U

#define GPIO_ADC3_IN9				3U
#define GPIO_ADC3_IN14				4U
#define GPIO_ADC3_IN15				5U
#define GPIO_868_SLEEP				8U
#define GPIO_868_RST				9U

#define GPIO_SD_CD					8U
#define GPIO_SAT_ON_OFF				10U
#define GPIO_LTE_ON_OFF				11U
#define GPIO_WIFI_ON_OFF			12U
#define GPIO_2_4_ON_OFF				13U

#define GPIO_SWDIO					13U
#define GPIO_SWCLK					14U

/*
 * Non-used outputs
 */
#define GPIOB_11					11U
#define GPIOB_12					12U
#define GPIOB_13					13U
#define GPIOB_14					14U
#define GPIOB_15					15U

#define GPIOC_3						3U
#define GPIOC_6						6U
#define GPIOC_7						7U
#define GPIOC_13					13U

#define GPIOC_OSC32_IN              14U
#define GPIOC_OSC32_OUT             15U


#define GPIOD_3						3U
#define GPIOD_4						4U
#define GPIOD_5						5U
#define GPIOD_7						7U
#define GPIOD_10					10U
#define GPIOD_11					11U
#define GPIOD_14					14U
#define GPIOD_15					15U

#define GPIOE_3						3U
#define GPIOE_4						4U
#define GPIOE_5						5U
#define GPIOE_7						7U
#define GPIOE_8						8U
#define GPIOE_9						9U
#define GPIOE_10					10U
#define GPIOE_11					11U
#define GPIOE_12					12U

#define GPIOF_0						0U
#define GPIOF_1						1U
#define GPIOF_2						2U
#define GPIOF_6						6U
#define GPIOF_7						7U
#define GPIOF_10					10U
#define GPIOF_11					11U
#define GPIOF_12					12U
#define GPIOF_13					13U
#define GPIOF_14					14U
#define GPIOF_15					15U

#define GPIOG_0						0U
#define GPIOG_1						1U
#define GPIOG_2						2U
#define GPIOG_3						3U
#define GPIOG_4						4U
#define GPIOG_5						5U
#define GPIOG_6						6U
#define GPIOG_7						7U
#define GPIOG_9						9U
#define GPIOG_14					14U
#define GPIOG_15					15U

/*
 * IO lines assignments.
 */
#define LINE_ZIO_D32                PAL_LINE(GPIOA, 0U)
#define LINE_TIM2_CH1               PAL_LINE(GPIOA, 0U)

#define LINE_USART1_TX				PAL_LINE(GPIOA, 9U)
#define LINE_USART1_RX				PAL_LINE(GPIOA, 10U)

#define LINE_USART2_TX				PAL_LINE(GPIOA, 2U)
#define LINE_USART2_RX				PAL_LINE(GPIOA, 3U)

#define LINE_USART3_TX				PAL_LINE(GPIOD, 8U)
#define LINE_USART3_RX				PAL_LINE(GPIOD, 9U)

#define LINE_USART4_TX				PAL_LINE(GPIOA, 0U)
#define LINE_USART4_RX				PAL_LINE(GPIOA, 1U)

#define LINE_USART5_TX				PAL_LINE(GPIOC, 12U)
#define LINE_USART5_RX				PAL_LINE(GPIOD, 2U)

#define LINE_USART7_TX				PAL_LINE(GPIOA, 15U)
#define LINE_USART7_RX				PAL_LINE(GPIOB, 3U)

#define LINE_SPI1_MOSI				PAL_LINE(GPIOA, 7U)
#define LINE_SPI1_MISO				PAL_LINE(GPIOA, 6U)
#define LINE_SPI1_CLK				PAL_LINE(GPIOA, 5U)

#define LINE_SPI2_MOSI				PAL_LINE(GPIOC, 1U)
#define LINE_SPI2_MISO				PAL_LINE(GPIOC, 2U)
#define LINE_SPI2_CLK				PAL_LINE(GPIOB, 10U)

#define LINE_SPI3_MOSI				PAL_LINE(GPIOD, 6U)
#define LINE_SPI3_MISO				PAL_LINE(GPIOC, 11U)
#define LINE_SPI3_CLK				PAL_LINE(GPIOC, 10U)

#define LINE_I2C1_SCL				PAL_LINE(GPIOB, 6U)
#define LINE_I2C1_SDA				PAL_LINE(GPIOB, 7U)

#define LINE_I2C3_SCL				PAL_LINE(GPIOA, 8U)
#define LINE_I2C3_SDA				PAL_LINE(GPIOC, 9U)

#define LINE_I2C4_SCL				PAL_LINE(GPIOD, 12U)
#define LINE_I2C4_SDA				PAL_LINE(GPIOD, 13U)

#define LINE_CAN_RX					PAL_LINE(GPIOD, 0U)
#define LINE_CAN_TX					PAL_LINE(GPIOD, 1U)

#define LINE_868_CS					PAL_LINE(GPIOA, 4U)
#define LINE_USB_DM					PAL_LINE(GPIOA, 11U)
#define LINE_USB_DP					PAL_LINE(GPIOA, 12U)

#define LINE_LED_ORANGE				PAL_LINE(GPIOB, 0U)
#define LINE_LED_GREEN				PAL_LINE(GPIOB, 1U)
#define LINE_LED_RED				PAL_LINE(GPIOB, 2U)
#define LINE_BUZZER					PAL_LINE(GPIOB, 4U)
#define LINE_RTK_STAT				PAL_LINE(GPIOB, 5U)
#define LINE_2_4_CS					PAL_LINE(GPIOB, 8U)
#define LINE_M8T_RST				PAL_LINE(GPIOB, 9U)
#define LINE_MPU_CS					PAL_LINE(GPIOC, 0U)
#define LINE_MCU_CS					PAL_LINE(GPIOC, 4U)
#define LINE_868_SPI_ATTN			PAL_LINE(GPIOC, 5U)
#define LINE_SD_CS					PAL_LINE(GPIOC, 8U)
#define LINE_SAT_NTW_AV				PAL_LINE(GPIOE, 0U)
#define LINE_SAT_RING_SGN			PAL_LINE(GPIOE, 1U)
#define LINE_RAM_CS1				PAL_LINE(GPIOE, 2U)
#define LINE_RAM_CS2				PAL_LINE(GPIOE, 3U)
#define LINE_M8T_CS					PAL_LINE(GPIOE, 5U)
#define LINE_KEY1					PAL_LINE(GPIOE, 13U)
#define LINE_KEY2					PAL_LINE(GPIOE, 14U)
#define LINE_KEY3					PAL_LINE(GPIOE, 15U)

#define LINE_ADC3_IN9				PAL_LINE(GPIOF, 3U)
#define LINE_ADC3_IN14				PAL_LINE(GPIOF, 4U)
#define LINE_ADC3_IN15				PAL_LINE(GPIOF, 5U)
#define LINE_868_SLEEP				PAL_LINE(GPIOF, 8U)
#define LINE_868_RST				PAL_LINE(GPIOF, 9U)

#define LINE_SD_CD					PAL_LINE(GPIOG, 8U)
#define LINE_SAT_ON_OFF				PAL_LINE(GPIOG, 10U)
#define LINE_LTE_ON_OFF				PAL_LINE(GPIOG, 11U)
#define LINE_WIFI_ON_OFF			PAL_LINE(GPIOG, 12U)
#define LINE_2_4_ON_OFF				PAL_LINE(GPIOG, 13U)

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

/*
 * GPIOA setup:
 *
 * PA0  - USART4_RX		         	(alternate 8).
 * PA1  - USART4_TX    	          	(alternate 8).
 * PA2  - USART2_TX                 (alternate 7).
 * PA3  - USART2_RX         		(alternate 7).
 * PA4  - 868_CS			        (output pushpull maximum).
 * PA5  - SPI1_SCK          		(alternate 5).
 * PA6  - SPI1_MISO         		(alternate 5).
 * PA7  - SPI1_MOSI					(alternate 5).
 * PA8  - I2C3_SCL                  (alternate 4).
 * PA9  - USART1_TX                 (alternate 7).
 * PA10 - USART1_RX                 (alternate 7).
 * PA11 - USB_DM                    (alternate 10).
 * PA12 - USB_DP                    (alternate 10).
 * PA13 - SWDIO                     (alternate 0).
 * PA14 - SWCLK                     (alternate 0).
 * PA15 - USART7_TX		            (alternate 12).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_ALTERNATE(GPIO_USART4_RX) |        \
                                     PIN_MODE_ALTERNATE(GPIO_USART4_TX) |\
                                     PIN_MODE_ALTERNATE(GPIO_USART2_TX) |  \
                                     PIN_MODE_ALTERNATE(GPIO_USART2_RX) |         \
                                     PIN_MODE_OUTPUT(GPIO_868_CS) |        \
                                     PIN_MODE_ALTERNATE(GPIO_SPI1_CLK) |        \
                                     PIN_MODE_ALTERNATE(GPIO_SPI1_MISO) |        \
                                     PIN_MODE_ALTERNATE(GPIO_SPI1_MOSI) |    \
                                     PIN_MODE_ALTERNATE(GPIO_I2C3_SCL) |    \
                                     PIN_MODE_ALTERNATE(GPIO_USART1_TX) |      \
                                     PIN_MODE_ALTERNATE(GPIO_USART1_RX) |     \
                                     PIN_MODE_ALTERNATE(GPIO_USB_DM) |     \
                                     PIN_MODE_ALTERNATE(GPIO_USB_DP) |     \
                                     PIN_MODE_ALTERNATE(GPIO_SWDIO) |      \
                                     PIN_MODE_ALTERNATE(GPIO_SWCLK) |      \
                                     PIN_MODE_ALTERNATE(GPIO_USART7_TX))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIO_USART4_RX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_USART4_TX) |\
                                     PIN_OTYPE_PUSHPULL(GPIO_USART2_TX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIO_USART2_RX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIO_868_CS) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_SPI1_CLK) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_SPI1_MISO) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_SPI1_MOSI) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_I2C3_SCL) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_USART1_TX) |   \
                                     PIN_OTYPE_PUSHPULL(GPIO_USART1_RX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIO_USB_DM) |     \
                                     PIN_OTYPE_PUSHPULL(GPIO_USB_DP) |     \
                                     PIN_OTYPE_PUSHPULL(GPIO_SWDIO) |      \
                                     PIN_OTYPE_PUSHPULL(GPIO_SWCLK) |      \
                                     PIN_OTYPE_PUSHPULL(GPIO_USART7_TX))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_HIGH(GPIO_USART4_RX) |       \
                                     PIN_OSPEED_HIGH(GPIO_USART4_TX) |  \
                                     PIN_OSPEED_HIGH(GPIO_USART2_TX) |     \
                                     PIN_OSPEED_HIGH(GPIO_USART2_RX) |        \
                                     PIN_OSPEED_HIGH(GPIO_868_CS) |       \
                                     PIN_OSPEED_HIGH(GPIO_SPI1_CLK) |       \
                                     PIN_OSPEED_HIGH(GPIO_SPI1_MISO) |       \
                                     PIN_OSPEED_HIGH(GPIO_SPI1_MOSI) |       \
                                     PIN_OSPEED_HIGH(GPIO_I2C3_SCL) |       \
                                     PIN_OSPEED_HIGH(GPIO_USART1_TX) |      \
                                     PIN_OSPEED_HIGH(GPIO_USART1_RX) |        \
                                     PIN_OSPEED_HIGH(GPIO_USB_DM) |        \
                                     PIN_OSPEED_HIGH(GPIO_USB_DP) |        \
                                     PIN_OSPEED_HIGH(GPIO_SWDIO) |         \
                                     PIN_OSPEED_HIGH(GPIO_SWCLK) |         \
                                     PIN_OSPEED_HIGH(GPIO_USART7_TX))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_PULLUP(GPIO_USART4_RX) |      \
                                     PIN_PUPDR_PULLUP(GPIO_USART4_TX) |\
                                     PIN_PUPDR_PULLUP(GPIO_USART2_TX) |    \
                                     PIN_PUPDR_PULLUP(GPIO_USART2_RX) |       \
                                     PIN_PUPDR_PULLUP(GPIO_868_CS) |      \
                                     PIN_PUPDR_PULLUP(GPIO_SPI1_CLK) |      \
                                     PIN_PUPDR_PULLUP(GPIO_SPI1_MISO) |      \
                                     PIN_PUPDR_PULLUP(GPIO_SPI1_MOSI) |      \
                                     PIN_PUPDR_FLOATING(GPIO_I2C3_SCL) |    \
                                     PIN_PUPDR_PULLUP(GPIO_USART1_TX) |   \
                                     PIN_PUPDR_FLOATING(GPIO_USART1_RX) |     \
                                     PIN_PUPDR_FLOATING(GPIO_USB_DM) |     \
                                     PIN_PUPDR_FLOATING(GPIO_USB_DP) |     \
                                     PIN_PUPDR_FLOATING(GPIO_SWDIO) |      \
                                     PIN_PUPDR_FLOATING(GPIO_SWCLK) |      \
                                     PIN_PUPDR_PULLUP(GPIO_USART7_TX))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIO_USART4_RX) |          \
                                     PIN_ODR_HIGH(GPIO_USART4_TX) |     \
                                     PIN_ODR_HIGH(GPIO_USART2_TX) |        \
                                     PIN_ODR_HIGH(GPIO_USART2_RX) |           \
                                     PIN_ODR_HIGH(GPIO_868_CS) |          \
                                     PIN_ODR_HIGH(GPIO_SPI1_CLK) |          \
                                     PIN_ODR_HIGH(GPIO_SPI1_MISO) |          \
                                     PIN_ODR_HIGH(GPIO_SPI1_MOSI) |          \
                                     PIN_ODR_HIGH(GPIO_I2C3_SCL) |          \
                                     PIN_ODR_HIGH(GPIO_USART1_TX) |         \
                                     PIN_ODR_HIGH(GPIO_USART1_RX) |           \
                                     PIN_ODR_HIGH(GPIO_USB_DM) |           \
                                     PIN_ODR_HIGH(GPIO_USB_DP) |           \
                                     PIN_ODR_HIGH(GPIO_SWDIO) |            \
                                     PIN_ODR_HIGH(GPIO_SWCLK) |            \
                                     PIN_ODR_HIGH(GPIO_USART7_TX))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIO_USART4_RX, 8U) |       \
                                     PIN_AFIO_AF(GPIO_USART4_TX, 8U) | \
                                     PIN_AFIO_AF(GPIO_USART2_TX, 7U) |    \
                                     PIN_AFIO_AF(GPIO_USART2_RX, 7U) |        \
                                     PIN_AFIO_AF(GPIO_868_CS, 0U) |       \
                                     PIN_AFIO_AF(GPIO_SPI1_CLK, 5U) |       \
                                     PIN_AFIO_AF(GPIO_SPI1_MISO, 5U) |       \
                                     PIN_AFIO_AF(GPIO_SPI1_MOSI, 5U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIO_I2C3_SCL, 4U) |      \
                                     PIN_AFIO_AF(GPIO_USART1_TX, 7U) |      \
                                     PIN_AFIO_AF(GPIO_USART1_RX, 7U) |       \
                                     PIN_AFIO_AF(GPIO_USB_DM, 10U) |       \
                                     PIN_AFIO_AF(GPIO_USB_DP, 10U) |       \
                                     PIN_AFIO_AF(GPIO_SWDIO, 0U) |         \
                                     PIN_AFIO_AF(GPIO_SWCLK, 0U) |         \
                                     PIN_AFIO_AF(GPIO_USART7_TX, 12U))


/*
 * GPIOB setup:
 *
 * PB0  - ORANGE_LED		        (output pushpull maximum).
 * PB1  - GREEN_LED		     	    (output pushpull maximum).
 * PB2  - RED_LED		            (output pushpull maximum).
 * PB3  - USART7_RX		            (alternate 12).
 * PB4  - BUZZER		            (alternate 2). - timer 3 ch1
 * PB5  - RTK_STAT		            (input pullup).
 * PB6  - I2C1_SCL			        (alternate 4).
 * PB7  - I2C1_SDA                  (alternate 4).
 * PB8  - 2_4_CS		            (output pushpull maximum).
 * PB9  - M8T_RST		            (output pushpull maximum).
 * PB10 - SPI2_CLK		            (alternate 5).
 * PB11 - GPIOB_11   		        (input pullup).
 * PB12 - GPIOB_12  	            (input pullup).
 * PB13 - GPIOB_13 					(input pullup).
 * PB14 - GPIOB_14                  (input pullup).
 * PB15 - GPIOB_15					(input pullup).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_OUTPUT(GPIO_LED_ORANGE) |       \
                                     PIN_MODE_OUTPUT(GPIO_LED_GREEN) |         \
                                     PIN_MODE_OUTPUT(GPIO_LED_RED) |        \
                                     PIN_MODE_ALTERNATE(GPIO_USART7_RX) |        \
                                     PIN_MODE_ALTERNATE(GPIO_BUZZER) |        \
                                     PIN_MODE_INPUT(GPIO_RTK_STAT) |        \
                                     PIN_MODE_ALTERNATE(GPIO_I2C1_SCL) |        \
                                     PIN_MODE_ALTERNATE(GPIO_I2C1_SDA) |          \
                                     PIN_MODE_OUTPUT(GPIO_2_4_CS) |        \
                                     PIN_MODE_OUTPUT(GPIO_M8T_RST) |        \
                                     PIN_MODE_ALTERNATE(GPIO_SPI2_CLK) |        \
                                     PIN_MODE_INPUT(GPIOB_11) |        \
                                     PIN_MODE_INPUT(GPIOB_12) |        \
                                     PIN_MODE_INPUT(GPIOB_13) |    \
                                     PIN_MODE_INPUT(GPIOB_14) |          \
                                     PIN_MODE_INPUT(GPIOB_15))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIO_LED_ORANGE) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_LED_GREEN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIO_LED_RED) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_USART7_RX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_BUZZER) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_RTK_STAT) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_I2C1_SCL) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_I2C1_SDA) |       \
                                     PIN_OTYPE_PUSHPULL(GPIO_2_4_CS) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_M8T_RST) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_SPI2_CLK) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_11) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_12) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_13) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_14) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_15))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_HIGH(GPIO_LED_ORANGE) |       \
                                     PIN_OSPEED_HIGH(GPIO_LED_GREEN) |        \
                                     PIN_OSPEED_HIGH(GPIO_LED_RED) |       \
                                     PIN_OSPEED_HIGH(GPIO_USART7_RX) |       \
                                     PIN_OSPEED_HIGH(GPIO_BUZZER) |       \
                                     PIN_OSPEED_HIGH(GPIO_RTK_STAT) |       \
                                     PIN_OSPEED_HIGH(GPIO_I2C1_SCL) |       \
                                     PIN_OSPEED_HIGH(GPIO_I2C1_SDA) |          \
                                     PIN_OSPEED_HIGH(GPIO_2_4_CS) |       \
                                     PIN_OSPEED_HIGH(GPIO_M8T_RST) |       \
                                     PIN_OSPEED_HIGH(GPIO_SPI2_CLK) |       \
                                     PIN_OSPEED_HIGH(GPIOB_11) |       \
                                     PIN_OSPEED_HIGH(GPIOB_12) |       \
                                     PIN_OSPEED_HIGH(GPIOB_13) |       \
                                     PIN_OSPEED_HIGH(GPIOB_14) |          \
                                     PIN_OSPEED_HIGH(GPIOB_15))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLUP(GPIO_LED_ORANGE) |    \
                                     PIN_PUPDR_PULLUP(GPIO_LED_GREEN) |       \
                                     PIN_PUPDR_PULLUP(GPIO_LED_RED) |      \
                                     PIN_PUPDR_FLOATING(GPIO_USART7_RX) |      \
                                     PIN_PUPDR_PULLUP(GPIO_BUZZER) |      \
                                     PIN_PUPDR_PULLUP(GPIO_RTK_STAT) |      \
                                     PIN_PUPDR_FLOATING(GPIO_I2C1_SCL) |      \
                                     PIN_PUPDR_FLOATING(GPIO_I2C1_SDA) |       \
                                     PIN_PUPDR_PULLUP(GPIO_2_4_CS) |      \
                                     PIN_PUPDR_PULLUP(GPIO_M8T_RST) |      \
                                     PIN_PUPDR_PULLUP(GPIO_SPI2_CLK) |      \
                                     PIN_PUPDR_PULLUP(GPIOB_11) |      \
                                     PIN_PUPDR_PULLUP(GPIOB_12) |      \
                                     PIN_PUPDR_PULLUP(GPIOB_13) |      \
                                     PIN_PUPDR_PULLUP(GPIOB_14) |       \
                                     PIN_PUPDR_PULLUP(GPIOB_15))
#define VAL_GPIOB_ODR               (PIN_ODR_LOW(GPIO_LED_ORANGE) |           \
                                     PIN_ODR_LOW(GPIO_LED_GREEN) |           \
                                     PIN_ODR_LOW(GPIO_LED_RED) |          \
                                     PIN_ODR_HIGH(GPIO_USART7_RX) |          \
                                     PIN_ODR_HIGH(GPIO_BUZZER) |          \
                                     PIN_ODR_HIGH(GPIO_RTK_STAT) |          \
                                     PIN_ODR_HIGH(GPIO_I2C1_SCL) |          \
                                     PIN_ODR_HIGH(GPIO_I2C1_SDA) |              \
                                     PIN_ODR_HIGH(GPIO_2_4_CS) |          \
                                     PIN_ODR_HIGH(GPIO_M8T_RST) |          \
                                     PIN_ODR_HIGH(GPIO_SPI2_CLK) |          \
                                     PIN_ODR_HIGH(GPIOB_11) |          \
                                     PIN_ODR_HIGH(GPIOB_12) |          \
                                     PIN_ODR_HIGH(GPIOB_13) |          \
                                     PIN_ODR_HIGH(GPIOB_14) |              \
                                     PIN_ODR_HIGH(GPIOB_15))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIO_LED_ORANGE, 0U) |       \
                                     PIN_AFIO_AF(GPIO_LED_GREEN, 0U) |        \
                                     PIN_AFIO_AF(GPIO_LED_RED, 0U) |       \
                                     PIN_AFIO_AF(GPIO_USART7_RX, 12U) |       \
                                     PIN_AFIO_AF(GPIO_BUZZER, 2U) |       \
                                     PIN_AFIO_AF(GPIO_RTK_STAT, 0U) |       \
                                     PIN_AFIO_AF(GPIO_I2C1_SCL, 4U) |       \
                                     PIN_AFIO_AF(GPIO_I2C1_SDA, 4U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIO_2_4_CS, 0U) |       \
                                     PIN_AFIO_AF(GPIO_M8T_RST, 0U) |       \
                                     PIN_AFIO_AF(GPIO_SPI2_CLK, 5U) |       \
                                     PIN_AFIO_AF(GPIOB_11, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_12, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_13, 0U) |      \
                                     PIN_AFIO_AF(GPIOB_14, 0U) |          \
                                     PIN_AFIO_AF(GPIOB_15, 0U))

/*
 * GPIOC setup:
 *
 * PC0  - MPU_CS			        (output pushpull maximum).
 * PC1  - SPI2_MOSI                 (alternate 5).
 * PC2  - SPI2_MISO			        (alternate 5).
 * PC3  - GPIOC_3			        (input pullup).
 * PC4  - MCU_CS	                (output pushpull maximum).
 * PC5  - 868_SPI_ATTN              (output pushpull maximum).
 * PC6  - GPIOC_6		            (input pullup).
 * PC7  - GPIOC_7		            (input pullup).
 * PC8  - SD_CS			            (output pushpull maximum).
 * PC9  - I2C3_SDA		            (alternate 4).
 * PC10 - SPI3_CLK		            (alternate 6).
 * PC11 - SPI3_MISO		            (alternate 6).
 * PC12 - USART5_TX		            (alternate 8).
 * PC13 - GPIOC_13                  (input floating).
 * PC14 - OSC32_IN                  (input floating).
 * PC15 - OSC32_OUT                 (input floating).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_OUTPUT(GPIO_MPU_CS) |         \
                                     PIN_MODE_ALTERNATE(GPIO_SPI2_MOSI) |   \
                                     PIN_MODE_ALTERNATE(GPIO_SPI2_MISO) |         \
                                     PIN_MODE_INPUT(GPIOC_3) |         \
                                     PIN_MODE_OUTPUT(GPIO_MCU_CS) |  \
                                     PIN_MODE_OUTPUT(GPIO_868_SPI_ATTN) |  \
                                     PIN_MODE_INPUT(GPIOC_6) |        \
                                     PIN_MODE_INPUT(GPIOC_7) |        \
                                     PIN_MODE_INPUT(GPIO_SD_CS) |        \
                                     PIN_MODE_INPUT(GPIO_I2C3_SDA) |        \
                                     PIN_MODE_INPUT(GPIO_SPI3_CLK) |        \
                                     PIN_MODE_INPUT(GPIO_SPI3_MISO) |        \
                                     PIN_MODE_INPUT(GPIO_USART5_TX) |        \
                                     PIN_MODE_INPUT(GPIOC_13) |         \
                                     PIN_MODE_INPUT(GPIOC_OSC32_IN) |       \
                                     PIN_MODE_INPUT(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIO_MPU_CS) |     \
                                     PIN_OTYPE_PUSHPULL(GPIO_SPI2_MOSI) |   \
                                     PIN_OTYPE_PUSHPULL(GPIO_SPI2_MISO) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIO_MCU_CS) |  \
                                     PIN_OTYPE_PUSHPULL(GPIO_868_SPI_ATTN) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_6) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOC_7) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_SD_CS) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_I2C3_SDA) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_SPI3_CLK) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_SPI3_MISO) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_USART5_TX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOC_13) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_HIGH(GPIO_MPU_CS) |        \
                                     PIN_OSPEED_HIGH(GPIO_SPI2_MOSI) |      \
                                     PIN_OSPEED_HIGH(GPIO_SPI2_MISO) |        \
                                     PIN_OSPEED_HIGH(GPIOC_3) |        \
                                     PIN_OSPEED_HIGH(GPIO_MCU_CS) |     \
                                     PIN_OSPEED_HIGH(GPIO_868_SPI_ATTN) |     \
                                     PIN_OSPEED_HIGH(GPIOC_6) |       \
                                     PIN_OSPEED_HIGH(GPIOC_7) |       \
                                     PIN_OSPEED_HIGH(GPIO_SD_CS) |       \
                                     PIN_OSPEED_HIGH(GPIO_I2C3_SDA) |       \
                                     PIN_OSPEED_HIGH(GPIO_SPI3_CLK) |       \
                                     PIN_OSPEED_HIGH(GPIO_SPI3_MISO) |       \
                                     PIN_OSPEED_HIGH(GPIO_USART5_TX) |       \
                                     PIN_OSPEED_HIGH(GPIOC_13) |        \
                                     PIN_OSPEED_VERYLOW(GPIOC_OSC32_IN) |   \
                                     PIN_OSPEED_VERYLOW(GPIOC_OSC32_OUT))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_PULLUP(GPIO_MPU_CS) |       \
                                     PIN_PUPDR_FLOATING(GPIO_SPI2_MOSI) |   \
                                     PIN_PUPDR_PULLUP(GPIO_SPI2_MISO) |       \
                                     PIN_PUPDR_PULLUP(GPIOC_3) |       \
                                     PIN_PUPDR_FLOATING(GPIO_MCU_CS) |  \
                                     PIN_PUPDR_FLOATING(GPIO_868_SPI_ATTN) |  \
                                     PIN_PUPDR_PULLUP(GPIOC_6) |      \
                                     PIN_PUPDR_PULLUP(GPIOC_7) |      \
                                     PIN_PUPDR_PULLUP(GPIO_SD_CS) |      \
                                     PIN_PUPDR_PULLUP(GPIO_I2C3_SDA) |      \
                                     PIN_PUPDR_PULLUP(GPIO_SPI3_CLK) |      \
                                     PIN_PUPDR_PULLUP(GPIO_SPI3_MISO) |      \
                                     PIN_PUPDR_PULLUP(GPIO_USART5_TX) |      \
                                     PIN_PUPDR_FLOATING(GPIOC_13) |     \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_IN) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIO_MPU_CS) |           \
                                     PIN_ODR_HIGH(GPIO_SPI2_MOSI) |         \
                                     PIN_ODR_HIGH(GPIO_SPI2_MISO) |           \
                                     PIN_ODR_HIGH(GPIOC_3) |           \
                                     PIN_ODR_HIGH(GPIO_MCU_CS) |        \
                                     PIN_ODR_HIGH(GPIO_868_SPI_ATTN) |        \
                                     PIN_ODR_HIGH(GPIOC_6) |          \
                                     PIN_ODR_HIGH(GPIOC_7) |          \
                                     PIN_ODR_HIGH(GPIO_SD_CS) |          \
                                     PIN_ODR_HIGH(GPIO_I2C3_SDA) |          \
                                     PIN_ODR_HIGH(GPIO_SPI3_CLK) |          \
                                     PIN_ODR_HIGH(GPIO_SPI3_MISO) |          \
                                     PIN_ODR_HIGH(GPIO_USART5_TX) |          \
                                     PIN_ODR_HIGH(GPIOC_13) |           \
                                     PIN_ODR_HIGH(GPIOC_OSC32_IN) |         \
                                     PIN_ODR_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIO_MPU_CS, 0U) |        \
                                     PIN_AFIO_AF(GPIO_SPI2_MOSI, 5U) |     \
                                     PIN_AFIO_AF(GPIO_SPI2_MISO, 5U) |        \
                                     PIN_AFIO_AF(GPIOC_3, 0U) |        \
                                     PIN_AFIO_AF(GPIO_MCU_CS, 11U) |    \
                                     PIN_AFIO_AF(GPIO_868_SPI_ATTN, 11U) |    \
                                     PIN_AFIO_AF(GPIOC_6, 0U) |       \
                                     PIN_AFIO_AF(GPIOC_7, 0U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIO_SD_CS, 0U) |       \
                                     PIN_AFIO_AF(GPIO_I2C3_SDA, 4U) |       \
                                     PIN_AFIO_AF(GPIO_SPI3_CLK, 6U) |       \
                                     PIN_AFIO_AF(GPIO_SPI3_MISO, 6U) |       \
                                     PIN_AFIO_AF(GPIO_USART5_TX, 8U) |       \
                                     PIN_AFIO_AF(GPIOC_13, 0U) |        \
                                     PIN_AFIO_AF(GPIOC_OSC32_IN, 0U) |      \
                                     PIN_AFIO_AF(GPIOC_OSC32_OUT, 0U))

/*
 * GPIOD setup:
 *
 * PD0  - CAN1_RX        		    (alternate 9).
 * PD1  - CAN1_TX         		    (alternate 9).
 * PD2  - USART5_RX		            (alternate 8).
 * PD3  - GPIOD_3			        (input pullup).
 * PD4  - GPIOD_4        			(input pullup).
 * PD5  - GPIOD_5         			(input pullup).
 * PD6  - SPI3_MOSI			        (alternate 5).
 * PD7  - GPIOD_7			        (input pullup).
 * PD8  - USART3_RX		            (alternate 7).
 * PD9  - USART3_TX		            (alternate 7).
 * PD10 - GPIOD_10                  (input pullup).
 * PD11 - GPIOD_11			        (input pullup).
 * PD12 - I2C4_SCL			        (alternate 4).
 * PD13 - I2C4_SDA			        (alternate 4).
 * PD14 - GPIOD_14       		    (input pullup).
 * PD15 - GPIOD_15		            (input pullup).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_ALTERNATE(GPIO_CAN1_RX) |        \
                                     PIN_MODE_ALTERNATE(GPIO_CAN1_TX) |        \
                                     PIN_MODE_ALTERNATE(GPIO_USART5_RX) |        \
                                     PIN_MODE_INPUT(GPIOD_3) |        \
                                     PIN_MODE_INPUT(GPIOD_4) |        \
                                     PIN_MODE_INPUT(GPIOD_5) |        \
                                     PIN_MODE_ALTERNATE(GPIO_SPI3_MOSI) |        \
                                     PIN_MODE_INPUT(GPIOD_7) |        \
                                     PIN_MODE_ALTERNATE(GPIO_USART3_RX) |  \
                                     PIN_MODE_ALTERNATE(GPIO_USART3_TX) |  \
                                     PIN_MODE_INPUT(GPIOD_10) |          \
                                     PIN_MODE_INPUT(GPIOD_11) |        \
                                     PIN_MODE_ALTERNATE(GPIO_I2C4_SCL) |        \
                                     PIN_MODE_ALTERNATE(GPIO_I2C4_SDA) |        \
                                     PIN_MODE_INPUT(GPIOD_14) |        \
                                     PIN_MODE_INPUT(GPIOD_15))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIO_CAN1_RX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_CAN1_TX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_USART5_RX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_3) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_4) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_5) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_SPI3_MOSI) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_7) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_USART3_RX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIO_USART3_TX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_11) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_I2C4_SCL) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_I2C4_SDA) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_14) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_15))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_HIGH(GPIO_CAN1_RX) |       \
                                     PIN_OSPEED_HIGH(GPIO_CAN1_TX) |       \
                                     PIN_OSPEED_HIGH(GPIO_USART5_RX) |       \
                                     PIN_OSPEED_HIGH(GPIOD_3) |       \
                                     PIN_OSPEED_HIGH(GPIOD_4) |       \
                                     PIN_OSPEED_HIGH(GPIOD_5) |       \
                                     PIN_OSPEED_HIGH(GPIO_SPI3_MOSI) |       \
                                     PIN_OSPEED_HIGH(GPIOD_7) |       \
                                     PIN_OSPEED_HIGH(GPIO_USART3_RX) |     \
                                     PIN_OSPEED_HIGH(GPIO_USART3_TX) |     \
                                     PIN_OSPEED_HIGH(GPIOD_10) |      \
                                     PIN_OSPEED_HIGH(GPIOD_11) |       \
                                     PIN_OSPEED_HIGH(GPIO_I2C4_SCL) |       \
                                     PIN_OSPEED_HIGH(GPIO_I2C4_SDA) |       \
                                     PIN_OSPEED_HIGH(GPIOD_14) |       \
                                     PIN_OSPEED_HIGH(GPIOD_15))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_PULLUP(GPIO_CAN1_RX) |      \
                                     PIN_PUPDR_PULLUP(GPIO_CAN1_TX) |      \
                                     PIN_PUPDR_FLOATING(GPIO_USART5_RX) |      \
                                     PIN_PUPDR_PULLUP(GPIOD_3) |      \
                                     PIN_PUPDR_PULLUP(GPIOD_4) |      \
                                     PIN_PUPDR_PULLUP(GPIOD_5) |      \
                                     PIN_PUPDR_PULLUP(GPIO_SPI3_MOSI) |      \
                                     PIN_PUPDR_PULLUP(GPIOD_7) |      \
                                     PIN_PUPDR_FLOATING(GPIO_USART3_RX) |  \
                                     PIN_PUPDR_FLOATING(GPIO_USART3_TX) |  \
                                     PIN_PUPDR_PULLUP(GPIOD_10) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_11) |      \
                                     PIN_PUPDR_FLOATING(GPIO_I2C4_SCL) |      \
                                     PIN_PUPDR_FLOATING(GPIO_I2C4_SDA) |      \
                                     PIN_PUPDR_PULLUP(GPIOD_14) |      \
                                     PIN_PUPDR_PULLUP(GPIOD_15))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIO_CAN1_RX) |          \
                                     PIN_ODR_HIGH(GPIO_CAN1_TX) |          \
                                     PIN_ODR_HIGH(GPIO_USART5_RX) |          \
                                     PIN_ODR_HIGH(GPIOD_3) |          \
                                     PIN_ODR_HIGH(GPIOD_4) |          \
                                     PIN_ODR_HIGH(GPIOD_5) |          \
                                     PIN_ODR_HIGH(GPIO_SPI3_MOSI) |          \
                                     PIN_ODR_HIGH(GPIOD_7) |          \
                                     PIN_ODR_HIGH(GPIO_USART3_RX) |        \
                                     PIN_ODR_HIGH(GPIO_USART3_TX) |        \
                                     PIN_ODR_HIGH(GPIOD_10) |            \
                                     PIN_ODR_HIGH(GPIOD_11) |          \
                                     PIN_ODR_HIGH(GPIO_I2C4_SCL) |          \
                                     PIN_ODR_HIGH(GPIO_I2C4_SDA) |          \
                                     PIN_ODR_HIGH(GPIOD_14) |          \
                                     PIN_ODR_HIGH(GPIOD_15))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIO_CAN1_RX, 9U) |       \
                                     PIN_AFIO_AF(GPIO_CAN1_TX, 9U) |       \
                                     PIN_AFIO_AF(GPIO_USART5_RX, 8U) |       \
                                     PIN_AFIO_AF(GPIOD_3, 0U) |       \
                                     PIN_AFIO_AF(GPIOD_4, 0U) |       \
                                     PIN_AFIO_AF(GPIOD_5, 0U) |       \
                                     PIN_AFIO_AF(GPIO_SPI3_MOSI, 5U) |       \
                                     PIN_AFIO_AF(GPIOD_7, 0U))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIO_USART3_RX, 7U) |     \
                                     PIN_AFIO_AF(GPIO_USART3_TX, 7U) |     \
                                     PIN_AFIO_AF(GPIOD_10, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_11, 0U) |       \
                                     PIN_AFIO_AF(GPIO_I2C4_SCL, 4U) |       \
                                     PIN_AFIO_AF(GPIO_I2C4_SDA, 4U) |       \
                                     PIN_AFIO_AF(GPIOD_14, 0U) |       \
                                     PIN_AFIO_AF(GPIOD_15, 0U))

/*
 * GPIOE setup:
 *
 * PE0  - SAT_NTW_AV	            (input pullup).
 * PE1  - SAT_RING_SGN              (input pullup).
 * PE2  - RAM_CS1					(output pushpull maximum).
 * PE3  - RAM_CS2			        (output pushpull maximum).
 * PE4  - GPIOE_4			        (input pullup).
 * PE5  - GPIOE_5			        (input pullup).
 * PE6  - M8T_CS			        (output pushpull maximum).
 * PE7  - GPIOE_7          			(input pullup).
 * PE8  - GPIOE_8         			(input pullup).
 * PE9  - GPIOE_9           		(input pullup).
 * PE10 - GPIOE_10         			(input pullup).
 * PE11 - GPIOE_11           		(input pullup).
 * PE12 - GPIOE_12         			(input pullup).
 * PE13 - KEY1			            (input pullup).
 * PE14 - KEY2             		    (input pullup).
 * PE15 - KEY3				        (input pullup).
 */
#define VAL_GPIOE_MODER             (PIN_MODE_INPUT(GPIO_SAT_NTW_AV) |        \
                                     PIN_MODE_INPUT(GPIO_SAT_RING_SGN) |           \
                                     PIN_MODE_OUTPUT(GPIO_RAM_CS1) |        \
                                     PIN_MODE_OUTPUT(GPIO_RAM_CS2) |        \
                                     PIN_MODE_INPUT(GPIOE_4) |        \
                                     PIN_MODE_INPUT(GPIOE_5) |        \
                                     PIN_MODE_OUTPUT(GPIO_M8T_CS) |        \
                                     PIN_MODE_INPUT(GPIOE_7) |        \
                                     PIN_MODE_INPUT(GPIOE_8) |        \
                                     PIN_MODE_INPUT(GPIOE_9) |         \
                                     PIN_MODE_INPUT(GPIOE_10) |        \
                                     PIN_MODE_INPUT(GPIOE_11) |         \
                                     PIN_MODE_INPUT(GPIOE_12) |        \
                                     PIN_MODE_INPUT(GPIO_KEY1) |         \
                                     PIN_MODE_INPUT(GPIO_KEY2) |        \
                                     PIN_MODE_INPUT(GPIO_KEY3))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIO_SAT_NTW_AV) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_SAT_RING_SGN) |       \
                                     PIN_OTYPE_PUSHPULL(GPIO_RAM_CS1) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_RAM_CS2) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_4) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_5) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_M8T_CS) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_7) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_8) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_9) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_10) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_11) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_12) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_KEY1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIO_KEY2) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_KEY3))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_HIGH(GPIO_SAT_NTW_AV) |       \
                                     PIN_OSPEED_VERYLOW(GPIO_SAT_RING_SGN) |       \
                                     PIN_OSPEED_HIGH(GPIO_RAM_CS1) |       \
                                     PIN_OSPEED_HIGH(GPIO_RAM_CS2) |       \
                                     PIN_OSPEED_HIGH(GPIOE_4) |       \
                                     PIN_OSPEED_HIGH(GPIOE_5) |       \
                                     PIN_OSPEED_HIGH(GPIO_M8T_CS) |       \
                                     PIN_OSPEED_HIGH(GPIOE_7) |       \
                                     PIN_OSPEED_HIGH(GPIOE_8) |       \
                                     PIN_OSPEED_HIGH(GPIOE_9) |        \
                                     PIN_OSPEED_HIGH(GPIOE_10) |       \
                                     PIN_OSPEED_HIGH(GPIOE_11) |        \
                                     PIN_OSPEED_HIGH(GPIOE_12) |       \
                                     PIN_OSPEED_HIGH(GPIO_KEY1) |        \
                                     PIN_OSPEED_HIGH(GPIO_KEY2) |    \
                                     PIN_OSPEED_HIGH(GPIO_KEY3))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_PULLUP(GPIO_SAT_NTW_AV) |      \
                                     PIN_PUPDR_PULLUP(GPIO_SAT_RING_SGN) |         \
                                     PIN_PUPDR_PULLUP(GPIO_RAM_CS1) |      \
                                     PIN_PUPDR_PULLUP(GPIO_RAM_CS2) |      \
                                     PIN_PUPDR_PULLUP(GPIOE_4) |      \
                                     PIN_PUPDR_PULLUP(GPIOE_5) |      \
                                     PIN_PUPDR_PULLUP(GPIO_M8T_CS) |      \
                                     PIN_PUPDR_PULLUP(GPIOE_7) |      \
                                     PIN_PUPDR_PULLUP(GPIOE_8) |      \
                                     PIN_PUPDR_PULLUP(GPIOE_9) |       \
                                     PIN_PUPDR_PULLUP(GPIOE_10) |      \
                                     PIN_PUPDR_PULLUP(GPIOE_11) |       \
                                     PIN_PUPDR_PULLUP(GPIOE_12) |      \
                                     PIN_PUPDR_PULLUP(GPIO_KEY1) |       \
                                     PIN_PUPDR_PULLUP(GPIO_KEY2) |      \
                                     PIN_PUPDR_PULLUP(GPIO_KEY3))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIO_SAT_NTW_AV) |          \
                                     PIN_ODR_HIGH(GPIO_SAT_RING_SGN) |             \
                                     PIN_ODR_HIGH(GPIO_RAM_CS1) |          \
                                     PIN_ODR_HIGH(GPIO_RAM_CS2) |          \
                                     PIN_ODR_HIGH(GPIOE_4) |          \
                                     PIN_ODR_HIGH(GPIOE_5) |          \
                                     PIN_ODR_HIGH(GPIO_M8T_CS) |          \
                                     PIN_ODR_HIGH(GPIOE_7) |          \
                                     PIN_ODR_HIGH(GPIOE_8) |          \
                                     PIN_ODR_HIGH(GPIOE_9) |           \
                                     PIN_ODR_HIGH(GPIOE_10) |          \
                                     PIN_ODR_HIGH(GPIOE_11) |           \
                                     PIN_ODR_HIGH(GPIOE_12) |          \
                                     PIN_ODR_HIGH(GPIO_KEY1) |           \
                                     PIN_ODR_HIGH(GPIO_KEY2) |          \
                                     PIN_ODR_HIGH(GPIO_KEY3))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIO_SAT_NTW_AV, 0U) |       \
                                     PIN_AFIO_AF(GPIO_SAT_RING_SGN, 0U) |          \
                                     PIN_AFIO_AF(GPIO_RAM_CS1, 0U) |       \
                                     PIN_AFIO_AF(GPIO_RAM_CS2, 0U) |       \
                                     PIN_AFIO_AF(GPIOE_4, 0U) |       \
                                     PIN_AFIO_AF(GPIOE_5, 0U) |       \
                                     PIN_AFIO_AF(GPIO_M8T_CS, 0U) |       \
                                     PIN_AFIO_AF(GPIOE_7, 0U))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_8, 0U) |       \
                                     PIN_AFIO_AF(GPIOE_9, 0U) |        \
                                     PIN_AFIO_AF(GPIOE_10, 0U) |       \
                                     PIN_AFIO_AF(GPIOE_11, 0U) |        \
                                     PIN_AFIO_AF(GPIOE_12, 0U) |       \
                                     PIN_AFIO_AF(GPIO_KEY1, 0U) |        \
                                     PIN_AFIO_AF(GPIO_KEY2, 0U) |       \
                                     PIN_AFIO_AF(GPIO_KEY3, 0U))

/*
 * GPIOF setup:
 *
 * PF0  - GPIOF_0          			  (input pullup).
 * PF1  - GPIOF_1          			  (input pullup).
 * PF2  - GPIOF_2         			  (input pullup).
 * PF3  - ADC3_IN9            		  (input pullup).
 * PF4  - ADC3_IN14          		  (input pullup).
 * PF5  - ADC3_IN15          	      (input pullup).
 * PF6  - GPIOF_6                     (input pullup).
 * PF7  - GPIOF_7       			  (input pullup).
 * PF8  - 868_SLEEP			          (output pushpull maximum).
 * PF9  - 868_RST         		      (output pushpull maximum).
 * PF10 - GPIOF_10			          (input pullup).
 * PF11 - GPIOF_11                    (input pullup).
 * PF12 - GPIOF_12                    (input pullup).
 * PF13 - GPIOF_13                    (input pullup).
 * PF14 - GPIOF_14                    (input pullup).
 * PF15 - GPIOF_15                    (input pullup).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_INPUT(GPIOF_0) |        \
                                     PIN_MODE_INPUT(GPIOF_1) |        \
                                     PIN_MODE_INPUT(GPIOF_2) |        \
                                     PIN_MODE_INPUT(GPIO_ADC3_IN9) |         \
                                     PIN_MODE_INPUT(GPIO_ADC3_IN14) |         \
                                     PIN_MODE_INPUT(GPIO_ADC3_IN15) |         \
                                     PIN_MODE_INPUT(GPIOF_6) |           \
                                     PIN_MODE_INPUT(GPIOF_7) |        \
                                     PIN_MODE_OUTPUT(GPIO_868_SLEEP) |        \
                                     PIN_MODE_OUTPUT(GPIO_868_RST) |        \
                                     PIN_MODE_INPUT(GPIOF_10) |         \
                                     PIN_MODE_INPUT(GPIOF_11) |          \
                                     PIN_MODE_INPUT(GPIOF_12) |         \
                                     PIN_MODE_INPUT(GPIOF_13) |         \
                                     PIN_MODE_INPUT(GPIOF_14) |         \
                                     PIN_MODE_INPUT(GPIOF_15))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_0) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOF_1) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOF_2) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_ADC3_IN9) |     \
                                     PIN_OTYPE_PUSHPULL(GPIO_ADC3_IN14) |     \
                                     PIN_OTYPE_PUSHPULL(GPIO_ADC3_IN15) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_7) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_868_SLEEP) |    \
                                     PIN_OTYPE_PUSHPULL(GPIO_868_RST) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOF_10) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_12) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_13) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_14) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_15))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_HIGH(GPIOF_0) |       \
                                     PIN_OSPEED_HIGH(GPIOF_1) |       \
                                     PIN_OSPEED_HIGH(GPIOF_2) |       \
                                     PIN_OSPEED_HIGH(GPIO_ADC3_IN9) |        \
                                     PIN_OSPEED_HIGH(GPIO_ADC3_IN14) |        \
                                     PIN_OSPEED_HIGH(GPIO_ADC3_IN15) |        \
                                     PIN_OSPEED_VERYLOW(GPIOF_6) |       \
                                     PIN_OSPEED_HIGH(GPIOF_7) |       \
                                     PIN_OSPEED_HIGH(GPIO_868_SLEEP) |       \
                                     PIN_OSPEED_HIGH(GPIO_868_RST) |       \
                                     PIN_OSPEED_HIGH(GPIOF_10) |        \
                                     PIN_OSPEED_VERYLOW(GPIOF_11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_12) |     \
                                     PIN_OSPEED_VERYLOW(GPIOF_13) |     \
                                     PIN_OSPEED_VERYLOW(GPIOF_14) |     \
                                     PIN_OSPEED_VERYLOW(GPIOF_15))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_PULLUP(GPIOF_0) |      \
                                     PIN_PUPDR_PULLUP(GPIOF_1) |      \
                                     PIN_PUPDR_PULLUP(GPIOF_2) |      \
                                     PIN_PUPDR_PULLUP(GPIO_ADC3_IN9) |       \
                                     PIN_PUPDR_PULLUP(GPIO_ADC3_IN14) |       \
                                     PIN_PUPDR_PULLUP(GPIO_ADC3_IN15) |       \
                                     PIN_PUPDR_PULLUP(GPIOF_6) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_7) |      \
                                     PIN_PUPDR_PULLUP(GPIO_868_SLEEP) |      \
                                     PIN_PUPDR_PULLUP(GPIO_868_RST) |      \
                                     PIN_PUPDR_PULLUP(GPIOF_10) |       \
                                     PIN_PUPDR_PULLUP(GPIOF_11) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_12) |       \
                                     PIN_PUPDR_PULLUP(GPIOF_13) |       \
                                     PIN_PUPDR_PULLUP(GPIOF_14) |       \
                                     PIN_PUPDR_PULLUP(GPIOF_15))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_0) |          \
                                     PIN_ODR_HIGH(GPIOF_1) |          \
                                     PIN_ODR_HIGH(GPIOF_2) |          \
                                     PIN_ODR_HIGH(GPIO_ADC3_IN9) |           \
                                     PIN_ODR_HIGH(GPIO_ADC3_IN14) |           \
                                     PIN_ODR_HIGH(GPIO_ADC3_IN15) |           \
                                     PIN_ODR_HIGH(GPIOF_6) |             \
                                     PIN_ODR_HIGH(GPIOF_7) |          \
                                     PIN_ODR_HIGH(GPIO_868_SLEEP) |          \
                                     PIN_ODR_HIGH(GPIO_868_RST) |          \
                                     PIN_ODR_HIGH(GPIOF_10) |           \
                                     PIN_ODR_HIGH(GPIOF_11) |            \
                                     PIN_ODR_HIGH(GPIOF_12) |           \
                                     PIN_ODR_HIGH(GPIOF_13) |           \
                                     PIN_ODR_HIGH(GPIOF_14) |           \
                                     PIN_ODR_HIGH(GPIOF_15))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_0, 0U) |       \
                                     PIN_AFIO_AF(GPIOF_1, 0U) |       \
                                     PIN_AFIO_AF(GPIOF_2, 0U) |       \
                                     PIN_AFIO_AF(GPIO_ADC3_IN9, 0U) |        \
                                     PIN_AFIO_AF(GPIO_ADC3_IN14, 0U) |        \
                                     PIN_AFIO_AF(GPIO_ADC3_IN15, 0U) |        \
                                     PIN_AFIO_AF(GPIOF_6, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_7, 0U))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIO_868_SLEEP, 0U) |       \
                                     PIN_AFIO_AF(GPIO_868_RST, 0U) |       \
                                     PIN_AFIO_AF(GPIOF_10, 0U) |        \
                                     PIN_AFIO_AF(GPIOF_11, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_12, 0U) |        \
                                     PIN_AFIO_AF(GPIOF_13, 0U) |        \
                                     PIN_AFIO_AF(GPIOF_14, 0U) |        \
                                     PIN_AFIO_AF(GPIOF_15, 0U))

/*
 * GPIOG setup:
 *
 * PG0  - GPIOG_0                   (input pullup).
 * PG1  - GPIOG_1                   (input pullup).
 * PG2  - GPIOG_2                   (input pullup).
 * PG3  - GPIOG_3                   (input pullup).
 * PG4  - GPIOG_4                   (input pullup).
 * PG5  - GPIOG_5                   (input pullup).
 * PG6  - GPIOG_6              		(input pullup).
 * PG7  - GPIOG_7               	(input pullup).
 * PG8  - SD_CD                     (input pullup).
 * PG9  - GPIOG_9          			(input pullup).
 * PG10 - SAT_ON_OFF                (output pushpull maximum).
 * PG11 - LTE_ON_OFF                (output pushpull maximum).
 * PG12 - WIFI_ON_OFF               (output pushpull maximum).
 * PG13 - 2_4_ON_OFF                (output pushpull maximum).
 * PG14 - GPIOG_14 			        (input pullup).
 * PG15 - GPIOG_15                  (input pullup).
 */
#define VAL_GPIOG_MODER             (PIN_MODE_INPUT(GPIOG_0) |        \
                                     PIN_MODE_INPUT(GPIOG_1) |        \
                                     PIN_MODE_INPUT(GPIOG_2) |        \
                                     PIN_MODE_INPUT(GPIOG_3) |        \
                                     PIN_MODE_INPUT(GPIOG_4) |           \
                                     PIN_MODE_INPUT(GPIOG_5) |           \
                                     PIN_MODE_INPUT(GPIOG_6) |   \
                                     PIN_MODE_INPUT(GPIOG_7) |    \
                                     PIN_MODE_INPUT(GPIO_SD_CD) |           \
                                     PIN_MODE_INPUT(GPIOG_9) |         \
                                     PIN_MODE_OUTPUT(GPIO_SAT_ON_OFF) |          \
                                     PIN_MODE_OUTPUT(GPIO_LTE_ON_OFF) | \
                                     PIN_MODE_OUTPUT(GPIO_WIFI_ON_OFF) |          \
                                     PIN_MODE_OUTPUT(GPIO_2_4_ON_OFF) |  \
                                     PIN_MODE_INPUT(GPIOG_14) |         \
                                     PIN_MODE_INPUT(GPIOG_15))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOG_0) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_1) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_2) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_3) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_6) |\
                                     PIN_OTYPE_PUSHPULL(GPIOG_7) |\
                                     PIN_OTYPE_PUSHPULL(GPIO_SD_CD) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_9) |     \
                                     PIN_OTYPE_PUSHPULL(GPIO_SAT_ON_OFF) |      \
                                     PIN_OTYPE_PUSHPULL(GPIO_LTE_ON_OFF) | \
                                     PIN_OTYPE_PUSHPULL(GPIO_WIFI_ON_OFF) |      \
                                     PIN_OTYPE_PUSHPULL(GPIO_2_4_ON_OFF) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_14) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_15))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOG_0) |    \
                                     PIN_OSPEED_VERYLOW(GPIOG_1) |    \
                                     PIN_OSPEED_VERYLOW(GPIOG_2) |    \
                                     PIN_OSPEED_VERYLOW(GPIOG_3) |    \
                                     PIN_OSPEED_VERYLOW(GPIOG_4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_5) |       \
                                     PIN_OSPEED_HIGH(GPIOG_6) |  \
                                     PIN_OSPEED_HIGH(GPIOG_7) |   \
                                     PIN_OSPEED_HIGH(GPIO_SD_CD) |       \
                                     PIN_OSPEED_HIGH(GPIOG_9) |        \
                                     PIN_OSPEED_HIGH(GPIO_SAT_ON_OFF) |      \
                                     PIN_OSPEED_HIGH(GPIO_LTE_ON_OFF) |    \
                                     PIN_OSPEED_HIGH(GPIO_WIFI_ON_OFF) |      \
                                     PIN_OSPEED_HIGH(GPIO_2_4_ON_OFF) |     \
                                     PIN_OSPEED_HIGH(GPIOG_14) |        \
                                     PIN_OSPEED_HIGH(GPIOG_15))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_PULLUP(GPIOG_0) |      \
                                     PIN_PUPDR_PULLUP(GPIOG_1) |      \
                                     PIN_PUPDR_PULLUP(GPIOG_2) |      \
                                     PIN_PUPDR_PULLUP(GPIOG_3) |      \
                                     PIN_PUPDR_PULLUP(GPIOG_4) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_5) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_6) | \
                                     PIN_PUPDR_PULLUP(GPIOG_7) |  \
                                     PIN_PUPDR_PULLUP(GPIO_SD_CD) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_9) |       \
                                     PIN_PUPDR_PULLUP(GPIO_SAT_ON_OFF) |        \
                                     PIN_PUPDR_PULLUP(GPIO_LTE_ON_OFF) | \
                                     PIN_PUPDR_PULLUP(GPIO_WIFI_ON_OFF) |        \
                                     PIN_PUPDR_PULLUP(GPIO_2_4_ON_OFF) |  \
                                     PIN_PUPDR_PULLUP(GPIOG_14) |       \
                                     PIN_PUPDR_PULLUP(GPIOG_15))
#define VAL_GPIOG_ODR               (PIN_ODR_HIGH(GPIOG_0) |          \
                                     PIN_ODR_HIGH(GPIOG_1) |          \
                                     PIN_ODR_HIGH(GPIOG_2) |          \
                                     PIN_ODR_HIGH(GPIOG_3) |          \
                                     PIN_ODR_HIGH(GPIOG_4) |             \
                                     PIN_ODR_HIGH(GPIOG_5) |             \
                                     PIN_ODR_HIGH(GPIOG_6) |     \
                                     PIN_ODR_HIGH(GPIOG_7) |      \
                                     PIN_ODR_HIGH(GPIO_SD_CD) |             \
                                     PIN_ODR_HIGH(GPIOG_9) |           \
                                     PIN_ODR_HIGH(GPIO_SAT_ON_OFF) |            \
                                     PIN_ODR_HIGH(GPIO_LTE_ON_OFF) |       \
                                     PIN_ODR_HIGH(GPIO_WIFI_ON_OFF) |            \
                                     PIN_ODR_HIGH(GPIO_2_4_ON_OFF) |        \
                                     PIN_ODR_HIGH(GPIOG_14) |           \
                                     PIN_ODR_HIGH(GPIOG_15))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_0, 0U) |       \
                                     PIN_AFIO_AF(GPIOG_1, 0U) |       \
                                     PIN_AFIO_AF(GPIOG_2, 0U) |       \
                                     PIN_AFIO_AF(GPIOG_3, 0U) |       \
                                     PIN_AFIO_AF(GPIOG_4, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_5, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_6, 0U) |  \
                                     PIN_AFIO_AF(GPIOG_7, 0U))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIO_SD_CD, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_9, 0U) |        \
                                     PIN_AFIO_AF(GPIO_SAT_ON_OFF, 0U) |         \
                                     PIN_AFIO_AF(GPIO_LTE_ON_OFF, 0U) |   \
                                     PIN_AFIO_AF(GPIO_WIFI_ON_OFF, 0U) |         \
                                     PIN_AFIO_AF(GPIO_2_4_ON_OFF, 0U) |    \
                                     PIN_AFIO_AF(GPIOG_14, 0U) |        \
                                     PIN_AFIO_AF(GPIOG_15, 0U))
#define GPIOH_OSC_IN                0U
#define GPIOH_OSC_OUT               1U
#define GPIOH_PIN2                  2U
#define GPIOH_PIN3                  3U
#define GPIOH_PIN4                  4U
#define GPIOH_PIN5                  5U
#define GPIOH_PIN6                  6U
#define GPIOH_PIN7                  7U
#define GPIOH_PIN8                  8U
#define GPIOH_PIN9                  9U
#define GPIOH_PIN10                 10U
#define GPIOH_PIN11                 11U
#define GPIOH_PIN12                 12U
#define GPIOH_PIN13                 13U
#define GPIOH_PIN14                 14U
#define GPIOH_PIN15                 15U

#define GPIOI_PIN0                  0U
#define GPIOI_PIN1                  1U
#define GPIOI_PIN2                  2U
#define GPIOI_PIN3                  3U
#define GPIOI_PIN4                  4U
#define GPIOI_PIN5                  5U
#define GPIOI_PIN6                  6U
#define GPIOI_PIN7                  7U
#define GPIOI_PIN8                  8U
#define GPIOI_PIN9                  9U
#define GPIOI_PIN10                 10U
#define GPIOI_PIN11                 11U
#define GPIOI_PIN12                 12U
#define GPIOI_PIN13                 13U
#define GPIOI_PIN14                 14U
#define GPIOI_PIN15                 15U

#define GPIOJ_PIN0                  0U
#define GPIOJ_PIN1                  1U
#define GPIOJ_PIN2                  2U
#define GPIOJ_PIN3                  3U
#define GPIOJ_PIN4                  4U
#define GPIOJ_PIN5                  5U
#define GPIOJ_PIN6                  6U
#define GPIOJ_PIN7                  7U
#define GPIOJ_PIN8                  8U
#define GPIOJ_PIN9                  9U
#define GPIOJ_PIN10                 10U
#define GPIOJ_PIN11                 11U
#define GPIOJ_PIN12                 12U
#define GPIOJ_PIN13                 13U
#define GPIOJ_PIN14                 14U
#define GPIOJ_PIN15                 15U

#define GPIOK_PIN0                  0U
#define GPIOK_PIN1                  1U
#define GPIOK_PIN2                  2U
#define GPIOK_PIN3                  3U
#define GPIOK_PIN4                  4U
#define GPIOK_PIN5                  5U
#define GPIOK_PIN6                  6U
#define GPIOK_PIN7                  7U
#define GPIOK_PIN8                  8U
#define GPIOK_PIN9                  9U
#define GPIOK_PIN10                 10U
#define GPIOK_PIN11                 11U
#define GPIOK_PIN12                 12U
#define GPIOK_PIN13                 13U
#define GPIOK_PIN14                 14U
#define GPIOK_PIN15                 15U

/*
 * GPIOH setup:
 *
 * PH0  - OSC_IN                    (input floating).
 * PH1  - OSC_OUT                   (input floating).
 * PH2  - PIN2                      (input pullup).
 * PH3  - PIN3                      (input pullup).
 * PH4  - PIN4                      (input pullup).
 * PH5  - PIN5                      (input pullup).
 * PH6  - PIN6                      (input pullup).
 * PH7  - PIN7                      (input pullup).
 * PH8  - PIN8                      (input pullup).
 * PH9  - PIN9                      (input pullup).
 * PH10 - PIN10                     (input pullup).
 * PH11 - PIN11                     (input pullup).
 * PH12 - PIN12                     (input pullup).
 * PH13 - PIN13                     (input pullup).
 * PH14 - PIN14                     (input pullup).
 * PH15 - PIN15                     (input pullup).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) |         \
                                     PIN_MODE_INPUT(GPIOH_OSC_OUT) |        \
                                     PIN_MODE_INPUT(GPIOH_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN15))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_OSC_IN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOH_OSC_OUT) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN15))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_HIGH(GPIOH_OSC_IN) |        \
                                     PIN_OSPEED_HIGH(GPIOH_OSC_OUT) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN15))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_FLOATING(GPIOH_OSC_IN) |     \
                                     PIN_PUPDR_FLOATING(GPIOH_OSC_OUT) |    \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN15))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_OSC_IN) |           \
                                     PIN_ODR_HIGH(GPIOH_OSC_OUT) |          \
                                     PIN_ODR_HIGH(GPIOH_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN15))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_OSC_IN, 0U) |        \
                                     PIN_AFIO_AF(GPIOH_OSC_OUT, 0U) |       \
                                     PIN_AFIO_AF(GPIOH_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN7, 0U))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN15, 0U))

/*
 * GPIOI setup:
 *
 * PI0  - PIN0                      (input pullup).
 * PI1  - PIN1                      (input pullup).
 * PI2  - PIN2                      (input pullup).
 * PI3  - PIN3                      (input pullup).
 * PI4  - PIN4                      (input pullup).
 * PI5  - PIN5                      (input pullup).
 * PI6  - PIN6                      (input pullup).
 * PI7  - PIN7                      (input pullup).
 * PI8  - PIN8                      (input pullup).
 * PI9  - PIN9                      (input pullup).
 * PI10 - PIN10                     (input pullup).
 * PI11 - PIN11                     (input pullup).
 * PI12 - PIN12                     (input pullup).
 * PI13 - PIN13                     (input pullup).
 * PI14 - PIN14                     (input pullup).
 * PI15 - PIN15                     (input pullup).
 */
#define VAL_GPIOI_MODER             (PIN_MODE_INPUT(GPIOI_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN15))
#define VAL_GPIOI_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOI_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN15))
#define VAL_GPIOI_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOI_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN15))
#define VAL_GPIOI_PUPDR             (PIN_PUPDR_PULLUP(GPIOI_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN15))
#define VAL_GPIOI_ODR               (PIN_ODR_HIGH(GPIOI_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN15))
#define VAL_GPIOI_AFRL              (PIN_AFIO_AF(GPIOI_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN7, 0U))
#define VAL_GPIOI_AFRH              (PIN_AFIO_AF(GPIOI_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN15, 0U))

/*
 * GPIOJ setup:
 *
 * PJ0  - PIN0                      (input pullup).
 * PJ1  - PIN1                      (input pullup).
 * PJ2  - PIN2                      (input pullup).
 * PJ3  - PIN3                      (input pullup).
 * PJ4  - PIN4                      (input pullup).
 * PJ5  - PIN5                      (input pullup).
 * PJ6  - PIN6                      (input pullup).
 * PJ7  - PIN7                      (input pullup).
 * PJ8  - PIN8                      (input pullup).
 * PJ9  - PIN9                      (input pullup).
 * PJ10 - PIN10                     (input pullup).
 * PJ11 - PIN11                     (input pullup).
 * PJ12 - PIN12                     (input pullup).
 * PJ13 - PIN13                     (input pullup).
 * PJ14 - PIN14                     (input pullup).
 * PJ15 - PIN15                     (input pullup).
 */
#define VAL_GPIOJ_MODER             (PIN_MODE_INPUT(GPIOJ_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOJ_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOJ_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOJ_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOJ_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOJ_PIN15))
#define VAL_GPIOJ_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOJ_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN15))
#define VAL_GPIOJ_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOJ_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN15))
#define VAL_GPIOJ_PUPDR             (PIN_PUPDR_PULLUP(GPIOJ_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN15))
#define VAL_GPIOJ_ODR               (PIN_ODR_HIGH(GPIOJ_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOJ_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOJ_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOJ_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOJ_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOJ_PIN15))
#define VAL_GPIOJ_AFRL              (PIN_AFIO_AF(GPIOJ_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN7, 0U))
#define VAL_GPIOJ_AFRH              (PIN_AFIO_AF(GPIOJ_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOJ_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOJ_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOJ_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOJ_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOJ_PIN15, 0U))

/*
 * GPIOK setup:
 *
 * PK0  - PIN0                      (input pullup).
 * PK1  - PIN1                      (input pullup).
 * PK2  - PIN2                      (input pullup).
 * PK3  - PIN3                      (input pullup).
 * PK4  - PIN4                      (input pullup).
 * PK5  - PIN5                      (input pullup).
 * PK6  - PIN6                      (input pullup).
 * PK7  - PIN7                      (input pullup).
 * PK8  - PIN8                      (input pullup).
 * PK9  - PIN9                      (input pullup).
 * PK10 - PIN10                     (input pullup).
 * PK11 - PIN11                     (input pullup).
 * PK12 - PIN12                     (input pullup).
 * PK13 - PIN13                     (input pullup).
 * PK14 - PIN14                     (input pullup).
 * PK15 - PIN15                     (input pullup).
 */
#define VAL_GPIOK_MODER             (PIN_MODE_INPUT(GPIOK_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOK_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOK_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOK_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOK_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOK_PIN15))
#define VAL_GPIOK_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOK_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN15))
#define VAL_GPIOK_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOK_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN15))
#define VAL_GPIOK_PUPDR             (PIN_PUPDR_PULLUP(GPIOK_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN15))
#define VAL_GPIOK_ODR               (PIN_ODR_HIGH(GPIOK_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOK_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOK_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOK_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOK_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOK_PIN15))
#define VAL_GPIOK_AFRL              (PIN_AFIO_AF(GPIOK_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN7, 0U))
#define VAL_GPIOK_AFRH              (PIN_AFIO_AF(GPIOK_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOK_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOK_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOK_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOK_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOK_PIN15, 0U))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_H */
