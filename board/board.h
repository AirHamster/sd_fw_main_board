/*
    ChibiOS - Copyright (C) 2006..2017 Giovanni Di Sirio

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
 * Setup for SailData main CPU board board.
 */

/*
 * Board identifier.
 */
#define BOARD_SD_main
#define BOARD_NAME                  "SailData main CPU board"

/*
 * Ethernet PHY type.
 */
#define BOARD_PHY_ID                MII_LAN8742A_ID
#define BOARD_PHY_RMII

/*
 * Board oscillators-related settings.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                32768U
#endif

#define STM32_LSEDRV                (3U << 3U)

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                12000000U
#endif

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
#define GPIOA_USART4_RX             0U
#define GPIOA_USART4_TX             1U
#define GPIOA_USART2_TX             2U
#define GPIOA_USART2_RX             3U
#define GPIOA_RF_868_CS             4U
#define GPIOA_SPI1_SCK              5U
#define GPIOA_SPI1_MISO             6U
#define GPIOA_SPI1_MOSI             7U
#define GPIOA_I2C3_SCL              8U
#define GPIOA_USART1_TX             9U
#define GPIOA_USART1_RX             10U
#define GPIOA_USB_DM                11U
#define GPIOA_USB_DP                12U
#define GPIOA_SWDIO                 13U
#define GPIOA_SWCLK                 14U
#define GPIOA_GPIOA_15             15U

#define GPIOB_GREEN_LED            0U
#define GPIOB_ORANGE_LED             1U
#define GPIOB_RED_LED               2U
#define GPIOB_GPIOB_3             3U
#define GPIOB_BUZZER                4U
#define GPIOB_RTK_STAT              5U
#define GPIOB_I2C1_SCL              6U
#define GPIOB_I2C1_SDA              7U
#define GPIOB_RF_2_4_CS             8U
#define GPIOB_M8T_RST               9U
#define GPIOB_SPI2_CLK              10U
#define GPIOB_GPIOB_11              11U
#define GPIOB_GPIOB_12              12U
#define GPIOB_GPIOB_13              13U
#define GPIOB_GPIOB_14              14U
#define GPIOB_GPIOB_15              15U

#define GPIOC_MPU_CS                0U
#define GPIOC_SPI2_MOSI             1U
#define GPIOC_SPI2_MISO             2U
#define GPIOC_GPIOC_3               3U
#define GPIOC_MCU_CS                4U
#define GPIOC_NINA_CTS       		5U
#define GPIOC_GPIOC_6               6U
#define GPIOC_GPIOC_7               7U
#define GPIOC_GPIOC_8                8U
#define GPIOC_I2C3_SDA              9U
#define GPIOC_SPI3_CLK              10U
#define GPIOC_SPI3_MISO             11U
#define GPIOC_USART5_TX             12U
#define GPIOC_GPIOC_13              13U
#define GPIOC_OSC32_IN              14U
#define GPIOC_OSC32_OUT             15U

#define GPIOD_CAN1_RX               0U
#define GPIOD_CAN1_TX               1U
#define GPIOD_USART5_RX             2U
#define GPIOD_GPIOD_3               3U
#define GPIOD_SD_CS	               4U
#define GPIOD_GPIOD_5               5U
#define GPIOD_SPI3_MOSI             6U
#define GPIOD_GPIOD_7               7U
#define GPIOD_USART3_RX             8U
#define GPIOD_USART3_TX             9U
#define GPIOD_GPIOD_10              10U
#define GPIOD_GPIOD_11              11U
#define GPIOD_I2C4_SCL              12U
#define GPIOD_I2C4_SDA              13U
#define GPIOD_GPIOD_14              14U
#define GPIOD_GPIOD_15              15U

#define GPIOE_UART8_RX      	    0U		//OK
#define GPIOE_UART8_TX		        1U		//OK
#define GPIOE_SPI4_SCK              2U		//OK
#define GPIOE_RAM_CS2               3U
#define GPIOE_M8T_CS		        4U		//OK
#define GPIOE_SPI4_MISO             5U		//OK
#define GPIOE_SPI4_MOSI             6U		//OK
#define GPIOE_GPIOE_7               7U
#define GPIOE_GPIOE_8               8U
#define GPIOE_GPIOE_9               9U
#define GPIOE_GPIOE_10              10U
#define GPIOE_GPIOE_11              11U
#define GPIOE_GPIOE_12              12U
#define GPIOE_KEY1                  13U
#define GPIOE_KEY2                  14U
#define GPIOE_KEY3                  15U

#define GPIOF_I2C2_SDA              0U
#define GPIOF_I2C2_SCL              1U
#define GPIOF_RF_868_SPI_ATTN               2U
#define GPIOF_ADC3_IN9              3U
#define GPIOF_ADC3_IN14             4U
#define GPIOF_ADC3_IN15             5U
#define GPIOF_USART7_RX             6U
#define GPIOF_USART7_TX             7U
#define GPIOF_RF_868_SLEEP          8U
#define GPIOF_RF_868_RST            9U
#define GPIOF_GPIOF_10              10U
#define GPIOF_GPIOF_11              11U
#define GPIOF_GPIOF_12              12U
#define GPIOF_GPIOF_13              13U
#define GPIOF_GPIOF_14              14U
#define GPIOF_GPIOF_15              15U

#define GPIOG_ZIO_D65               0U
#define GPIOG_ZIO_D64               1U
#define GPIOG_ZIO_D49               2U
#define GPIOG_ZIO_D50               3U
#define GPIOG_PIN4                  4U
#define GPIOG_PIN5                  5U
#define GPIOG_USB_GPIO_OUT          6U
#define GPIOG_USB_GPIO_IN           7U
#define GPIOG_PIN8                  8U
#define GPIOG_ARD_D0                9U
#define GPIOG_USART6_RX             9U
#define GPIOG_PIN10                 10U
#define GPIOG_RMII_TX_EN            11U
#define GPIOG_PIN12                 12U
#define GPIOG_RMII_TXD0             13U
#define GPIOG_ARD_D1                14U
#define GPIOG_USART6_TX             14U
#define GPIOG_PIN15                 15U

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
 * IO lines assignments.
 */
#define LINE_USART4_RX              PAL_LINE(GPIOA, 0U)
#define LINE_USART4_TX              PAL_LINE(GPIOA, 1U)
#define LINE_USART2_TX              PAL_LINE(GPIOA, 2U)
#define LINE_USART2_RX              PAL_LINE(GPIOA, 3U)
#define LINE_RF_868_CS              PAL_LINE(GPIOA, 4U)
#define LINE_SPI1_SCK               PAL_LINE(GPIOA, 5U)
#define LINE_SPI1_MISO              PAL_LINE(GPIOA, 6U)
#define LINE_SPI1_MOSI              PAL_LINE(GPIOA, 7U)
#define LINE_I2C3_SCL               PAL_LINE(GPIOA, 8U)
#define LINE_USART1_TX              PAL_LINE(GPIOA, 9U)
#define LINE_USART1_RX              PAL_LINE(GPIOA, 10U)
#define LINE_USB_DM                 PAL_LINE(GPIOA, 11U)
#define LINE_USB_DP                 PAL_LINE(GPIOA, 12U)
#define LINE_SWDIO                  PAL_LINE(GPIOA, 13U)
#define LINE_SWCLK                  PAL_LINE(GPIOA, 14U)
#define LINE_GPIOA_15              PAL_LINE(GPIOA, 15U)
#define LINE_GREEN_LED             	PAL_LINE(GPIOB, 1U)
#define LINE_ORANGE_LED             PAL_LINE(GPIOB, 0U)
#define LINE_RED_LED                PAL_LINE(GPIOB, 2U)
#define LINE_USART7_RX              PAL_LINE(GPIOB, 3U)
#define LINE_BUZZER                 PAL_LINE(GPIOB, 4U)
#define LINE_RTK_STAT               PAL_LINE(GPIOB, 5U)
#define LINE_I2C1_SCL               PAL_LINE(GPIOB, 6U)
#define LINE_I2C1_SDA               PAL_LINE(GPIOB, 7U)
#define LINE_RF_2_4_CS              PAL_LINE(GPIOB, 8U)
#define LINE_M8T_RST                PAL_LINE(GPIOB, 9U)
#define LINE_SPI2_CLK               PAL_LINE(GPIOB, 10U)
#define LINE_GPIOB_11               PAL_LINE(GPIOB, 11U)
#define LINE_GPIOB_12               PAL_LINE(GPIOB, 12U)
#define LINE_GPIOB_13               PAL_LINE(GPIOB, 13U)
#define LINE_GPIOB_14               PAL_LINE(GPIOB, 14U)
#define LINE_GPIOB_15               PAL_LINE(GPIOB, 15U)
#define LINE_MPU_CS                 PAL_LINE(GPIOC, 0U)
#define LINE_SPI2_MOSI              PAL_LINE(GPIOC, 1U)
#define LINE_SPI2_MISO              PAL_LINE(GPIOC, 2U)
#define LINE_GPIOC_3                PAL_LINE(GPIOC, 3U)
#define LINE_MCU_CS                 PAL_LINE(GPIOC, 4U)
#define LINE_NINA_CTS				PAL_LINE(GPIOC, 5U)
#define LINE_RF_868_SPI_ATTN        PAL_LINE(GPIOF, 2U)
#define LINE_GPIOC_6                PAL_LINE(GPIOC, 6U)
#define LINE_GPIOC_7                PAL_LINE(GPIOC, 7U)
#define LINE_GPIOC_8                PAL_LINE(GPIOC, 8U)
#define LINE_I2C3_SDA               PAL_LINE(GPIOC, 9U)
#define LINE_SPI3_CLK               PAL_LINE(GPIOC, 10U)
#define LINE_SPI3_MISO              PAL_LINE(GPIOC, 11U)
#define LINE_USART5_TX              PAL_LINE(GPIOC, 12U)
#define LINE_GPIOC_13               PAL_LINE(GPIOC, 13U)
#define LINE_OSC32_IN               PAL_LINE(GPIOC, 14U)
#define LINE_OSC32_OUT              PAL_LINE(GPIOC, 15U)
#define LINE_CAN1_RX                PAL_LINE(GPIOD, 0U)
#define LINE_CAN1_TX                PAL_LINE(GPIOD, 1U)
#define LINE_USART5_RX              PAL_LINE(GPIOD, 2U)
#define LINE_GPIOD_3                PAL_LINE(GPIOD, 3U)
#define LINE_SD_CS               	PAL_LINE(GPIOD, 4U)
#define LINE_GPIOD_5                PAL_LINE(GPIOD, 5U)
#define LINE_SPI3_MOSI              PAL_LINE(GPIOD, 6U)
#define LINE_GPIOD_7                PAL_LINE(GPIOD, 7U)
#define LINE_USART3_RX              PAL_LINE(GPIOD, 8U)
#define LINE_USART3_TX              PAL_LINE(GPIOD, 9U)
#define LINE_GPIOD_10               PAL_LINE(GPIOD, 10U)
#define LINE_GPIOD_11               PAL_LINE(GPIOD, 11U)
#define LINE_I2C4_SCL               PAL_LINE(GPIOD, 12U)
#define LINE_I2C4_SDA               PAL_LINE(GPIOD, 13U)
#define LINE_GPIOD_14               PAL_LINE(GPIOD, 14U)
#define LINE_GPIOD_15               PAL_LINE(GPIOD, 15U)
#define LINE_SAT_NTW_AV             PAL_LINE(GPIOE, 0U)
#define LINE_SAT_RING_SGN           PAL_LINE(GPIOE, 1U)
#define LINE_RAM_CS1                PAL_LINE(GPIOE, 2U)
#define LINE_RAM_CS2                PAL_LINE(GPIOE, 3U)
#define LINE_NEO_CS                 PAL_LINE(GPIOE, 4U)
#define LINE_GPIOE_5                PAL_LINE(GPIOE, 5U)
#define LINE_GPIOE_6                PAL_LINE(GPIOE, 6U)
#define LINE_GPIOE_7                PAL_LINE(GPIOE, 7U)
#define LINE_GPIOE_8                PAL_LINE(GPIOE, 8U)
#define LINE_GPIOE_9                PAL_LINE(GPIOE, 9U)
#define LINE_GPIOE_10               PAL_LINE(GPIOE, 10U)
#define LINE_GPIOE_11               PAL_LINE(GPIOE, 11U)
#define LINE_GPIOE_12               PAL_LINE(GPIOE, 12U)
#define LINE_KEY1                   PAL_LINE(GPIOE, 13U)
#define LINE_KEY2                   PAL_LINE(GPIOE, 14U)
#define LINE_KEY3                   PAL_LINE(GPIOE, 15U)
#define LINE_I2C2_SDA               PAL_LINE(GPIOF, 0U)
#define LINE_I2C2_SCL               PAL_LINE(GPIOF, 1U)
#define LINE_GPIOF_2                PAL_LINE(GPIOF, 2U)
#define LINE_ADC3_IN9               PAL_LINE(GPIOF, 3U)
#define LINE_ADC3_IN14              PAL_LINE(GPIOF, 4U)
#define LINE_ADC3_IN15              PAL_LINE(GPIOF, 5U)
#define LINE_GPIOF_6              PAL_LINE(GPIOF, 6U)
#define LINE_USART7_TX              PAL_LINE(GPIOF, 7U)
#define LINE_RF_868_SLEEP           PAL_LINE(GPIOF, 8U)
#define LINE_RF_868_RST             PAL_LINE(GPIOF, 9U)
#define LINE_GPIOF_10               PAL_LINE(GPIOF, 10U)
#define LINE_GPIOF_11               PAL_LINE(GPIOF, 11U)
#define LINE_GPIOF_12               PAL_LINE(GPIOF, 12U)
#define LINE_GPIOF_13               PAL_LINE(GPIOF, 13U)
#define LINE_GPIOF_14               PAL_LINE(GPIOF, 14U)
#define LINE_GPIOF_15               PAL_LINE(GPIOF, 15U)
#define LINE_ZIO_D65                PAL_LINE(GPIOG, 0U)
#define LINE_ZIO_D64                PAL_LINE(GPIOG, 1U)
#define LINE_ZIO_D49                PAL_LINE(GPIOG, 2U)
#define LINE_ZIO_D50                PAL_LINE(GPIOG, 3U)
#define LINE_USB_GPIO_OUT           PAL_LINE(GPIOG, 6U)
#define LINE_USB_GPIO_IN            PAL_LINE(GPIOG, 7U)
#define LINE_ARD_D0                 PAL_LINE(GPIOG, 9U)
#define LINE_USART6_RX              PAL_LINE(GPIOG, 9U)
#define LINE_RMII_TX_EN             PAL_LINE(GPIOG, 11U)
#define LINE_RMII_TXD0              PAL_LINE(GPIOG, 13U)
#define LINE_ARD_D1                 PAL_LINE(GPIOG, 14U)
#define LINE_USART6_TX              PAL_LINE(GPIOG, 14U)
#define LINE_OSC_IN                 PAL_LINE(GPIOH, 0U)
#define LINE_OSC_OUT                PAL_LINE(GPIOH, 1U)

#define LINE_SUSART1_RX				PAL_LINE(GPIOC, 9U)
#define LINE_SUSART1_TX				PAL_LINE(GPIOA, 8U)
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
 * PA0  - USART4_RX                 (input pullup).
 * PA1  - USART4_TX                 (alternate 8).
 * PA2  - USART2_TX                 (alternate 7).
 * PA3  - USART2_RX                 (alternate 7).
 * PA4  - RF_868_CS                 (output pushpull maximum).
 * PA5  - SPI1_SCK                  (alternate 5).
 * PA6  - SPI1_MISO                 (alternate 5).
 * PA7  - SPI1_MOSI                 (alternate 5).
 * PA8  - I2C3_SCL                  (alternate 4).
 * PA9  - USART1_TX                 (alternate 7).
 * PA10 - USART1_RX                 (alternate 7).
 * PA11 - USB_DM                    (alternate 10).
 * PA12 - USB_DP                    (alternate 10).
 * PA13 - SWDIO                     (alternate 0).
 * PA14 - SWCLK                     (alternate 0).
 * PA15 - USART7_TX                 (alternate 12).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_USART4_RX) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_USART4_TX) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_USART2_TX) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_USART2_RX) |  \
                                     PIN_MODE_OUTPUT(GPIOA_RF_868_CS) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_SCK) |   \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_MISO) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_MOSI) |  \
                                     PIN_MODE_OUTPUT(GPIOA_I2C3_SCL) |   \
                                     PIN_MODE_ALTERNATE(GPIOA_USART1_TX) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_USART1_RX) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_USB_DM) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_USB_DP) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_GPIOA_15))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_USART4_RX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USART4_TX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USART2_TX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USART2_RX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_RF_868_CS) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_SCK) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_MISO) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_MOSI) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_I2C3_SCL) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USART1_TX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USART1_RX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_DM) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_DP) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_GPIOA_15))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_HIGH(GPIOA_USART4_RX) |     \
                                     PIN_OSPEED_HIGH(GPIOA_USART4_TX) |     \
                                     PIN_OSPEED_HIGH(GPIOA_USART2_TX) |     \
                                     PIN_OSPEED_HIGH(GPIOA_USART2_RX) |     \
                                     PIN_OSPEED_HIGH(GPIOA_RF_868_CS) |     \
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_SCK) |      \
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_MISO) |     \
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_MOSI) |     \
                                     PIN_OSPEED_HIGH(GPIOA_I2C3_SCL) |      \
                                     PIN_OSPEED_HIGH(GPIOA_USART1_TX) |     \
                                     PIN_OSPEED_HIGH(GPIOA_USART1_RX) |     \
                                     PIN_OSPEED_HIGH(GPIOA_USB_DM) |        \
                                     PIN_OSPEED_HIGH(GPIOA_USB_DP) |        \
                                     PIN_OSPEED_HIGH(GPIOA_SWDIO) |         \
                                     PIN_OSPEED_HIGH(GPIOA_SWCLK) |         \
                                     PIN_OSPEED_HIGH(GPIOA_GPIOA_15))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_PULLUP(GPIOA_USART4_RX) |    \
                                     PIN_PUPDR_PULLUP(GPIOA_USART4_TX) |    \
                                     PIN_PUPDR_PULLUP(GPIOA_USART2_TX) |    \
                                     PIN_PUPDR_PULLUP(GPIOA_USART2_RX) |    \
                                     PIN_PUPDR_PULLUP(GPIOA_RF_868_CS) |    \
                                     PIN_PUPDR_PULLUP(GPIOA_SPI1_SCK) |     \
                                     PIN_PUPDR_PULLUP(GPIOA_SPI1_MISO) |    \
                                     PIN_PUPDR_PULLUP(GPIOA_SPI1_MOSI) |    \
                                     PIN_PUPDR_PULLUP(GPIOA_I2C3_SCL) |     \
                                     PIN_PUPDR_PULLUP(GPIOA_USART1_TX) |    \
                                     PIN_PUPDR_PULLUP(GPIOA_USART1_RX) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_USB_DM) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_USB_DP) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_SWDIO) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_SWCLK) |      \
                                     PIN_PUPDR_PULLUP(GPIOA_GPIOA_15))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_USART4_RX) |        \
                                     PIN_ODR_HIGH(GPIOA_USART4_TX) |        \
                                     PIN_ODR_HIGH(GPIOA_USART2_TX) |        \
                                     PIN_ODR_HIGH(GPIOA_USART2_RX) |        \
                                     PIN_ODR_HIGH(GPIOA_RF_868_CS) |        \
                                     PIN_ODR_HIGH(GPIOA_SPI1_SCK) |         \
                                     PIN_ODR_HIGH(GPIOA_SPI1_MISO) |        \
                                     PIN_ODR_HIGH(GPIOA_SPI1_MOSI) |        \
                                     PIN_ODR_HIGH(GPIOA_I2C3_SCL) |         \
                                     PIN_ODR_HIGH(GPIOA_USART1_TX) |        \
                                     PIN_ODR_HIGH(GPIOA_USART1_RX) |        \
                                     PIN_ODR_HIGH(GPIOA_USB_DM) |           \
                                     PIN_ODR_HIGH(GPIOA_USB_DP) |           \
                                     PIN_ODR_HIGH(GPIOA_SWDIO) |            \
                                     PIN_ODR_HIGH(GPIOA_SWCLK) |            \
                                     PIN_ODR_HIGH(GPIOA_GPIOA_15))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_USART4_RX, 8U) |     \
                                     PIN_AFIO_AF(GPIOA_USART4_TX, 8U) |     \
                                     PIN_AFIO_AF(GPIOA_USART2_TX, 7U) |     \
                                     PIN_AFIO_AF(GPIOA_USART2_RX, 7U) |     \
                                     PIN_AFIO_AF(GPIOA_RF_868_CS, 0U) |     \
                                     PIN_AFIO_AF(GPIOA_SPI1_SCK, 5U) |      \
                                     PIN_AFIO_AF(GPIOA_SPI1_MISO, 5U) |     \
                                     PIN_AFIO_AF(GPIOA_SPI1_MOSI, 5U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_I2C3_SCL, 4U) |      \
                                     PIN_AFIO_AF(GPIOA_USART1_TX, 7U) |     \
                                     PIN_AFIO_AF(GPIOA_USART1_RX, 7U) |     \
                                     PIN_AFIO_AF(GPIOA_USB_DM, 10U) |       \
                                     PIN_AFIO_AF(GPIOA_USB_DP, 10U) |       \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_GPIOA_15, 0U))

/*
 * GPIOB setup:
 *
 * PB0  - ORANGE_LED                (output pushpull maximum).
 * PB1  - GREEN_LED                 (output pushpull maximum).
 * PB2  - RED_LED                   (output pushpull maximum).
 * PB3  - USART7_RX                 (alternate 12).
 * PB4  - BUZZER                    (alternate 2).  //CHANGE TO UART7_TX!!!!
 * PB5  - RTK_STAT                  (input pullup).
 * PB6  - I2C1_SCL                  (alternate 4).m
 * PB7  - I2C1_SDA                  (alternate 4).
 * PB8  - RF_2_4_CS                 (output pushpull maximum).
 * PB9  - M8T_RST                   (output pushpull maximum).
 * PB10 - SPI2_CLK                  (alternate 5).
 * PB11 - GPIOB_11                  (input pullup).
 * PB12 - GPIOB_12                  (input pullup).
 * PB13 - GPIOB_13                  (input pullup).
 * PB14 - GPIOB_14                  (input floating).
 * PB15 - GPIOB_15                  (input pullup).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_OUTPUT(GPIOB_ORANGE_LED) |    \
                                     PIN_MODE_OUTPUT(GPIOB_GREEN_LED) |     \
                                     PIN_MODE_OUTPUT(GPIOB_RED_LED) |       \
                                     PIN_MODE_ALTERNATE(GPIOB_GPIOB_3) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_BUZZER) |     \
                                     PIN_MODE_INPUT(GPIOB_RTK_STAT) |       \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SCL) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SDA) |   \
                                     PIN_MODE_OUTPUT(GPIOB_RF_2_4_CS) |     \
                                     PIN_MODE_OUTPUT(GPIOB_M8T_RST) |       \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI2_CLK) |   \
                                     PIN_MODE_INPUT(GPIOB_GPIOB_11) |       \
                                     PIN_MODE_INPUT(GPIOB_GPIOB_12) |       \
                                     PIN_MODE_INPUT(GPIOB_GPIOB_13) |       \
                                     PIN_MODE_INPUT(GPIOB_GPIOB_14) |       \
                                     PIN_MODE_INPUT(GPIOB_GPIOB_15))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_ORANGE_LED) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_GREEN_LED) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_RED_LED) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_GPIOB_3) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_BUZZER) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_RTK_STAT) |   \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SCL) |   \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SDA) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_RF_2_4_CS) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_M8T_RST) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI2_CLK) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_GPIOB_11) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_GPIOB_12) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_GPIOB_13) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_GPIOB_14) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_GPIOB_15))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_HIGH(GPIOB_ORANGE_LED) |    \
                                     PIN_OSPEED_HIGH(GPIOB_GREEN_LED) |     \
                                     PIN_OSPEED_HIGH(GPIOB_RED_LED) |       \
                                     PIN_OSPEED_HIGH(GPIOB_GPIOB_3) |     \
                                     PIN_OSPEED_HIGH(GPIOB_BUZZER) |        \
                                     PIN_OSPEED_HIGH(GPIOB_RTK_STAT) |      \
                                     PIN_OSPEED_HIGH(GPIOB_I2C1_SCL) |      \
                                     PIN_OSPEED_HIGH(GPIOB_I2C1_SDA) |      \
                                     PIN_OSPEED_HIGH(GPIOB_RF_2_4_CS) |     \
                                     PIN_OSPEED_HIGH(GPIOB_M8T_RST) |       \
                                     PIN_OSPEED_HIGH(GPIOB_SPI2_CLK) |      \
                                     PIN_OSPEED_HIGH(GPIOB_GPIOB_11) |      \
                                     PIN_OSPEED_HIGH(GPIOB_GPIOB_12) |      \
                                     PIN_OSPEED_HIGH(GPIOB_GPIOB_13) |      \
                                     PIN_OSPEED_HIGH(GPIOB_GPIOB_14) |      \
                                     PIN_OSPEED_HIGH(GPIOB_GPIOB_15))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLUP(GPIOB_ORANGE_LED) |   \
                                     PIN_PUPDR_PULLUP(GPIOB_GREEN_LED) |    \
                                     PIN_PUPDR_PULLUP(GPIOB_RED_LED) |      \
                                     PIN_PUPDR_PULLUP(GPIOB_GPIOB_3) |    \
                                     PIN_PUPDR_PULLUP(GPIOB_BUZZER) |       \
                                     PIN_PUPDR_PULLUP(GPIOB_RTK_STAT) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_I2C1_SCL) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_I2C1_SDA) |     \
                                     PIN_PUPDR_PULLUP(GPIOB_RF_2_4_CS) |    \
                                     PIN_PUPDR_PULLUP(GPIOB_M8T_RST) |      \
                                     PIN_PUPDR_PULLUP(GPIOB_SPI2_CLK) |     \
                                     PIN_PUPDR_PULLUP(GPIOB_GPIOB_11) |     \
                                     PIN_PUPDR_PULLUP(GPIOB_GPIOB_12) |     \
                                     PIN_PUPDR_PULLUP(GPIOB_GPIOB_13) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_GPIOB_14) |   \
                                     PIN_PUPDR_PULLUP(GPIOB_GPIOB_15))
#define VAL_GPIOB_ODR               (PIN_ODR_LOW(GPIOB_ORANGE_LED) |        \
                                     PIN_ODR_LOW(GPIOB_GREEN_LED) |         \
                                     PIN_ODR_LOW(GPIOB_RED_LED) |           \
                                     PIN_ODR_HIGH(GPIOB_GPIOB_3) |        \
                                     PIN_ODR_HIGH(GPIOB_BUZZER) |           \
                                     PIN_ODR_HIGH(GPIOB_RTK_STAT) |         \
                                     PIN_ODR_HIGH(GPIOB_I2C1_SCL) |         \
                                     PIN_ODR_HIGH(GPIOB_I2C1_SDA) |         \
                                     PIN_ODR_HIGH(GPIOB_RF_2_4_CS) |        \
                                     PIN_ODR_HIGH(GPIOB_M8T_RST) |          \
                                     PIN_ODR_HIGH(GPIOB_SPI2_CLK) |         \
                                     PIN_ODR_HIGH(GPIOB_GPIOB_11) |         \
                                     PIN_ODR_HIGH(GPIOB_GPIOB_12) |         \
                                     PIN_ODR_HIGH(GPIOB_GPIOB_13) |         \
                                     PIN_ODR_LOW(GPIOB_GPIOB_14) |          \
                                     PIN_ODR_HIGH(GPIOB_GPIOB_15))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_ORANGE_LED, 0U) |    \
                                     PIN_AFIO_AF(GPIOB_GREEN_LED, 0U) |     \
                                     PIN_AFIO_AF(GPIOB_RED_LED, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_GPIOB_3, 0U) |    \
                                     PIN_AFIO_AF(GPIOB_BUZZER, 2U) |        \
                                     PIN_AFIO_AF(GPIOB_RTK_STAT, 0U) |      \
                                     PIN_AFIO_AF(GPIOB_I2C1_SCL, 4U) |      \
                                     PIN_AFIO_AF(GPIOB_I2C1_SDA, 4U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_RF_2_4_CS, 0U) |     \
                                     PIN_AFIO_AF(GPIOB_M8T_RST, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_SPI2_CLK, 5U) |      \
                                     PIN_AFIO_AF(GPIOB_GPIOB_11, 0U) |      \
                                     PIN_AFIO_AF(GPIOB_GPIOB_12, 0U) |      \
                                     PIN_AFIO_AF(GPIOB_GPIOB_13, 0U) |      \
                                     PIN_AFIO_AF(GPIOB_GPIOB_14, 0U) |      \
                                     PIN_AFIO_AF(GPIOB_GPIOB_15, 0U))

/*
 * GPIOC setup:
 *
 * PC0  - MPU_CS                    (output pushpull maximum).
 * PC1  - SPI2_MOSI                 (alternate 5).
 * PC2  - SPI2_MISO                 (alternate 5).
 * PC3  - GPIOC_3                   (input pullup).
 * PC4  - MCU_CS                    (output pushpull maximum).
 * PC5  - NINA_CTS           		(output_pushpull).
 * PC6  - GPIOC_6                   (input pullup).
 * PC7  - GPIOC_7                   (input pullup).
 * PC8  - GPIOC_8                     (input_pullup).
 * PC9  - I2C3_SDA                  (alternate 4).
 * PC10 - SPI3_CLK                  (alternate 6).
 * PC11 - SPI3_MISO                 (alternate 6).
 * PC12 - USART5_TX                 (alternate 8).
 * PC13 - GPIOC_13                  (input floating).
 * PC14 - OSC32_IN                  (input floating).
 * PC15 - OSC32_OUT                 (input floating).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_OUTPUT(GPIOC_MPU_CS) |        \
                                     PIN_MODE_ALTERNATE(GPIOC_SPI2_MOSI) |  \
                                     PIN_MODE_ALTERNATE(GPIOC_SPI2_MISO) |  \
                                     PIN_MODE_INPUT(GPIOC_GPIOC_3) |        \
                                     PIN_MODE_OUTPUT(GPIOC_MCU_CS) |        \
                                     PIN_MODE_OUTPUT(GPIOC_NINA_CTS) |\
                                     PIN_MODE_INPUT(GPIOC_GPIOC_6) |        \
                                     PIN_MODE_INPUT(GPIOC_GPIOC_7) |        \
                                     PIN_MODE_INPUT(GPIOC_GPIOC_8) |         \
                                     PIN_MODE_INPUT(GPIOC_I2C3_SDA) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_SPI3_CLK) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_SPI3_MISO) |  \
                                     PIN_MODE_ALTERNATE(GPIOC_USART5_TX) |  \
                                     PIN_MODE_INPUT(GPIOC_GPIOC_13) |       \
                                     PIN_MODE_INPUT(GPIOC_OSC32_IN) |       \
                                     PIN_MODE_INPUT(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_MPU_CS) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SPI2_MOSI) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SPI2_MISO) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_GPIOC_3) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOC_MCU_CS) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_NINA_CTS) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_GPIOC_6) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOC_GPIOC_7) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOC_GPIOC_8) |      \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_I2C3_SDA) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SPI3_CLK) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SPI3_MISO) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_USART5_TX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_GPIOC_13) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_HIGH(GPIOC_MPU_CS) |        \
                                     PIN_OSPEED_HIGH(GPIOC_SPI2_MOSI) |     \
                                     PIN_OSPEED_HIGH(GPIOC_SPI2_MISO) |     \
                                     PIN_OSPEED_HIGH(GPIOC_GPIOC_3) |       \
                                     PIN_OSPEED_HIGH(GPIOC_MCU_CS) |        \
                                     PIN_OSPEED_HIGH(GPIOC_NINA_CTS) |\
                                     PIN_OSPEED_HIGH(GPIOC_GPIOC_6) |       \
                                     PIN_OSPEED_HIGH(GPIOC_GPIOC_7) |       \
                                     PIN_OSPEED_HIGH(GPIOC_GPIOC_8) |         \
                                     PIN_OSPEED_HIGH(GPIOC_I2C3_SDA) |      \
                                     PIN_OSPEED_HIGH(GPIOC_SPI3_CLK) |      \
                                     PIN_OSPEED_HIGH(GPIOC_SPI3_MISO) |     \
                                     PIN_OSPEED_HIGH(GPIOC_USART5_TX) |     \
                                     PIN_OSPEED_HIGH(GPIOC_GPIOC_13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOC_OSC32_IN) |   \
                                     PIN_OSPEED_VERYLOW(GPIOC_OSC32_OUT))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_PULLUP(GPIOC_MPU_CS) |       \
                                     PIN_PUPDR_PULLUP(GPIOC_SPI2_MOSI) |    \
                                     PIN_PUPDR_PULLUP(GPIOC_SPI2_MISO) |    \
                                     PIN_PUPDR_PULLUP(GPIOC_GPIOC_3) |      \
                                     PIN_PUPDR_PULLUP(GPIOC_MCU_CS) |       \
                                     PIN_PUPDR_PULLUP(GPIOC_NINA_CTS) |\
                                     PIN_PUPDR_PULLUP(GPIOC_GPIOC_6) |      \
                                     PIN_PUPDR_PULLUP(GPIOC_GPIOC_7) |      \
                                     PIN_PUPDR_PULLUP(GPIOC_GPIOC_8) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_I2C3_SDA) |     \
                                     PIN_PUPDR_PULLUP(GPIOC_SPI3_CLK) |     \
                                     PIN_PUPDR_PULLUP(GPIOC_SPI3_MISO) |    \
                                     PIN_PUPDR_PULLUP(GPIOC_USART5_TX) |    \
                                     PIN_PUPDR_FLOATING(GPIOC_GPIOC_13) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_IN) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_MPU_CS) |           \
                                     PIN_ODR_HIGH(GPIOC_SPI2_MOSI) |        \
                                     PIN_ODR_HIGH(GPIOC_SPI2_MISO) |        \
                                     PIN_ODR_HIGH(GPIOC_GPIOC_3) |          \
                                     PIN_ODR_HIGH(GPIOC_MCU_CS) |           \
                                     PIN_ODR_HIGH(GPIOC_NINA_CTS) |  \
                                     PIN_ODR_HIGH(GPIOC_GPIOC_6) |          \
                                     PIN_ODR_HIGH(GPIOC_GPIOC_7) |          \
                                     PIN_ODR_HIGH(GPIOC_GPIOC_8) |            \
                                     PIN_ODR_HIGH(GPIOC_I2C3_SDA) |         \
                                     PIN_ODR_HIGH(GPIOC_SPI3_CLK) |         \
                                     PIN_ODR_HIGH(GPIOC_SPI3_MISO) |        \
                                     PIN_ODR_HIGH(GPIOC_USART5_TX) |        \
                                     PIN_ODR_HIGH(GPIOC_GPIOC_13) |         \
                                     PIN_ODR_HIGH(GPIOC_OSC32_IN) |         \
                                     PIN_ODR_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_MPU_CS, 0U) |        \
                                     PIN_AFIO_AF(GPIOC_SPI2_MOSI, 5U) |     \
                                     PIN_AFIO_AF(GPIOC_SPI2_MISO, 5U) |     \
                                     PIN_AFIO_AF(GPIOC_GPIOC_3, 0U) |       \
                                     PIN_AFIO_AF(GPIOC_MCU_CS, 0U) |        \
                                     PIN_AFIO_AF(GPIOC_NINA_CTS, 0U) |\
                                     PIN_AFIO_AF(GPIOC_GPIOC_6, 0U) |       \
                                     PIN_AFIO_AF(GPIOC_GPIOC_7, 0U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_GPIOC_8, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_I2C3_SDA, 4U) |      \
                                     PIN_AFIO_AF(GPIOC_SPI3_CLK, 6U) |      \
                                     PIN_AFIO_AF(GPIOC_SPI3_MISO, 6U) |     \
                                     PIN_AFIO_AF(GPIOC_USART5_TX, 8U) |     \
                                     PIN_AFIO_AF(GPIOC_GPIOC_13, 0U) |      \
                                     PIN_AFIO_AF(GPIOC_OSC32_IN, 0U) |      \
                                     PIN_AFIO_AF(GPIOC_OSC32_OUT, 0U))

/*
 * GPIOD setup:
 *
 * PD0  - CAN1_RX                   (alternate 9).	//!!!! While previous version of pcb using this pins to parce wind sensor
 * PD1  - CAN1_TX                   (alternate 9).	//	on UART4
 * PD2  - USART5_RX                 (alternate 8).
 * PD3  - GPIOD_3                   (input pullup).
 * PD4  - SD_CS		                (output).
 * PD5  - GPIOD_5                   (input pullup).
 * PD6  - SPI3_MOSI                 (alternate 5).
 * PD7  - GPIOD_7                   (input pullup).
 * PD8  - USART3_RX                 (alternate 7).
 * PD9  - USART3_TX                 (alternate 7).
 * PD10 - GPIOD_10                  (input pullup).
 * PD11 - GPIOD_11                  (input pullup).
 * PD12 - I2C4_SCL                  (alternate 4).
 * PD13 - I2C4_SDA                  (alternate 4).
 * PD14 - GPIOD_14                  (input pullup).
 * PD15 - GPIOD_15                  (input pullup).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_ALTERNATE(GPIOD_CAN1_RX) |    \
                                     PIN_MODE_ALTERNATE(GPIOD_CAN1_TX) |    \
                                     PIN_MODE_ALTERNATE(GPIOD_USART5_RX) |  \
                                     PIN_MODE_INPUT(GPIOD_GPIOD_3) |        \
                                     PIN_MODE_OUTPUT(GPIOD_SD_CS) |        \
                                     PIN_MODE_INPUT(GPIOD_GPIOD_5) |        \
                                     PIN_MODE_ALTERNATE(GPIOD_SPI3_MOSI) |  \
                                     PIN_MODE_INPUT(GPIOD_GPIOD_7) |        \
                                     PIN_MODE_ALTERNATE(GPIOD_USART3_RX) |  \
                                     PIN_MODE_ALTERNATE(GPIOD_USART3_TX) |  \
                                     PIN_MODE_INPUT(GPIOD_GPIOD_10) |       \
                                     PIN_MODE_INPUT(GPIOD_GPIOD_11) |       \
                                     PIN_MODE_ALTERNATE(GPIOD_I2C4_SCL) |   \
                                     PIN_MODE_ALTERNATE(GPIOD_I2C4_SDA) |   \
                                     PIN_MODE_INPUT(GPIOD_GPIOD_14) |       \
                                     PIN_MODE_INPUT(GPIOD_GPIOD_15))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_CAN1_RX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_CAN1_TX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_USART5_RX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_GPIOD_3) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_SD_CS) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_GPIOD_5) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_SPI3_MOSI) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_GPIOD_7) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_USART3_RX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_USART3_TX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_GPIOD_10) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_GPIOD_11) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_I2C4_SCL) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_I2C4_SDA) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_GPIOD_14) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_GPIOD_15))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_HIGH(GPIOD_CAN1_RX) |       \
                                     PIN_OSPEED_HIGH(GPIOD_CAN1_TX) |       \
                                     PIN_OSPEED_HIGH(GPIOD_USART5_RX) |     \
                                     PIN_OSPEED_HIGH(GPIOD_GPIOD_3) |       \
                                     PIN_OSPEED_HIGH(GPIOD_SD_CS) |       \
                                     PIN_OSPEED_HIGH(GPIOD_GPIOD_5) |       \
                                     PIN_OSPEED_HIGH(GPIOD_SPI3_MOSI) |     \
                                     PIN_OSPEED_HIGH(GPIOD_GPIOD_7) |       \
                                     PIN_OSPEED_HIGH(GPIOD_USART3_RX) |     \
                                     PIN_OSPEED_HIGH(GPIOD_USART3_TX) |     \
                                     PIN_OSPEED_VERYLOW(GPIOD_GPIOD_10) |   \
                                     PIN_OSPEED_HIGH(GPIOD_GPIOD_11) |      \
                                     PIN_OSPEED_HIGH(GPIOD_I2C4_SCL) |      \
                                     PIN_OSPEED_HIGH(GPIOD_I2C4_SDA) |      \
                                     PIN_OSPEED_HIGH(GPIOD_GPIOD_14) |      \
                                     PIN_OSPEED_HIGH(GPIOD_GPIOD_15))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_PULLUP(GPIOD_CAN1_RX) |      \
                                     PIN_PUPDR_PULLUP(GPIOD_CAN1_TX) |      \
                                     PIN_PUPDR_PULLUP(GPIOD_USART5_RX) |    \
                                     PIN_PUPDR_PULLUP(GPIOD_GPIOD_3) |      \
                                     PIN_PUPDR_PULLUP(GPIOD_SD_CS) |      \
                                     PIN_PUPDR_PULLUP(GPIOD_GPIOD_5) |      \
                                     PIN_PUPDR_PULLUP(GPIOD_SPI3_MOSI) |    \
                                     PIN_PUPDR_PULLUP(GPIOD_GPIOD_7) |      \
                                     PIN_PUPDR_FLOATING(GPIOD_USART3_RX) |  \
                                     PIN_PUPDR_FLOATING(GPIOD_USART3_TX) |  \
                                     PIN_PUPDR_PULLUP(GPIOD_GPIOD_10) |     \
                                     PIN_PUPDR_PULLUP(GPIOD_GPIOD_11) |     \
                                     PIN_PUPDR_PULLUP(GPIOD_I2C4_SCL) |     \
                                     PIN_PUPDR_PULLUP(GPIOD_I2C4_SDA) |     \
                                     PIN_PUPDR_PULLUP(GPIOD_GPIOD_14) |     \
                                     PIN_PUPDR_PULLUP(GPIOD_GPIOD_15))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_CAN1_RX) |          \
                                     PIN_ODR_HIGH(GPIOD_CAN1_TX) |          \
                                     PIN_ODR_HIGH(GPIOD_USART5_RX) |        \
                                     PIN_ODR_HIGH(GPIOD_GPIOD_3) |          \
                                     PIN_ODR_HIGH(GPIOD_SD_CS) |          \
                                     PIN_ODR_HIGH(GPIOD_GPIOD_5) |          \
                                     PIN_ODR_HIGH(GPIOD_SPI3_MOSI) |        \
                                     PIN_ODR_HIGH(GPIOD_GPIOD_7) |          \
                                     PIN_ODR_HIGH(GPIOD_USART3_RX) |        \
                                     PIN_ODR_HIGH(GPIOD_USART3_TX) |        \
                                     PIN_ODR_HIGH(GPIOD_GPIOD_10) |         \
                                     PIN_ODR_HIGH(GPIOD_GPIOD_11) |         \
                                     PIN_ODR_HIGH(GPIOD_I2C4_SCL) |         \
                                     PIN_ODR_HIGH(GPIOD_I2C4_SDA) |         \
                                     PIN_ODR_HIGH(GPIOD_GPIOD_14) |         \
                                     PIN_ODR_HIGH(GPIOD_GPIOD_15))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_CAN1_RX, 9U) |       \
                                     PIN_AFIO_AF(GPIOD_CAN1_TX, 9U) |       \
                                     PIN_AFIO_AF(GPIOD_USART5_RX, 8U) |     \
                                     PIN_AFIO_AF(GPIOD_GPIOD_3, 0U) |       \
                                     PIN_AFIO_AF(GPIOD_SD_CS, 0U) |       \
                                     PIN_AFIO_AF(GPIOD_GPIOD_5, 0U) |       \
                                     PIN_AFIO_AF(GPIOD_SPI3_MOSI, 5U) |     \
                                     PIN_AFIO_AF(GPIOD_GPIOD_7, 0U))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_USART3_RX, 7U) |     \
                                     PIN_AFIO_AF(GPIOD_USART3_TX, 7U) |     \
                                     PIN_AFIO_AF(GPIOD_GPIOD_10, 0U) |      \
                                     PIN_AFIO_AF(GPIOD_GPIOD_11, 0U) |      \
                                     PIN_AFIO_AF(GPIOD_I2C4_SCL, 4U) |      \
                                     PIN_AFIO_AF(GPIOD_I2C4_SDA, 4U) |      \
                                     PIN_AFIO_AF(GPIOD_GPIOD_14, 0U) |      \
                                     PIN_AFIO_AF(GPIOD_GPIOD_15, 0U))

/*
 * GPIOE setup:
 *
 * PE0  - GPIOE_UART8_RX            (alternate 8).
 * PE1  - GPIOE_UART8_TX            (alternate 8).
 * PE2  - SPI4_SCK                  (alternate 5).
 * PE3  - RAM_CS2                   (output pushpull maximum).
 * PE4  - M8T_CS	                (output pushpull maximum).
 * PE5  - SPI4_MISO                 (alternate 5).
 * PE6  - SPI4_MOSI                 (alternate 5).
 * PE7  - GPIOE_7                   (input pullup).
 * PE8  - GPIOE_8                   (input pullup).
 * PE9  - GPIOE_9                   (input pullup).
 * PE10 - GPIOE_10                  (input pullup).
 * PE11 - GPIOE_11                  (input pullup).
 * PE12 - GPIOE_12                  (input pullup).
 * PE13 - KEY1                      (input pullup).
 * PE14 - KEY2                      (input pullup).
 * PE15 - KEY3                      (input pullup).
 */
#define VAL_GPIOE_MODER             (PIN_MODE_ALTERNATE(GPIOE_UART8_RX) |   \
                                     PIN_MODE_ALTERNATE(GPIOE_UART8_TX) |   \
                                     PIN_MODE_ALTERNATE(GPIOE_SPI4_SCK) |   \
                                     PIN_MODE_OUTPUT(GPIOE_RAM_CS2) |       \
                                     PIN_MODE_OUTPUT(GPIOE_M8T_CS) |        \
                                     PIN_MODE_ALTERNATE(GPIOE_SPI4_MISO) |  \
                                     PIN_MODE_ALTERNATE(GPIOE_SPI4_MOSI) |  \
                                     PIN_MODE_INPUT(GPIOE_GPIOE_7) |        \
                                     PIN_MODE_INPUT(GPIOE_GPIOE_8) |        \
                                     PIN_MODE_INPUT(GPIOE_GPIOE_9) |        \
                                     PIN_MODE_INPUT(GPIOE_GPIOE_10) |       \
                                     PIN_MODE_INPUT(GPIOE_GPIOE_11) |       \
                                     PIN_MODE_INPUT(GPIOE_GPIOE_12) |       \
                                     PIN_MODE_INPUT(GPIOE_KEY1) |           \
                                     PIN_MODE_INPUT(GPIOE_KEY2) |           \
                                     PIN_MODE_INPUT(GPIOE_KEY3))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_UART8_RX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_UART8_TX) |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_SPI4_SCK) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_RAM_CS2) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_M8T_CS) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SPI4_MISO) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SPI4_MOSI) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_GPIOE_7) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_GPIOE_8) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_GPIOE_9) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_GPIOE_10) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_GPIOE_11) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_GPIOE_12) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_KEY1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_KEY2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_KEY3))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_HIGH(GPIOE_UART8_RX) |    \
                                     PIN_OSPEED_HIGH(GPIOE_UART8_TX) |\
                                     PIN_OSPEED_HIGH(GPIOE_SPI4_SCK) |       \
                                     PIN_OSPEED_HIGH(GPIOE_RAM_CS2) |       \
                                     PIN_OSPEED_HIGH(GPIOE_M8T_CS) |       \
                                     PIN_OSPEED_HIGH(GPIOE_SPI4_MISO) |       \
                                     PIN_OSPEED_HIGH(GPIOE_SPI4_MOSI) |        \
                                     PIN_OSPEED_HIGH(GPIOE_GPIOE_7) |       \
                                     PIN_OSPEED_HIGH(GPIOE_GPIOE_8) |       \
                                     PIN_OSPEED_HIGH(GPIOE_GPIOE_9) |       \
                                     PIN_OSPEED_HIGH(GPIOE_GPIOE_10) |      \
                                     PIN_OSPEED_HIGH(GPIOE_GPIOE_11) |      \
                                     PIN_OSPEED_HIGH(GPIOE_GPIOE_12) |      \
                                     PIN_OSPEED_HIGH(GPIOE_KEY1) |          \
                                     PIN_OSPEED_VERYLOW(GPIOE_KEY2) |       \
                                     PIN_OSPEED_HIGH(GPIOE_KEY3))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_PULLUP(GPIOE_UART8_RX) |   \
                                     PIN_PUPDR_PULLUP(GPIOE_UART8_TX) | \
                                     PIN_PUPDR_PULLUP(GPIOE_SPI4_SCK) |      \
                                     PIN_PUPDR_PULLUP(GPIOE_RAM_CS2) |      \
                                     PIN_PUPDR_PULLUP(GPIOE_M8T_CS) |      \
                                     PIN_PUPDR_PULLUP(GPIOE_SPI4_MISO) |      \
                                     PIN_PUPDR_PULLUP(GPIOE_SPI4_MOSI) |       \
                                     PIN_PUPDR_PULLUP(GPIOE_GPIOE_7) |      \
                                     PIN_PUPDR_PULLUP(GPIOE_GPIOE_8) |      \
                                     PIN_PUPDR_PULLUP(GPIOE_GPIOE_9) |      \
                                     PIN_PUPDR_PULLUP(GPIOE_GPIOE_10) |     \
                                     PIN_PUPDR_PULLUP(GPIOE_GPIOE_11) |     \
                                     PIN_PUPDR_PULLUP(GPIOE_GPIOE_12) |     \
                                     PIN_PUPDR_PULLUP(GPIOE_KEY1) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_KEY2) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_KEY3))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_UART8_RX) |       \
                                     PIN_ODR_HIGH(GPIOE_UART8_TX) |     \
                                     PIN_ODR_HIGH(GPIOE_SPI4_SCK) |          \
                                     PIN_ODR_HIGH(GPIOE_RAM_CS2) |          \
                                     PIN_ODR_HIGH(GPIOE_M8T_CS) |          \
                                     PIN_ODR_HIGH(GPIOE_SPI4_MISO) |          \
                                     PIN_ODR_HIGH(GPIOE_SPI4_MOSI) |           \
                                     PIN_ODR_HIGH(GPIOE_GPIOE_7) |          \
                                     PIN_ODR_HIGH(GPIOE_GPIOE_8) |          \
                                     PIN_ODR_HIGH(GPIOE_GPIOE_9) |          \
                                     PIN_ODR_HIGH(GPIOE_GPIOE_10) |         \
                                     PIN_ODR_HIGH(GPIOE_GPIOE_11) |         \
                                     PIN_ODR_HIGH(GPIOE_GPIOE_12) |         \
                                     PIN_ODR_HIGH(GPIOE_KEY1) |             \
                                     PIN_ODR_HIGH(GPIOE_KEY2) |             \
                                     PIN_ODR_HIGH(GPIOE_KEY3))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_UART8_RX, 8U) |    \
                                     PIN_AFIO_AF(GPIOE_UART8_TX, 8U) |  \
                                     PIN_AFIO_AF(GPIOE_SPI4_SCK, 5U) |       \
                                     PIN_AFIO_AF(GPIOE_RAM_CS2, 0U) |       \
                                     PIN_AFIO_AF(GPIOE_M8T_CS, 0U) |       \
                                     PIN_AFIO_AF(GPIOE_SPI4_MISO, 5U) |       \
                                     PIN_AFIO_AF(GPIOE_SPI4_MOSI, 5U) |        \
                                     PIN_AFIO_AF(GPIOE_GPIOE_7, 0U))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_GPIOE_8, 0U) |       \
                                     PIN_AFIO_AF(GPIOE_GPIOE_9, 0U) |       \
                                     PIN_AFIO_AF(GPIOE_GPIOE_10, 0U) |      \
                                     PIN_AFIO_AF(GPIOE_GPIOE_11, 0U) |      \
                                     PIN_AFIO_AF(GPIOE_GPIOE_12, 0U) |      \
                                     PIN_AFIO_AF(GPIOE_KEY1, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_KEY2, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_KEY3, 0U))

/*
 * GPIOF setup:
 *
 * PF0  - GPIOF_I2C2_SDA            (alternate 4).
 * PF1  - GPIOF_I2C2_SCL            (alternate 4).
 * PF2  - GPIOF_RF_868_SPI_ATTN     (input pullup).
 * PF3  - ADC3_IN9                  (analog).
 * PF4  - ADC3_IN14                 (analog).
 * PF5  - ADC3_IN15                 (analog).
 * PF6  - USART7_RX                 (alternate 8).
 * PF7  - USART7_TX                 (alternate 8).
 * PF8  - RF_868_SLEEP              (output pushpull maximum).
 * PF9  - RF_868_RST                (output open drain).
 * PF10 - GPIOF_10                  (input pullup).
 * PF11 - GPIOF_11                  (input pullup).
 * PF12 - GPIOF_11                  (input pullup).
 * PF13 - GPIOF_13                  (input pullup).
 * PF14 - GPIOF_14                  (input pullup).
 * PF15 - GPIOF_15                  (input pullup).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_ALTERNATE(GPIOF_I2C2_SDA) |        \
                                     PIN_MODE_ALTERNATE(GPIOF_I2C2_SCL) |        \
                                     PIN_MODE_ANALOG(GPIOF_RF_868_SPI_ATTN) |        \
                                     PIN_MODE_ANALOG(GPIOF_ADC3_IN9) |      \
                                     PIN_MODE_ANALOG(GPIOF_ADC3_IN14) |     \
                                     PIN_MODE_ANALOG(GPIOF_ADC3_IN15) |     \
                                     PIN_MODE_ALTERNATE(GPIOF_USART7_RX) |        \
                                     PIN_MODE_ALTERNATE(GPIOF_USART7_TX) |        \
                                     PIN_MODE_OUTPUT(GPIOF_RF_868_SLEEP) |  \
                                     PIN_MODE_OUTPUT(GPIOF_RF_868_RST) |    \
                                     PIN_MODE_INPUT(GPIOF_GPIOF_10) |       \
                                     PIN_MODE_INPUT(GPIOF_GPIOF_11) |       \
                                     PIN_MODE_INPUT(GPIOF_GPIOF_11) |       \
                                     PIN_MODE_INPUT(GPIOF_GPIOF_13) |       \
                                     PIN_MODE_INPUT(GPIOF_GPIOF_14) |       \
                                     PIN_MODE_INPUT(GPIOF_GPIOF_15))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOF_I2C2_SDA) |    \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_I2C2_SCL) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOF_RF_868_SPI_ATTN) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOF_ADC3_IN9) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOF_ADC3_IN14) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOF_ADC3_IN15) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOF_USART7_RX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOF_USART7_TX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOF_RF_868_SLEEP) |\
                                     PIN_OTYPE_OPENDRAIN(GPIOF_RF_868_RST) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_GPIOF_10) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOF_GPIOF_11) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOF_GPIOF_11) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOF_GPIOF_13) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOF_GPIOF_14) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOF_GPIOF_15))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_HIGH(GPIOF_I2C2_SDA) |       \
                                     PIN_OSPEED_HIGH(GPIOF_I2C2_SCL) |       \
                                     PIN_OSPEED_HIGH(GPIOF_RF_868_SPI_ATTN) |       \
                                     PIN_OSPEED_HIGH(GPIOF_ADC3_IN9) |      \
                                     PIN_OSPEED_HIGH(GPIOF_ADC3_IN14) |     \
                                     PIN_OSPEED_HIGH(GPIOF_ADC3_IN15) |     \
                                     PIN_OSPEED_HIGH(GPIOF_USART7_RX) |    \
                                     PIN_OSPEED_HIGH(GPIOF_USART7_TX) |       \
                                     PIN_OSPEED_HIGH(GPIOF_RF_868_SLEEP) |  \
                                     PIN_OSPEED_HIGH(GPIOF_RF_868_RST) |    \
                                     PIN_OSPEED_HIGH(GPIOF_GPIOF_10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_GPIOF_11) |   \
                                     PIN_OSPEED_VERYLOW(GPIOF_GPIOF_11) |   \
                                     PIN_OSPEED_VERYLOW(GPIOF_GPIOF_13) |   \
                                     PIN_OSPEED_VERYLOW(GPIOF_GPIOF_14) |   \
                                     PIN_OSPEED_VERYLOW(GPIOF_GPIOF_15))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_PULLUP(GPIOF_I2C2_SDA) |      \
                                     PIN_PUPDR_PULLUP(GPIOF_I2C2_SCL) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_RF_868_SPI_ATTN) |      \
                                     PIN_PUPDR_PULLUP(GPIOF_ADC3_IN9) |     \
                                     PIN_PUPDR_PULLUP(GPIOF_ADC3_IN14) |    \
                                     PIN_PUPDR_PULLUP(GPIOF_ADC3_IN15) |    \
                                     PIN_PUPDR_PULLUP(GPIOF_USART7_RX) |      \
                                     PIN_PUPDR_PULLUP(GPIOF_USART7_TX) |      \
                                     PIN_PUPDR_PULLUP(GPIOF_RF_868_SLEEP) | \
                                     PIN_PUPDR_PULLUP(GPIOF_RF_868_RST) |   \
                                     PIN_PUPDR_PULLUP(GPIOF_GPIOF_10) |     \
                                     PIN_PUPDR_PULLUP(GPIOF_GPIOF_11) |     \
                                     PIN_PUPDR_PULLUP(GPIOF_GPIOF_11) |     \
                                     PIN_PUPDR_PULLUP(GPIOF_GPIOF_13) |     \
                                     PIN_PUPDR_PULLUP(GPIOF_GPIOF_14) |     \
                                     PIN_PUPDR_PULLUP(GPIOF_GPIOF_15))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_I2C2_SDA) |          \
                                     PIN_ODR_HIGH(GPIOF_I2C2_SCL) |          \
                                     PIN_ODR_HIGH(GPIOF_RF_868_SPI_ATTN) |          \
                                     PIN_ODR_HIGH(GPIOF_ADC3_IN9) |         \
                                     PIN_ODR_HIGH(GPIOF_ADC3_IN14) |        \
                                     PIN_ODR_HIGH(GPIOF_ADC3_IN15) |        \
                                     PIN_ODR_HIGH(GPIOF_USART7_RX) |          \
                                     PIN_ODR_HIGH(GPIOF_USART7_TX) |          \
                                     PIN_ODR_HIGH(GPIOF_RF_868_SLEEP) |     \
                                     PIN_ODR_HIGH(GPIOF_RF_868_RST) |       \
                                     PIN_ODR_HIGH(GPIOF_GPIOF_10) |         \
                                     PIN_ODR_HIGH(GPIOF_GPIOF_11) |         \
                                     PIN_ODR_HIGH(GPIOF_GPIOF_11) |         \
                                     PIN_ODR_HIGH(GPIOF_GPIOF_13) |         \
                                     PIN_ODR_HIGH(GPIOF_GPIOF_14) |         \
                                     PIN_ODR_HIGH(GPIOF_GPIOF_15))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_I2C2_SDA, 4U) |       \
                                     PIN_AFIO_AF(GPIOF_I2C2_SCL, 4U) |       \
                                     PIN_AFIO_AF(GPIOF_RF_868_SPI_ATTN, 0U) |       \
                                     PIN_AFIO_AF(GPIOF_ADC3_IN9, 0U) |      \
                                     PIN_AFIO_AF(GPIOF_ADC3_IN14, 0U) |     \
                                     PIN_AFIO_AF(GPIOF_ADC3_IN15, 0U) |     \
                                     PIN_AFIO_AF(GPIOF_USART7_RX, 8U) |       \
                                     PIN_AFIO_AF(GPIOF_USART7_TX, 8U))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_RF_868_SLEEP, 0U) |  \
                                     PIN_AFIO_AF(GPIOF_RF_868_RST, 0U) |    \
                                     PIN_AFIO_AF(GPIOF_GPIOF_10, 0U) |      \
                                     PIN_AFIO_AF(GPIOF_GPIOF_11, 0U) |      \
                                     PIN_AFIO_AF(GPIOF_GPIOF_11, 0U) |      \
                                     PIN_AFIO_AF(GPIOF_GPIOF_13, 0U) |      \
                                     PIN_AFIO_AF(GPIOF_GPIOF_14, 0U) |      \
                                     PIN_AFIO_AF(GPIOF_GPIOF_15, 0U))

/*
 * GPIOG setup:
 *
 * PG0  - ZIO_D65                   (input pullup).
 * PG1  - ZIO_D64                   (input pullup).
 * PG2  - ZIO_D49                   (input pullup).
 * PG3  - ZIO_D50                   (input pullup).
 * PG4  - PIN4                      (input pullup).
 * PG5  - PIN5                      (input pullup).
 * PG6  - USB_GPIO_OUT              (input pullup).
 * PG7  - USB_GPIO_IN               (input pullup).
 * PG8  - PIN8                      (input pullup).
 * PG9  - ARD_D0 USART6_RX          (input pullup).
 * PG10 - PIN10                     (input pullup).
 * PG11 - RMII_TX_EN                (alternate 11).
 * PG12 - PIN12                     (input pullup).
 * PG13 - RMII_TXD0                 (alternate 11).
 * PG14 - ARD_D1 USART6_TX          (input pullup).
 * PG15 - PIN15                     (input pullup).
 */
#define VAL_GPIOG_MODER             (PIN_MODE_INPUT(GPIOG_ZIO_D65) |        \
                                     PIN_MODE_INPUT(GPIOG_ZIO_D64) |        \
                                     PIN_MODE_INPUT(GPIOG_ZIO_D49) |        \
                                     PIN_MODE_INPUT(GPIOG_ZIO_D50) |        \
                                     PIN_MODE_INPUT(GPIOG_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOG_USB_GPIO_OUT) |   \
                                     PIN_MODE_INPUT(GPIOG_USB_GPIO_IN) |    \
                                     PIN_MODE_INPUT(GPIOG_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOG_ARD_D0) |         \
                                     PIN_MODE_INPUT(GPIOG_PIN10) |          \
                                     PIN_MODE_ALTERNATE(GPIOG_RMII_TX_EN) | \
                                     PIN_MODE_INPUT(GPIOG_PIN12) |          \
                                     PIN_MODE_ALTERNATE(GPIOG_RMII_TXD0) |  \
                                     PIN_MODE_INPUT(GPIOG_ARD_D1) |         \
                                     PIN_MODE_INPUT(GPIOG_PIN15))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOG_ZIO_D65) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_ZIO_D64) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_ZIO_D49) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_ZIO_D50) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_USB_GPIO_OUT) |\
                                     PIN_OTYPE_PUSHPULL(GPIOG_USB_GPIO_IN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_ARD_D0) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_RMII_TX_EN) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_RMII_TXD0) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_ARD_D1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN15))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOG_ZIO_D65) |    \
                                     PIN_OSPEED_VERYLOW(GPIOG_ZIO_D64) |    \
                                     PIN_OSPEED_VERYLOW(GPIOG_ZIO_D49) |    \
                                     PIN_OSPEED_VERYLOW(GPIOG_ZIO_D50) |    \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN5) |       \
                                     PIN_OSPEED_HIGH(GPIOG_USB_GPIO_OUT) |  \
                                     PIN_OSPEED_HIGH(GPIOG_USB_GPIO_IN) |   \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN8) |       \
                                     PIN_OSPEED_HIGH(GPIOG_ARD_D0) |        \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN10) |      \
                                     PIN_OSPEED_HIGH(GPIOG_RMII_TX_EN) |    \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN12) |      \
                                     PIN_OSPEED_HIGH(GPIOG_RMII_TXD0) |     \
                                     PIN_OSPEED_HIGH(GPIOG_ARD_D1) |        \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN15))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_PULLUP(GPIOG_ZIO_D65) |      \
                                     PIN_PUPDR_PULLUP(GPIOG_ZIO_D64) |      \
                                     PIN_PUPDR_PULLUP(GPIOG_ZIO_D49) |      \
                                     PIN_PUPDR_PULLUP(GPIOG_ZIO_D50) |      \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_USB_GPIO_OUT) | \
                                     PIN_PUPDR_PULLUP(GPIOG_USB_GPIO_IN) |  \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_ARD_D0) |       \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN10) |        \
                                     PIN_PUPDR_FLOATING(GPIOG_RMII_TX_EN) | \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN12) |        \
                                     PIN_PUPDR_FLOATING(GPIOG_RMII_TXD0) |  \
                                     PIN_PUPDR_PULLUP(GPIOG_ARD_D1) |       \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN15))
#define VAL_GPIOG_ODR               (PIN_ODR_HIGH(GPIOG_ZIO_D65) |          \
                                     PIN_ODR_HIGH(GPIOG_ZIO_D64) |          \
                                     PIN_ODR_HIGH(GPIOG_ZIO_D49) |          \
                                     PIN_ODR_HIGH(GPIOG_ZIO_D50) |          \
                                     PIN_ODR_HIGH(GPIOG_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOG_USB_GPIO_OUT) |     \
                                     PIN_ODR_HIGH(GPIOG_USB_GPIO_IN) |      \
                                     PIN_ODR_HIGH(GPIOG_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOG_ARD_D0) |           \
                                     PIN_ODR_HIGH(GPIOG_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOG_RMII_TX_EN) |       \
                                     PIN_ODR_HIGH(GPIOG_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOG_RMII_TXD0) |        \
                                     PIN_ODR_HIGH(GPIOG_ARD_D1) |           \
                                     PIN_ODR_HIGH(GPIOG_PIN15))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_ZIO_D65, 0U) |       \
                                     PIN_AFIO_AF(GPIOG_ZIO_D64, 0U) |       \
                                     PIN_AFIO_AF(GPIOG_ZIO_D49, 0U) |       \
                                     PIN_AFIO_AF(GPIOG_ZIO_D50, 0U) |       \
                                     PIN_AFIO_AF(GPIOG_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_USB_GPIO_OUT, 0U) |  \
                                     PIN_AFIO_AF(GPIOG_USB_GPIO_IN, 0U))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_ARD_D0, 0U) |        \
                                     PIN_AFIO_AF(GPIOG_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_RMII_TX_EN, 11U) |   \
                                     PIN_AFIO_AF(GPIOG_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_RMII_TXD0, 11U) |    \
                                     PIN_AFIO_AF(GPIOG_ARD_D1, 0U) |        \
                                     PIN_AFIO_AF(GPIOG_PIN15, 0U))

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
