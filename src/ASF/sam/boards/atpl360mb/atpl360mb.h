/**
 * \file
 *
 * \brief ATPL360MB Board Definition.
 *
 * Copyright (c) 2017 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef ATPL360MB_H_INCLUDED
#define ATPL360MB_H_INCLUDED

#include "compiler.h"
#include "conf_board.h"

/**
 * \ingroup group_common_boards
 * \defgroup atpl360mb_group "ATPL360MB"
 * Develop Board for plc devices.
 *
 * @{
 */

/**
 * \defgroup ATPL360MB_board_info_group "ATPL360MB - Board informations"
 * Definitions related to the board description.
 *
 * @{
 */

/** Name of the board */
#define BOARD_NAME "ATPL360MB"
/** Board definition */
#define atpl360mb /** Family definition (already defined) */
#define sam4c
/** Core definition */
#define cortexm4
/** Board revision definition */
#define BOARD_REV_1     1

#ifndef BOARD_REV
#define BOARD_REV BOARD_REV_1
#endif

/* @} */

/**
 *  \defgroup ATPL360MB_opfreq_group "ATPL360MB - Operating frequencies"
 *  Definitions related to the board operating frequency.
 *
 *  @{
 */

/**
 * \name Board oscillator settings
 * @{
 */
#define BOARD_FREQ_SLCK_XTAL        (32768U)
#define BOARD_FREQ_SLCK_BYPASS      (32768U)
#define BOARD_FREQ_MAINCK_XTAL      (12000000U)
#define BOARD_FREQ_MAINCK_BYPASS    (12000000U)
/* @} */

/** Master clock frequency */
#define BOARD_MCK                   CHIP_FREQ_CPU_MAX

/** board main clock xtal statup time */
#define BOARD_OSC_STARTUP_US        15625U

/* @} */

/**
 * \defgroup ATPL360MB_features_group "ATPL360MB - Features"
 * Symbols that describe features and capabilities of the board.
 *
 * @{
 */

/**
 * \name LED #0 pin definition
 * @{
 */
#define LED0_GPIO                        (PIO_PA15_IDX)
#define LED0_ACTIVE_LEVEL                IOPORT_PIN_LEVEL_LOW
#define LED0_INACTIVE_LEVEL              IOPORT_PIN_LEVEL_HIGH

/* Wrapper macros to ensure common naming across all boards */
#define LED_0_NAME                       "Green LED"
#define PIN_LED_0                        {PIO_PA15, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}
#define PIN_LED_0_MASK                   PIO_PA15
#define PIN_LED_0_PIO                    PIOA
#define PIN_LED_0_ID                     ID_PIOA
#define PIN_LED_0_TYPE                   PIO_OUTPUT_1
#define PIN_LED_0_ATTR                   PIO_DEFAULT
/* @} */

/**
 * \name LED #1 pin definition
 * @{
 */
#define LED1_GPIO                        (PIO_PA16_IDX)
#define LED1_ACTIVE_LEVEL                IOPORT_PIN_LEVEL_LOW
#define LED1_INACTIVE_LEVEL              IOPORT_PIN_LEVEL_HIGH

/* Wrapper macros to ensure common naming across all boards */
#define LED_1_NAME                       "Red LED"
#define PIN_LED_1                        {PIO_PA16, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}
#define PIN_LED_1_MASK                   PIO_PA16
#define PIN_LED_1_PIO                    PIOA
#define PIN_LED_1_ID                     ID_PIOA
#define PIN_LED_1_TYPE                   PIO_OUTPUT_1
#define PIN_LED_1_ATTR                   PIO_DEFAULT
/* @} */

/**
 * \PWM LED0 pin definitions.
 * @{
 */
#define PIN_PWM_LED0_GPIO       PIO_PC6_IDX
#define PIN_PWM_LED0_FLAGS      (PIO_PERIPH_A | PIO_DEFAULT)
#define PIN_PWM_LED0_CHANNEL    PWM_CHANNEL_0

/**
 * \PWM LED1 pin definitions.
 * @{
 */
#define PIN_PWM_LED1_GPIO       PIO_PC7_IDX
#define PIN_PWM_LED1_FLAGS      (PIO_PERIPH_A | PIO_DEFAULT)
#define PIN_PWM_LED1_CHANNEL    PWM_CHANNEL_1

/**
 * \name UART0 pins (UTXD0 and URXD0) definitions
 * @{
 */
#define PINS_UART0                       (PIO_PB4A_URXD0 | PIO_PB5A_UTXD0)
#define PINS_UART0_FLAGS                 (IOPORT_MODE_MUX_A)

#define PINS_UART0_PORT                  IOPORT_PIOB
#define PINS_UART0_MASK                  (PIO_PB4A_URXD0 | PIO_PB5A_UTXD0)
#define PINS_UART0_PIO                   PIOB
#define PINS_UART0_ID                    ID_PIOB
#define PINS_UART0_TYPE                  PIO_PERIPH_A
#define PINS_UART0_ATTR                  PIO_DEFAULT
/* @} */

/**
 * \name UART1 pins (UTXD1 and URXD1) definitions
 * @{
 */
#define PINS_UART1                       (PIO_PC1A_URXD1 | PIO_PC0A_UTXD1)
#define PINS_UART1_FLAGS                 (IOPORT_MODE_MUX_A)

#define PINS_UART1_PORT                  IOPORT_PIOC
#define PINS_UART1_MASK                  (PIO_PC1A_URXD1 | PIO_PC0A_UTXD1)
#define PINS_UART1_PIO                   PIOC
#define PINS_UART1_ID                    ID_PIOC
#define PINS_UART1_TYPE                  PIO_PERIPH_A
#define PINS_UART1_ATTR                  PIO_DEFAULT
/* @} */

/**
 * \name SPI pin definitions
 * @{
 */
/** SPI0 MISO pin definition. */
#define SPI0_MISO_GPIO                   (PIO_PA6_IDX)
#define SPI0_MISO_FLAGS                  (IOPORT_MODE_MUX_A | IOPORT_MODE_PULLDOWN)
/** SPI0 MOSI pin definition. */
#define SPI0_MOSI_GPIO                   (PIO_PA7_IDX)
#define SPI0_MOSI_FLAGS                  (IOPORT_MODE_MUX_A)
/** SPI0 SPCK pin definition. */
#define SPI0_SPCK_GPIO                   (PIO_PA8_IDX)
#define SPI0_SPCK_FLAGS                  (IOPORT_MODE_MUX_A)
/** SPI0 chip select 0 pin definition. */
#define SPI0_NPCS0_GPIO                  (PIO_PA5_IDX)
#define SPI0_NPCS0_FLAGS                 (IOPORT_MODE_MUX_A)
/* @} */

/**
 * \name TWIx pin definitions
 * @{
 */
/*! TWI0 Data pin for EEPROM */
#define TWIO_DATA_GPIO                   PIO_PA24_IDX
#define TWIO_DATA_FLAG                   IOPORT_MODE_MUX_A
/*! TWI0 Clock pin for EEPROM */
#define TWIO_CLK_GPIO                    PIO_PA25_IDX
#define TWIO_CLK_FLAG                    IOPORT_MODE_MUX_A
#define BOARD_CLK_TWI_EEPROM             TWIO_CLK_GPIO
#define BOARD_CLK_TWI_MUX_EEPROM         TWIO_CLK_FLAG
/*! TWI0 Address for EEPROM */
#define BOARD_AT24C_ADDRESS              0x50u
/*! TWI0 Address for EEPROM */
# define BOARD_AT24C_TWI_INSTANCE        TWI0
/*! AT24CXX internal address length */
#define AT24C_MEM_ADDR_LEN   2
/* @} */

/**
 * \name Voltage Monitor pins definition
 * @{
 */
#define VDD_SENSE_GPIO                   PIO_PB13_IDX
#define POWER_FAULT_GPIO                 PIO_PA14_IDX
/* @} */

/**
 * \name Zero Cross Monitor pins definition
 * @{
 */
#define VZC_GPIO                         PIO_PB10_IDX
/* @} */

/**
 * \name LCD Reset pin definition
 * @{
 */
#define LCD_RESET_GPIO                   PIO_PA17_IDX
#define LCD_RESET_ACTIVE_LEVEL           IOPORT_PIN_LEVEL_LOW
#define LCD_RESET_INACTIVE_LEVEL         IOPORT_PIN_LEVEL_HIGH
/* @} */

/**
 * \name ATPL360 Reset pin definition
 * @{
 */
#define ATPL360_RESET_GPIO               PIO_PB15_IDX
#define ATPL360_RESET_ACTIVE_LEVEL       IOPORT_PIN_LEVEL_LOW
#define ATPL360_RESET_INACTIVE_LEVEL     IOPORT_PIN_LEVEL_HIGH
/* @} */

/**
 * \name ATPL360 LDO Enable pin definition
 * @{
 */
#define ATPL360_LDO_EN_GPIO              PIO_PB14_IDX
#define ATPL360_LDO_EN_ACTIVE_LEVEL      IOPORT_PIN_LEVEL_HIGH
#define ATPL360_LDO_EN_INACTIVE_LEVEL    IOPORT_PIN_LEVEL_LOW
/* @} */

/**
 * \name ATPL360 interrupt pin definition
 * @{
 */
#define ATPL360_INT_GPIO                 PIO_PB12_IDX
#define ATPL360_INT_FLAGS                IOPORT_MODE_DEBOUNCE
#define ATPL360_INT_SENSE                IOPORT_SENSE_FALLING

#define ATPL360_INT                      {PIO_PB12, PIOB, ID_PIOB, PIO_INPUT, PIO_DEGLITCH | PIO_IT_LOW_LEVEL}
#define ATPL360_INT_MASK                 PIO_PB12
#define ATPL360_INT_PIO                  PIOB
#define ATPL360_INT_ID                   ID_PIOB
#define ATPL360_INT_TYPE                 PIO_INPUT
#define ATPL360_INT_ATTR                 (PIO_DEGLITCH | PIO_IT_LOW_LEVEL)
#define ATPL360_INT_IRQn                 PIOB_IRQn
/* @} */

/**
 * \name ATPL360 Carrier Detect Enable pin definition
 * @{
 */
#define ATPL360_CD_EN_GPIO               PIO_PA10_IDX
/* @} */

/**
 * \name ATPL360 GPIO pins definition
 * @{
 */
#define ATPL360_EN_GPIO0                 PIO_PA10_IDX
#define ATPL360_EN_GPIO1                 PIO_PA9_IDX
#define ATPL360_EN_GPIO2                 PIO_PB11_IDX
#define ATPL360_EN_GPIO3                 PIO_PB12_IDX
/* @} */

/**
 * \name ATPL360 Relays Control pin definition
 * @{
 */
#define RELAY_CONTROL0_GPIO              PIO_PA18_IDX
#define RELAY_CONTROL0_ACTIVE_LEVEL      IOPORT_PIN_LEVEL_HIGH
#define RELAY_CONTROL0_INACTIVE_LEVEL    IOPORT_PIN_LEVEL_LOW

#define RELAY_CONTROL1_GPIO              PIO_PA19_IDX
#define RELAY_CONTROL1_ACTIVE_LEVEL      IOPORT_PIN_LEVEL_HIGH
#define RELAY_CONTROL1_INACTIVE_LEVEL    IOPORT_PIN_LEVEL_LOW

#define RELAY_CONTROL2_GPIO              PIO_PA26_IDX
#define RELAY_CONTROL2_ACTIVE_LEVEL      IOPORT_PIN_LEVEL_HIGH
#define RELAY_CONTROL2_INACTIVE_LEVEL    IOPORT_PIN_LEVEL_LOW
/* @} */

/* @} */ /* End of ATPL360MB_features_group */

/* @} */ /* End of ATPL360MB_group */

#endif  /* ATPL360MB_H_INCLUDED */
