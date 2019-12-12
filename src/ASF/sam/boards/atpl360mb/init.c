/**
 * \file
 *
 * \brief ATPL360MB board init.
 *
 * Copyright (c) 2014 Atmel Corporation. All rights reserved.
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

#include "compiler.h"

#include "board.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "asf.h"

/**
 * \addtogroup atpl360mb_group
 * @{
 */

/**
 * \brief Set peripheral mode for IOPORT pins.
 * It will configure port mode and disable pin mode (but enable peripheral).
 * \param port IOPORT port to configure
 * \param masks IOPORT pin masks to configure
 * \param mode Mode masks to configure for the specified pin (\ref ioport_modes)
 */
#define ioport_set_port_peripheral_mode(port, masks, mode) \
	do { \
		ioport_set_port_mode(port, masks, mode); \
		ioport_disable_port(port, masks); \
	} \
	while (0)

/**
 * \brief Set peripheral mode for one single IOPORT pin.
 * It will configure port mode and disable pin mode (but enable peripheral).
 * \param pin IOPORT pin to configure
 * \param mode Mode masks to configure for the specified pin (\ref ioport_modes)
 */
#define ioport_set_pin_peripheral_mode(pin, mode) \
	do { \
		ioport_set_pin_mode(pin, mode);	\
		ioport_disable_pin(pin); \
	} \
	while (0)

/**
 * \brief Set input mode for one single IOPORT pin.
 * It will configure port mode and disable pin mode (but enable peripheral).
 * \param pin IOPORT pin to configure
 * \param mode Mode masks to configure for the specified pin (\ref ioport_modes)
 * \param sense Sense for interrupt detection (\ref ioport_sense)
 */
#define ioport_set_pin_input_mode(pin, mode, sense) \
	do { \
		ioport_set_pin_dir(pin, IOPORT_DIR_INPUT); \
		ioport_set_pin_mode(pin, mode);	\
		ioport_set_pin_sense_mode(pin, sense); \
	} \
	while (0)

void board_init(void)
{
#ifndef CONF_BOARD_KEEP_WATCHDOG_AT_INIT
	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;
#else
	WDT->WDT_MR = (WDT->WDT_MR & 0xFFFFF000) | 0x7FF; /* 8 secs */
#endif

	/* Select the crystal oscillator to be the source of the slow clock,
	 * as it provides a more accurate frequency
	 */
#ifdef CONF_BOARD_32K_XTAL
	supc_switch_sclk_to_32kxtal(SUPC, 0);
#endif

	/* GPIO has been deprecated, the old code just keeps it for compatibility.
	 * In new designs IOPORT is used instead.
	 * Here IOPORT must be initialized for others to use before setting up IO.
	 */
	ioport_init();

	/* Configure the pins connected to LEDs as output and set their
	 * default initial state to high (LEDs off).
	 */
	ioport_set_pin_dir(LED0_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED0_GPIO, LED0_INACTIVE_LEVEL);
	ioport_set_pin_dir(LED1_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED1_GPIO, LED0_INACTIVE_LEVEL);

#ifdef CONF_BOARD_ATPL360
	/* Configure SPI pins */
	ioport_set_pin_peripheral_mode(SPI0_MISO_GPIO, SPI0_MISO_FLAGS);
	ioport_set_pin_peripheral_mode(SPI0_MOSI_GPIO, SPI0_MOSI_FLAGS);
	ioport_set_pin_peripheral_mode(SPI0_SPCK_GPIO, SPI0_SPCK_FLAGS);
	ioport_set_pin_peripheral_mode(SPI0_NPCS0_GPIO, SPI0_NPCS0_FLAGS);

	/* Configure LDO Enable pin */
	ioport_set_pin_dir(ATPL360_LDO_EN_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(ATPL360_LDO_EN_GPIO, ATPL360_LDO_EN_INACTIVE_LEVEL);

	/* Configure RST Enable pin */
	ioport_set_pin_dir(ATPL360_RESET_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(ATPL360_RESET_GPIO, ATPL360_RESET_ACTIVE_LEVEL);

	/* Configure Carrier Detect Enable pin */
#ifdef CONF_BOARD_ATPL360_CD_EN
	ioport_set_pin_dir(ATPL360_CD_EN_GPIO, IOPORT_DIR_INPUT);
#endif
#endif

	/* Configure UART0 pins */
#ifdef CONF_BOARD_UART0
	ioport_set_port_peripheral_mode(PINS_UART0_PORT, PINS_UART0, PINS_UART0_FLAGS);
#endif

	/* Configure UART1 pins */
#ifdef CONF_BOARD_UART1
	ioport_set_port_peripheral_mode(PINS_UART1_PORT, PINS_UART1, PINS_UART1_FLAGS);
#endif

	/* Configure Console UART pins */
#ifdef CONF_BOARD_UART_CONSOLE
	switch (CONSOLE_UART_ID){
	case ID_UART0:
		ioport_set_port_peripheral_mode(PINS_UART0_PORT, PINS_UART0, PINS_UART0_FLAGS);
		break;
	case ID_UART1:
		ioport_set_port_peripheral_mode(PINS_UART1_PORT, PINS_UART1, PINS_UART1_FLAGS);
		break;
	}
#endif

#ifdef CONF_BOARD_LCD_EN
	/* Configure LCD Reset pin */
	ioport_set_pin_level(LCD_RESET_GPIO, LCD_RESET_INACTIVE_LEVEL);
	ioport_set_pin_dir(LCD_RESET_GPIO, IOPORT_DIR_OUTPUT);

	/* Configure TWI pins */
	ioport_set_pin_peripheral_mode(TWIO_DATA_GPIO, TWIO_DATA_FLAG);
	ioport_set_pin_peripheral_mode(TWIO_CLK_GPIO, TWIO_CLK_FLAG);
#endif

#ifdef CONF_BOARD_PWM_LED0
	/* Configure PWM LED0 pin */
	gpio_configure_pin(PIN_PWM_LED0_GPIO, PIN_PWM_LED0_FLAGS);
#endif

#ifdef CONF_BOARD_PWM_LED1
	/* Configure PWM LED1 pin */
	gpio_configure_pin(PIN_PWM_LED1_GPIO, PIN_PWM_LED1_FLAGS);
#endif
}

/* @} */
