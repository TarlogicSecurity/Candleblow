/**
 * \file
 *
 * \brief API driver for C0216CiZ COG (Chip-on-Glass) Liquid Crystal Display component.
 *
 * Copyright (c) 2016 Atmel Corporation. All rights reserved.
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

/**
 * \defgroup c0216CiZ_display_group Display - C0216CiZ Controller
 *
 * Low-level driver for the C0216CiZ LCD controller. This driver provides access to the main
 * features of the ILI9488 controller.
 *
 * \{
 */

/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include "conf_c0216CiZ.h"
#include "c0216CiZ.h"
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include "pio.h"
#include "delay.h"
#include "twi_master.h"
#include "ioport.h"

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

static void _lcd_reset(void)
{
	/* wait 50ms to stable */
	delay_ms(50);

	/* reset enable */
	ioport_set_pin_level(LCD_RESET_GPIO, LCD_RESET_ACTIVE_LEVEL);

	/* apply reset during 5ms */
	delay_ms(5);

	/* reset disable */
	ioport_set_pin_level(LCD_RESET_GPIO, LCD_RESET_INACTIVE_LEVEL);

	/* wait 50ms to stable */
	delay_ms(50);
}

static void _send_cmd(uint8_t uc_cmd)
{
	twi_package_t x_twi_packet;

	x_twi_packet.chip = C0216CiZ_SLAVE_ADDRESS;
	x_twi_packet.addr[0] = C0216CiZ_CONTROL_WR_CMD;
	x_twi_packet.addr_length = 1;
	x_twi_packet.length = 1;
	x_twi_packet.buffer = &uc_cmd;

	/* Perform a single-byte write access then check the result. */
	while (twi_master_write(C0216CIZ_TWI, &x_twi_packet) != TWI_SUCCESS) {
	}

	delay_us(30);
}

static void _send_text(const char *text)
{
	twi_package_t x_twi_packet;

	x_twi_packet.chip = C0216CiZ_SLAVE_ADDRESS;
	x_twi_packet.addr[0] = C0216CiZ_CONTROL_WR_DAT;
	x_twi_packet.addr_length = 1;
	x_twi_packet.buffer = (uint8_t *)text;
	x_twi_packet.length = strlen(text);

	/* Perform a multi-byte write access then check the result. */
	while (twi_master_write(C0216CIZ_TWI, &x_twi_packet) != TWI_SUCCESS) {
	}
}

/**
 * \brief Initialize the C0216CiZ lcd driver.
 *
 * \note Make sure below works have been done before calling ili9488_init()\n
 * 1. C0216CiZ related Pins have been initialized correctly. \n
 *
 * \param p_opt pointer to C0216CiZ option structure.
 *
 * \return 0 if initialization succeeds, otherwise fails.
 */
uint8_t c0216CiZ_init(void)
{
	twi_master_options_t twi_opt;

	_lcd_reset();

	/* TWI master initialization options. */
	memset((void *)&twi_opt, 0, sizeof(twi_opt));
	twi_opt.speed = C0216CIZ_TWI_BAUDRATE;
	twi_opt.chip  = C0216CiZ_SLAVE_ADDRESS;

	/* Initialize the TWI master driver. */
	twi_master_setup(C0216CIZ_TWI, &twi_opt);

	c0216CiZ_set_function(C0216CiZ_FSET_8_BITS | C0216CiZ_FSET_N_2LN | C0216CiZ_FSET_DH_8 | C0216CiZ_FSET_IS_NOR);
	delay_ms(10);
	c0216CiZ_set_function(C0216CiZ_FSET_8_BITS | C0216CiZ_FSET_N_2LN | C0216CiZ_FSET_DH_8 | C0216CiZ_FSET_IS_EXT);
	delay_ms(10);
	c0216CiZ_set_internal_osc(C0216CiZ_INT_OSC_BS_1_5 | C0216CiZ_INT_OSC_F_ADJ_183);
	c0216CiZ_set_contrast(0x08);
	c0216CiZ_set_pic(C0216CiZ_ICON_DISPLAY_ON | C0216CiZ_ICON_BOOSTER_ON | 0x01);
	c0216CiZ_set_follower(C0216CiZ_FOLLOWER_ON | 0x05);
	delay_ms(200);
	c0216CiZ_set_display(C0216CiZ_DISPLAY_ON | C0216CiZ_CURSOR_OFF | C0216CiZ_CURSOR_BLINK_DIS);

	c0216CiZ_clear_display();
	c0216CiZ_set_entry_mode(C0216CiZ_SHIFT_INC_DDRAM_ADDR);
	delay_ms(10);

	return 0;
}

void c0216CiZ_clear_display(void)
{
	_send_cmd(C0216CiZ_CMD_CLEAR_DISPLAY);
}

void c0216CiZ_return_home(void)
{
	_send_cmd(C0216CiZ_CMD_RETURN_HOME);
}

void c0216CiZ_set_entry_mode(uint8_t uc_flags)
{
	if (uc_flags < C0216CiZ_CMD_ENTRY_MODE_SET) {
		_send_cmd(C0216CiZ_CMD_ENTRY_MODE_SET | uc_flags);
	}
}

void c0216CiZ_set_display(uint8_t uc_flags)
{
	if (uc_flags < C0216CiZ_CMD_DISPLAY_ON) {
		_send_cmd(C0216CiZ_CMD_DISPLAY_ON | uc_flags);
	}
}

void c0216CiZ_set_function(uint8_t uc_flags)
{
	if (uc_flags < C0216CiZ_CMD_FUNCTION_SET) {
		_send_cmd(C0216CiZ_CMD_FUNCTION_SET | uc_flags);
		delay_us(30);
	}
}

void c0216CiZ_set_ddram_addr(uint8_t uc_addr)
{
	if (uc_addr < C0216CiZ_CMD_SET_DDRAM_ADDR) {
		_send_cmd(C0216CiZ_CMD_SET_DDRAM_ADDR | uc_addr);
	}
}

void c0216CiZ_set_cd_shift(uint8_t uc_flags)
{
	if (uc_flags < C0216CiZ_CMD_CD_SHIFT) {
		_send_cmd(C0216CiZ_CMD_CD_SHIFT | uc_flags);
	}
}

void c0216CiZ_set_cgram(uint8_t uc_addr)
{
	if (uc_addr < C0216CiZ_CMD_SET_CGRAM_ADDR) {
		_send_cmd(C0216CiZ_CMD_SET_CGRAM_ADDR | uc_addr);
	}
}

void c0216CiZ_set_internal_osc(uint8_t uc_flags)
{
	if (uc_flags < C0216CiZ_CMD_SET_INTERNAL_OSC) {
		_send_cmd(C0216CiZ_CMD_SET_INTERNAL_OSC | uc_flags);
	}
}

void c0216CiZ_set_icon_addr(uint8_t uc_flags)
{
	if (uc_flags < C0216CiZ_CMD_SET_ICON_ADDR) {
		_send_cmd(C0216CiZ_CMD_SET_ICON_ADDR | uc_flags);
	}
}

void c0216CiZ_set_pic(uint8_t uc_flags)
{
	if (uc_flags < C0216CiZ_CMD_SET_PIC) {
		_send_cmd(C0216CiZ_CMD_SET_PIC | uc_flags);
	}
}

void c0216CiZ_set_follower(uint8_t uc_flags)
{
	if (uc_flags < C0216CiZ_CMD_SET_FOLLOWER_C) {
		_send_cmd(C0216CiZ_CMD_SET_FOLLOWER_C | uc_flags);
	}
}

void c0216CiZ_set_contrast(uint8_t uc_value)
{
	if (uc_value <= 0x0F) {
		_send_cmd(C0216CiZ_CMD_SET_CONTRAST | uc_value);
	}
}

void c0216CiZ_show(const char *puc_char)
{
	if (strlen(puc_char) < C0216CiZ_LINE_CHARS) {
		_send_text(puc_char);
	}
}

void c0216CiZ_set_cursor(uint8_t uc_line, uint8_t uc_pos)
{
	if ((uc_pos < C0216CiZ_LINE_CHARS) && (uc_line < C0216CiZ_LINES)) {
		if (uc_line == 0) {
			c0216CiZ_set_ddram_addr(uc_pos);
		} else {
			c0216CiZ_set_ddram_addr(40 + uc_pos);
		}
	}
}

void c0216CiZ_shift_cursor_right(void)
{
	c0216CiZ_set_cd_shift(C0216CiZ_CD_SHIFT_CURSOR_SEL | C0216CiZ_CD_SHIFT_RIGHT);
}

void c0216CiZ_shift_cursor_left(void)
{
	c0216CiZ_set_cd_shift(C0216CiZ_CD_SHIFT_CURSOR_SEL | C0216CiZ_CD_SHIFT_LEFT);
}

void c0216CiZ_shift_display_right(void)
{
	c0216CiZ_set_cd_shift(C0216CiZ_CD_SHIFT_SCREEN_SEL | C0216CiZ_CD_SHIFT_RIGHT);
}

void c0216CiZ_shift_display_left(void)
{
	c0216CiZ_set_cd_shift(C0216CiZ_CD_SHIFT_SCREEN_SEL | C0216CiZ_CD_SHIFT_LEFT);
}

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */

/**
 * \}
 */
