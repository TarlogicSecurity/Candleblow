/**
 * \file
 *
 * \brief Bootloader interface.
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

/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include "atpl360.h"
#include "atpl360_hal_spi.h"
#include "atpl360_boot.h"

/* HAL wrapper */
extern atpl360_hal_wrapper_t sx_atpl360_hal_wrapper;

#ifdef __cplusplus
extern "C" {
#endif

static uint32_t sul_binary_address;
static uint32_t sul_binary_len;

#ifdef ATPL360_SEC_BOOT_MODE
static void _inverse_buf_16(uint8_t *puc_buf)
{
	uint8_t tmp_buf[16];

	for (uint8_t uc_idx = 0; uc_idx < 16; uc_idx++) {
		tmp_buf[uc_idx] = puc_buf[15 - uc_idx];
	}

	memcpy(puc_buf, tmp_buf, 16);
}

#endif

inline void atpl360_boot_without_load(void)
{
#ifdef __ATPL360A__
	atpl360_spi_boot_disable();
#else
	#ifdef __ATPL360B__
	atpl360_spi_clk_boot_disable();
	#else
		#error ERROR in ATPL360 mode definition. (__ATPL360A__, __ATPL360B__)
	#endif
#endif

	sx_atpl360_hal_wrapper.plc_delay(DELAY_TREF_MS, 5);
}

void atpl360_boot_init(uint32_t ul_binary_address, uint32_t ul_binary_len)
{
	sul_binary_address = ul_binary_address;
	sul_binary_len = ul_binary_len;
}

void atpl360_boot_download_firmware(void)
{
	sx_atpl360_hal_wrapper.plc_enable_int(false);
	sx_atpl360_hal_wrapper.plc_reset();

	atpl360_spi_boot_write_cmd_enable();
	atpl360_spi_boot_config_sys();

#ifndef ATPL360_SEC_BOOT_MODE
	atpl360_spi_update_fw(sul_binary_address, sul_binary_len);
#else
	atpl360_boot_sec_info_t x_sec_info;
	uint32_t ul_addr;

	/* avoid warnings */
	(void)sul_binary_len;

	/* Extract bin security info */
	ul_addr = sul_binary_address;
	x_sec_info.us_num_pkts = (*(uint8_t *)ul_addr++) << 8;
	x_sec_info.us_num_pkts += (*(uint8_t *)ul_addr);
	ul_addr += 15;
	memcpy(x_sec_info.puc_iv, (uint8_t *)ul_addr, 16);
	_inverse_buf_16(x_sec_info.puc_iv);
	ul_addr += 16;
	memcpy(x_sec_info.puc_sign, (uint8_t *)ul_addr, 16);
	_inverse_buf_16(x_sec_info.puc_sign);
	ul_addr += 16;

	atpl360_spi_boot_sec_set_num_pkts(x_sec_info.us_num_pkts - 1);
	atpl360_spi_boot_sec_set_init_vector(x_sec_info.puc_iv);
	atpl360_spi_boot_sec_set_signature(x_sec_info.puc_sign);
	atpl360_spi_update_fw_sec(ul_addr, x_sec_info.us_num_pkts << 4);
#endif /* #ifndef ATPL360_SEC_BOOT_MODE */

	atpl360_spi_boot_disable_cpuwait();

#ifdef __ATPL360A__
	atpl360_spi_boot_disable();
#else
	#ifdef __ATPL360B__
	atpl360_spi_clk_boot_disable();
	#else
		#error ERROR in ATPL360 mode definition. (__ATPL360A__, __ATPL360B__)
	#endif
#endif

	sx_atpl360_hal_wrapper.plc_delay(DELAY_TREF_MS, 10);

	sx_atpl360_hal_wrapper.plc_enable_int(true);
}

/* ! @} */

#ifdef __cplusplus
}
#endif
