/**
 * \file
 *
 * \brief Connection of the ATPL360 to SPI interface driver.
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

#include "atpl360_hal_spi.h"
#include "atpl360.h"
#include "atpl360_boot.h"
#include "compiler.h"

extern atpl360_hal_wrapper_t sx_atpl360_hal_wrapper;

#ifdef __cplusplus
extern "C" {
#endif

/** \name Functions to connect the ATPL360 component with the SPI Multiple-Sector Access Functions.
 */
/* ! @{ */
/** PDC buffer us_size to firmware update process */
#define PDC_SPI_FUP_BUFFER_SIZE        512

/* SPI Status static variable */
static atpl360_spi_status_t s_spi_status;

/* SPI Status Info static variable */
static atpl360_spi_status_info_t spx_status_info;

/* SPI protection counter */
static uint8_t suc_spi_flag_err;

static void _check_spi_status(atpl360_spi_status_info_t *px_spi_status)
{
	/* Check who is in the other side (bootloader / atpl360) */
	if (ATPL360_CHECK_ID_BOOT_HEADER(px_spi_status->us_header_id)) {
		if (ATPL360_CHECK_CORTEX_RESET(px_spi_status->ul_flags)) {
			/* Debug Reset */
			atpl360_boot_without_load();
		} else {
			atpl360_boot_download_firmware();
		}

		s_spi_status = ATPL360_SPI_STATUS_FW_VALIDATING;
	} else if (ATPL360_CHECK_ID_CORTEX_HEADER(px_spi_status->us_header_id)) {
		if (s_spi_status == ATPL360_SPI_STATUS_UNKNOWN || s_spi_status == ATPL360_SPI_STATUS_CORTEX || s_spi_status == ATPL360_SPI_STATUS_FW_VALIDATING) {
			s_spi_status = ATPL360_SPI_STATUS_CORTEX;
			/* Protection against SPI corruption */
			if (px_spi_status->ul_flags & 0xFF00) {
				px_spi_status->ul_flags = 0;
				if (++suc_spi_flag_err > 3) {
					/* Unexpected s_spi_status -> Reset HW ATPL360 */
					s_spi_status = ATPL360_SPI_STATUS_UNKNOWN;
				}
			} else {
				suc_spi_flag_err = 0;
			}
		} else {
			/* Unexpected s_spi_status -> Reset HW ATPL360 */
			s_spi_status = ATPL360_SPI_STATUS_UNKNOWN;
		}
	} else {
		/* Unexpected ID value -> Reset HW ATPL360 */
		s_spi_status = ATPL360_SPI_STATUS_UNKNOWN;
	}

	return;
}

static uint32_t _check_boot_status(void)
{
	uint8_t puc_value[4];

	memset(puc_value, 0, sizeof(puc_value));
	sx_atpl360_hal_wrapper.plc_send_boot_cmd(ATPL360_BOOT_READ_BOOT_STATUS, 0, sizeof(puc_value), puc_value, puc_value);

	return (*(uint32_t *)puc_value);
}

/**
 * \brief Write ATPL360 firmware through bootloader commands
 *
 * \param *px_data	Pointer to SPI data info
 *
 */
#ifdef ATPL360_ENABLE_SPI_DEBUG_MODE
uint8_t puc_bin[512];
#endif
void atpl360_spi_update_fw(uint32_t ul_fw_addr, uint32_t ul_fw_size)
{
	uint8_t *puc_fw_data;
	uint32_t ul_fw_prog_addr;
	uint32_t ul_fw_pending_len;
	uint16_t us_fw_fragment_len;
	uint8_t uc_padding;

	ul_fw_pending_len = ul_fw_size;
	puc_fw_data = (uint8_t *)ul_fw_addr;
	ul_fw_prog_addr = ATPL360_BOOT_PROGRAM_ADDR;
	uc_padding = 0;
	while (ul_fw_pending_len) {
		if (ul_fw_pending_len > PDC_SPI_FUP_BUFFER_SIZE) {
			us_fw_fragment_len = PDC_SPI_FUP_BUFFER_SIZE;
		} else {
			us_fw_fragment_len = ul_fw_pending_len;
			uc_padding = us_fw_fragment_len % 4;
			us_fw_fragment_len += uc_padding;
		}

		/* Write fw block data */
		sx_atpl360_hal_wrapper.plc_send_boot_cmd(ATPL360_BOOT_CMD_WRITE_BUF, ul_fw_prog_addr, us_fw_fragment_len, puc_fw_data, NULL);

		/* Update counters */
		ul_fw_pending_len -= (us_fw_fragment_len - uc_padding);
		puc_fw_data += us_fw_fragment_len;
		ul_fw_prog_addr += us_fw_fragment_len;
	}

#ifdef ATPL360_ENABLE_SPI_DEBUG_MODE
	/* READ BIN: Only for debug */
	uint8_t *puc_bin_data;
	memset(puc_bin, 0, sizeof(puc_bin));
	puc_bin_data = puc_bin;
	puc_fw_data = (uint8_t *)ul_fw_addr;
	ul_fw_prog_addr = ATPL360_BOOT_PROGRAM_ADDR;
	/* Read fw block data */
	sx_atpl360_hal_wrapper.plc_send_boot_cmd(ATPL360_BOOT_CMD_READ_BUF, ul_fw_prog_addr, us_fw_fragment_len, puc_fw_data, puc_bin_data);
#endif
}

/**
 * \brief Write ATPL360 firmware through bootloader commands
 *
 * \param *px_data	Pointer to SPI data info
 *
 */
void atpl360_spi_update_fw_sec(uint32_t ul_fw_addr, uint32_t ul_fw_size)
{
	uint8_t *puc_fw_data;
	uint32_t ul_reg_value;
	uint32_t ul_fw_prog_addr;
	uint32_t ul_fw_pending_len;
	uint16_t us_fw_fragment_len;
	uint8_t uc_padding;

	/* Send sequencial binary data */
	ul_fw_pending_len = ul_fw_size;
	puc_fw_data = (uint8_t *)ul_fw_addr;
	ul_fw_prog_addr = ATPL360_BOOT_PROGRAM_ADDR;
	uc_padding = 0;
	while (ul_fw_pending_len) {
		if (ul_fw_pending_len > PDC_SPI_FUP_BUFFER_SIZE) {
			us_fw_fragment_len = PDC_SPI_FUP_BUFFER_SIZE;
		} else {
			us_fw_fragment_len = ul_fw_pending_len;
			uc_padding = us_fw_fragment_len % 4;
			us_fw_fragment_len += uc_padding;
		}

		/* Write fw block data */
		sx_atpl360_hal_wrapper.plc_send_boot_cmd(ATPL360_BOOT_CMD_WRITE_BUF, ul_fw_prog_addr, us_fw_fragment_len, puc_fw_data, NULL);

		/* Update counters */
		ul_fw_pending_len -= (us_fw_fragment_len - uc_padding);
		puc_fw_data += us_fw_fragment_len;
		ul_fw_prog_addr += us_fw_fragment_len;
	}

#ifdef ATPL360_ENABLE_SPI_DEBUG_MODE
	/* READ BIN: Only for debug */
	uint8_t *puc_bin_data;
	memset(puc_bin, 0, sizeof(puc_bin));
	puc_bin_data = puc_bin;
	puc_fw_data = (uint8_t *)ul_fw_addr;
	ul_fw_prog_addr = ATPL360_BOOT_PROGRAM_ADDR;
	/* Read fw block data */
	sx_atpl360_hal_wrapper.plc_send_boot_cmd(ATPL360_BOOT_CMD_READ_BUF, ul_fw_prog_addr, us_fw_fragment_len, puc_fw_data, puc_bin_data);
#endif

	/* Send Start Decryption */
	ul_reg_value = 0;
	sx_atpl360_hal_wrapper.plc_send_boot_cmd(ATPL360_BOOT_START_DECRYPT, 0, 4, (uint8_t *)&ul_reg_value, NULL);

	/* Test Bootloader status : wait to AES block */
	while (_check_boot_status() & ATPL360_FUSES_BOOT_ST_AES_ACT) {
		sx_atpl360_hal_wrapper.plc_delay(DELAY_TREF_MS, ATPL360_DELAY_BOOT_CMD);
	}

#ifdef ATPL360_ENABLE_SPI_DEBUG_MODE
	/* Only for debug */
	uint32_t ul_boot_dbg = _check_boot_status();
	if (ul_boot_dbg & ATPL360_FUSES_BOOT_ST_SIGN_OK) {
		printf("SIGNATURE OK. Boot_status: 0x%08x\r\n", ul_boot_dbg);
	} else {
		printf("Error in SIGNATURE. Boot_status: 0x%08x\r\n", ul_boot_dbg);
	}

	memset(puc_bin, 0, sizeof(puc_bin));
	puc_bin_data = puc_bin;
	puc_fw_data = (uint8_t *)ul_fw_addr;
	ul_fw_prog_addr = ATPL360_BOOT_PROGRAM_ADDR;
	/* Read fw block data */
	sx_atpl360_hal_wrapper.plc_send_boot_cmd(ATPL360_BOOT_CMD_READ_BUF, ul_fw_prog_addr, 512, puc_fw_data, puc_bin_data);
#endif
}

/**
 * \brief Write CPUWAIT = 0
 *
 */
void atpl360_spi_boot_disable_cpuwait(void)
{
	uint32_t ul_reg_value;
	uint8_t puc_value[4];

	ul_reg_value = ATPL360_MISCR_PPM_CALIB_OFF | ATPL360_MISCR_MEM_96_96_CFG | ATPL360_MISCR_EN_ACCESS_ERROR | ATPL360_MISCR_SET_GPIO_12_ZC;
	puc_value[3] = (uint8_t)(ul_reg_value >> 24);
	puc_value[2] = (uint8_t)(ul_reg_value >> 16);
	puc_value[1] = (uint8_t)(ul_reg_value >> 8);
	puc_value[0] = (uint8_t)(ul_reg_value);
	sx_atpl360_hal_wrapper.plc_send_boot_cmd(ATPL360_BOOT_CMD_WRITE_WORD, ATPL360_MISCR, sizeof(puc_value), puc_value, NULL);
}

/**
 * \brief Config system
 *
 */
void atpl360_spi_boot_config_sys(void)
{
	uint32_t ul_reg_value;
	uint8_t puc_value[4];

	/* Send CPU Wait Cmd */
	ul_reg_value = ATPL360_MISCR_CPUWAIT | ATPL360_MISCR_PPM_CALIB_OFF | ATPL360_MISCR_MEM_96_96_CFG |
			ATPL360_MISCR_EN_ACCESS_ERROR | ATPL360_MISCR_SET_GPIO_12_ZC;
	puc_value[3] = (uint8_t)(ul_reg_value >> 24);
	puc_value[2] = (uint8_t)(ul_reg_value >> 16);
	puc_value[1] = (uint8_t)(ul_reg_value >> 8);
	puc_value[0] = (uint8_t)(ul_reg_value);
	sx_atpl360_hal_wrapper.plc_send_boot_cmd(ATPL360_BOOT_CMD_WRITE_WORD, ATPL360_MISCR, sizeof(puc_value), puc_value, NULL);

#ifdef ATPL360_ENABLE_SPI_DEBUG_MODE
	memset(puc_value, 0, 4);
	sx_atpl360_hal_wrapper.plc_send_boot_cmd(ATPL360_BOOT_CMD_READ_WORD, ATPL360_MISCR, sizeof(puc_value), puc_value, puc_value);
	ul_reg_value = *(uint32_t *)puc_value;
#endif
}

/**
 * \brief Reset boot command
 *
 */
void atpl360_spi_boot_reset_cmd(void)
{
	uint32_t ul_reg_value;
	uint8_t puc_value[4];

	/* Send Reset Cmd */
	ul_reg_value = ATPL360_RSTR_VALUE_CONST_RST | ATPL360_RSTR_EN_PERSLCK_RESET | ATPL360_RSTR_EN_PER_RESET | ATPL360_RSTR_EN_PROC_RESET;
	puc_value[3] = (uint8_t)(ul_reg_value >> 24);
	puc_value[2] = (uint8_t)(ul_reg_value >> 16);
	puc_value[1] = (uint8_t)(ul_reg_value >> 8);
	puc_value[0] = (uint8_t)(ul_reg_value);
	sx_atpl360_hal_wrapper.plc_send_boot_cmd(ATPL360_BOOT_CMD_WRITE_WORD, ATPL360_RSTR, sizeof(puc_value), puc_value, NULL);
}

/**
 * \brief Disable ATPL360 bootloader (SPI)
 *
 */
void atpl360_spi_boot_disable(void)
{
	sx_atpl360_hal_wrapper.plc_send_boot_cmd(ATPL360_BOOT_CMD_DIS_SPI_CTRL, 0, 0, NULL, NULL);
}

/**
 * \brief Disable ATPL360 bootloader (SPI, CLK)
 *
 */
void atpl360_spi_clk_boot_disable(void)
{
	sx_atpl360_hal_wrapper.plc_send_boot_cmd(ATPL360_BOOT_CMD_DIS_SPI_CLK_CTRL, 0, 0, NULL, NULL);
}

/**
 * \brief Write buffer
 *
 * \param px_data                         Pointer to SPI tx data info
 *
 */
void atpl360_spi_write_buf(atpl360_spi_data_t *px_data)
{
	sx_atpl360_hal_wrapper.plc_write_read_cmd(ATPL360_CMD_WRITE, (void *)px_data, (void *)&spx_status_info);
	_check_spi_status((atpl360_spi_status_info_t *)&spx_status_info);
}

/**
 * \brief Read buffer
 *
 * \param px_data          Pointer to SPI rx data info
 * \return  true if there is no error, otherwise returns false.
 */
bool atpl360_spi_read_buf(atpl360_spi_data_t *px_data)
{
	bool b_res;

	b_res = sx_atpl360_hal_wrapper.plc_write_read_cmd(ATPL360_CMD_READ, (void *)px_data, (void *)&spx_status_info);
	_check_spi_status((atpl360_spi_status_info_t *)&spx_status_info);

	return b_res;
}

/**
 * \brief Initialize ATPL360 communication interface
 *
 */
void atpl360_spi_initialize(void)
{
	/* Status Initialization */
	s_spi_status = ATPL360_SPI_STATUS_UNKNOWN;
}

/**
 * \brief Enable Write operation in bootloader
 *
 */
void atpl360_spi_boot_write_cmd_enable(void)
{
	uint8_t auc_value[4];

	auc_value[3] = (uint8_t)(ATPL360_BOOT_WRITE_KEY >> 24);
	auc_value[2] = (uint8_t)(ATPL360_BOOT_WRITE_KEY >> 16);
	auc_value[1] = (uint8_t)(ATPL360_BOOT_WRITE_KEY >> 8);
	auc_value[0] = (uint8_t)(ATPL360_BOOT_WRITE_KEY);
	sx_atpl360_hal_wrapper.plc_send_boot_cmd(ATPL360_BOOT_CMD_ENABLE_WRITE, 0, sizeof(auc_value), auc_value, NULL);

	auc_value[3] = (uint8_t)(ATPL360_BOOT_WRITE_KEY >> 8);
	auc_value[2] = (uint8_t)(ATPL360_BOOT_WRITE_KEY);
	auc_value[1] = (uint8_t)(ATPL360_BOOT_WRITE_KEY >> 24);
	auc_value[0] = (uint8_t)(ATPL360_BOOT_WRITE_KEY >> 16);
	sx_atpl360_hal_wrapper.plc_send_boot_cmd(ATPL360_BOOT_CMD_ENABLE_WRITE, 0, sizeof(auc_value), auc_value, NULL);
}

void atpl360_spi_boot_sec_set_num_pkts(uint16_t us_num_pkts)
{
	uint32_t ul_reg_value;
	uint8_t puc_value[4];

	ul_reg_value = (uint32_t)us_num_pkts;

	puc_value[3] = (uint8_t)(ul_reg_value >> 24);
	puc_value[2] = (uint8_t)(ul_reg_value >> 16);
	puc_value[1] = (uint8_t)(ul_reg_value >> 8);
	puc_value[0] = (uint8_t)(ul_reg_value);
	sx_atpl360_hal_wrapper.plc_send_boot_cmd(ATPL360_BOOT_SET_DEC_NUM_PKTS, 0, 4, puc_value, NULL);
}

void atpl360_spi_boot_sec_set_init_vector(uint8_t *puc_iv)
{
	/* Send decryption initial vector */
	sx_atpl360_hal_wrapper.plc_send_boot_cmd(ATPL360_BOOT_SET_DEC_INIT_VECT, 0, 16, puc_iv, NULL);
}

void atpl360_spi_boot_sec_set_signature(uint8_t *puc_signature)
{
	/* Send decryption signature */
	sx_atpl360_hal_wrapper.plc_send_boot_cmd(ATPL360_BOOT_SET_DEC_SIGN, 0, 16, puc_signature, NULL);
}

atpl360_spi_status_t atpl360_spi_get_status(void)
{
	return s_spi_status;
}

void atpl360_spi_get_status_info(atpl360_spi_status_info_t *px_status_info)
{
	memcpy(px_status_info, (void const *)&spx_status_info, sizeof(atpl360_spi_status_info_t));
}

/* ! @} */

#ifdef __cplusplus
}
#endif
