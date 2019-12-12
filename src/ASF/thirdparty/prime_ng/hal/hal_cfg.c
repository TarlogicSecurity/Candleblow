/**
 * \file
 *
 * \brief HAL_CFG: Configuration related functions
 *
 * Copyright (c) 2015 Atmel Corporation. All rights reserved.
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

#include "asf.h"
#include "string.h"
#include "board.h"
#include "hal_private.h"
#include "conf_app_example.h"

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

static uint32_t ul_user_sign_buf[HAL_USER_SIGNATURE_SIZE];

static const uint8_t user_signature_offset_list[CONFIG_TYPE_END_LIST] = {
	HAL_MACCFG_OFFSET_USER_SIGN,
	HAL_PHYCFG_OFFSET_USER_SIGN,
	HAL_BNINFO_OFFSET_USER_SIGN,
	HAL_PRIME_MODE_OFFSET_USER_SIGN,
	HAL_SECURITY_OFFSET_USER_SIGN,
	HAL_BOOT_INFO_OFFSET_USER_SIGN
};

/**
 * \brief Get information relative to configuration paramaters from non-volatile memory
 *
 * \param cfg_type    Configuration type
 * \param us_size     Number of bytes to read
 * \param pv_data     pointer to get the data value
 *
 * \return TRUE if there is no error, otherwise returns FALSE.
 */
bool hal_get_config_info(config_info_type_t cfg_type, uint16_t us_size, void *pv_data)
{
	uint8_t *puc_data_src;
	uint16_t us_write_end_address;
	uint8_t uc_position_offset;
	bool b_is_int_disabled;

	if (cfg_type >= CONFIG_TYPE_END_LIST) {
		/* data_id out of signature data */
		return false;
	}

	uc_position_offset = user_signature_offset_list[cfg_type];

	/* The user signature should be no longer than 512 bytes */
	us_write_end_address = us_size + uc_position_offset;
	if (us_write_end_address > HAL_FLASH_PAGE_SIZE) {
		return false;
	}

	/* set critical region */
	b_is_int_disabled = __get_PRIMASK();
	if (!b_is_int_disabled) {
		Disable_global_interrupt();
	}

#ifdef CONF_BOARD_ENABLE_CACHE
        SCB_InvalidateDCache_by_Addr ((uint32_t*)ul_user_sign_buf, HAL_USER_SIGNATURE_SIZE);
#endif
	/* Read user signature */
	flash_read_user_signature((void *)ul_user_sign_buf, HAL_USER_SIGNATURE_SIZE);

	/* end critical region */
	if (!b_is_int_disabled) {
		Enable_global_interrupt();
	}

	/* Provide data parameters */
	puc_data_src = (uint8_t *)ul_user_sign_buf + uc_position_offset;
#if (BOARD == ATPL360AMB) || (BOARD == ATPL360MB) || (BOARD == PL360G55CF_EK)
	/* Consider endianness for compatibility */
	uint8_t uc_temp;
	uc_temp = puc_data_src[0];
	puc_data_src[0] = puc_data_src[1];
	puc_data_src[1] = uc_temp;
#endif
	memcpy((uint8_t *)pv_data, (uint8_t *)puc_data_src, us_size);



	return true;
}

/**
 * \brief Set information relative to configuration paramaters in non-volatile memory
 *
 * \param cfg_type    Configuration type
 * \param us_size     Number of bytes to write
 * \param pv_data     pointer to set the data value
 *
 * \return TRUE if there is no error, otherwise returns FALSE.
 */
bool hal_set_config_info(config_info_type_t cfg_type, uint16_t us_size, void *pv_data)
{
	uint8_t *puc_dst;
	uint8_t uc_ret;
	uint16_t us_write_end_address;
	uint8_t uc_position_offset;
	bool b_is_int_disabled;


	if (cfg_type >= CONFIG_TYPE_END_LIST) {
		/* data_id out of signature data */
		return false;
	}

	uc_position_offset = user_signature_offset_list[cfg_type];

	/* The user signature should be no longer than 512 bytes */
	us_write_end_address = us_size + uc_position_offset;
	if (us_write_end_address > HAL_FLASH_PAGE_SIZE) {
		return false;
	}

	/* set critical region */
	b_is_int_disabled = __get_PRIMASK();
	if (!b_is_int_disabled) {
		Disable_global_interrupt();
	}

	/* Read user signature */
        uc_ret = flash_read_user_signature((void *)ul_user_sign_buf, HAL_USER_SIGNATURE_SIZE);
#ifdef CONF_BOARD_ENABLE_CACHE
        SCB_InvalidateDCache_by_Addr ((uint32_t*)ul_user_sign_buf, HAL_USER_SIGNATURE_SIZE);
#endif

        if (uc_ret == FLASH_RC_OK) {


            /* Check MAC config key... it should be present! */
            if ((ul_user_sign_buf[0] & 0xFFFF) != HAL_MAC_CONFIG_KEY) {

                /* Error... not a valid MAC Address valid... system will enter in MPT mode*/
                /* retry...*/
                flash_read_user_signature((void *)ul_user_sign_buf, HAL_USER_SIGNATURE_SIZE);
            }
        } else {
                /*error!*/
        }

	/* provide data parameters */
	puc_dst = (uint8_t *)ul_user_sign_buf + uc_position_offset;
	memcpy((uint8_t *)puc_dst, (uint8_t *)pv_data, us_size);
#if (BOARD == ATPL360AMB) || (BOARD == ATPL360MB) || (BOARD == PL360G55CF_EK)
	/* Consider endianness for compatibility */
	uint8_t uc_temp;
	uc_temp = puc_dst[0];
	puc_dst[0] = puc_dst[1];
	puc_dst[1] = uc_temp;
#endif

#ifdef CONF_BOARD_ENABLE_CACHE
        SCB_InvalidateDCache_by_Addr ((uint32_t*)ul_user_sign_buf, HAL_USER_SIGNATURE_SIZE);
#endif
	/* erase user signature */
	flash_erase_user_signature();

	/* write user signature */
	flash_write_user_signature((void *)ul_user_sign_buf, HAL_USER_SIGNATURE_SIZE);

	/* end critical region */
	if (!b_is_int_disabled) {
		Enable_global_interrupt();
	}

	return true;
}

#ifdef HAL_NWK_RECOVERY_INTERFACE
/**
 * \brief Initialize the network recovery
 *
 * \return Size of the reserved region
 */
uint32_t hal_nwk_recovery_init(void)
{
	uint32_t pul_flash;
	uint32_t pul_flash_end;
	uint32_t ul_size;
	uint8_t uc_16pages_counter;
	uint8_t i;

	ul_size = PRIME_MAX_SIZE_NWK_RECOVERY;
	if (ul_size == 0) {
		return 0;
	}

	pul_flash = PRIME_FLASH_LOCATION_NWK_RECOVERY;
	pul_flash_end = (uint32_t)(pul_flash + ul_size);
	uc_16pages_counter = (uint8_t)(ul_size / HAL_FLASH_16PAGE_SIZE);
	if (ul_size % HAL_FLASH_16PAGE_SIZE) {
		uc_16pages_counter++;
	}

	/* Unlock region for later erasing and writing */
	for (i = 1; i < uc_16pages_counter + 1; i++) {
		flash_unlock(pul_flash, pul_flash + HAL_FLASH_16PAGE_SIZE - 1, 0, 0);

		if (pul_flash > pul_flash_end) {
			break;
		}

		pul_flash += HAL_FLASH_16PAGE_SIZE; /* 16 pages */
	}

	return PRIME_MAX_SIZE_NWK_RECOVERY;
}

/**
 * \brief Read data from memory
 *
 * \param ul_addr		Address to read
 * \param puc_buf		Ptr to buffer for information
 * \param us_size		Number of bytes to get
 *
 */
void hal_nwk_recovery_read(uint32_t ul_addr, uint8_t *puc_buf, uint16_t us_size)
{
	uint32_t ul_read_address;

	ul_read_address = PRIME_FLASH_LOCATION_NWK_RECOVERY + ul_addr;
	if (us_size) {
		memcpy(puc_buf, (uint8_t *)ul_read_address, us_size);
	}
}

/**
 * \brief Write data to memory
 *
 * \param ul_addr		Address to write
 * \param puc_buf		Ptr to information to save
 * \param us_size		Number of bytes to put
 *
 * \return 1 if there is no error, otherwise returns 0.
 */
uint8_t hal_nwk_recovery_write(uint32_t ul_addr, uint8_t *puc_buf, uint16_t us_size)
{
	uint8_t uc_res = 0;
	uint32_t ul_write_address;

	ul_write_address = PRIME_FLASH_LOCATION_NWK_RECOVERY + ul_addr;

	/* Delete next eight pages only if address is multiple of 8 */
	if (!((ul_write_address / HAL_FLASH_PAGE_SIZE) % 8)) {
		flash_erase_page(ul_write_address, IFLASH_ERASE_PAGES_8);
	}

	if (us_size) {
		uint32_t ul_rc;

		ul_rc = flash_write(ul_write_address, puc_buf, us_size, 0);

		if (ul_rc == FLASH_RC_OK) {
			uc_res = 1;
		} else {
			uc_res = 0;
		}
	}

	return uc_res;
}
#endif

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */
