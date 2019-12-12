/**
 * \file
 *
 * \brief HAL_PIB
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
#include "hal_private.h"
#include "hal.h"

/* User PIB attributes */
#define PIB_USER_RESET_INFO                     0xF000
#define PIB_USER_PC                             0xF001
#define PIB_USER_LR                             0xF002
#define PIB_USER_PSR                            0xF003
#define PIB_USER_HFSR                           0xF004
#define PIB_USER_CFSR                           0xF005
#define PIB_USER_R0                             0xF006
#define PIB_USER_R1                             0xF007
#define PIB_USER_R2                             0xF008
#define PIB_USER_R3                             0xF009
#define PIB_USER_R12                            0xF00A

/* PIB values */
static uint32_t sul_pib_values[11];

/* Callback function pointers */
static void (*_pib_get_request_cb_function)(uint8_t uc_result, uint16_t us_pib_attrib, void *pv_pib_value, uint8_t uc_pib_size);
static void (*_pib_set_request_cb_function)(uint8_t uc_result);

/**
 * \brief Get request
 *
 * \param us_pib_attrib   PIB attribute
 *
 */
void hal_pib_get_request(uint16_t us_pib_attrib)
{
	uint32_t ul_value;
	uint8_t uc_result;

	/* Check PIB value */
	if ((us_pib_attrib >= PIB_USER_RESET_INFO) && (us_pib_attrib <= PIB_USER_R12)) {
		uc_result = true;
		ul_value = sul_pib_values[us_pib_attrib & 0x000F];
	} else {
		uc_result = false;
		ul_value = 0;
	}

	/* Return result */
	_pib_get_request_cb_function(uc_result, us_pib_attrib, &ul_value, 4);
}

/**
 * \brief Set request
 *
 * \param us_pib_attrib   PIB attribute
 * \param pv_pib_value    PIB value
 * \param uc_pib_size     PIB size
 *
 */
void hal_pib_set_request(uint16_t us_pib_attrib, void *pv_pib_value, uint8_t uc_pib_size)
{
	(void)us_pib_attrib;
	(void)pv_pib_value;
	(void)uc_pib_size;

	/* Return result */
	_pib_set_request_cb_function(false);
}

/**
 * \brief Set handler to Get Request callback
 *
 * \param p_handler   Get request callback function pointer
 *
 */
void hal_pib_get_request_set_callback(void (*p_handler)(uint8_t uc_result, uint16_t us_pib_attrib, void *pv_pib_value, uint8_t uc_pib_size))
{
	_pib_get_request_cb_function = p_handler;
}

/**
 * \brief Set handler to Set Request callback
 *
 * \param p_handler   Set request callback function pointer
 *
 */
void hal_pib_set_request_set_callback(void (*p_handler)(uint8_t uc_result))
{
	_pib_set_request_cb_function = p_handler;
}

/**
 * \brief Initialize module
 *
 */
void hal_pib_init(void)
{
	_pib_get_request_cb_function = NULL;
	_pib_set_request_cb_function = NULL;

#ifndef DISABLE_PIB_HANDLING
	/* Store PIB values */
#if SAM4C || SAM4CP || SAM4CM 
#if PRIME_MODE == PRIME_SN
	sul_pib_values[PIB_USER_RESET_INFO & 0x000F] = gpbr_read(GPBR5);
	sul_pib_values[PIB_USER_PC & 0x000F] = gpbr_read(GPBR6);
	sul_pib_values[PIB_USER_LR & 0x000F] = gpbr_read(GPBR7);
	sul_pib_values[PIB_USER_PSR & 0x000F] = gpbr_read(GPBR8);
	sul_pib_values[PIB_USER_HFSR & 0x000F] = gpbr_read(GPBR9);
	sul_pib_values[PIB_USER_CFSR & 0x000F] = gpbr_read(GPBR10);
	sul_pib_values[PIB_USER_R0 & 0x000F] = gpbr_read(GPBR11);
	sul_pib_values[PIB_USER_R1 & 0x000F] = gpbr_read(GPBR12);
	sul_pib_values[PIB_USER_R2 & 0x000F] = gpbr_read(GPBR13);
	sul_pib_values[PIB_USER_R3 & 0x000F] = gpbr_read(GPBR14);
	sul_pib_values[PIB_USER_R12 & 0x000F] = 0;
#else
	sul_pib_values[PIB_USER_RESET_INFO & 0x000F] = gpbr_read(GPBR0);
	sul_pib_values[PIB_USER_PC & 0x000F] = gpbr_read(GPBR1);
	sul_pib_values[PIB_USER_LR & 0x000F] = gpbr_read(GPBR2);
	sul_pib_values[PIB_USER_PSR & 0x000F] = gpbr_read(GPBR3);
	sul_pib_values[PIB_USER_HFSR & 0x000F] = gpbr_read(GPBR4);
	sul_pib_values[PIB_USER_CFSR & 0x000F] = gpbr_read(GPBR5);
	sul_pib_values[PIB_USER_R0 & 0x000F] = gpbr_read(GPBR6);
	sul_pib_values[PIB_USER_R1 & 0x000F] = gpbr_read(GPBR7);
	sul_pib_values[PIB_USER_R2 & 0x000F] = gpbr_read(GPBR8);
	sul_pib_values[PIB_USER_R3 & 0x000F] = gpbr_read(GPBR9);
	sul_pib_values[PIB_USER_R12 & 0x000F] = gpbr_read(GPBR10);
#endif
#endif
#if SAMG55
#if PRIME_MODE == PRIME_SN
	sul_pib_values[PIB_USER_RESET_INFO & 0x000F] = gpbr_read(GPBR5);
	sul_pib_values[PIB_USER_PC & 0x000F] = gpbr_read(GPBR6);
	sul_pib_values[PIB_USER_LR & 0x000F] = gpbr_read(GPBR7);
	sul_pib_values[PIB_USER_PSR & 0x000F] = 0;
	sul_pib_values[PIB_USER_HFSR & 0x000F] = 0;
	sul_pib_values[PIB_USER_CFSR & 0x000F] = 0;
	sul_pib_values[PIB_USER_R0 & 0x000F] = 0;
	sul_pib_values[PIB_USER_R1 & 0x000F] = 0;
	sul_pib_values[PIB_USER_R2 & 0x000F] = 0;
	sul_pib_values[PIB_USER_R3 & 0x000F] = 0;
	sul_pib_values[PIB_USER_R12 & 0x000F] = 0;
#else
	sul_pib_values[PIB_USER_RESET_INFO & 0x000F] = gpbr_read(GPBR0);
	sul_pib_values[PIB_USER_PC & 0x000F] = gpbr_read(GPBR1);
	sul_pib_values[PIB_USER_LR & 0x000F] = gpbr_read(GPBR2);
	sul_pib_values[PIB_USER_PSR & 0x000F] = gpbr_read(GPBR3);
	sul_pib_values[PIB_USER_HFSR & 0x000F] = gpbr_read(GPBR4);
	sul_pib_values[PIB_USER_CFSR & 0x000F] = gpbr_read(GPBR5);
	sul_pib_values[PIB_USER_R0 & 0x000F] = gpbr_read(GPBR6);
	sul_pib_values[PIB_USER_R1 & 0x000F] = gpbr_read(GPBR7);
	sul_pib_values[PIB_USER_R2 & 0x000F] = 0;
	sul_pib_values[PIB_USER_R3 & 0x000F] = 0;
	sul_pib_values[PIB_USER_R12 & 0x000F] = 0;
#endif
#endif
#if SAME70
	sul_pib_values[PIB_USER_RESET_INFO & 0x000F] = gpbr_read(GPBR0);
	sul_pib_values[PIB_USER_PC & 0x000F] = gpbr_read(GPBR1);
	sul_pib_values[PIB_USER_LR & 0x000F] = gpbr_read(GPBR2);
	sul_pib_values[PIB_USER_PSR & 0x000F] = gpbr_read(GPBR3);
	sul_pib_values[PIB_USER_HFSR & 0x000F] = gpbr_read(GPBR4);
	sul_pib_values[PIB_USER_CFSR & 0x000F] = gpbr_read(GPBR5);
	sul_pib_values[PIB_USER_R0 & 0x000F] = gpbr_read(GPBR6);
	sul_pib_values[PIB_USER_R1 & 0x000F] = gpbr_read(GPBR7);
	sul_pib_values[PIB_USER_R2 & 0x000F] = 0;
	sul_pib_values[PIB_USER_R3 & 0x000F] = 0;
	sul_pib_values[PIB_USER_R12 & 0x000F] = 0;
#endif
#if SAM4E
	sul_pib_values[PIB_USER_RESET_INFO & 0x000F] = gpbr_read(GPBR0);
	sul_pib_values[PIB_USER_PC & 0x000F] = gpbr_read(GPBR1);
	sul_pib_values[PIB_USER_LR & 0x000F] = gpbr_read(GPBR2);
	sul_pib_values[PIB_USER_PSR & 0x000F] = gpbr_read(GPBR3);
	sul_pib_values[PIB_USER_HFSR & 0x000F] = gpbr_read(GPBR4);
	sul_pib_values[PIB_USER_CFSR & 0x000F] = gpbr_read(GPBR5);
	sul_pib_values[PIB_USER_R0 & 0x000F] = gpbr_read(GPBR6);
	sul_pib_values[PIB_USER_R1 & 0x000F] = gpbr_read(GPBR7);
	sul_pib_values[PIB_USER_R2 & 0x000F] = gpbr_read(GPBR8);
	sul_pib_values[PIB_USER_R3 & 0x000F] = gpbr_read(GPBR9);
	sul_pib_values[PIB_USER_R12 & 0x000F] = gpbr_read(GPBR10);
#endif

	/* Clear registers (except reset information) */
#if SAM4C || SAM4CP || SAM4CM 
#if PRIME_MODE == PRIME_SN
	gpbr_write(GPBR6, 0);
	gpbr_write(GPBR7, 0);
	gpbr_write(GPBR8, 0);
	gpbr_write(GPBR9, 0);
	gpbr_write(GPBR10, 0);
	gpbr_write(GPBR11, 0);
	gpbr_write(GPBR12, 0);
	gpbr_write(GPBR13, 0);
	gpbr_write(GPBR14, 0);
#else
	gpbr_write(GPBR1, 0);
	gpbr_write(GPBR2, 0);
	gpbr_write(GPBR3, 0);
	gpbr_write(GPBR4, 0);
	gpbr_write(GPBR5, 0);
	gpbr_write(GPBR6, 0);
	gpbr_write(GPBR7, 0);
	gpbr_write(GPBR8, 0);
	gpbr_write(GPBR9, 0);
	gpbr_write(GPBR10, 0);
#endif
#endif
#if SAMG55
#if PRIME_MODE == PRIME_SN
	gpbr_write(GPBR6, 0);
	gpbr_write(GPBR7, 0);
#else
	gpbr_write(GPBR1, 0);
	gpbr_write(GPBR2, 0);
	gpbr_write(GPBR3, 0);
	gpbr_write(GPBR4, 0);
	gpbr_write(GPBR5, 0);
	gpbr_write(GPBR6, 0);
	gpbr_write(GPBR7, 0);
#endif
#endif
#if SAME70
	gpbr_write(GPBR1, 0);
	gpbr_write(GPBR2, 0);
	gpbr_write(GPBR3, 0);
	gpbr_write(GPBR4, 0);
	gpbr_write(GPBR5, 0);
	gpbr_write(GPBR6, 0);
	gpbr_write(GPBR7, 0);
#endif
#if SAM4E
	gpbr_write(GPBR1, 0);
	gpbr_write(GPBR2, 0);
	gpbr_write(GPBR3, 0);
	gpbr_write(GPBR4, 0);
	gpbr_write(GPBR5, 0);
	gpbr_write(GPBR6, 0);
	gpbr_write(GPBR7, 0);
	gpbr_write(GPBR8, 0);
	gpbr_write(GPBR9, 0);
	gpbr_write(GPBR10, 0);
#endif
#endif /* #ifndef DISABLE_PIB_HANDLING */
}
