/**
 * \file
 *
 * \brief HAL_REGIONS : PRIME Hardware Regions Definitions
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

#ifndef HAL_REGIONS_H_INCLUDE
#define HAL_REGIONS_H_INCLUDE

#include "board.h"

/* @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* @endcond */

#if (SAM4CP || SAM4CMP16_0 || SAM4CMS16_0 || SAM4C || SAM4E || SAMG || SAME70)
#define HAL_FLASH_PAGE_SIZE                    IFLASH_PAGE_SIZE
#elif (SAM4SD32)
#define HAL_FLASH_PAGE_SIZE                    IFLASH0_PAGE_SIZE
#else
#  error No known regions for Atmel PLC boot defined
#endif

typedef struct {
	uint32_t ul_address;
	uint32_t ul_size;
	bool in_use;
} x_fu_region_cfg_t;

/** \brief Configuration parameters */
/* @{ */

#define HAL_USER_SIGNATURE_SIZE                (HAL_FLASH_PAGE_SIZE / (sizeof(uint32_t)))
#define HAL_MACCFG_OFFSET_USER_SIGN            0
#define HAL_PHYCFG_OFFSET_USER_SIGN            16
#define HAL_BNINFO_OFFSET_USER_SIGN            32
#define HAL_PRIME_MODE_OFFSET_USER_SIGN        48
#define HAL_SECURITY_OFFSET_USER_SIGN          64
#define HAL_BOOT_INFO_OFFSET_USER_SIGN         112

/* @} */

/** Firmware configuration */
#define HAL_FLASH_4PAGE_SIZE                   (HAL_FLASH_PAGE_SIZE << 2)
#define HAL_FLASH_16PAGE_SIZE                  (HAL_FLASH_PAGE_SIZE << 4)
#define HAL_FLASH_PAGES_PER_SECTOR             128
#define HAL_FLASH_SECTOR_SIZE                  (HAL_FLASH_PAGES_PER_SECTOR * HAL_FLASH_PAGE_SIZE)

/* @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* @endcond */
#endif /* HAL_REGIONS_H_INCLUDE */
