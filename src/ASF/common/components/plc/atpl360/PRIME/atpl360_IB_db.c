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

#include "string.h"
#include "atpl360_IB_db_api.h"
#include "atpl360_comm.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PRODUCT_DESCRIPTION_LEN 10

/** ATPL360_HOST_DESCRIPTION_ID : Product Description identifier(ro)*/
static char puc_description[PRODUCT_DESCRIPTION_LEN];

/** ATPL360_HOST_MODEL_ID : Model identifier (ro) */
static uint16_t us_model_id;

/** ATPL360_HOST_VERSION_ID : Version number identifier */
static uint32_t ul_version;

/** ATPL360_HOST_PRODUCT_ID : Product identifier (ro) */
static uint16_t us_product_id;

static const atpl360_db_param_t param_id[ATPL360_IB_PARAM_END] = {
	{ATPL360_HOST_DESCRIPTION_ID, &puc_description[0], (sizeof(char) * PRODUCT_DESCRIPTION_LEN), true},
	{ATPL360_HOST_MODEL_ID, &us_model_id, sizeof(uint16_t), true},
	{ATPL360_HOST_VERSION_ID, &ul_version, sizeof(uint32_t), true},
	{ATPL360_HOST_PRODUCT_ID, &us_product_id, sizeof(uint16_t), true},
};

/**
 * \brief Initialize all static vars
 *
 */
void atpl360_ib_db_init(void)
{
	memcpy(puc_description, ATPL360_HOST_DESCRIPTION, sizeof(char) * PRODUCT_DESCRIPTION_LEN);
	us_model_id = ATPL360_HOST_MODEL;
	ul_version = ATPL360_HOST_VERSION;
	us_product_id = ATPL360_HOST_PRODUCT;
}

const atpl360_db_param_t *atpl360_get_address_params_table(void)
{
	return param_id;
}

#ifdef __cplusplus
}
#endif
