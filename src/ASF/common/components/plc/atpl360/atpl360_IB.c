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
#include "atpl360_IB.h"
#include "string.h"
/* Address to the table of with all parameters of database*/
static const atpl360_db_param_t *px_db;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Initializes the addresses of the parameters of the information base
 */
void atpl360_ib_init(void)
{
	px_db = atpl360_get_address_params_table();
	atpl360_ib_db_init();
}

/** \name Functions to access to Information Base of ATPL360 component
 */
/* ! @{ */

/**
 * \brief retrieve if possible a parameter from the information base
 *
 * \param id_param the identifier of the parameter
 * \param value pointer where the parameter is going to be stored
 * \param size_param size of the parameter
 */
bool atpl360_ib_get_param(atpl360_id_param_t id_param, void *value, uint8_t size_param)
{
	uint16_t us_i;
	uint16_t us_idx = ATPL360_IB_PARAM_END;

	for (us_i = 0; us_i < ATPL360_IB_PARAM_END; us_i++) {
		if (id_param == px_db[us_i].id) {
			us_idx = us_i;
			break;
		}
	}

	/* Checking that is a correct ID */
	if (us_idx >= ATPL360_IB_PARAM_END) {
		return false;
	}

	/* Checking that is right length */
	if (size_param != px_db[us_idx].size) {
		return false;
	}

	/* Now the parameter can be copied safely */
	memcpy(value, px_db[us_idx].ptr_storage, size_param);

	return true;
}

/**
 * \brief set if possible the value of a parameter in the information base
 *
 * \param id_param the identifier of the parameter
 * \param value pointer where the parameter is going to be stored
 * \param size_param size of the parameter
 */
bool atpl360_ib_set_param(atpl360_id_param_t id_param, void *value, uint8_t size_param)
{
	uint16_t us_i;
	uint16_t us_idx = ATPL360_IB_PARAM_END;
	/* Find index in db table */

	for (us_i = 0; us_i < ATPL360_IB_PARAM_END; us_i++) {
		if (id_param == px_db[us_i].id) {
			us_idx = us_i;
			break;
		}
	}

	/* Checking that is a correct ID */
	if (us_idx >= ATPL360_IB_PARAM_END) {
		return false;
	}

	/* Checking that is right length */
	if (size_param != px_db[us_idx].size) {
		return false;
	}

	/* Checking that is not read only */
	if (px_db[us_idx].b_read_only) {
		return false;
	}

	/* Now the parameter can be copied safely */
	memcpy(px_db[us_idx].ptr_storage, value, size_param);

	return true;
}

/* ! @} */

#ifdef __cplusplus
}
#endif
