/**
 * \file
 *
 * \brief HAL_TRNG
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
#include <stdlib.h>
#include "asf.h"
#include "hal_private.h"

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

/**
 * \brief This function initializes the True Random Number Generator (TRNG)
 */
void hal_trng_init(void)
{
#if (SAMG || SAM4E)
	/* srand seed must change for each service node: (mac value ) */
	x_mac_cfg_t x_eui48;
	uint32_t value;
	uint8_t i;

	hal_get_config_info(CONFIG_TYPE_MAC_INFO, sizeof(x_eui48), &x_eui48);

	value = 0;
	/* read mac as uint32_t  */
	for (i = 0; i < 5; i++) {
		value += (x_eui48.uc_mac[5 - i] << (8 * i));
	}

	/*Init srand module with mac number seed */
	srand(value);
#else
	/* Enable TRNG clock domain */
	pmc_enable_periph_clk(ID_TRNG);

	/* Enable TRNG */
	trng_enable(TRNG);
#endif
}

/**
 * \brief  This function reads out information from the TRNG
 * \return Data read from the TRNG
 */
uint32_t hal_trng_read(void)
{
#if (SAMG || SAM4E)
	return (uint32_t)rand();

#else
	/* Wait until the device is ready to be read */
	while ((trng_get_interrupt_status(TRNG) & TRNG_ISR_DATRDY) != TRNG_ISR_DATRDY) {
	}

	/* Read out TRNG*/
	return trng_read_output_data(TRNG);
#endif
}

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */
