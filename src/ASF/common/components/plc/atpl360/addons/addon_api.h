/**
 * \file
 *
 * \brief ATPL360_Host Serial Interface for Physical layer
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

#ifndef ADDON_API_H_INCLUDED
#define ADDON_API_H_INCLUDED

#include "compiler.h"
#include "atpl360_comm.h"
#include "atpl360.h"

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

/* ! \name ATPL360 descriptor */
typedef struct atpl360_addon_descriptor {
	pf_send_data_t send_data;
	pf_mng_get_cfg_t get_config;
	pf_mng_set_cfg_t set_config;
	uint8_t *puc_addon_buffer;
} atpl360_addon_descriptor_t;

/* ! \name Addon interface */
/* @{ */
void atpl360_addon_init(atpl360_addon_descriptor_t *desc);
void atpl360_addon_set_event_callback(pf_addons_event_t pf_event_callback);
uint16_t atpl360_addon_stringify_ind(uint8_t *puc_ind_data, rx_msg_t *px_ind_data);
uint16_t atpl360_addon_stringify_cfm(uint8_t *puc_cfm_data, tx_cfm_t *px_cfm_data);
uint16_t atpl360_addon_stringify_tx(uint8_t *puc_tx_data, tx_msg_t *px_tx_data);
uint16_t atpl360_addon_stringify_reg(uint8_t *puc_dst_data, uint8_t *puc_src_data, uint16_t us_reg_size);
void atpl360_addon_cmd(uint8_t *px_msg, uint16_t us_len);

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */

#endif   /* ADDON_API_H_INCLUDED */
