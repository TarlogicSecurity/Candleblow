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

#include "compiler.h"
/* Phy includes */

/* Serial interface */
#include "serial_if.h"
#include "addon_api.h"
#include "atpl360_IB_db.h"

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

#define MAX_CFG_CMD_LEN      256

static atpl360_addon_descriptor_t *spx_desc;
static pf_addons_event_t pf_trigger_addon_event;
static uint8_t spuc_cfg_value_buf[MAX_CFG_CMD_LEN];

/* Var to store the last reg id to send response after interruption */
static atpl360_id_param_t sus_last_id_get;

/**
 * \internal
 * \brief Memcopy with byte order reversal.
 *
 * Copies puc_buf[] into puc_dst[], re-ordering the bytes to adapt to the serial communication.
 *
 * \param puc_dst    Pointer to buffer where the data will be copied
 * \param puc_src    Pointer to buffer source data
 * \param us_len     Length of data to copy
 */
static void _memcpy_rev(uint8_t *puc_dst, uint8_t *puc_src, uint16_t us_len)
{
	uint8_t *ptr_uc_mem_dst, *ptr_uc_mem_src;
	uint16_t us_idx;

	if (us_len <= 4) {
		ptr_uc_mem_dst = puc_dst + us_len - 1;
		ptr_uc_mem_src = puc_src;
		for (us_idx = 0; us_idx < us_len; us_idx++) {
			*ptr_uc_mem_dst-- = (uint8_t)*ptr_uc_mem_src++;
		}
	} else {
		memcpy(puc_dst, puc_src, us_len);
	}
}

/**
 * \brief Converts Phy REG info to byte buffering in order to report data through ADDONs  (NOT USE IN SNIFFER ADDON)
 *
 * \param puc_dst_data       Pointer to destiny buffer
 * \param puc_src_data       Pointer to buffer to extract reg value
 * \param us_reg_size        Register size
 *
 * \return Length of data to buffering
 */
uint16_t atpl360_addon_stringify_reg(uint8_t *puc_dst_data, uint8_t *puc_src_data, uint16_t us_reg_size)
{
	uint8_t *puc_dst_buf;

	puc_dst_buf = puc_dst_data;

	*puc_dst_buf++ = (uint8_t)SERIAL_IF_PHY_COMMAND_GET_CFG_RSP;
	*puc_dst_buf++ = (uint8_t)(sus_last_id_get >> 8);
	*puc_dst_buf++ = (uint8_t)sus_last_id_get;
	*puc_dst_buf++ = (uint8_t)us_reg_size;
	_memcpy_rev(puc_dst_buf, puc_src_data, us_reg_size);

	return (us_reg_size + 4);
}

/**
 * \brief Function to init addon module
 *
 * \param sx_desc  Pointer to addon descriptor
 *
 */
void atpl360_addon_init(atpl360_addon_descriptor_t *sx_desc)
{
	spx_desc = sx_desc;
}

/**
 * \brief Function to set event callback
 *
 * \param pf_event_callback  Pointer to callback function to call
 *
 */
void atpl360_addon_set_event_callback(pf_addons_event_t pf_event_callback)
{
	pf_trigger_addon_event = pf_event_callback;
}

/**
 * \brief Function to execute addons commnad
 *
 * \param px_msg  Pointer to command message
 * \param us_len  Length of the command message
 *
 */
void atpl360_addon_cmd(uint8_t *px_msg, uint16_t us_len)
{
	uint8_t *puc_rx;
	uint8_t uc_serial_if_cmd;

	/* Protection for invalid us_length */
	if (!us_len) {
		return;
	}

	/* Process received message */
	uc_serial_if_cmd = px_msg[0];
	puc_rx = &px_msg[1];

	switch (uc_serial_if_cmd) {
	/* GET command */
	case SERIAL_IF_PHY_COMMAND_GET_CFG:
	{
		atpl360_id_param_t us_id;
		uint8_t uc_id_len;

		us_id = (atpl360_id_param_t)(((uint16_t)*puc_rx++) << 8);
		us_id += (atpl360_id_param_t)((uint16_t)*puc_rx++);
		sus_last_id_get = us_id;
		uc_id_len = *puc_rx++;
		if (spx_desc->get_config(us_id, spuc_cfg_value_buf, uc_id_len, false)) {
			uint8_t *puc_buf;

			puc_buf = spx_desc->puc_addon_buffer;

			*puc_buf++ = SERIAL_IF_PHY_COMMAND_GET_CFG_RSP;
			*puc_buf++ = (uint8_t)(us_id >> 8);
			*puc_buf++ = (uint8_t)(us_id);
			*puc_buf++ = uc_id_len;
			if (uc_id_len > 4) {
				memcpy(puc_buf, spuc_cfg_value_buf, uc_id_len);
			} else {
				_memcpy_rev(puc_buf, spuc_cfg_value_buf, uc_id_len);
			}

			puc_buf += uc_id_len;

			pf_trigger_addon_event(spx_desc->puc_addon_buffer, puc_buf - spx_desc->puc_addon_buffer);
		}
	}
	break;

	/* SET command */
	case SERIAL_IF_PHY_COMMAND_SET_CFG:
	{
		atpl360_id_param_t us_id;
		uint8_t uc_id_len;

		us_id = (atpl360_id_param_t)(((uint16_t)*puc_rx++) << 8);
		us_id += (atpl360_id_param_t)((uint16_t)*puc_rx++);
		uc_id_len = *puc_rx++;
		_memcpy_rev(spuc_cfg_value_buf, puc_rx, uc_id_len);
		if (spx_desc->set_config(us_id, spuc_cfg_value_buf, uc_id_len)) {
			uint8_t *puc_buf;

			puc_buf = spx_desc->puc_addon_buffer;

			*puc_buf++ = SERIAL_IF_PHY_COMMAND_SET_CFG_RSP;
			*puc_buf++ = (uint8_t)(us_id >> 8);
			*puc_buf++ = (uint8_t)(us_id);
			*puc_buf++ = uc_id_len;
			*puc_buf++ = (uint8_t)true;

			pf_trigger_addon_event(spx_desc->puc_addon_buffer, puc_buf - spx_desc->puc_addon_buffer);
		}
	}
	break;

	/* CMD command (operations over bitfields, only in 8 bits) */
	case SERIAL_IF_PHY_COMMAND_CMD_CFG:
		/* TBD */
		break;

	/* Write command (send data msg) */
	case SERIAL_IF_PHY_COMMAND_SEND_MSG:
	{
		tx_msg_t tx_msg;

		serial_if_parse_tx_msg(&tx_msg, puc_rx);
		spx_desc->send_data(&tx_msg);
	}
	break;

	default:
		break;
	}
}

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */
