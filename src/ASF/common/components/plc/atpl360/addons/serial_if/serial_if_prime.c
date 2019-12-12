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

/* System includes */
#include "compiler.h"
#include "atpl360_comm.h"

/* Serial interface */
#include "serial_if.h"
#include "addon_api.h"
#include "general_defs.h"

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif

/**INDENT-ON**/
/* / @endcond */

/* Carriers Buffer Len */
#define SERIAL_IF_CARR_BUFFER_LEN  16

static uint8_t spuc_serial_data_buf[808];  /* !<  Receive working buffer */

/**
 * \brief Converts Phy RX Data struct to byte buffering in order to report data through ADDONs
 *
 * \param puc_ind_data       Pointer to destiny buffer
 * \param pv_src             Pointer to struct to convert
 *
 * \return Length of data to buffering
 */
uint16_t atpl360_addon_stringify_ind(uint8_t *puc_ind_data, rx_msg_t *px_ind_data)
{
	uint8_t *puc_dst_buf;

	puc_dst_buf = puc_ind_data;

	*puc_dst_buf++ = (uint8_t)SERIAL_IF_PHY_COMMAND_RECEIVE_MSG;
	*puc_dst_buf++ = (uint8_t)px_ind_data->uc_mod_type;
	*puc_dst_buf++ = (uint8_t)px_ind_data->uc_scheme;
	*puc_dst_buf++ = (uint8_t)px_ind_data->uc_header_type;
	*puc_dst_buf++ = (uint8_t)px_ind_data->uc_ber_soft;
	*puc_dst_buf++ = (uint8_t)px_ind_data->uc_ber_soft_max;
	*puc_dst_buf++ = (uint8_t)px_ind_data->uc_cinr_min;
	*puc_dst_buf++ = (uint8_t)px_ind_data->uc_cinr_avg;
	*puc_dst_buf++ = (uint8_t)px_ind_data->uc_rssi_avg;
	*puc_dst_buf++ = (uint8_t)(px_ind_data->us_evm_header >> 8);
	*puc_dst_buf++ = (uint8_t)(px_ind_data->us_evm_header);
	*puc_dst_buf++ = (uint8_t)(px_ind_data->us_evm_payload >> 8);
	*puc_dst_buf++ = (uint8_t)(px_ind_data->us_evm_payload);
	*puc_dst_buf++ = (uint8_t)(px_ind_data->ul_evm_header_acum >> 24);
	*puc_dst_buf++ = (uint8_t)(px_ind_data->ul_evm_header_acum >> 16);
	*puc_dst_buf++ = (uint8_t)(px_ind_data->ul_evm_header_acum >> 8);
	*puc_dst_buf++ = (uint8_t)(px_ind_data->ul_evm_header_acum);
	*puc_dst_buf++ = (uint8_t)(px_ind_data->ul_evm_payload_acum >> 24);
	*puc_dst_buf++ = (uint8_t)(px_ind_data->ul_evm_payload_acum >> 16);
	*puc_dst_buf++ = (uint8_t)(px_ind_data->ul_evm_payload_acum >> 8);
	*puc_dst_buf++ = (uint8_t)(px_ind_data->ul_evm_payload_acum);
	*puc_dst_buf++ = (uint8_t)(px_ind_data->ul_rx_time >> 24);
	*puc_dst_buf++ = (uint8_t)(px_ind_data->ul_rx_time >> 16);
	*puc_dst_buf++ = (uint8_t)(px_ind_data->ul_rx_time >> 8);
	*puc_dst_buf++ = (uint8_t)(px_ind_data->ul_rx_time);
	*puc_dst_buf++ = (uint8_t)px_ind_data->uc_nar_bnd_percent;
	*puc_dst_buf++ = (uint8_t)px_ind_data->uc_imp_percent;

	if (px_ind_data->uc_mod_type == MODE_TYPE_A) {
		/* remove Generic Data Frame Header */
		uint16_t us_data_len;

		us_data_len = px_ind_data->us_data_len - 3;
		*puc_dst_buf++ = (uint8_t)(us_data_len >> 8);
		*puc_dst_buf++ = (uint8_t)(us_data_len);
		memcpy(puc_dst_buf, px_ind_data->puc_data_buf + 3, us_data_len);
		puc_dst_buf += us_data_len;
	} else {
		*puc_dst_buf++ = (uint8_t)(px_ind_data->us_data_len >> 8);
		*puc_dst_buf++ = (uint8_t)(px_ind_data->us_data_len);
		memcpy(puc_dst_buf, px_ind_data->puc_data_buf, px_ind_data->us_data_len);
		puc_dst_buf += px_ind_data->us_data_len;
	}

	return (puc_dst_buf - puc_ind_data);
}

/**
 * \brief Converts Phy TX Cfm struct to byte buffering in order to report data through ADDONs
 *
 * \param puc_cfm_data       Pointer to destiny buffer
 * \param *px_cfm_data       Pointer to struct to convert
 *
 * \return Length of data to buffering
 */
uint16_t atpl360_addon_stringify_cfm(uint8_t *puc_cfm_data, tx_cfm_t *px_cfm_data)
{
	uint8_t *puc_dst_buf;

	puc_dst_buf = puc_cfm_data;

	*puc_dst_buf++ = (uint8_t)SERIAL_IF_PHY_COMMAND_SEND_MSG_RSP;
	*puc_dst_buf++ = (uint8_t)px_cfm_data->uc_buffer_id;
	*puc_dst_buf++ = (uint8_t)px_cfm_data->uc_tx_result;
	*puc_dst_buf++ = (uint8_t)(px_cfm_data->ul_rms_calc >> 24);
	*puc_dst_buf++ = (uint8_t)(px_cfm_data->ul_rms_calc >> 16);
	*puc_dst_buf++ = (uint8_t)(px_cfm_data->ul_rms_calc >> 8);
	*puc_dst_buf++ = (uint8_t)(px_cfm_data->ul_rms_calc);
	*puc_dst_buf++ = (uint8_t)(px_cfm_data->ul_tx_time >> 24);
	*puc_dst_buf++ = (uint8_t)(px_cfm_data->ul_tx_time >> 16);
	*puc_dst_buf++ = (uint8_t)(px_cfm_data->ul_tx_time >> 8);
	*puc_dst_buf++ = (uint8_t)(px_cfm_data->ul_tx_time);

	return (puc_dst_buf - puc_cfm_data);
}

/**
 * \brief Converts Phy TX info to byte buffering in order to report data through ADDONs   (NOT USE IN SERIAL ADDON)
 *
 * \param puc_tx_data       Pointer to destiny buffer
 * \param px_tx_data        Pointer to struct to convert
 *
 * \return Length of data to buffering
 */
uint16_t atpl360_addon_stringify_tx(uint8_t *puc_tx_data, tx_msg_t *px_tx_data)
{
	(void)puc_tx_data;
	(void)px_tx_data;

	return 0;
}

/**
 * \brief Extract TX info from data buffer
 *
 * \param px_tx_msg         Pointer to struct to extract data
 * \param puc_tx_data       Pointer to tx data buffer
 *
 */
void serial_if_parse_tx_msg(tx_msg_t *px_tx_msg, uint8_t *puc_tx_data)
{
	uint8_t *puc_data;

	puc_data = puc_tx_data;

	/* Update write parameters of TX struct */
	px_tx_msg->uc_buffer_id = (enum buffer_id)*puc_data++;
	px_tx_msg->uc_att_level = *puc_data++;
	px_tx_msg->uc_mod_type = (enum mode_types)*puc_data++;
	px_tx_msg->uc_scheme = (enum mod_schemes)*puc_data++;
	px_tx_msg->uc_tx_mode = *puc_data++;
	px_tx_msg->ul_tx_time = ((uint32_t)*puc_data++) << 24;
	px_tx_msg->ul_tx_time += ((uint32_t)*puc_data++) << 16;
	px_tx_msg->ul_tx_time += ((uint32_t)*puc_data++) << 8;
	px_tx_msg->ul_tx_time += (uint32_t)*puc_data++;
	px_tx_msg->uc_disable_rx = *puc_data++;
	px_tx_msg->us_data_len = ((uint16_t)*puc_data++) << 8;
	px_tx_msg->us_data_len += (uint16_t)*puc_data++;
	px_tx_msg->puc_data_buf = spuc_serial_data_buf;

	/* protect memory: check data len */
	if (px_tx_msg->us_data_len > sizeof(spuc_serial_data_buf)) {
		px_tx_msg->us_data_len = 0;
		return;
	}

	if (px_tx_msg->uc_mod_type == MODE_TYPE_A) {
		/* Generic Data Frame */
		spuc_serial_data_buf[0] = 0xAA;
		spuc_serial_data_buf[1] = 0xAA;
		spuc_serial_data_buf[2] = 0xAA;
		px_tx_msg->us_data_len += 3;
		/* copy data */
		memcpy(spuc_serial_data_buf + 3, puc_data, px_tx_msg->us_data_len);
	} else {
		/* copy data */
		memcpy(spuc_serial_data_buf, puc_data, px_tx_msg->us_data_len);
	}
}

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */
