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

/* Sniffer interface */
#include "sniffer_if.h"
#include "addon_api.h"
#include "general_defs.h"

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif

/**INDENT-ON**/
/* / @endcond */

/** USI protocol */
#define SNIFFER_USI_PROTOCOL                            0x13

/** \brief SNIFFER version */
/* @{ */
#define SNIFFER_ATPL230                                 0x01
#define SNIFFER_PL360_PRIME                             0x11
#define SNIFFER_VERSION                                 0x14
/* @} */

/** \brief PRIME Time definitions */
/* @{ */
#define TIME_PRIME_1_3_PREAMBLE_US                      2048L
#define TIME_PRIME_1_3_HEADER_US                        4480L
#define TIME_OFDM_SYMBOL_US                             2240L
#define TIME_PRIME_PLUS_PREAMBLE_US                     (TIME_PRIME_1_3_PREAMBLE_US * 4)
#define TIME_PRIME_PLUS_HEADER_US                       (TIME_OFDM_SYMBOL_US * 4)
/* @} */

static atpl360_addon_descriptor_t *sx_atpl360_snif_desc;

static uint8_t spuc_tx_msg_buf[2][544];  /* !<  Tx working copy buffer to report to sniffer in cfm event */
static uint32_t spul_last_buf_len[2];

/**
 * \brief Get duration message in microseconds
 *
 * \param mod_type       PRIME mode type
 * \param uc_symbols     Number of symbols in message
 *
 * \return Duration of message in microseconds
 */
static uint32_t _get_msg_duration_time(enum mode_types mod_type, uint8_t uc_symbols)
{
	uint32_t ul_duration;

	if (mod_type == MODE_TYPE_A) {
		ul_duration = TIME_PRIME_1_3_PREAMBLE_US + TIME_PRIME_1_3_HEADER_US + (uc_symbols *  TIME_OFDM_SYMBOL_US);
	} else if (mod_type == MODE_TYPE_B) {
		ul_duration = TIME_PRIME_PLUS_PREAMBLE_US + TIME_PRIME_PLUS_HEADER_US + (uc_symbols * TIME_OFDM_SYMBOL_US);
	} else if (mod_type == MODE_TYPE_BC) {
		ul_duration = TIME_PRIME_1_3_PREAMBLE_US + TIME_PRIME_1_3_HEADER_US + TIME_PRIME_PLUS_PREAMBLE_US + TIME_PRIME_PLUS_HEADER_US +
				(uc_symbols * TIME_OFDM_SYMBOL_US);
	} else {
		ul_duration = 0;
	}

	return ul_duration;
}

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
	uint32_t ul_time_ini, ul_time_end, ul_duration, ul_len;
	uint8_t *puc_dst_buf;
	uint8_t uc_symbols, uc_channel, uc_snr;

	if (px_ind_data->us_data_len == 0) {
		return 0;
	}

	puc_dst_buf = puc_ind_data;

	if (px_ind_data->uc_mod_type == MODE_TYPE_A) {
		*puc_dst_buf++ = SNIFFER_IF_PHY_MESSAGE_TYPE_A;
	} else if (px_ind_data->uc_mod_type == MODE_TYPE_B) {
		*puc_dst_buf++ = SNIFFER_IF_PHY_MESSAGE_TYPE_B;
	} else if (px_ind_data->uc_mod_type == MODE_TYPE_BC) {
		*puc_dst_buf++ = SNIFFER_IF_PHY_MESSAGE_TYPE_BC;
	} else {
		return 0;
	}

	*puc_dst_buf++ = (uint8_t)SNIFFER_VERSION;
	*puc_dst_buf++ = (uint8_t)SNIFFER_PL360_PRIME;
	*puc_dst_buf++ = (uint8_t)px_ind_data->uc_scheme;

	sx_atpl360_snif_desc->get_config(ATPL360_REG_RX_PAY_SYMBOLS, &uc_symbols, 1, true);
	*puc_dst_buf++ = uc_symbols;

	/* SNR = (QT/(3 * 4)) + 1 */
	/* QT = CINR_MIN + 12 */
	uc_snr = ((px_ind_data->uc_cinr_min + 12) / 12) + 1;
	if (uc_snr > 7) {
		uc_snr = 7;
	}

	*puc_dst_buf++ = uc_snr;
	*puc_dst_buf++ = (px_ind_data->uc_cinr_avg + 12 + 2) / 4; /* Added 2 extra to make round instead trunk */

	sx_atpl360_snif_desc->get_config(ATPL360_REG_CHANNEL_CFG, &uc_channel, 1, true);
	*puc_dst_buf++ = uc_channel;

	*puc_dst_buf++ = (uint8_t)px_ind_data->uc_cinr_min;
	*puc_dst_buf++ = (uint8_t)px_ind_data->uc_ber_soft;
	*puc_dst_buf++ = (uint8_t)px_ind_data->uc_ber_soft_max;

	/* padding (reserved bytes) */
	*puc_dst_buf++ = 0;
	*puc_dst_buf++ = 0;
	*puc_dst_buf++ = 0;
	*puc_dst_buf++ = 0;
	*puc_dst_buf++ = 0;
	*puc_dst_buf++ = 0;
	*puc_dst_buf++ = 0;
	*puc_dst_buf++ = 0;

	/* Adapt RX TIME to 10us base */
	ul_time_ini = px_ind_data->ul_rx_time;

	ul_duration = _get_msg_duration_time(px_ind_data->uc_mod_type, uc_symbols);
	ul_time_end = ul_time_ini + ul_duration;

	*puc_dst_buf++ = (uint8_t)(ul_time_ini >> 24);
	*puc_dst_buf++ = (uint8_t)(ul_time_ini >> 16);
	*puc_dst_buf++ = (uint8_t)(ul_time_ini >> 8);
	*puc_dst_buf++ = (uint8_t)(ul_time_ini);

	*puc_dst_buf++ = (uint8_t)(ul_time_end >> 24);
	*puc_dst_buf++ = (uint8_t)(ul_time_end >> 16);
	*puc_dst_buf++ = (uint8_t)(ul_time_end >> 8);
	*puc_dst_buf++ = (uint8_t)(ul_time_end);

	*puc_dst_buf++ = 0;
	*puc_dst_buf++ = px_ind_data->uc_rssi_avg;

	/* mac_enable not supported */
	*puc_dst_buf++ = 0;

	/* Compute length with CRC, */
	ul_len = px_ind_data->us_data_len;

	*puc_dst_buf++ = (uint8_t)(ul_len >> 8);
	*puc_dst_buf++ = (uint8_t)(ul_len);

	memcpy(puc_dst_buf, px_ind_data->puc_data_buf, ul_len);
	puc_dst_buf += ul_len;

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
	uint32_t ul_time_ini, ul_time_end, ul_duration, ul_len;
	uint8_t *puc_dst_buf;
	uint8_t uc_symbols;

	if (px_cfm_data->uc_tx_result != TX_RESULT_SUCCESS) {
		return 0;
	}

	puc_dst_buf = spuc_tx_msg_buf[px_cfm_data->uc_buffer_id];

	puc_dst_buf += 4;

	/* Get Tx symbols */
	sx_atpl360_snif_desc->get_config(ATPL360_REG_TX_PAY_SYMBOLS, &uc_symbols, 1, true);
	*puc_dst_buf++ = uc_symbols;

	puc_dst_buf += 14;

	/* adapat time base ref */
	ul_time_ini = px_cfm_data->ul_tx_time;

	/* get Tx time ini */
	*puc_dst_buf++ = (uint8_t)(ul_time_ini >> 24);
	*puc_dst_buf++ = (uint8_t)(ul_time_ini >> 16);
	*puc_dst_buf++ = (uint8_t)(ul_time_ini >> 8);
	*puc_dst_buf++ = (uint8_t)(ul_time_ini);

	/* Tx time end will fill in cfm event */
	ul_duration = _get_msg_duration_time(px_cfm_data->uc_mod_type, uc_symbols);
	ul_time_end = ul_time_ini + ul_duration;

	*puc_dst_buf++ = (uint8_t)(ul_time_end >> 24);
	*puc_dst_buf++ = (uint8_t)(ul_time_end >> 16);
	*puc_dst_buf++ = (uint8_t)(ul_time_end >> 8);
	*puc_dst_buf = (uint8_t)(ul_time_end);

	/* restore buffering len */
	ul_len = spul_last_buf_len[px_cfm_data->uc_buffer_id];

	/* copy internal buffering to app buffer */
	memcpy(puc_cfm_data, spuc_tx_msg_buf[px_cfm_data->uc_buffer_id], ul_len);

	/* Return buffering len stored in tx event */
	return ul_len;
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
	(void)puc_dst_data;
	(void)puc_src_data;
	(void)us_reg_size;

	return 0;
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
	uint32_t ul_len;
	uint8_t *puc_dst_buf;
	uint8_t uc_channel;

	(void)puc_tx_data;

	puc_dst_buf = spuc_tx_msg_buf[px_tx_data->uc_buffer_id];

	if (px_tx_data->uc_mod_type == MODE_TYPE_A) {
		*puc_dst_buf++ = SNIFFER_IF_PHY_MESSAGE_TYPE_A;
	} else if (px_tx_data->uc_mod_type == MODE_TYPE_B) {
		*puc_dst_buf++ = SNIFFER_IF_PHY_MESSAGE_TYPE_B;
	} else if (px_tx_data->uc_mod_type == MODE_TYPE_BC) {
		*puc_dst_buf++ = SNIFFER_IF_PHY_MESSAGE_TYPE_BC;
	} else {
		return 0;
	}

	*puc_dst_buf++ = (uint8_t)SNIFFER_VERSION;
	*puc_dst_buf++ = (uint8_t)SNIFFER_PL360_PRIME;
	*puc_dst_buf++ = (uint8_t)px_tx_data->uc_scheme;

	/* Tx symbols will fill in cfm event */
	*puc_dst_buf++ = 0;

	*puc_dst_buf++ = 7;               /* SNR */
	*puc_dst_buf++ = 60;              /* EX_SNR */

	/* channel */
	sx_atpl360_snif_desc->get_config(ATPL360_REG_CHANNEL_CFG, &uc_channel, 1, true);
	*puc_dst_buf++ = uc_channel;

	*puc_dst_buf++ = 255;
	*puc_dst_buf++ = 0;
	*puc_dst_buf++ = 0;

	/* padding (reserved bytes) */
	*puc_dst_buf++ = 0;
	*puc_dst_buf++ = 0;
	*puc_dst_buf++ = 0;
	*puc_dst_buf++ = 0;
	*puc_dst_buf++ = 0;
	*puc_dst_buf++ = 0;
	*puc_dst_buf++ = 0;
	*puc_dst_buf++ = 0;

	/* Tx time ini will fill in cfm event */
	*puc_dst_buf++ = 0;
	*puc_dst_buf++ = 0;
	*puc_dst_buf++ = 0;
	*puc_dst_buf++ = 0;
	/* Tx time end will fill in cfm event */
	*puc_dst_buf++ = 0;
	*puc_dst_buf++ = 0;
	*puc_dst_buf++ = 0;
	*puc_dst_buf++ = 0;

	*puc_dst_buf++ = 0;
	/* Fix RSSI */
	*puc_dst_buf++ = 140;

	/* mac_enable not supported */
	*puc_dst_buf++ = 0;

	/* Compute length with CRC, */
	ul_len = px_tx_data->us_data_len;

	*puc_dst_buf++ = (uint8_t)(ul_len >> 8);
	*puc_dst_buf++ = (uint8_t)(ul_len);

	memcpy(puc_dst_buf, px_tx_data->puc_data_buf, ul_len);
	puc_dst_buf += ul_len;

	/* Store buffering len to used in cfm event */
	ul_len = puc_dst_buf - spuc_tx_msg_buf[px_tx_data->uc_buffer_id];
	spul_last_buf_len[px_tx_data->uc_buffer_id] = ul_len;

	return ul_len;
}

/**
 * \brief Function to execute addons commnad
 *
 * \param puc_rx_msg  Pointer to command message
 * \param us_len  Length of the command message
 *
 */
void atpl360_addon_cmd(uint8_t *puc_rx_msg, uint16_t us_len)
{
	uint8_t uc_sniffer_if_cmd;
	uint8_t uc_data;

	/* Protection for invalid length */
	if (!us_len) {
		return;
	}

	/* Process received message */
	uc_sniffer_if_cmd  = puc_rx_msg[0];
	uc_data = puc_rx_msg[1];

	switch (uc_sniffer_if_cmd) {
	/* GET command */
	case SNIFFER_IF_PHY_COMMAND_SET_CHANNEL:
		sx_atpl360_snif_desc->set_config(ATPL360_REG_CHANNEL_CFG, (void *)&uc_data, 1);
		break;

	case SNIFFER_IF_PHY_COMMAND_ENABLE_PRIME_PLUS_ROBUST:
	case SNIFFER_IF_PHY_COMMAND_MESSAGE:
		/* NOT IMPLEMENTED */
		break;

	default:
		break;
	}
}

void sniffer_if_init(atpl360_addon_descriptor_t *sx_desc)
{
	sx_atpl360_snif_desc = sx_desc;
}

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */
