/**
 * \file
 *
 * \brief HAL_USI: PLC Service Universal Serial Interface
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

#include <string.h>
#include "conf_usi.h"
#include "hal_private.h"

#if BOARD == PL360G55CF_EK
#include "conf_usb.h"
#endif

/* @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* @endcond */

#ifdef NUM_PORTS

/** Default empty PHY serialization function */
uint8_t Dummy_serial_parser(uint8_t *puc_rx_msg, uint16_t us_len);

/** Invalid port index */
#define INVALID_PORT_IDX  0xFF

/** \brief Minimum overhead introduced by the USI protocol */
/** \note (1 Start Byte, 2 Bytes (Len+Protocol), 1 End Byte, 1 CRC Byte) */
/* @{ */
#define MIN_OVERHEAD    5
/* @} */

/** \brief Special characters used by USI */
/* @{ */
/** Start/end mark in message */
#define MSGMARK       0x7e
/** Escaped start/end mark */
#define ESC_MSGMARK   0x5e
/** Escape mark in message */
#define ESCMARK       0x7d
/** Escaped escape mark */
#define ESC_ESCMARK   0x5d
/* @} */

/** \brief MSG started and ended flags */
/* @{ */
#define MSG_START 1
#define MSG_END   0
/* @} */

/** \brief USI header and CRC lengths */
/* @{ */
#define HEADER_LEN  2
#define CRC8_LEN    1
#define CRC16_LEN   2
#define CRC32_LEN   4
/* @} */

/** \brief USI flow control internal protocol
 * \note This internal protocol has no effect on NUM_PORTS constants
 * defined in conf_usi.h because it has no interaction with other
 * modules messages. It aims to provide flow control between the embedded USI
 * and the USI Host.
 */
/* @{ */
/** Commands for internal protocol messages */
/* @{ */
#define CMD_LOCK_PORT        0
#define CMD_UNLOCK_PORT      1
/* @} */
/** Lenght of internal messages */
/* @{ */
#define INTERNAL_MSG_LEN               1
#define MAX_ESCAPED_INTERNAL_MSG_LEN  10
/* @} */
/** Timeouts for port blocking (in 10ms) and update function callback */
/* @{ */
/** 50ms */
#define TX_BLOCK_TIMEOUT   5
#define RX_BLOCK_TIMEOUT   5
#define TXRX_BLOCK_UPDATE  1
/* @} */
/* @} */

/** \brief USI protocol header format */
/* @{ */
#define TYPE_PROTOCOL_OFFSET       1
#define TYPE_PROTOCOL_MSK       0x3F
#define LEN_PROTOCOL_HI_OFFSET     0
#define LEN_PROTOCOL_HI_MSK     0xFF
#define LEN_PROTOCOL_HI_SHIFT      2
#define LEN_PROTOCOL_LO_OFFSET     1
#define LEN_PROTOCOL_LO_MSK     0xC0
#define LEN_PROTOCOL_LO_SHIFT      6
#define XLEN_PROTOCOL_OFFSET       2
#define XLEN_PROTOCOL_MSK       0x80
#define XLEN_PROTOCOL_SHIFT_L      3
#define XLEN_PROTOCOL_SHIFT_R     10
#define PAYLOAD_OFFSET             2
#define CMD_PROTOCOL_OFFSET        2
#define CMD_PROTOCOL_MSK        0x7F
/* @} */

/** \brief Macro operators
 * \note A: HI, B: LO, C:Xlen
 */
/* @{ */
#define TYPE_PROTOCOL(A)       ((A)&TYPE_PROTOCOL_MSK)
#define LEN_PROTOCOL(A, B)     ((((uint16_t)(A)) << LEN_PROTOCOL_HI_SHIFT) + ((B) >> LEN_PROTOCOL_LO_SHIFT))
#define XLEN_PROTOCOL(A, B, C) ((((uint16_t)(A)) << LEN_PROTOCOL_HI_SHIFT) \
	+ ((B) >> LEN_PROTOCOL_LO_SHIFT) \
	+ (((uint16_t)(C)&XLEN_PROTOCOL_MSK) << XLEN_PROTOCOL_SHIFT_L))
#define LEN_HI_PROTOCOL(A)    (((uint16_t)(A) >> LEN_PROTOCOL_HI_SHIFT) & LEN_PROTOCOL_HI_MSK)
#define LEN_LO_PROTOCOL(A)    (((uint16_t)(A) << LEN_PROTOCOL_LO_SHIFT) & LEN_PROTOCOL_LO_MSK)
#define LEN_EX_PROTOCOL(A)    (((uint16_t)(A & 0x0400)) >> 3)
#define CMD_PROTOCOL(A)       ((A)&CMD_PROTOCOL_MSK)
/* @} */

/** USI command structure */
typedef struct {
	/** Protocol Type */
	uint8_t uc_p_type;
	/** Pointer to data buffer */
	uint8_t *puc_buf;
	/** Length of data */
	uint16_t us_len;
} cmd_params_t;

/** USI communication control parameters structure */
typedef struct {
	/** Buffer index */
	uint16_t us_idx_in;
	/** Buffer data length */
	uint16_t us_len;
} USI_param_t;

/** Reception states */
enum {
	/** Inactive */
	RX_IDLE,
	/** Receiving message */
	RX_MSG,
	/** Processing escape char */
	RX_ESC,
	/** Message received correctly */
	RX_EORX
};

/** Transmission states */
enum {
	/** Inactive */
	TX_IDLE,
	/** Transmitting message */
	TX_MSG
};

/** USI communication control parameters (one entry per port) */
static USI_param_t usi_cfg_param[NUM_PORTS];

/** Flag to check len integrity in usi process */
static bool sb_check_len;

/** \brief Types of serial port */
/* @{ */
#define UART_TYPE   0
#define USART_TYPE  1
#define NULL_DEV_TYPE  2
#define USB_TYPE  3
/* @} */

/** Protocol configuration structure */
typedef struct {
	/** Serialization function pointer */
	uint8_t (*serialization_function)(uint8_t *puc_rx_msg, uint16_t us_len);
	/** Protocol communication port */
	uint8_t uc_port_idx;
} protocol_config_t;

/** Port configuration */
typedef struct {
	/** Serial Communication Type */
	uint8_t uc_s_type;
	/** Port number */
	uint8_t uc_chn;
	/** Port speed (bauds) */
	uint32_t ul_speed;
	/** Port buffer size in transmission */
	uint16_t us_tx_size;
	/** Port buffer size in reception */
	uint16_t us_rx_size;
} map_ports_t;

/** Buffer handling structure */
typedef struct {
	/** Size of data in the buffer */
	uint16_t us_size;
	/** Size of the first USI message in the buffer */
	uint16_t us_current_msg_size;
	/** Pointer to the buffer */
	uint8_t *const puc_buf;
} map_buffers_t;

/** Function Pointers for USI protocols */
#undef CONF_PORT
#define CONF_PORT(type, channel, speed, tx_size, rx_size) {type, channel, speed, tx_size, rx_size}

/** \brief Port Mapping. Configured with the values provided in conf_usi.h:
 * \note MapPorts[PORT TYPE, PORT CHANNEL, PORT SPEED, TX BUFFER SIZE, RX BUFFER SIZE]
 */
static const map_ports_t usiMapPorts[NUM_PORTS + 1] = {
#ifdef PORT_0
	PORT_0,
#endif
#ifdef PORT_1
	PORT_1,
#endif
#ifdef PORT_2
	PORT_2,
#endif
#ifdef PORT_3
	PORT_3,
#endif
	{0xff, 0xff, 0, 0, 0}
};

/** Port configuration for buffers */
#undef CONF_PORT
#define CONF_PORT(type, channel, speed, tx_size, rx_size) rx_size

/** \brief Reception buffers */
/* @{ */
#ifdef PORT_0
static uint8_t puc_rxbuf0[PORT_0];
#endif

#ifdef PORT_1
static uint8_t puc_rxbuf1[PORT_1];
#endif

#ifdef PORT_2
static uint8_t puc_rxbuf2[PORT_2];
#endif

#ifdef PORT_3
static uint8_t puc_rxbuf3[PORT_3];
#endif
/* @} */

/** Reception Buffers mapping */
static map_buffers_t usi_rx_buf[NUM_PORTS + 1] = {
#ifdef PORT_0
	{0, 0, &puc_rxbuf0[0]},
#endif
#ifdef PORT_1
	{0, 0, &puc_rxbuf1[0]},
#endif
#ifdef PORT_2
	{0, 0, &puc_rxbuf2[0]},
#endif
#ifdef PORT_3
	{0, 0, &puc_rxbuf3[0]},
#endif
	{0xFF, 0, NULL}
};

/** \brief Port configuration for buffers */
#undef CONF_PORT
#define CONF_PORT(type, channel, speed, tx_size, rx_size) tx_size

/** \brief Transmission Buffers */
/* @{ */
#ifdef PORT_0
static uint8_t puc_txbuf0[PORT_0];
#endif
#ifdef PORT_1
static uint8_t puc_txbuf1[PORT_1];
#endif
#ifdef PORT_2
static uint8_t puc_txbuf2[PORT_2];
#endif
#ifdef PORT_3
static uint8_t puc_txbuf3[PORT_3];
#endif
/* @} */

/** \brief Transmission Buffers mapping */
static const map_buffers_t usi_tx_buf[NUM_PORTS + 1] = {
#ifdef PORT_0
	{PORT_0, 0, &puc_txbuf0[0]},
#endif
#ifdef PORT_1
	{PORT_1, 0, &puc_txbuf1[0]},
#endif
#ifdef PORT_2
	{PORT_2, 0, &puc_txbuf2[0]},
#endif
#ifdef PORT_3
	{PORT_3, 0, &puc_txbuf3[0]},
#endif
	{0xFF, 0, NULL}
};

/** \brief Transmission buffers size configuration */
/* @{ */
#define TXAUX_SIZE 0
#ifdef PORT_0
  #if (PORT_0 > TXAUX_SIZE)
    #undef TXAUX_SIZE
    #define TXAUX_SIZE     PORT_0
  #endif
#endif
#ifdef PORT_1
  #if (PORT_1 > TXAUX_SIZE)
    #undef TXAUX_SIZE
    #define TXAUX_SIZE     PORT_1
  #endif
#endif
#ifdef PORT_2
  #if (PORT_2 > TXAUX_SIZE)
    #undef TXAUX_SIZE
    #define TXAUX_SIZE     PORT_2
  #endif
#endif
#ifdef PORT_3
  #if (PORT_3 > TXAUX_SIZE)
    #undef TXAUX_SIZE
    #define TXAUX_SIZE     PORT_3
  #endif
#endif
/* @} */

/** Configuration of mapped ports */
const map_ports_t *const usi_cfg_map_ports = &usiMapPorts[0];

/** Configuration of transmission buffers */
const map_buffers_t *const usi_cfg_tx_buf = &usi_tx_buf[0];

/** Configuration of reception buffers */
map_buffers_t *const usi_cfg_rx_buf = &usi_rx_buf[0];

/** Configuration of mapped protocols */
static protocol_config_t usi_cfg_map_protocols[USI_NUMBER_OF_PROTOCOLS];

/**
 * \brief Function that translates protocol IDs to indexes for the
 *        protocol tables (configuration and callback functions).
 *
 * \param protocol_id    Protocol identifier
 *
 * \return Protocol index
 */
static uint8_t _usi_prot_id2idx(usi_protocol_t protocol_id)
{
	uint8_t uc_prot_idx;

	switch (protocol_id) {
	case PROTOCOL_MNGP_PRIME_GETQRY:
	case PROTOCOL_MNGP_PRIME_GETRSP:
	case PROTOCOL_MNGP_PRIME_SET:
	case PROTOCOL_MNGP_PRIME_RESET:
	case PROTOCOL_MNGP_PRIME_REBOOT:
	case PROTOCOL_MNGP_PRIME_FU:
	case PROTOCOL_MNGP_PRIME_GETQRY_EN:
	case PROTOCOL_MNGP_PRIME_GETRSP_EN:
		uc_prot_idx = 0;
		break;

	case PROTOCOL_SNIF_PRIME:
		uc_prot_idx = 1;
		break;

	case PROTOCOL_ATPL230:
		uc_prot_idx = 2;
		break;

	case PROTOCOL_USER_DEFINED:
		uc_prot_idx = 3;
		break;

	case PROTOCOL_PRIME_API:
		uc_prot_idx = 4;
		break;

	case PROTOCOL_INTERNAL:
		uc_prot_idx = 5;
		break;

	case PROTOCOL_PHY_SERIAL:
		uc_prot_idx = 6;
		break;

	default:
		uc_prot_idx = USI_INVALID_PROTOCOL_IDX;
	}

	return(uc_prot_idx);
}

/**
 * \brief  This function processes the complete received message.
 *
 * \retval true if there is no error
 * \retval false if length is invalid or command is wrong
 */
static uint8_t _process_msg(uint8_t *puc_rx_buf)
{
	uint8_t (*pf_serialization_function)(uint8_t *, uint16_t);
	uint16_t us_len;
	uint8_t uc_type;
	uint8_t uc_result = false;

	/* Extract protocol */
	uc_type = TYPE_PROTOCOL(puc_rx_buf[TYPE_PROTOCOL_OFFSET]);

	/* Extract length */
	if (uc_type == PROTOCOL_PRIME_API) {
		/* Extract LEN using XLEN */
		us_len = XLEN_PROTOCOL(puc_rx_buf[LEN_PROTOCOL_HI_OFFSET], puc_rx_buf[LEN_PROTOCOL_LO_OFFSET], puc_rx_buf[XLEN_PROTOCOL_OFFSET]);
	} else {
		/* Extract LEN using LEN */
		us_len = LEN_PROTOCOL(puc_rx_buf[LEN_PROTOCOL_HI_OFFSET], puc_rx_buf[LEN_PROTOCOL_LO_OFFSET]);
	}

	/* Protection for invalid length */
	if (!us_len && !((uc_type == PROTOCOL_MNGP_PRIME_RESET) || (uc_type == PROTOCOL_MNGP_PRIME_REBOOT))) {
		return false;
	}

	/* Call decoding function depending on uc_type */
	switch (uc_type) {
	/* PRIME spec.v.1.3.E */
	case PROTOCOL_MNGP_PRIME_GETQRY:
	case PROTOCOL_MNGP_PRIME_GETRSP:
	case PROTOCOL_MNGP_PRIME_SET:
	case PROTOCOL_MNGP_PRIME_RESET:
	case PROTOCOL_MNGP_PRIME_REBOOT:
	case PROTOCOL_MNGP_PRIME_FU:
	case PROTOCOL_MNGP_PRIME_GETQRY_EN:
	case PROTOCOL_MNGP_PRIME_GETRSP_EN:
		pf_serialization_function = usi_cfg_map_protocols[_usi_prot_id2idx(PROTOCOL_MNGP_PRIME)].serialization_function;
		if (pf_serialization_function) {
			/* MNGL spec. including header 2bytes) */
			uc_result = pf_serialization_function(puc_rx_buf, us_len + 2);
		}

		break;

	/* Atmel's serialized protocols */
	case PROTOCOL_PHY_SERIAL:
		pf_serialization_function = usi_cfg_map_protocols[_usi_prot_id2idx(PROTOCOL_PHY_SERIAL)].serialization_function;
		if (pf_serialization_function) {
			uc_result = pf_serialization_function(&puc_rx_buf[PAYLOAD_OFFSET], us_len);
		}

		break;

	case PROTOCOL_ATPL230:
		pf_serialization_function = usi_cfg_map_protocols[_usi_prot_id2idx(PROTOCOL_ATPL230)].serialization_function;
		if (pf_serialization_function) {
			uc_result = pf_serialization_function(&puc_rx_buf[PAYLOAD_OFFSET], us_len);
		}

		break;

	case PROTOCOL_SNIF_PRIME:
		pf_serialization_function = usi_cfg_map_protocols[_usi_prot_id2idx(PROTOCOL_SNIF_PRIME)].serialization_function;
		if (pf_serialization_function) {
			uc_result = pf_serialization_function(&puc_rx_buf[PAYLOAD_OFFSET], us_len);
		}

		break;

	case PROTOCOL_PRIME_API:
		pf_serialization_function = usi_cfg_map_protocols[_usi_prot_id2idx(PROTOCOL_PRIME_API)].serialization_function;
		if (pf_serialization_function) {
			uc_result = pf_serialization_function(&puc_rx_buf[PAYLOAD_OFFSET], us_len);
		}

		break;

	case PROTOCOL_USER_DEFINED:
		pf_serialization_function = usi_cfg_map_protocols[_usi_prot_id2idx(PROTOCOL_USER_DEFINED)].serialization_function;
		if (pf_serialization_function) {
			uc_result = pf_serialization_function(&puc_rx_buf[PAYLOAD_OFFSET], us_len);
		}

		break;

	default:
		break;
	}

	return uc_result;
}

/**
 * \brief  This function processes received message
 *
 *  \param  puc_rx_buf       Pointer to the received buffer
 *  \param  us_msg_size      Buffer length
 *
 *  \return true      if message is OK
 *          false     if message is not OK
 */
static uint8_t _doEoMsg(uint8_t *puc_rx_buf, uint16_t us_msg_size)
{
	uint32_t ul_rx_crc;
	uint32_t ul_ev_crc;
	uint8_t *puc_tb;
	uint16_t us_len;
	uint8_t uc_type;

	/* Get buffer and number of bytes */
	if (us_msg_size < 4) {    /* insufficient data */
		hal_debug_report(USI_NOT_ENOUGH_DATA);
		return false;
	}

	/* Extract length and protocol */
	us_len = LEN_PROTOCOL(puc_rx_buf[LEN_PROTOCOL_HI_OFFSET], puc_rx_buf[LEN_PROTOCOL_LO_OFFSET]);
	uc_type = TYPE_PROTOCOL(puc_rx_buf[TYPE_PROTOCOL_OFFSET]);

	/* Protection for invalid length */
	if (!us_len && !((uc_type == PROTOCOL_MNGP_PRIME_RESET) || (uc_type == PROTOCOL_MNGP_PRIME_REBOOT))) {
		hal_debug_report(USI_BAD_LENGTH);
		return false;
	}

	/* Evaluate CRC depending on protocol */
	switch (uc_type) {
	case PROTOCOL_MNGP_PRIME_GETQRY:
	case PROTOCOL_MNGP_PRIME_GETRSP:
	case PROTOCOL_MNGP_PRIME_SET:
	case PROTOCOL_MNGP_PRIME_RESET:
	case PROTOCOL_MNGP_PRIME_REBOOT:
	case PROTOCOL_MNGP_PRIME_FU:
	case PROTOCOL_MNGP_PRIME_GETQRY_EN:
	case PROTOCOL_MNGP_PRIME_GETRSP_EN:
		/* Get received CRC 32 */
		puc_tb = &puc_rx_buf[us_msg_size - 4];
		ul_rx_crc = (((uint32_t)puc_tb[0]) << 24) | (((uint32_t)puc_tb[1]) << 16) | (((uint32_t)puc_tb[2]) << 8) | ((uint32_t)puc_tb[3]);
		/* Calculate CRC */
		/* +2 header bytes: included in CRC */
		ul_ev_crc = hal_pcrc_calc(puc_rx_buf, us_len + 2, HAL_PCRC_HT_USI, HAL_PCRC_CRC_TYPE_32, false);
		break;

	case PROTOCOL_SNIF_PRIME:
	case PROTOCOL_PHY_SERIAL:
	case PROTOCOL_ATPL230:
	case PROTOCOL_USER_DEFINED:
		/* Get received CRC 16 */
		puc_tb = &puc_rx_buf[us_msg_size - 2];
		ul_rx_crc = (((uint32_t)puc_tb[0]) << 8) | ((uint32_t)puc_tb[1]);
		/* Calculate CRC */
		/* +2 header bytes: included in CRC */
		ul_ev_crc = hal_pcrc_calc(puc_rx_buf, us_len + 2, HAL_PCRC_HT_USI, HAL_PCRC_CRC_TYPE_16, false);
		break;

	/* Length is up to 2Kb ... use XLEN field */
	case PROTOCOL_PRIME_API:
		/* Get received CRC 8: use XLEN */
		us_len = XLEN_PROTOCOL(puc_rx_buf[LEN_PROTOCOL_HI_OFFSET], puc_rx_buf[LEN_PROTOCOL_LO_OFFSET], puc_rx_buf[XLEN_PROTOCOL_OFFSET]);
		puc_tb = &puc_rx_buf[us_msg_size - 1];
		ul_rx_crc = (uint32_t)puc_tb[0];
		/* Calculate CRC */
		/* +2 header bytes: included in CRC */
		ul_ev_crc = hal_pcrc_calc(puc_rx_buf, us_len + 2, HAL_PCRC_HT_USI, HAL_PCRC_CRC_TYPE_8, false);
		break;

	default:
		hal_debug_report(USI_BAD_PROTOCOL);
		return false;
	}

	/* Return CRC ok or not */
	if (ul_rx_crc == ul_ev_crc) {
		return true;
	} else {
		hal_debug_report(USI_BAD_CRC);
		return false;
	}
}

/**
 * \brief     Shifts the transmission buffer one byte to the right, starting
 *            from the byte following the byte to escape, and thus creating
 *            room for the escaped byte.
 *            Note that the check to see if there is enough space for it
 *            must be done previous to calling this function.
 *
 */
static void _usi_shift_buffer_right(uint8_t *puc_tx_buf, uint16_t us_idx, uint16_t us_len)
{
	uint16_t us_orig, i;
	uint8_t uc_backup_chars[2];
	uint8_t uc_backup_char_idx;

	/* Get current buffer index (byte following the character to escape)*/
	us_orig = us_idx;

	/* Start with the second element in the backup char array */
	uc_backup_char_idx = 1;
	uc_backup_chars[uc_backup_char_idx] = puc_tx_buf[us_orig++];
	/* Toggle backup char index */
	uc_backup_char_idx = 1 - uc_backup_char_idx;
	i = 0;
	while (i < us_len) {
		uc_backup_chars[uc_backup_char_idx] = puc_tx_buf[us_orig + i];
		/* Toggle backup char index */
		uc_backup_char_idx = 1 - uc_backup_char_idx;
		/* Write the stored char in its adjacent position to the right */
		puc_tx_buf[us_orig + i] = uc_backup_chars[uc_backup_char_idx];
		i++;
	}
}

/**
 * \brief     Encodes the escape characters and transmits the message
 *
 *  \param    uc_port_idx  Port to transmit through
 *  \param    msg       Pointer to data to be transmitted
 *
 *
 *  \return   Result of operation:  USI_OK: Sent
 *                                  USI_STATUS_TX_FAILED: Not sent
 */
static usi_status_t _usi_encode_and_send(uint8_t uc_port_idx, cmd_params_t *msg)
{
	uint32_t ul_idx_in_orig;
	uint32_t ul_idx_aux = 0;
	uint32_t ul_crc;
	uint32_t ul_size_coded = 0;
	uint16_t us_len = 0;
	uint16_t us_len_token = 0;
	uint16_t us_sent_chars;
	uint8_t *puc_tx_buf;
	uint8_t *puc_tx_buf_ini;
	uint8_t *puc_next_token = NULL;
	uint8_t *puc_aux_next_token = NULL;
	uint8_t uc_cmd;
	uint8_t uc_delimiter = MSGMARK;
	uint8_t uc_escape    = ESCMARK;
	uint8_t uc_escaped_byte = 0;
	uint8_t uc_p_type = msg->uc_p_type;

	/* Len protection */
	if (msg->us_len == 0) {
		return USI_STATUS_FORMAT_ERROR;
	}

	us_len = msg->us_len;

	/* Get ptr to TxBuffer */
	puc_tx_buf = usi_cfg_tx_buf[uc_port_idx].puc_buf;
	puc_tx_buf_ini = puc_tx_buf;

	/* Start Escape */
	ul_idx_in_orig = usi_cfg_param[uc_port_idx].us_idx_in;
	ul_idx_aux = usi_cfg_param[uc_port_idx].us_idx_in;
	*puc_tx_buf++ = MSGMARK;
	/* Copy message to TX buffer including header */
	*puc_tx_buf++ = LEN_HI_PROTOCOL(us_len);
	*puc_tx_buf++ = LEN_LO_PROTOCOL(us_len) + TYPE_PROTOCOL(uc_p_type);

	memcpy(puc_tx_buf, msg->puc_buf, us_len);
	/* Adjust XLEN if uc_p_type is internal protocol */
	uc_cmd = msg->puc_buf[0];
	if (uc_p_type == PROTOCOL_PRIME_API) {
		puc_tx_buf[0] = LEN_EX_PROTOCOL(us_len) + CMD_PROTOCOL(uc_cmd);
	}

	puc_tx_buf += us_len;

	/* Update the index to the end of the data to send */
	ul_idx_aux = puc_tx_buf - puc_tx_buf_ini;

	/* Add 1 MSGMARK + 2 header bytes to LEN */
	us_len += 3;

	/* Calculate CRC */
	switch (uc_p_type) {
	case PROTOCOL_MNGP_PRIME_GETQRY:
	case PROTOCOL_MNGP_PRIME_GETRSP:
	case PROTOCOL_MNGP_PRIME_SET:
	case PROTOCOL_MNGP_PRIME_RESET:
	case PROTOCOL_MNGP_PRIME_REBOOT:
	case PROTOCOL_MNGP_PRIME_FU:
	case PROTOCOL_MNGP_PRIME_GETQRY_EN:
	case PROTOCOL_MNGP_PRIME_GETRSP_EN:
		ul_crc = hal_pcrc_calc(&puc_tx_buf_ini[ul_idx_in_orig + 1], msg->us_len + 2, HAL_PCRC_HT_USI, HAL_PCRC_CRC_TYPE_32, false);
		*puc_tx_buf++ = (uint8_t)(ul_crc >> 24);
		*puc_tx_buf++ = (uint8_t)(ul_crc >> 16);
		*puc_tx_buf++ = (uint8_t)(ul_crc >> 8);
		*puc_tx_buf++ = (uint8_t)ul_crc;
		us_len += 4;
		break;

	case PROTOCOL_SNIF_PRIME:
	case PROTOCOL_PHY_SERIAL:
	case PROTOCOL_ATPL230:
	case PROTOCOL_USER_DEFINED:
		ul_crc = hal_pcrc_calc(&puc_tx_buf_ini[ul_idx_in_orig + 1], msg->us_len + 2, HAL_PCRC_HT_USI, HAL_PCRC_CRC_TYPE_16, false);
		*puc_tx_buf++ = (uint8_t)(ul_crc >> 8);
		*puc_tx_buf++ = (uint8_t)(ul_crc);
		us_len += 2;
		break;

	case PROTOCOL_PRIME_API:
	default:
		ul_crc = hal_pcrc_calc(&puc_tx_buf_ini[ul_idx_in_orig + 1], msg->us_len + 2, HAL_PCRC_HT_USI, HAL_PCRC_CRC_TYPE_8, false);
		*puc_tx_buf++ = (uint8_t)(ul_crc);
		us_len += 1;
		break;
	}

	/* Fill tx buffer adding required escapes */
	/* +1 for the still missing end MSGMARKs */
	ul_size_coded = us_len + 1;
	/* Check if there is still room */
	if (ul_idx_in_orig + ul_size_coded > usi_cfg_tx_buf[uc_port_idx].us_size) {
		/* No Room. Return error */
		return USI_STATUS_TX_BUFFER_OVERFLOW;
	}

	/* Reposition the index to the beginning of the data to send */
	ul_idx_aux = ul_idx_in_orig + 1; /* Skip the initial MSGMARK */
	us_len--; /* Skip the initial MSGMARK */

	puc_tx_buf = puc_tx_buf_ini;
	while (us_len) {
		/* Look for the next MSGMARK present within the data */
		puc_next_token = memchr(&puc_tx_buf[ul_idx_in_orig + ul_idx_aux], uc_delimiter, us_len);
		if (puc_next_token == NULL) {
			/* MSGMARK not found -> look for the next ESCMARK
			 * present within the data */
			puc_next_token = memchr(&puc_tx_buf[ul_idx_in_orig + ul_idx_aux], uc_escape, us_len);
			if (puc_next_token != NULL) {
				uc_escaped_byte = ESC_ESCMARK;
			}
		} else {
			/* Check if there is an ESCMARK before the MSGMARK */
			puc_aux_next_token = memchr(&puc_tx_buf[ul_idx_in_orig + ul_idx_aux],
					uc_escape,
					puc_next_token - &puc_tx_buf[ul_idx_in_orig + ul_idx_aux]);
			if (puc_aux_next_token != NULL) {
				uc_escaped_byte = ESC_ESCMARK;
				puc_next_token = puc_aux_next_token;
			} else {
				uc_escaped_byte = ESC_MSGMARK;
			}
		}

		/* Perform the codification of the MSGMARK or the ESCMARK */
		if (puc_next_token != NULL) {
			ul_size_coded++;

			/* Check if there is still room */
			if (ul_idx_in_orig + ul_size_coded > usi_cfg_tx_buf[uc_port_idx].us_size) {
				/* No Room. Reset index and return error */
				return USI_STATUS_TX_BUFFER_OVERFLOW;
			}

			us_len_token = puc_next_token - &puc_tx_buf[ul_idx_in_orig + ul_idx_aux];
			/* Encode the special character */
			ul_idx_aux += us_len_token;
			puc_tx_buf[ul_idx_aux++] = ESCMARK;
			us_len -= us_len_token + 1;

			/* Shift the whole buffer by one byte to make room for
			 * the escape character */
			_usi_shift_buffer_right(puc_tx_buf, ul_idx_aux, us_len);

			puc_tx_buf[ul_idx_aux++] = uc_escaped_byte;
		} else { /* No MSGMARK or ESCMARK found. The buffer is complete */
			ul_idx_aux += us_len;
			us_len = 0;
		}
	}

	puc_tx_buf[ul_idx_aux++] = MSGMARK;

	/* Message ready to be sent */
	usi_cfg_param[uc_port_idx].us_idx_in = ul_idx_aux;
	us_len = ul_idx_aux;

	/* Check if there is something to transmit */
	if (usi_cfg_map_ports[uc_port_idx].uc_s_type == UART_TYPE) {
		while (!hal_uart_is_free(usi_cfg_map_ports[uc_port_idx].uc_chn)) {
			/* Wait last tx end */
		}
	} else if (usi_cfg_map_ports[uc_port_idx].uc_s_type == USART_TYPE) {
		while (!hal_usart_is_free(usi_cfg_map_ports[uc_port_idx].uc_chn)) {
			/* Wait last tx end */
		}
	} else if (usi_cfg_map_ports[uc_port_idx].uc_s_type == USB_TYPE) {
#ifdef UDI_CDC_PORT_NB
		while (!hal_usb_udc_is_tx_ready()) {
			/* Wait last tx end */
		}
#endif
	}

	while (usi_cfg_param[uc_port_idx].us_idx_in) {
		/* Send chars to device, checking how many have been
		 * really processed by device */
		if (usi_cfg_map_ports[uc_port_idx].uc_s_type == UART_TYPE) {
			us_sent_chars = hal_uart_write(usi_cfg_map_ports[uc_port_idx].uc_chn,
					usi_cfg_tx_buf[uc_port_idx].puc_buf,
					us_len);
		} else if (usi_cfg_map_ports[uc_port_idx].uc_s_type == USART_TYPE) {
			us_sent_chars = hal_usart_write(usi_cfg_map_ports[uc_port_idx].uc_chn,
					usi_cfg_tx_buf[uc_port_idx].puc_buf,
					us_len);
		} else if (usi_cfg_map_ports[uc_port_idx].uc_s_type == NULL_DEV_TYPE) {
			us_sent_chars = hal_null_dev_write(usi_cfg_map_ports[uc_port_idx].uc_chn,
					usi_cfg_tx_buf[uc_port_idx].puc_buf,
					us_len);
#ifdef UDI_CDC_PORT_NB
		} else if (usi_cfg_map_ports[uc_port_idx].uc_s_type == USB_TYPE) {
			us_sent_chars = hal_usb_udc_write_buf(usi_cfg_tx_buf[uc_port_idx].puc_buf, us_len);
#endif
		} else {
			/* Port wrongly mapped: can't send. Do not retry */
			us_sent_chars = us_len;
		}

		if (us_sent_chars > 0) {
			/* Adjust buffer values depending on sent chars */
			usi_cfg_param[uc_port_idx].us_idx_in -= us_sent_chars;
		} else {
			/* Discard msg */
			usi_cfg_param[uc_port_idx].us_idx_in -= us_len;
			/* USI_ERROR: UART/USART error */
			return USI_STATUS_UART_ERROR;
		}
	}

	return USI_STATUS_OK;
}

/**
 * \brief     Shifts the given buffer us_n bytes to the left, starting from
 *            the us_n-th + 1 byte (that will then become the first one).
 *            The us_n last positions of the buffer are padded with 0.
 *
 *  \param    puc_buf  Pointer to the byte to de-escape.
 *  \param    us_n     Number pf bytes to shift.
 *  \param    us_len   Size of the buffer to shift.
 *
 */
static void _usi_shift_buffer_left(uint8_t *puc_buf, int16_t us_n, int16_t us_len)
{
	uint16_t i = us_n;

	/* Start with the us_n-th element in the array */
	while (i < us_len) {
		puc_buf[i - us_n] = puc_buf[i];
		i++;
	}
	/* Zero pad the end of the buffer */
	i = us_len - us_n;
	while (i < us_len) {
		puc_buf[i++] = 0;
	}
}

/** \brief   Decodes the escape sequences and updates the input buffer
 *           accordingly.
 *
 *  \param   puc_start  pointer to the input buffer
 *  \param   puc_end    pointer to the output buffer
 *
 *  \return  Updated length of the decoded array,
 *           0 if error.
 */
static uint16_t _decode_copy(uint8_t *puc_start, uint8_t *puc_end)
{
	uint16_t i = 0;
	uint16_t us_curr_size;

	us_curr_size = puc_end - puc_start - 1;
	if (!us_curr_size) {
		return 0;
	}

	while (i < us_curr_size) {
		if (puc_start[i] == ESCMARK) {
			/* If the escaped byte is the MSGMARK, */
			/* copy it to the destination buffer */
			if (puc_start[i + 1] == ESC_MSGMARK) {
				puc_start[i] = MSGMARK;
				_usi_shift_buffer_left(&puc_start[i + 1], 1, us_curr_size - i);
				us_curr_size--;
				i++;
			} else if (puc_start[i + 1] == ESC_ESCMARK) {
				/* Idem, for ESCMARK */
				puc_start[i] = ESCMARK;
				_usi_shift_buffer_left(&puc_start[i + 1], 1, us_curr_size - i);
				us_curr_size--;
				i++;
			} else { /* Error: unknown escaped character */
				return 0;
			}
		} else {
			/* No escape mark: continue */
			i++;
		}
	}

	return(us_curr_size);
}

/**
 * \brief  This function checks len of the received message
 *
 *  \param  puc_rx_start       Pointer to start msg
 *  \param  us_msg_len         Message len to check
 *  \return true      if message is OK
 *          false     if message is not OK
 */
static bool _check_integrity_len(uint8_t *puc_rx_start, uint16_t us_msg_len)
{
	uint8_t *puc_data;
	uint16_t us_len;
	uint8_t uc_type;

	/* get msg size */
	puc_data = puc_rx_start + 1;

	/* Get buffer and number of bytes */
	if (us_msg_len < 4) {    /* insufficient data */
		return false;
	}

	/* Extract length and protocol */
	us_len = LEN_PROTOCOL(puc_data[LEN_PROTOCOL_HI_OFFSET], puc_data[LEN_PROTOCOL_LO_OFFSET]);
	uc_type = TYPE_PROTOCOL(puc_data[TYPE_PROTOCOL_OFFSET]);

	/* Add CRC bytes depending on protocol */
	switch (uc_type) {
	case PROTOCOL_MNGP_PRIME_GETQRY:
	case PROTOCOL_MNGP_PRIME_GETRSP:
	case PROTOCOL_MNGP_PRIME_SET:
	case PROTOCOL_MNGP_PRIME_RESET:
	case PROTOCOL_MNGP_PRIME_REBOOT:
	case PROTOCOL_MNGP_PRIME_FU:
	case PROTOCOL_MNGP_PRIME_GETQRY_EN:
	case PROTOCOL_MNGP_PRIME_GETRSP_EN:
		us_len += 4;
		break;

	case PROTOCOL_SNIF_PRIME:
	case PROTOCOL_PHY_SERIAL:
	case PROTOCOL_ATPL230:
	case PROTOCOL_USER_DEFINED:
		us_len += 2;
		break;

	/* Length is up to 2Kb ... use XLEN field */
	case PROTOCOL_PRIME_API:
		/* Get received CRC 8: use XLEN */
		us_len = XLEN_PROTOCOL(puc_data[LEN_PROTOCOL_HI_OFFSET], puc_data[LEN_PROTOCOL_LO_OFFSET], puc_data[XLEN_PROTOCOL_OFFSET]);
		us_len++;
		break;

	default:
		return false;
	}

	/* Add header bytes */
	us_len += 2;

	if (us_len == us_msg_len) {
		return true;
	} else {
		return false;
	}
}

/**
 * \brief Function to perform the USI RX process.
 */
void hal_usi_process(void)
{
	uint8_t *puc_rx_start;
	uint8_t *puc_rx_aux;
	uint8_t *puc_first_token;
	uint8_t *puc_last_token;
	uint16_t us_msg_size;
	uint16_t us_msg_size_new = 0;
	uint16_t us_msg_size_pending;
	uint8_t uc_port_idx;

	/* Check reception on every port */
	for (uc_port_idx = 0; uc_port_idx < NUM_PORTS; uc_port_idx++) {
		us_msg_size_pending = usi_cfg_rx_buf[uc_port_idx].us_size;
		puc_rx_aux = &usi_cfg_rx_buf[uc_port_idx].puc_buf[0] + us_msg_size_pending;
		puc_rx_start = &usi_cfg_rx_buf[uc_port_idx].puc_buf[0];

		/* Read all the data in the respective buffer (UART or USART) */
		if (usi_cfg_map_ports[uc_port_idx].uc_s_type == UART_TYPE) {
			us_msg_size_new = hal_uart_read(usi_cfg_map_ports[uc_port_idx].uc_chn, puc_rx_aux, usi_cfg_map_ports[uc_port_idx].us_rx_size);
		} else if (usi_cfg_map_ports[uc_port_idx].uc_s_type == USART_TYPE) {
			us_msg_size_new = hal_usart_read(usi_cfg_map_ports[uc_port_idx].uc_chn, puc_rx_aux, usi_cfg_map_ports[uc_port_idx].us_rx_size);
		} else if (usi_cfg_map_ports[uc_port_idx].uc_s_type == NULL_DEV_TYPE) {
			us_msg_size_new = hal_null_dev_read(usi_cfg_map_ports[uc_port_idx].uc_chn, puc_rx_aux, usi_cfg_map_ports[uc_port_idx].us_rx_size);
#ifdef UDI_CDC_PORT_NB
		} else if (usi_cfg_map_ports[uc_port_idx].uc_s_type == USB_TYPE) {
			us_msg_size_new = hal_usb_udc_read_buf(puc_rx_aux, usi_cfg_map_ports[uc_port_idx].us_rx_size);
#endif
		}

		/* Find first byte 0x7E */
		us_msg_size_pending += us_msg_size_new;

		if (us_msg_size_pending) {
			puc_first_token = memchr(puc_rx_start, (uint8_t)MSGMARK, us_msg_size_pending);
			if (puc_first_token) {
				/* Update RX pointer to first decoded data byte */
				puc_rx_aux = puc_first_token + 1;
				/* Process received data */
				puc_last_token = memchr(puc_rx_aux, (uint8_t)MSGMARK, us_msg_size_pending);
				if (puc_last_token) {
					uint16_t us_msg_dec_size;
					/* Decode message */
					us_msg_size = puc_last_token - puc_rx_start + 1;
					us_msg_dec_size = _decode_copy(puc_first_token, puc_last_token);
					/* Check integrity len to resync frames */
					if (sb_check_len) {
						sb_check_len = false;
						/* Check integrity LEN */
						if (!_check_integrity_len(puc_first_token, us_msg_dec_size)) {
							/* ERROR: discard message except last 0x7E. It is the first token of the next frame */
							_usi_shift_buffer_left(puc_rx_start, us_msg_size - 1, us_msg_size_pending);
							usi_cfg_rx_buf[uc_port_idx].us_size = us_msg_size_pending - (us_msg_size - 1);
							return;
						}
					}

					/* Calculate CRC */
					if (_doEoMsg(puc_rx_aux, us_msg_dec_size)) {
						/* CRC is OK: process the message */
						_process_msg(puc_rx_aux);
					}

					/* Update RX pointer to next process */
					_usi_shift_buffer_left(puc_rx_start, us_msg_size, us_msg_size_pending);
					usi_cfg_rx_buf[uc_port_idx].us_size = us_msg_size_pending - us_msg_size;
				} else {
					/* ERROR: Not 0x7E as last byte. Wait and check integrity through msg len in next process */
					sb_check_len = true;
					usi_cfg_rx_buf[uc_port_idx].us_size = us_msg_size_pending;
				}
			} else {
				/* ERROR: Not 0x7E as first byte. Discard RX */
				usi_cfg_rx_buf[uc_port_idx].us_size = 0;
				memset(puc_rx_start, 0, us_msg_size_pending);
			}
		}
	}
}

/**
 * \brief Create RX and TX USI tasks, and timer to update internal counters.
 */
void hal_usi_init(void)
{
	uint8_t i;

	/* Init callback functions list */
	for (i = 0; i < USI_NUMBER_OF_PROTOCOLS; i++) {
		usi_cfg_map_protocols[i].uc_port_idx = INVALID_PORT_IDX;
		usi_cfg_map_protocols[i].serialization_function = NULL;
	}

	/* Initialize Universal Serial Interface and start ports */
	for (i = 0; i < NUM_PORTS; i++) {
		/* Init Tx Parameters */
		usi_cfg_param[i].us_idx_in = 0;

		/* Start USI port */
		if (usi_cfg_map_ports[i].uc_s_type == UART_TYPE) {
			hal_uart_open(usi_cfg_map_ports[i].uc_chn, usi_cfg_map_ports[i].ul_speed);
		} else if (usi_cfg_map_ports[i].uc_s_type == USART_TYPE) {
			hal_usart_open(usi_cfg_map_ports[i].uc_chn, usi_cfg_map_ports[i].ul_speed);
		} else if (usi_cfg_map_ports[i].uc_s_type == NULL_DEV_TYPE) {
			hal_null_dev_open(usi_cfg_map_ports[i].uc_chn, usi_cfg_map_ports[i].ul_speed);
#ifdef UDI_CDC_PORT_NB
		} else if (usi_cfg_map_ports[i].uc_s_type == USB_TYPE) {
			hal_usb_udc_start();
#endif
		}
	}

	/* Init internal flag */
	sb_check_len = false;
}

/** @brief  Function to transmit data through USI
 *
 *  @param    msg    Pointer to message to be transmitted.
 *
 *  @return   USI operation result
 */
usi_status_t hal_usi_send_cmd(void *msg)
{
	uint16_t us_available_len;
	uint16_t us_len = ((cmd_params_t *)msg)->us_len;
	uint8_t uc_p_type = ((cmd_params_t *)msg)->uc_p_type;
	uint8_t uc_port_idx;
	uint8_t uc_protocol_idx;

	/* Get port index from protocol */
	uc_protocol_idx = _usi_prot_id2idx((usi_protocol_t)uc_p_type);
	if (uc_protocol_idx  == USI_INVALID_PROTOCOL_IDX) {
		return USI_STATUS_PROTOCOL_NOT_FOUND;
	}

	uc_port_idx = usi_cfg_map_protocols[uc_protocol_idx].uc_port_idx;
	/* Check if the protocol has been registered to use USI */
	if (uc_port_idx == INVALID_PORT_IDX) {
		return USI_STATUS_PROTOCOL_NOT_REGISTERED;
	}

	/* Get available length in buffer */
	us_available_len = usi_cfg_tx_buf[uc_port_idx].us_size - usi_cfg_param[uc_port_idx].us_idx_in;

	/* First checking, available length at least equal to minimum required space */
	if (us_available_len < (us_len + MIN_OVERHEAD)) {
		return USI_STATUS_RX_BUFFER_OVERFLOW;
	}

	return (_usi_encode_and_send(uc_port_idx, (cmd_params_t *)msg));
}

/**
 * \brief Sets a callback function for the serialization of a given protocol
 *
 * \param protocol_id    ID of the protocol to be serialized.
 * \param p_handler      Serialization callback function pointer.
 * \param usi_port       USI port for the protocol communication
 */
usi_status_t hal_usi_set_callback(usi_protocol_t protocol_id, uint8_t (*p_handler)(uint8_t *puc_rx_msg, uint16_t us_len), uint8_t usi_port)
{
	uint8_t uc_prot_idx;

	uc_prot_idx = _usi_prot_id2idx(protocol_id);

	if (uc_prot_idx  == USI_INVALID_PROTOCOL_IDX) {
		return(USI_STATUS_PROTOCOL_NOT_FOUND);
	}

	usi_cfg_map_protocols[uc_prot_idx].uc_port_idx = usi_port;
	usi_cfg_map_protocols[uc_prot_idx].serialization_function = p_handler;

	return(USI_STATUS_OK);
}

#else /* #ifdef NUM_PORTS */
void hal_usi_init(void)
{
}

usi_status_t hal_usi_set_callback(usi_protocol_t protocol_id, uint8_t (*p_handler)(uint8_t *puc_rx_msg, uint16_t us_len), uint8_t usi_port)
{
	(void)protocol_id;
	(void)p_handler;
	(void)usi_port;
	return USI_STATUS_INVALID;
}

void hal_usi_process(void)
{
}

usi_status_t hal_usi_send_cmd(void *msg)
{
	(void)msg;
	return USI_STATUS_INVALID;
}

#endif /* #ifdef NUM_PORTS */

/* @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* @endcond */
