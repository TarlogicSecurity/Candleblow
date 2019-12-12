/**
 * \file
 *
 * \brief atpl360_host PRIME Physical layer
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

#ifndef ATPL360_COMM_H_INCLUDED
#define ATPL360_COMM_H_INCLUDED

#include "general_defs.h"
/* #include "conf_atpl360.h" */

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

enum _imp_state {
	IMPEDANCE_STATE_HI,
	IMPEDANCE_STATE_LO,
	IMPEDANCE_STATE_VLO,
	IMPEDANCE_STATE_NUM,
};

#define NUM_COEF                               97

/* ! Defines relatives to some ATPL360 registers */
#define ATPL360_REG_ADC_MASK                   0x1000
#define ATPL360_REG_DAC_MASK                   0x2000
#define ATPL360_REG_MASK                       0x4000
#define ATPL360_FUSES_MASK                     0x8000
#define ATPL360_REG_ADC_BASE                   0x40000000
#define ATPL360_REG_DAC_BASE                   0x40004000
#define ATPL360_REG_BASE                       0x80000000
#define ATPL360_FUSES_BASE                     0x400E1800

typedef enum atpl360_reg_id {
	ATPL360_REG_PRODID = 0x4000,
	ATPL360_REG_MODEL,
	ATPL360_REG_VERSION_STR,
	ATPL360_REG_VERSION_NUM,
	ATPL360_REG_CFG_AUTODETECT_IMPEDANCE,
	ATPL360_REG_CFG_IMPEDANCE,
	ATPL360_REG_ZC_TIME,
	ATPL360_REG_RX_PAY_SYMBOLS,
	ATPL360_REG_TX_PAY_SYMBOLS,
	ATPL360_REG_RSV0,
	ATPL360_REG_MAX_RMS_TABLE_HI,
	ATPL360_REG_MAX_RMS_TABLE_VLO,
	ATPL360_REG_THRESHOLDS_TABLE_HI,
	ATPL360_REG_THRESHOLDS_TABLE_LO,
	ATPL360_REG_THRESHOLDS_TABLE_VLO,
	ATPL360_REG_PREDIST_COEF_TABLE_HI,
	ATPL360_REG_PREDIST_COEF_TABLE_LO,
	ATPL360_REG_PREDIST_COEF_TABLE_VLO,
	ATPL360_REG_GAIN_TABLE_HI,
	ATPL360_REG_GAIN_TABLE_LO,
	ATPL360_REG_GAIN_TABLE_VLO,
	ATPL360_REG_DACC_TABLE_CFG,
	ATPL360_REG_CHANNEL_CFG,
	ATPL360_REG_NUM_TX_LEVELS,
	ATPL360_REG_CORRECTED_RMS_CALC,
	ATPL360_REG_CURRENT_GAIN,
	ATPL360_REG_ZC_CONF_INV,
	ATPL360_REG_ZC_CONF_FREQ,
	ATPL360_REG_ZC_CONF_DELAY,
	ATPL360_REG_SIGNAL_CAPTURE_START,
	ATPL360_REG_SIGNAL_CAPTURE_STATUS,
	ATPL360_REG_SIGNAL_CAPTURE_FRAGMENT,
	ATPL360_REG_SIGNAL_CAPTURE_DATA,
	ATPL360_REG_ENABLE_AUTO_NOISE_CAPTURE,
	ATPL360_REG_TIME_BETWEEN_NOISE_CAPTURES,
	ATPL360_REG_DELAY_NOISE_CAPTURE_AFTER_RX,
	ATPL360_REG_RRC_NOTCH_ACTIVE,
	ATPL360_REG_RRC_NOTCH_INDEX,
	ATPL360_REG_NOISE_PEAK_POWER,
	ATPL360_REG_RRC_NOTCH_AUTODETECT,
	ATPL360_REG_RRC_NOTCH_THR_ON,
	ATPL360_REG_RRC_NOTCH_THR_OFF,
	ATPL360_REG_TX_TOTAL,
	ATPL360_REG_TX_TOTAL_BYTES,
	ATPL360_REG_TX_TOTAL_ERRORS,
	ATPL360_REG_TX_BAD_BUSY_TX,
	ATPL360_REG_TX_BAD_BUSY_CHANNEL,
	ATPL360_REG_TX_BAD_LEN,
	ATPL360_REG_TX_BAD_FORMAT,
	ATPL360_REG_TX_TIMEOUT,
	ATPL360_REG_RX_TOTAL,
	ATPL360_REG_RX_TOTAL_BYTES,
	ATPL360_REG_RX_EXCEPTIONS,
	ATPL360_REG_RX_BAD_LEN,
	ATPL360_REG_RX_BAD_CRC_FCH,
	ATPL360_REG_RX_FALSE_POSITIVE,
	ATPL360_REG_RX_BAD_FORMAT,
	ATPL360_REG_NOISE_PER_CARRIER,
	ATPL360_REG_PPM_CALIB_ON,
	ATPL360_REG_ZC_PERIOD,
	ATPL360_REG_SYNC_THRESHOLDS,
	ATPL360_REG_OBSOLETE_ID,
	ATPL360_REG_END_ID,
} atpl360_reg_id_t;

/* ! Internal Memory Map */
typedef enum atpl360_mem_id {
	ATPL360_STATUS_INFO_ID = 0,
	ATPL360_TX0_PARAM_ID,
	ATPL360_TX0_DATA_ID,
	ATPL360_TX0_CFM_ID,
	ATPL360_TX1_PARAM_ID,
	ATPL360_TX1_DATA_ID,
	ATPL360_TX1_CFM_ID,
	ATPL360_RX_PARAM_ID,
	ATPL360_RX_DATA_ID,
	ATPL360_REG_INFO_ID,
	ATPL360_IDS,
} atpl360_mem_id_t;

/* ! \name Structure defining configuration parameter type of access and address to be use for this access */
typedef struct cfg_access_info {
	uint32_t ul_address;
	uint16_t uc_sub_address;
} cfg_access_info_t;

/* Defines refering to Impedance Configuration */
#define HI_STATE                                0x00
#define LOW_STATE                               0x01
#define VLO_STATE                               0x02

/* ! \name TX Mode Bit Mask */
/* @{ */
/* ! TX Mode: Absolute transmission */
#define TX_MODE_ABSOLUTE             (0 << 0)
/* ! TX Mode: Delayed transmission */
#define TX_MODE_RELATIVE             (1 << 0)
/* ! TX Mode: Cancel transmission */
#define TX_MODE_CANCEL               (1 << 1)
/* ! TX Mode: Preamble Continuous transmission */
#define TX_MODE_PREAMBLE_CONTINUOUS  (1 << 2)
/* ! TX Mode: Syimbols Continuous transmission */
#define TX_MODE_SYMBOLS_CONTINUOUS   (1 << 3)

/* ! \name PRIME Mode types */
enum mode_types {
	MODE_TYPE_A = 0,
	MODE_TYPE_B = 2,
	MODE_TYPE_BC = 3,
};

/* ! \name Header types */
enum header_types {
	PHY_HT_GENERIC = 0,
	PHY_HT_PROMOTION = 1,
	PHY_HT_BEACON = 2,
};

/* ! \name PRIME Buffer ID */
enum buffer_id {
	TX_BUFFER_0 = 0,
	TX_BUFFER_1 = 1,
};

/* ! \name Modulation schemes */
enum mod_schemes {
	MOD_SCHEME_DBPSK = 0,
	MOD_SCHEME_DQPSK = 1,
	MOD_SCHEME_D8PSK = 2,
	MOD_SCHEME_DBPSK_C = 4,
	MOD_SCHEME_DQPSK_C = 5,
	MOD_SCHEME_D8PSK_C = 6,
	MOD_SCHEME_R_DBPSK = 12,
	MOD_SCHEME_R_DQPSK = 13,
};

/* ! \name TX Result values */
enum tx_result_values {
	TX_RESULT_PROCESS = 0,                  /* Transmission result: already in process */
	TX_RESULT_SUCCESS = 1,                  /* Transmission result: end successfully */
	TX_RESULT_INV_LENGTH = 2,               /* Transmission result: invalid length error */
	TX_RESULT_BUSY_CH = 3,                  /* Transmission result: busy channel error */
	TX_RESULT_BUSY_TX = 4,                  /* Transmission result: busy in transmission error */
	TX_RESULT_BUSY_RX = 5,                  /* Transmission result: busy in reception error */
	TX_RESULT_INV_SCHEME = 6,               /* Transmission result: invalid modulation scheme error */
	TX_RESULT_TIMEOUT = 7,                  /* Transmission result: timeout error */
	TX_RESULT_INV_BUFFER = 8,               /* Transmission result: invalid buffer identifier error */
	TX_RESULT_INV_MODE = 9,                 /* Transmission result: invalid Prime Mode error */
	TX_RESULT_NO_TX = 255,                  /* Transmission result: No transmission ongoing */
};

/* ! \name Event types */
enum atpl360_event_type {
	MSG_IND_DATA_EV_TYPE = 0,               /* Message data indication event */
	MSG_IND_PARAM_EV_TYPE,                  /* Message quality parameters indication event */
	MSG_CFM_EV_TYPE,                        /* Message confirm event */
	TIMER_EV_TYPE,                          /* Timer event */
	REG_EV_TYPE,                            /* Registers event */
	NUM_EV_TYPES,                           /* Number of event types */
};

/* ! \name communication results */
typedef enum atpl360_comm_status {
	ATPL360_COMM_SUCCESS = 0,               /* Current operation successful */
	ATPL360_COMM_ERROR,                     /* Current operation failed */
} atpl360_comm_status_t;

/* ! \name Noise Capture Mode Bit Mask */
#define SIGNAL_CAPTURE_CHANNEL_SHIFT    0
#define SIGNAL_CAPTURE_CHANNEL (0xFu << SIGNAL_CAPTURE_CHANNEL_SHIFT)
#define SIGNAL_CAPTURE_SIGNAL_SHIFT    4
#define SIGNAL_CAPTURE_SIGNAL_MODE (0x1u << SIGNAL_CAPTURE_SIGNAL_SHIFT)
#define SIGNAL_CAPTURE_SIGNAL_MODE_LOW (0x0u << SIGNAL_CAPTURE_SIGNAL_SHIFT)  /* Signal mode for low signal level : Only valid in SIGNAL_CAPTURE_BAND_MODE_FCC mode */
#define SIGNAL_CAPTURE_SIGNAL_MODE_HIGH (0x1u << SIGNAL_CAPTURE_SIGNAL_SHIFT) /* Signal mode for high signal level : Only valid in SIGNAL_CAPTURE_BAND_MODE_FCC mode */
#define SIGNAL_CAPTURE_BAND_MODE_SHIFT    5
#define SIGNAL_CAPTURE_BAND_MODE (0x1u << SIGNAL_CAPTURE_BAND_MODE_SHIFT)
#define SIGNAL_CAPTURE_BAND_MODE_CHN (0x0u << SIGNAL_CAPTURE_BAND_MODE_SHIFT) /* Frequency in Channel Mode */
#define SIGNAL_CAPTURE_BAND_MODE_FCC (0x1u << SIGNAL_CAPTURE_BAND_MODE_SHIFT) /* Frequency in all FCC band Mode */
#define SIGNAL_CAPTURE_TIME_MODE_SHIFT    6
#define SIGNAL_CAPTURE_TIME_MODE (0x1u << SIGNAL_CAPTURE_TIME_MODE_SHIFT)
#define SIGNAL_CAPTURE_TIME_MODE_ABS (0x0u << SIGNAL_CAPTURE_TIME_MODE_SHIFT) /* Time in Absolute Mode */
#define SIGNAL_CAPTURE_TIME_MODE_REL (0x1u << SIGNAL_CAPTURE_TIME_MODE_SHIFT) /* Time in Relative Mode */
#define SIGNAL_CAPTURE_CHN_1  0x01
#define SIGNAL_CAPTURE_CHN_2  0x02
#define SIGNAL_CAPTURE_CHN_3  0x03
#define SIGNAL_CAPTURE_CHN_4  0x04
#define SIGNAL_CAPTURE_CHN_5  0x05
#define SIGNAL_CAPTURE_CHN_6  0x06
#define SIGNAL_CAPTURE_CHN_7  0x07
#define SIGNAL_CAPTURE_CHN_8  0x08

/* ! \name Noise Capture States */
enum {
	SIGNAL_CAPTURE_IDLE,
	SIGNAL_CAPTURE_RUNNING,
	SIGNAL_CAPTURE_READY,
};

/* ! \name Structure defining Noise Capture status */
typedef struct signal_capture_st_info {
	uint8_t uc_num_frags;
	uint8_t uc_status;
} signal_capture_st_info_t;

#define SIGNAL_CAPTURE_FRAG_SIZE            255

/* Number of transmission buffers */
#define NUM_TX_BUFFERS                      2

#pragma pack(push,1)

/* ! \name Structure defining Rx message */
typedef struct rx_msg {
	/** Accumulated Error Vector Magnitude for header */
	uint32_t ul_evm_header_acum;
	/** Accumulated Error Vector Magnitude for payload */
	uint32_t ul_evm_payload_acum;
	/** Reception time in us */
	uint32_t ul_rx_time;
	/** Error Vector Magnitude for header */
	uint16_t us_evm_header;
	/** Error Vector Magnitude for payload */
	uint16_t us_evm_payload;
	/** Length of the data buffer. */
	uint16_t us_data_len;
	/** Modulation scheme of the last received message */
	enum mod_schemes uc_scheme;
	/** Type A, Type B or Type BC  */
	enum mode_types uc_mod_type;
	/** Header Type of the last received message */
	enum header_types uc_header_type;
	/** Average RSSI (Received Signal Strength Indication) */
	uint8_t uc_rssi_avg;
	/** Average CNIR (Carrier to Interference + Noise ratio) */
	uint8_t uc_cinr_avg;
	/** Minimum CNIR (Carrier to Interference + Noise ratio) */
	uint8_t uc_cinr_min;
	/** Average Soft BER (Bit Error Rate) */
	uint8_t uc_ber_soft;
	/** Maximum Soft BER (Bit Error Rate) */
	uint8_t uc_ber_soft_max;
	/** Percentage of carriers affected by narrow band noise */
	uint8_t uc_nar_bnd_percent;
	/** Percentage of symbols affected by impulsive noise */
	uint8_t uc_imp_percent;
	/** Pointer to local data buffer */
	uint8_t *puc_data_buf;
} rx_msg_t;

/* ! \name Structure defining Tx message */
typedef struct tx_msg {
	/** Time for transmission in us */
	uint32_t ul_tx_time;
	/** Length of the data buffer. */
	uint16_t us_data_len;
	/** Attenuation level with which the message must be transmitted */
	uint8_t uc_att_level;
	/** Modulation scheme of last transmitted message */
	enum mod_schemes uc_scheme;
	/** TX Forced */
	uint8_t uc_disable_rx;
	/** Type A, Type B or Type BC */
	enum mode_types uc_mod_type;
	/** Transmission Mode (absolute, relative, continuous tx, cancel tx). Constants above */
	uint8_t uc_tx_mode;
	/** Buffer Id used for transmission */
	enum buffer_id uc_buffer_id;
	/** Reserved byte */
	uint8_t uc_rsvd;
	/** Pointer to data buffer */
	uint8_t *puc_data_buf;
} tx_msg_t;

/* ! \name Structure defining result of a transmission */
typedef struct tx_cfm {
	/** Transmission time in us. */
	uint32_t ul_tx_time;
	/** RMS value emitted */
	uint32_t ul_rms_calc;
	/** Type mode: Type A, Type B or Type BC  */
	enum mode_types uc_mod_type;
	/** TX Result */
	enum tx_result_values uc_tx_result;
	/** Buffer Id used for transmission to confirm */
	enum buffer_id uc_buffer_id;
} tx_cfm_t;

/* ! \name Structure defining events */
typedef struct atpl360_events {
	/* HW Timer reference */
	uint32_t ul_timer_ref;
	/* Info relative to events */
	uint32_t ul_event_info;
	/* Indicate if TIMER event is enable. It returns the number of timer expired, 0 in otherwise */
	uint8_t uc_timer_expired;
	/* Flag to indicate if CONFIRMATION MSG event is enable */
	bool b_cfm_event_enable[NUM_TX_BUFFERS];
	/* Flag to indicate if DATA INDICATION MSG event is enable */
	bool b_data_ind_event_enable;
	/* Flag to indicate if QPAR INDICATION MSG event is enable */
	bool b_qpar_ind_event_enable;
	/* Flag to indicate if SYNCHRONIZATION event is enable */
	bool b_syn_event_enable;
	/* Flag to indicate if REGISTER DATA RESPONSE event is enable */
	bool b_reg_data_enable;
} atpl360_events_t;

#pragma pack(pop)

#define ATPL360_DELAY_TX_DATA_US                 100

#define ATPL360_EVENT_DATA_LENGTH                8

/* ! FLAG MASKs for set events */
#define ATPL360_TX0_CFM_FLAG_MASK                0x0001
#define ATPL360_TX1_CFM_FLAG_MASK                0x0002
#define ATPL360_RX_DATA_IND_FLAG_MASK            0x0004
#define ATPL360_CD_FLAG_MASK                     0x0008
#define ATPL360_REG_RSP_MASK                     0x0010
#define ATPL360_RX_QPAR_IND_FLAG_MASK            0x0020

/* ! Event Info MASKs */
#define ATPL360_EV_DAT_LEN_MASK                  0x0000FFFF
#define ATPL360_EV_REG_LEN_MASK                  0xFFFF0000
#define ATPL360_GET_EV_DAT_LEN_INFO(x)           ((uint32_t)x & ATPL360_EV_DAT_LEN_MASK)
#define ATPL360_GET_EV_REG_LEN_INFO(x)           (((uint32_t)x & ATPL360_EV_REG_LEN_MASK) >> 16)

uint16_t atpl360_comm_stringify(uint8_t *puc_dest, void *pv_src, uint16_t us_src_size);
atpl360_comm_status_t atpl360_comm_parse(void *pv_dst, uint8_t *puc_src, uint16_t us_dst_size);
uint16_t atpl360_comm_get_event_id(enum atpl360_event_type ev_type, uint8_t uc_buff_index);
uint16_t atpl360_comm_get_tx_params_id(tx_msg_t *px_msg);
uint16_t atpl360_comm_get_tx_data_id(tx_msg_t *px_msg);
void atpl360_comm_set_event_info(atpl360_events_t *px_events_info, uint16_t us_int_flags);
uint32_t atpl360_comm_get_cfg_param_access_type(uint16_t us_param_id);
uint32_t atpl360_comm_get_cfg_param_delay_us(uint16_t us_param_id);

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */

#endif /* ATPL360_COMM_H_INCLUDED */
