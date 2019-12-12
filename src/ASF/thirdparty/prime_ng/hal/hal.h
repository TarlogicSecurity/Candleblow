/**
 * \file
 *
 * \brief HAL: PRIME Hardware Abstraction Layer Header
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

#ifndef HAL_H_INCLUDE
#define HAL_H_INCLUDE

/* System includes */
#include <stdint.h>
#include <stdbool.h>

#include "conf_hal.h"

/* @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* @endcond */

/**
 * \ingroup prime_ng_group
 * \defgroup prime_hal_group PRIME Hardware Abstraction Layer
 *
 * This module provides configuration and utils for HAL layer.
 *
 * @{
 */

/** \brief Register definition for NVIC */
/* @{ */
/** (NVIC) Interrupt Set-enable register */
#define NVIC_ISER0        (0xE000E100U)
/** (NVIC) Interrupt Clear-enable register */
#define NVIC_ICER0        (0xE000E180U)
/** (NVIC) Interrupt Set-pending register */
#define NVIC_ISPR0        (0xE000E200U)
/** (NVIC) Interrupt Clear-pending register */
#define NVIC_ICPR0        (0xE000E280U)
/* @} */

/** \brief PPLC interruption priority */
/** \note In case of use of FreeRTOS, GROUP_PRIO is greater than configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY */
/* @{ */
#define HAL_PLC_PRIO                      11
/* @} */

#ifdef HAL_ATPL360_INTERFACE
/** \brief PLC communication parameters */
/* @{ */
#define HAL_PLC_DLYBS                       ATPL360_SPI_DLYBS
#define HAL_PLC_DLYBCT                      ATPL360_SPI_DLYBCT
/* @} */
/** \brief PLC commands */
/* @{ */
#define HAL_PLC_CMD_READ                    ATPL360_CMD_READ
#define HAL_PLC_CMD_WRITE                   ATPL360_CMD_WRITE
#define HAL_PLC_WR_RD_POS                   ATPL360_WR_RD_POS
#define HAL_PLC_LEN_MASK                    ATPL360_LEN_MASK

/** SPI Header field when bootload is in the other side of spi*/
#define HAL_PLC_SPI_HEADER_BOOT                     ATPL360_SPI_HEADER_BOOT
/** SPI Header MASK for bootloader heade*/
#define HAL_PLC_SPI_HEADER_BOOT_MASK                ATPL360_SPI_HEADER_BOOT_MASK
/** SPI Header field when atpl360 is in the other side of spi*/
#define HAL_PLC_SPI_HEADER_CORTEX                   ATPL360_SPI_HEADER_CORTEX

#define HAL_PLC_GET_ID_HEADER(b0, b1)               ATPL360_GET_ID_HEADER(b0, b1)
#define HAL_PLC_GET_FLAGS_FROM_BOOT(b0, b2, b3)     ATPL360_GET_FLAGS_FROM_BOOT(b0, b2, b3)
#define HAL_PLC_GET_FLAGS_FROM_CORTEX(b2, b3)       ATPL360_GET_FLAGS_FROM_CORTEX(b2, b3)

#define HAL_PLC_CHECK_ID_BOOT_HEADER(val)           ATPL360_CHECK_ID_BOOT_HEADER(val)
#define HAL_PLC_CHECK_ID_CORTEX_HEADER(val)         ATPL360_CHECK_ID_CORTEX_HEADER(val)
/* @} */
typedef struct spi_data {
	uint16_t us_len;
	uint16_t us_address;
	uint8_t *puc_data_buf;
} spi_data_t;

typedef struct spi_status_info {
	uint32_t ul_flags;
	uint16_t us_header_id;
} spi_status_info_t;

/* Avoid warning */
#define HAL_PLC_CMD_WRITE_REP              0

#else  /* !HAL_ATPL360_INTERFACE */
/** \brief PLC communication parameters */
/* @{ */
#define HAL_PLC_DLYBS                     10
#define HAL_PLC_DLYBCT                    0
/* @} */
/** Programmable Clock Settings (Hz) */
#define HAL_PLC_CLOCK                     9000000
/** \brief PLC commands */
/* @{ */
#define HAL_PLC_CMD_READ                   0x63
#define HAL_PLC_CMD_WRITE                  0x2a
#define HAL_PLC_CMD_WRITE_REP              0x1e
#define HAL_PLC_CMD_AND                    0x4c
#define HAL_PLC_CMD_OR                     0x71
#define HAL_PLC_CMD_XOR                    0x6d
/* @} */
#endif

/*
 * Monitoring Rate for Supply Monitor
 */
#define CONTINUOUS_MONITORING                0x00000100
#define MONITOR_ONE_OUT_OF_32SLCK_CYCLES     0x00000200
#define MONITOR_ONE_OUT_OF_256SLCK_CYCLES    0x00000200
#define MONITOR_ONE_OUT_OF_2048SLCK_CYCLES   0x00000200

/*
 * Threshold for Supply Monitor
 */
#define THRESHOLD_3V40                   0x0000000F
#define THRESHOLD_3V28                   0x0000000E
#define THRESHOLD_3V16                   0x0000000D
#define THRESHOLD_3V04                   0x0000000C
#define THRESHOLD_2V92                   0x0000000B
#define THRESHOLD_2V80                   0x0000000A
#define THRESHOLD_2V68                   0x00000009
#define THRESHOLD_2V56                   0x00000008
#define THRESHOLD_2V44                   0x00000007
#define THRESHOLD_2V32                   0x00000006
#define THRESHOLD_2V20                   0x00000005
#define THRESHOLD_2V08                   0x00000004

/** CRC types */
enum HAL_PCRC_CRC_types {
	HAL_PCRC_CRC_TYPE_8 = 0,
	HAL_PCRC_CRC_TYPE_16 = 1,
	HAL_PCRC_CRC_TYPE_24 = 2,
	HAL_PCRC_CRC_TYPE_32 = 3,
};

/** \brief Header type */

/** \note WARNING: duplicated definitions. Keep numerically consistent with those in atpl230.h
 */
/* @{ */
/** Header type: GENERIC PACKET */
#define HAL_PCRC_HT_GENERIC                0
/** Header type: PROMOTION PACKET */
#define HAL_PCRC_HT_PROMOTION              1
/** Header type: BEACON PACKET */
#define HAL_PCRC_HT_BEACON                 2
/** Header type: USI message */
#define HAL_PCRC_HT_USI                    3
/** Header type: SAR message */
#define HAL_PCRC_HT_SAR                    4
/* @} */

/** Invalid CRC */
#define HAL_PCRC_CRC_INVALID               0xFFFFFFFF

/** Signature algorithms */
typedef enum {
	HAL_FU_NO_SIGNATURE = 0,
	HAL_FU_RSA3072_SHA256,
	HAL_FU_ECDSA256_SHA256,
} hal_fu_signature_algorithm_t;

/** FU information */
typedef struct {
	uint32_t image_size;
	uint8_t page_size;
	hal_fu_signature_algorithm_t sign_algorithm;
	uint16_t sign_length;
} hal_fu_info_t;

/** FU result */
typedef enum {
	HAL_FU_SUCCESS,          /* Request to restart with new image */
	HAL_FU_CANCEL,           /* The FU has been killed */
	HAL_FU_CRC_ERROR,        /* CRC error */
	HAL_FU_FW_ERROR,         /* (Deprecated) */
	HAL_FU_FW_REVERT,        /* Request to restart with old image */
	HAL_FU_FW_CONFIRM,       /* The FU has been confirmed */
	HAL_FU_ERROR,            /* Error during FU */
	HAL_FU_SIGNATURE_ERROR,  /* Signature error (only PRIME 1.4) */
	HAL_FU_IMAGE_ERROR       /* Image verification (model/vendor) failed (only PRIME 1.4) */
} hal_fu_result_t;

/** FU verification result */
typedef enum {
	HAL_FU_VERIF_SUCCESS,
	HAL_FU_SIGNATURE_FAIL,
	HAL_FU_IMAGE_FAIL
} hal_fu_verif_result_t;

/** Frequency to poll internal usart buffer (Hz) */
#define FREQ_TIMER_POLL_USART              100

/** \brief Manage interruption priorities
 * \note In case of use of FreeRTOS, GROUP_PRIO has the same value as configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
 */
/* @{ */
#define TIMER_USART_PRIO                   10
#define USART0_PRIO                        10
#define USART1_PRIO                        10
#define USART2_PRIO                        10
#define USART3_PRIO                        10
#define USART4_PRIO                        10
/* @} */

/** Frequency to poll internal uart buffer (Hz) */
#define FREQ_TIMER_POLL_UART               100

/** \brief Manage interruption priorities
 * \note In case of use of FreeRTOS, GROUP_PRIO has the same value as configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
 */
/* @{ */
#define TIMER_UART_PRIO                    10
#define UART0_PRIO                         10
#define UART1_PRIO                         10
/* @} */

/** \brief UART definitions */
/* @{ */
#define SERIAL_UART_0                      0
#define SERIAL_UART_1                      1
/* @} */

/** GPIO direction */
typedef enum {
	HAL_GPIO_DIR_INPUT = 0,
	HAL_GPIO_DIR_OUTPUT
} hal_gpio_dir_t;

/** GPIO bank IDs */
typedef enum {
	/** ID_PIOA */
	HAL_GPIO_BANK_ID_A =       11,
	/** ID_PIOB */
	HAL_GPIO_BANK_ID_B =       12,
	/** ID_PIOC */
	HAL_GPIO_BANK_ID_C =       37,
	HAL_GPIO_BANK_ID_INVALID = 0xFF
} hal_gpio_bank_id_t;

/** \brief GPIO number of pins */
/* @{ */
#define HAL_GPIO_NUM_GPIOS_A               32
#define HAL_GPIO_NUM_GPIOS_B               32
#define HAL_GPIO_NUM_GPIOS_C               10
/* @} */

/** \brief USI Rx Errors */
/* @{ */
#define USI_NOT_ENOUGH_DATA                 9001
#define USI_BAD_LENGTH                      9002
#define USI_BAD_PROTOCOL                    9003
#define USI_BAD_CRC                         9004
/* @} */

/** \brief General Purpose Timers identificators */
typedef enum {
	HAL_GP_PLC_TIMER = 0,
	HAL_GP_TIMER_1,
	HAL_GP_TIMER_2,
	HAL_GP_TIMER_3,
	HAL_GP_TIMER_4,
	HAL_GP_TIMER_5,
	HAL_GP_TIMER_NUM
} hal_gp_timer_t;

/** \brief Timer mode */
typedef enum {
	HAL_TIMER_MODE_16_BITS = 0,
	HAL_TIMER_MODE_32_BITS,
	HAL_TIMER_MODE_NUM
} hal_timer_mode_t;

/** \brief Timer CLK source */
typedef enum {
	HAL_TIMER_CLK_SRC_CLK1 = (0x0u << 0),  /* TC_CMR_TCCLKS_TIMER_CLOCK1 */
	HAL_TIMER_CLK_SRC_CLK2 = (0x1u << 0),  /* TC_CMR_TCCLKS_TIMER_CLOCK2 */
	HAL_TIMER_CLK_SRC_CLK3 = (0x2u << 0),  /* TC_CMR_TCCLKS_TIMER_CLOCK3 */
	HAL_TIMER_CLK_SRC_CLK4 = (0x3u << 0),  /* TC_CMR_TCCLKS_TIMER_CLOCK4 */
	HAL_TIMER_CLK_SRC_CLK5 = (0x4u << 0),  /* TC_CMR_TCCLKS_TIMER_CLOCK5 */
} hal_timer_clk_src_t;

/** \brief Reset sources */
enum {
	GENERAL_RESET       = 0,
	BACKUP_RESET        = 1,
	WATCHDOG_RESET      = 2,
	SOFTWARE_RESET      = 3,
	USER_RESET          = 4,
	FU_RESET            = 5,
	USAGE_FAULT_RESET   = 6,
	BUS_FAULT_RESET     = 7,
	MEM_MANAGE_RESET    = 8,
	HARD_FAULT_RESET    = 9,
	VECTOR_FAULT_RESET  = 10,
} ;

/** \brief PLC Universal Serial Interface */
/* @{ */
/** Management Plane Protocol Spec and ATMEL serialized protocols */
typedef enum {
	PROTOCOL_MNGP_PRIME           = 0x00,
	PROTOCOL_MNGP_PRIME_GETQRY    = 0x00,
	PROTOCOL_MNGP_PRIME_GETRSP    = 0x01,
	PROTOCOL_MNGP_PRIME_SET       = 0x02,
	PROTOCOL_MNGP_PRIME_RESET     = 0x03,
	PROTOCOL_MNGP_PRIME_REBOOT    = 0x04,
	PROTOCOL_MNGP_PRIME_FU        = 0x05,
	PROTOCOL_MNGP_PRIME_GETQRY_EN = 0x06,
	PROTOCOL_MNGP_PRIME_GETRSP_EN = 0x07,
	PROTOCOL_SNIF_PRIME           = 0x13,
	PROTOCOL_PHY_SERIAL           = 0x1F,
	PROTOCOL_ATPL230              = 0x22,
	PROTOCOL_PRIME_API            = 0x30,
	PROTOCOL_INTERNAL             = 0x3F,
	PROTOCOL_USER_DEFINED         = 0xFE,
	PROTOCOL_INVALID              = 0xFF
} usi_protocol_t;

/** Number of USI supported protocols */
#define USI_NUMBER_OF_PROTOCOLS            7
/** Invalid protocol index */
#define USI_INVALID_PROTOCOL_IDX           0xFF

/** USI operation results */
typedef enum {
	USI_STATUS_PROTOCOL_NOT_FOUND,
	USI_STATUS_PROTOCOL_NOT_REGISTERED,
	USI_STATUS_TX_BUFFER_OVERFLOW,
	USI_STATUS_TX_BUSY,
	USI_STATUS_TX_BLOCKED,
	USI_STATUS_RX_BUFFER_OVERFLOW,
	USI_STATUS_RX_BLOCKED,
	USI_STATUS_UART_ERROR,
	USI_STATUS_FORMAT_ERROR,
	USI_STATUS_OK,
	USI_STATUS_INVALID
} usi_status_t;

/** Message Structure to communicate with USI layer */
typedef struct {
	uint8_t uc_protocol_type;    /* Protocol Type */
	uint8_t *ptr_buf;            /* Pointer to data buffer */
	uint16_t us_len;             /* Length of data */
} x_usi_serial_cmd_params_t;

/** \brief Types to manage commands */
/* @{ */
typedef uint8_t (*pf_usi_get_cmd)(void);
typedef void (*pf_usi_set_cmd)(uint8_t);
/* @} */
/* @} */

/** Configuration key to manage MAC address */
#define HAL_MAC_CONFIG_KEY      0xAA55

/** Type to manage MAC address */
typedef struct {
	uint16_t us_cfg_key;
	uint8_t uc_mac[6];
} x_mac_cfg_t;

/** Configuration key to manage PHY params */
#define HAL_PRIME_PHY_CONFIG_KEY  0xAA77

/** Type to manage PHY params */
typedef struct {
	uint16_t ul_cfg_key;
	uint16_t phy_cfg_load_threshold_1;
	uint16_t phy_cfg_load_threshold_2;
	uint8_t txrxChannel;
	uint8_t txrxChannelList;
} x_phy_cfg_t;

/** Configuration key to manage BN params */
#define HAL_PRIME_BN_INFO_CONFIG_KEY  0xAA55

/** Type to manage BN params */
typedef struct {
	uint16_t key;
	uint16_t mac_lnid_offset;
} x_bn_info_cfg_t;

/** Configuration key to manage prime board mode configuration */
#define HAL_PRIME_MODE_CONFIG_KEY  0xA55A

/** Type to manage prime board mode configuration.
 *  board_mode indicates board function ( PRIME_BN or PRIME_SN )
 *  prime_version indicates prime version protocol  PRIME_1_3, PRIME_1_4 or PRIME_BC */
typedef struct {
	uint16_t key;
	uint8_t prime_version;
	uint8_t board_mode;
} x_prime_mode_info_cfg_t;

/** Configuration key to manage DUK */
#define HAL_SEC_CONFIG_KEY      0x5AA5

/** Type to manage DUK */
typedef struct {
	uint16_t us_cfg_key;
	uint8_t duk[16];
	uint32_t reg_cnt;
} x_sec_cfg_t;

/** Configuration key to manage the boot information */
#define HAL_BOOT_CONFIG_KEY      0x55AA55AA

/** Type to manage the boot information */
typedef struct {
	uint32_t ul_cfg_key;
	uint32_t ul_img_size;
	uint32_t ul_orig_addr;
	uint32_t ul_dest_addr;
	uint8_t uc_pages_counter;
	uint8_t uc_boot_state;
} x_boot_info_cfg_t;

/* Type of information relative to configuration parameters */
typedef enum {
	CONFIG_TYPE_MAC_INFO = 0,
	CONFIG_TYPE_PHY_INFO = 1,
	CONFIG_TYPE_BN_INFO  = 2,
	CONFIG_TYPE_MODE_PRIME = 3,
	CONFIG_TYPE_SECURITY = 4,
	CONFIG_TYPE_BOOT_INFO = 5,
	CONFIG_TYPE_END_LIST
} config_info_type_t;

/** \brief HAL functions interface */
/* @{ */
typedef void (*hal_wrp_iface_t)(void);

/** HAL functions wrapper interface */
/* @{ */
typedef void (*hal_restart_system_t)(void);
typedef void (*hal_set_gp_timer_handler_t)(hal_gp_timer_t gpt, void (*p_handler)(void));
typedef void (*hal_clear_gp_timer_handler_t)(hal_gp_timer_t gpt);
typedef uint8_t (*hal_timer_init_t)(hal_gp_timer_t gpt, hal_timer_mode_t mode, hal_timer_clk_src_t clk_src);
typedef uint32_t (*hal_timer_count_get_t)(hal_gp_timer_t gpt);
typedef void (*hal_timer_stop_t)(hal_gp_timer_t gpt, hal_timer_mode_t mode);
typedef void (*hal_timer_plc_init_t)(uint32_t uc_time_us);
typedef void (*hal_timer_plc_stop_t)(void);
typedef void (*hal_set_plc_timer_handler_t)(void (*p_handler)(void));
typedef uint32_t (*hal_pcrc_calc_t)(uint8_t *puc_buf, uint32_t ul_len, uint8_t uc_header_type, uint8_t uc_crc_type, bool b_v14_mode);
typedef void (*hal_pcrc_config_sna_t)(uint8_t *puc_sna);
typedef void (*hal_fu_data_read_t)(uint32_t addr, uint8_t *puc_buf, uint16_t us_size);
typedef uint8_t (*hal_fu_data_write_t)(uint32_t addr, uint8_t *puc_buf, uint16_t us_size);
typedef void (*hal_fu_data_cfg_read_t)(void *pv_dst, uint16_t us_size);
typedef uint8_t (*hal_fu_data_cfg_write_t)(void *pv_src, uint16_t us_size);
typedef void (*hal_fu_start_t)(hal_fu_info_t *x_fu_info);
typedef void (*hal_fu_end_t)(hal_fu_result_t uc_hal_res);
typedef void (*hal_fu_revert_t)(void);
typedef void (*hal_fu_crc_calculate_t)(void);
typedef void (*hal_fu_crc_set_callback_t)(void (*p_handler)(uint32_t ul_crc));
typedef void (*hal_fu_signature_image_check_t)(void);
typedef void (*hal_fu_signature_image_check_set_callback_t)(void (*p_handler)(hal_fu_verif_result_t uc_result));
typedef uint16_t (*hal_fu_get_bitmap_t)(uint8_t *puc_bitmap, uint32_t *pus_num_rcv_pages);
typedef void (*hal_plc_init_t)(void);
typedef void (*hal_plc_reset_t)(void);
typedef int8_t (*hal_plc_cmd_op_t)(uint8_t uc_cmd, uint16_t us_addr, uint16_t us_len, uint8_t *ptr_buf, uint8_t uc_bytes_rep);
typedef void (*hal_plc_set_handler_t)(void (*p_handler)(void));
typedef void (*hal_plc_tx_signal_t)(void);
typedef void (*hal_plc_rx_signal_t)(void);
typedef bool (*hal_get_config_info_t)(config_info_type_t cfg_type, uint16_t us_size, void *pv_data);
typedef bool (*hal_set_config_info_t)(config_info_type_t cfg_type, uint16_t us_size, void *pv_data);
typedef usi_status_t (*hal_usi_set_callback_t)(usi_protocol_t protocol_id, uint8_t (*p_handler)(uint8_t *puc_rx_msg, uint16_t us_len), uint8_t usi_port);
typedef usi_status_t (*hal_usi_send_cmd_t)(void *msg);
typedef void (*hal_trng_init_t)(void);
typedef uint32_t (*hal_trng_read_t)(void);
typedef void (*hal_debug_report_t)(uint32_t ul_err_type);
typedef uint32_t (*hal_net_get_freq_t)(void);
typedef void (*hal_pib_get_request_t)(uint16_t us_pib_attrib);
typedef void (*hal_pib_get_request_set_callback_t)(void (*p_handler)(uint8_t uc_result, uint16_t us_pib_attrib, void *pv_pib_value, uint8_t uc_pib_size));
typedef void (*hal_pib_set_request_t)(uint16_t us_pib_attrib, void *pv_pib_value, uint8_t uc_pib_size);
typedef void (*hal_pib_set_request_set_callback_t)(void (*p_handler)(uint8_t uc_result));

#if PRIME_HAL_VERSION == HAL_PRIME_1_4
typedef void (*hal_aes_init_t)(void);
typedef void (*hal_aes_set_callback_t)(void (*p_handler)(void));
typedef void (*hal_aes_key_t)(uint8_t *puc_key, uint8_t uc_key_len);
typedef void (*hal_aes_crypt_t)(bool b_crypt_mode, uint8_t *puc_in_text, uint8_t *puc_out_text);
#endif

/* HAL USI Null Device Interface */
typedef uint16_t (*hal_null_dev_read_callback_t)(uint8_t chn, void *buffer, uint16_t len);
typedef uint16_t (*hal_null_dev_write_callback_t)(uint8_t chn, const void *buffer, uint16_t len);
bool hal_null_dev_set_read_callback(hal_null_dev_read_callback_t ptr_func);
bool hal_null_dev_set_write_callback(hal_null_dev_write_callback_t ptr_func);

#ifdef HAL_ATPL360_INTERFACE
/** Time reference delay */
typedef enum {
	HAL_TREF_SEC = 0,
	HAL_TREF_MS,
	HAL_TREF_US
} tref_delay_mode_t;

typedef bool (*hal_plc_send_boot_cmd_t)(uint16_t us_cmd, uint32_t ul_addr, uint32_t ul_data_len, uint8_t *puc_data_buf, uint8_t *puc_data_read);
typedef bool (*hal_plc_send_wrrd_cmd_t)(uint8_t uc_cmd, void *px_spi_data, void *px_spi_status_info);
typedef void (*hal_plc_enable_interrupt_t)(bool enable);
typedef void (*hal_plc_delay_t)(uint8_t uc_tref, uint32_t ul_delay);
typedef bool (*hal_plc_get_cd_t)(void);
#endif

#ifdef HAL_NWK_RECOVERY_INTERFACE
typedef uint32_t (*hal_nwk_recovery_init_t)(void);
typedef void (*hal_nwk_recovery_read_t)(uint32_t addr, uint8_t *puc_buf, uint16_t us_size);
typedef uint8_t (*hal_nwk_recovery_write_t)(uint32_t addr, uint8_t *puc_buf, uint16_t us_size);
#endif
/* @} */

/** Structure of HAL functions interface */
typedef struct {
	hal_restart_system_t restart_system;

	hal_set_gp_timer_handler_t set_gp_timer_handler;
	hal_clear_gp_timer_handler_t clear_gp_timer_handler;
	hal_timer_init_t timer_init;
	hal_timer_count_get_t timer_count_get;
	hal_timer_stop_t timer_stop;
	hal_timer_plc_init_t timer_plc_init;
	hal_timer_plc_stop_t timer_plc_stop;
	hal_set_plc_timer_handler_t set_plc_timer_handler;

	hal_pcrc_calc_t pcrc_calc;
	hal_pcrc_config_sna_t pcrc_config_sna;

	hal_fu_data_read_t fu_data_read;
	hal_fu_data_write_t fu_data_write;
	hal_fu_data_cfg_read_t fu_data_cfg_read;
	hal_fu_data_cfg_write_t fu_data_cfg_write;
	hal_fu_start_t fu_start;
	hal_fu_end_t fu_end;
	hal_fu_revert_t fu_revert;
	hal_fu_crc_calculate_t fu_crc_calculate;
	hal_fu_crc_set_callback_t fu_crc_set_callback;
	hal_fu_signature_image_check_t fu_signature_image_check;
	hal_fu_signature_image_check_set_callback_t fu_signature_image_check_set_callback;
	hal_fu_get_bitmap_t fu_get_bitmap;

	hal_plc_init_t plc_init;
	hal_plc_reset_t plc_reset;
	hal_plc_cmd_op_t plc_cmd_op;
	hal_plc_set_handler_t plc_set_handler;
	hal_plc_tx_signal_t plc_tx_signal;
	hal_plc_rx_signal_t plc_rx_signal;

	hal_get_config_info_t get_config_info;
	hal_set_config_info_t set_config_info;

	hal_usi_set_callback_t usi_set_callback;
	hal_usi_send_cmd_t usi_send_cmd;

	hal_trng_init_t trng_init;
	hal_trng_read_t trng_read;

	hal_debug_report_t debug_report;

	hal_net_get_freq_t net_get_freq;

#if PRIME_HAL_VERSION == HAL_PRIME_1_4
	hal_aes_init_t aes_init;
	hal_aes_set_callback_t aes_set_callback;
	hal_aes_key_t aes_key;
	hal_aes_crypt_t aes_crypt;
#endif

#ifdef HAL_ATPL360_INTERFACE
	hal_plc_send_boot_cmd_t plc_send_boot_cmd;
	hal_plc_send_wrrd_cmd_t plc_send_wrrd_cmd;
	hal_plc_enable_interrupt_t plc_enable_int;
	hal_plc_delay_t plc_delay;
	hal_plc_get_cd_t plc_get_cd;
#endif

#ifdef HAL_NWK_RECOVERY_INTERFACE
	hal_nwk_recovery_init_t nwk_recovery_init;
	hal_nwk_recovery_read_t nwk_recovery_read;
	hal_nwk_recovery_write_t nwk_recovery_write;
#endif

	hal_pib_get_request_t pib_get_request;
	hal_pib_get_request_set_callback_t pib_get_request_set_callback;
	hal_pib_set_request_t pib_set_request;
	hal_pib_set_request_set_callback_t pib_set_request_set_callback;
} hal_api_t;

/* @} */

/* @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* @endcond */
#endif /* HAL_H_INCLUDE */
