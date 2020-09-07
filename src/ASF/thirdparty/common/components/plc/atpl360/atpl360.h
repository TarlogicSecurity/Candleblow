/**
 * \file
 *
 * \brief API driver for ATPL360 PLC transceiver.
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

#ifndef ATPL360_H_INCLUDED
#define ATPL360_H_INCLUDED

/* System includes */
#include "compiler.h"
#include "atpl360_comm.h"
#include "atpl360_IB_db.h"
#include "atpl360_exception.h"
#include "atpl360_hal_spi.h"
#include "conf_atpl360.h"

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

/** ATPL360 Restuls */
typedef enum atpl360_res {
	ATPL360_SUCCESS = 0,
	ATPL360_ERROR,
} atpl360_res_t;

/** Time reference delay */
enum atpl360_delay_mode {
	DELAY_TREF_SEC = 0,
	DELAY_TREF_MS,
	DELAY_TREF_US
};

typedef void (*pf_data_confirm_t)(tx_cfm_t *px_msg_cfm);
typedef void (*pf_data_indication_t)(rx_msg_t *px_msg);
typedef void (*pf_addons_event_t)(uint8_t *px_msg, uint16_t us_len);

/* ATPL360 device Callbacks */
typedef struct atpl360_dev_callbacks {
	/* Callback for TX Data Confirm */
	pf_data_confirm_t data_confirm;
	/* Callback for RX Data Indication */
	pf_data_indication_t data_indication;
	/* Callback for Serial Data Event (see addons) */
	pf_addons_event_t addons_event;
	/* Callback for Exceptions triggered by the component */
	pf_exeption_event_t exception_event;
} atpl360_dev_callbacks_t;

typedef void (*pf_set_callbacks_t)(atpl360_dev_callbacks_t *dev_cb);
typedef uint8_t (*pf_send_data_t)(tx_msg_t *px_msg);
typedef bool (*pf_mng_get_cfg_t)(uint16_t us_param_id, void *px_value, uint8_t uc_len, bool b_sync);
typedef bool (*pf_mng_set_cfg_t)(uint16_t us_param_id, void *px_value, uint16_t us_len);
typedef uint32_t (*pf_get_timer_ref_t)(void);

/* ATPL360 descriptor */
typedef struct atpl360_descriptor {
	pf_set_callbacks_t set_callbacks;
	pf_send_data_t send_data;
	pf_mng_get_cfg_t get_config;
	pf_mng_set_cfg_t set_config;
	pf_addons_event_t send_addons_cmd;
} atpl360_descriptor_t;

/* ATPL360 Hardware Wrapper */
typedef void (*pf_plc_init_t)(void);
typedef void (*pf_plc_reset_t)(void);
typedef void (*pf_plc_set_handler_t)(void (*p_handler)(void));
typedef bool (*pf_plc_bootloader_cmd_t)(uint16_t us_cmd, uint32_t ul_addr, uint32_t ul_data_len, uint8_t *puc_data_buf, uint8_t *puc_data_read);
typedef bool (*pf_plc_write_read_cmd_t)(uint8_t uc_cmd, void *px_spi_data, void *px_spi_status_info);
typedef void (*pf_plc_enable_int_t)(bool enable);
typedef void (*pf_plc_delay_t)(uint8_t uc_tref, uint32_t ul_delay);

typedef struct atpl360_hal_wrapper {
	pf_plc_init_t plc_init;
	pf_plc_reset_t plc_reset;
	pf_plc_set_handler_t plc_set_handler;
	pf_plc_bootloader_cmd_t plc_send_boot_cmd;
	pf_plc_write_read_cmd_t plc_write_read_cmd;
	pf_plc_enable_int_t plc_enable_int;
	pf_plc_delay_t plc_delay;
} atpl360_hal_wrapper_t;

/* ! \name ATPL360 Interface */
/* @{ */
void atpl360_init(atpl360_descriptor_t *const descr, atpl360_hal_wrapper_t *px_hal_wrapper);
atpl360_res_t atpl360_enable(uint32_t ul_binary_address, uint32_t ul_binary_len);
void atpl360_disable(void);
void atpl360_handle_events(void);

/* @} */

/* ! @} */

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */

#endif /* ATPL360_H_INCLUDED */
