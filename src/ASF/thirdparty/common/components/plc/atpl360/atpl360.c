/**
 * \file
 *
 * \brief Management of the ATPL360 PLC transceiver.
 * This file manages the accesses to the ATPL360 component.
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

/* ATPL360 includes */
#include "conf_atpl360.h"
#include "atpl360_hal_spi.h"
#include "atpl360_IB.h"
#include "atpl360_boot.h"
#include "atpl360.h"

#ifdef ATPL360_ADDONS_ENABLE
#include "addon_api.h"
#endif

/* HAL wrapper */
atpl360_hal_wrapper_t sx_atpl360_hal_wrapper;

#define ATPL360_G3_MODE_MASK                      0x0007

#define ATPL360_MAX_STATUS_CHECK                  2

#define ATPL360_REG_CMD_RD                        (0 << 10)
#define ATPL360_REG_CMD_WR                        (1 << 10)
#define ATPL360_REG_LEN_MASK                      0x1FF
#define ATPL360_REG_ID_MASK                       0xF000
#define ATPL360_REG_OFFSET_MASK                   0x0FFF

#define ATPL360_DATA_PKT_SIZE                     512
#define ATPL360_TX_PKT_SIZE                       sizeof(tx_msg_t) + ATPL360_DATA_PKT_SIZE
#define ATPL360_RX_PKT_SIZE                       sizeof(rx_msg_t) + ATPL360_DATA_PKT_SIZE
#define ATPL360_CMF_PKT_SIZE                      sizeof(tx_cfm_t)
#define ATPL360_REG_PKT_SIZE                      ATPL360_DATA_PKT_SIZE

/* Buffer definition to communicate with ATPL360 */
static uint8_t spuc_tx_buffer[ATPL360_TX_PKT_SIZE];
static uint8_t spuc_ind_buffer[ATPL360_RX_PKT_SIZE];
static uint8_t spuc_cfm_buffer[NUM_TX_BUFFERS][ATPL360_CMF_PKT_SIZE];
static uint8_t spuc_reg_buffer[ATPL360_REG_PKT_SIZE];

#ifdef ATPL360_ADDONS_ENABLE
static uint8_t spuc_addon_buffer[ATPL360_RX_PKT_SIZE + 5];
static atpl360_addon_descriptor_t sx_addon_desc;
#endif

/* Flags to manage events */
static volatile uint8_t suc_waiting_tx_cfm;
static volatile bool sb_report_tx_cfm_error_by_rst;
static volatile bool spb_cfm_event_enable[NUM_TX_BUFFERS];
static volatile bool sb_data_ind_event_enable;
static volatile bool sb_param_ind_event_enable;
static volatile uint16_t sus_reg_event_len;

/* Device Enable/Disable Status */
static bool sb_component_enabled;

/* Internal Handlers to manage ATPL360 event notifications */
pf_data_confirm_t _data_confirm_cb_handler;
pf_data_indication_t _data_indication_cb_handler;
pf_addons_event_t _addons_event_cb_handler;
pf_exeption_event_t _exception_event_cb_handler;

/* Counters of interrupt */
static uint32_t ul_int_cnt;
static uint32_t ul_int_rx_data_cnt;
static uint32_t ul_int_rx_qpar_cnt;
static uint32_t ul_int_tx_cnt;
static uint32_t ul_int_reg_cnt;

static uint16_t sus_prod_info;

/* SPI Commands to get/set spi data buffer */
#define SPI_RD_CMD      0
#define SPI_WR_CMD      1

static bool _set_config(uint16_t us_param_id, void *px_value, uint16_t us_len);

/**
 * \brief Function to read/write through SPI
 *
 * \return  ATPL360_SUCCESS if there is no error, otherwise returns ATPL360_ERROR.
 */
static atpl360_res_t _spi_send_cmd(uint8_t uc_cmd, atpl360_spi_data_t *px_spi_data)
{
	uint8_t uc_cnt;
	atpl360_spi_status_t uc_status;

	if (uc_cmd == SPI_WR_CMD) {
		atpl360_spi_write_buf(px_spi_data);
	} else {
		atpl360_spi_read_buf(px_spi_data);
	}

	uc_cnt = ATPL360_MAX_STATUS_CHECK;
	uc_status = atpl360_spi_get_status();

	while (uc_status != ATPL360_SPI_STATUS_CORTEX) {
		if (uc_status == ATPL360_SPI_STATUS_UNKNOWN) {
			/* Safety condition in loop */
			if (_exception_event_cb_handler) {
				/* Report SPI Error to Application */
				_exception_event_cb_handler(ATPL360_EXCEPTION_UNEXPECTED_SPI_STATUS);
			}

			/* CRITICAL Safety condition in loop */
			if (!uc_cnt--) {
				if (_exception_event_cb_handler) {
					/* Report Critical SPI Error to Application */
					_exception_event_cb_handler(ATPL360_EXCEPTION_SPI_CRITICAL_ERROR);
				}

				uc_cnt = ATPL360_MAX_STATUS_CHECK;
				return ATPL360_ERROR;
			}

			/* Wait to system re-start */
			sx_atpl360_hal_wrapper.plc_delay(DELAY_TREF_MS, ATPL360_RST_WAIT_MS);

			/* ATPL360 hard reset */
			sx_atpl360_hal_wrapper.plc_reset();
		} else if (uc_status == ATPL360_SPI_STATUS_FW_VALIDATING) {
			/* Report Reset detection to Application */
			if (_exception_event_cb_handler) {
				_exception_event_cb_handler(ATPL360_EXCEPTION_RESET);
			}
		}

		/* Validate phase */
		if (uc_cmd == SPI_WR_CMD) {
			atpl360_spi_write_buf(px_spi_data);
		} else {
			atpl360_spi_read_buf(px_spi_data);
		}

		/* Check TX cfm flag */
		if (suc_waiting_tx_cfm) {
			/* Report CFM Error to Application. Pending report TX_CFM and abort due to ATPL360 reset*/
			sb_report_tx_cfm_error_by_rst = true;
		}

		/* Update SPI status */
		uc_status = atpl360_spi_get_status();
	}

	return ATPL360_SUCCESS;
}

/**
 * \brief Function to get Interrupt info
 *
 * \return Interrupt info in case of ATPl360 is enable. 0 in otherwise
 */
static void _get_interrupt_info(atpl360_events_t *px_events_info, atpl360_spi_status_info_t *x_status_info)
{
	atpl360_spi_data_t x_spi_data;
	uint8_t *puc_info;
	uint8_t puc_int_buffer[ATPL360_EVENT_DATA_LENGTH];

	puc_info = puc_int_buffer;

	/* Read Time Ref and Event flags */
	x_spi_data.us_mem_id = ATPL360_STATUS_INFO_ID;
	x_spi_data.us_len = sizeof(puc_int_buffer);
	x_spi_data.puc_data_buf = puc_info;
	if (_spi_send_cmd(SPI_RD_CMD, &x_spi_data) == ATPL360_SUCCESS) {
		/* Extract events info */
		atpl360_spi_get_status_info(x_status_info);
		atpl360_comm_set_event_info(px_events_info, x_status_info->ul_flags);

		/* Extract Timer info */
		px_events_info->ul_timer_ref = ((uint32_t)*puc_info++);
		px_events_info->ul_timer_ref += ((uint32_t)*puc_info++) << 8;
		px_events_info->ul_timer_ref += ((uint32_t)*puc_info++) << 16;
		px_events_info->ul_timer_ref += ((uint32_t)*puc_info++) << 24;

		/* Extract Event info */
		px_events_info->ul_event_info = ((uint32_t)*puc_info++);
		px_events_info->ul_event_info += ((uint32_t)*puc_info++) << 8;
		px_events_info->ul_event_info += ((uint32_t)*puc_info++) << 16;
		px_events_info->ul_event_info += ((uint32_t)*puc_info++) << 24;
	}
}

/**
 * \brief External interrupt handler
 */
static void _handler_atpl360_ext_int(void)
{
	atpl360_events_t x_events_info;
	atpl360_spi_data_t x_spi_data;
	atpl360_spi_status_info_t x_spi_status_info;
	uint8_t uc_i;

	ul_int_cnt++;

	if (sb_component_enabled) {
		/* Time guard */
		sx_atpl360_hal_wrapper.plc_delay(DELAY_TREF_US, 20);

		/* capture information relative to ATPL360 events */
		_get_interrupt_info(&x_events_info, &x_spi_status_info);

		/* Check MSG_CFM_EV_TYPE event */
		for (uc_i = 0; uc_i < NUM_TX_BUFFERS; uc_i++) {
			if (x_events_info.b_cfm_event_enable[uc_i]) {
				/* Read confirm message */
				x_spi_data.us_mem_id = atpl360_comm_get_event_id(MSG_CFM_EV_TYPE, uc_i);
				x_spi_data.puc_data_buf = spuc_cfm_buffer[uc_i];
				x_spi_data.us_len = sizeof(spuc_cfm_buffer[uc_i]);
				if (_spi_send_cmd(SPI_RD_CMD, &x_spi_data) == ATPL360_SUCCESS) {
					spb_cfm_event_enable[uc_i] = true;
					ul_int_tx_cnt++;
				}
			}
		}

		/* Check MSG_IND_DATA_EV_TYPE event (First event in rx) */
		if (x_events_info.b_data_ind_event_enable) {
			/* Read DATA from indication message */
			x_spi_data.us_mem_id = atpl360_comm_get_event_id(MSG_IND_DATA_EV_TYPE, NULL);
			x_spi_data.puc_data_buf = spuc_ind_buffer + sizeof(rx_msg_t) - 4;
			x_spi_data.us_len = ATPL360_GET_EV_DAT_LEN_INFO(x_events_info.ul_event_info);
			if ((x_spi_data.us_len == 0) || (x_spi_data.us_len > ATPL360_DATA_PKT_SIZE)) {
				x_spi_data.us_len = 1;
			}

			if (_spi_send_cmd(SPI_RD_CMD, &x_spi_data) == ATPL360_SUCCESS) {
				sb_data_ind_event_enable = true;
				ul_int_rx_data_cnt++;
			}
		}

		/* Check MSG_IND_PARAM_EV_TYPE event (Second event in rx) */
		if (x_events_info.b_qpar_ind_event_enable) {
			/* Read PARAMS from indication message */
			x_spi_data.us_mem_id = atpl360_comm_get_event_id(MSG_IND_PARAM_EV_TYPE, NULL);
			x_spi_data.puc_data_buf = spuc_ind_buffer;
			x_spi_data.us_len = sizeof(rx_msg_t) - 4;
			if (_spi_send_cmd(SPI_RD_CMD, &x_spi_data) == ATPL360_SUCCESS) {
				sb_param_ind_event_enable = true;
				ul_int_rx_qpar_cnt++;
			}
		}

		/* Check REG_RSP_EV_TYPE event */
		if (x_events_info.b_reg_data_enable) {
			/* Extract data and pkt len from event info */
			sus_reg_event_len = ATPL360_GET_EV_REG_LEN_INFO(x_events_info.ul_event_info);
			if ((sus_reg_event_len == 0) || (sus_reg_event_len > ATPL360_REG_PKT_SIZE)) {
				sus_reg_event_len = 1;
			}

			x_spi_data.us_mem_id = atpl360_comm_get_event_id(REG_EV_TYPE, NULL);
			x_spi_data.puc_data_buf = spuc_reg_buffer;
			x_spi_data.us_len = sus_reg_event_len;
			_spi_send_cmd(SPI_RD_CMD, &x_spi_data);
			ul_int_reg_cnt++;
		}

		/* Time guard */
		sx_atpl360_hal_wrapper.plc_delay(DELAY_TREF_US, 20);
	} else {
		/* Disable EXT INT */
		sx_atpl360_hal_wrapper.plc_enable_int(false);
	}
}

/**
 * \brief Function to get a configuration parameter
 *
 * \param param_id   Identification number of configuration parameter
 * \param px_val     Pointer to value of the configuration parameter
 * \param uc_len     Size in bytes of the configuration parameter
 *
 * \return true if success, false in otherwise
 */
static bool _get_config(uint16_t us_param_id, void *px_value, uint8_t uc_len, bool b_sync)
{
	uint32_t ul_base_int;
	uint16_t us_sec_cnt;

	if (sb_component_enabled) {
		if (us_param_id & ATPL360_REG_ID_MASK) {
			/* set reg value through spi */
			atpl360_spi_data_t x_spi_data;
			uint32_t ul_reg_addr;
			uint32_t ul_offset_addr;
			uint16_t us_reg_len;
			uint8_t *puc_buf;

			/* Set register address */
			ul_reg_addr = (uint32_t)(us_param_id & ATPL360_REG_OFFSET_MASK);

			/* Get base address */
			ul_offset_addr = atpl360_comm_get_cfg_param_access_type(us_param_id);
			if (ul_offset_addr == 0) {
				/* Param error */
				*(uint8_t *)px_value = 0;
				return false;
			}

			ul_reg_addr += ul_offset_addr;

			puc_buf = spuc_reg_buffer;

			x_spi_data.us_len = 8;
			x_spi_data.us_mem_id = ATPL360_REG_INFO_ID;
			x_spi_data.puc_data_buf = puc_buf;

			/* Set cmd and length */
			us_reg_len = ATPL360_REG_CMD_RD | (uint16_t)uc_len;

			/* Build command */
			*puc_buf++ = (uint8_t)(ul_reg_addr >> 24);
			*puc_buf++ = (uint8_t)(ul_reg_addr >> 16);
			*puc_buf++ = (uint8_t)(ul_reg_addr >> 8);
			*puc_buf++ = (uint8_t)(ul_reg_addr);
			*puc_buf++ = (uint8_t)(us_reg_len >> 8);
			*puc_buf++ = (uint8_t)(us_reg_len);

			/* Send command */
			_spi_send_cmd(SPI_WR_CMD, &x_spi_data);

			if (!b_sync) {
				/* wait to interrupt to get reg value */
				return false;
			} else {
				us_sec_cnt = 0xFFFF;

				/* Check interrupt system */
				ul_base_int = __get_BASEPRI();
				if (ul_base_int) {
					__set_BASEPRI(0);
				}

				/* Wait to the response */
				while (!sus_reg_event_len) {
					if (!us_sec_cnt--) {
						/* Restore interrupt system */
						if (ul_base_int) {
							__set_BASEPRI(ul_base_int);
						}

						/* Error in get config cmd */
						return false;
					}
				}

				/* Restore interrupt system */
				if (ul_base_int) {
					__set_BASEPRI(ul_base_int);
				}

				/* copy reg info in data pointer */
				memcpy((uint8_t *)px_value, spuc_reg_buffer, sus_reg_event_len);
				/* Reset event flag */
				sus_reg_event_len = 0;

				return true;
			}
		} else if (us_param_id == ATPL360_TIME_REF_ID) {
			atpl360_spi_data_t x_spi_data;

			x_spi_data.us_len = uc_len;
			x_spi_data.us_mem_id = ATPL360_STATUS_INFO_ID;
			x_spi_data.puc_data_buf = (uint8_t *)px_value;
			/* Send command */
			_spi_send_cmd(SPI_RD_CMD, &x_spi_data);
			return true; /* Don't wait to interrupt to get reg value */
		} else {
			return atpl360_ib_get_param((atpl360_id_param_t)us_param_id, px_value, uc_len);
		}
	} else {
		return false;
	}
}

/**
 * \brief Function to set a configuration parameter
 *
 * \param param_id   Identification number of configuration parameter
 * \param px_value   Pointer to value of the configuration parameter
 * \param us_len     Size of the configuration parameter
 *
 */
static bool _set_config(uint16_t us_param_id, void *px_value, uint16_t us_len)
{
	if (us_param_id & ATPL360_REG_ID_MASK) {
		/* set reg value through spi */
		atpl360_spi_data_t x_spi_data;
		uint32_t ul_reg_addr;
		uint16_t us_reg_len;
		uint8_t *puc_buf;
		uint8_t *pul_ptr;
		uint32_t ul_delay;

		/* Set register address */
		ul_reg_addr = (uint32_t)(us_param_id & ATPL360_REG_OFFSET_MASK);

		/* Get Access type and address */
		ul_reg_addr += atpl360_comm_get_cfg_param_access_type(us_param_id);

		puc_buf = spuc_tx_buffer;

		x_spi_data.us_len = us_len + 8;
		x_spi_data.us_mem_id = ATPL360_REG_INFO_ID;
		x_spi_data.puc_data_buf = puc_buf;

		/* Set cmd and length */
		us_reg_len = ATPL360_REG_CMD_WR | (us_len & ATPL360_REG_LEN_MASK);

		/* Build command */
		*puc_buf++ = (uint8_t)(ul_reg_addr >> 24);
		*puc_buf++ = (uint8_t)(ul_reg_addr >> 16);
		*puc_buf++ = (uint8_t)(ul_reg_addr >> 8);
		*puc_buf++ = (uint8_t)(ul_reg_addr);
		*puc_buf++ = (uint8_t)(us_reg_len >> 8);
		*puc_buf++ = (uint8_t)(us_reg_len);
		pul_ptr = (uint8_t *)px_value;
		if (us_len == 4) {
			*puc_buf++ = *pul_ptr++;
			*puc_buf++ = *pul_ptr++;
			*puc_buf++ = *pul_ptr++;
			*puc_buf++ = *pul_ptr;
		} else if (us_len == 2) {
			*puc_buf++ = *pul_ptr++;
			*puc_buf++ = *pul_ptr++;
		} else {
			memcpy(puc_buf, pul_ptr, us_len);
			puc_buf += us_len;
		}

		/* Update spi data len */
		x_spi_data.us_len = puc_buf - spuc_tx_buffer;

		/* Send command */
		_spi_send_cmd(SPI_WR_CMD, &x_spi_data);

		/* Guard delay to ensure writing operation completion. */
		ul_delay = atpl360_comm_get_cfg_param_delay_us(us_param_id);
		sx_atpl360_hal_wrapper.plc_delay(DELAY_TREF_US, ul_delay);

		return true;
	} else {
		return atpl360_ib_set_param((atpl360_id_param_t)us_param_id, px_value, (uint8_t)us_len);
	}
}

/**
 * \brief Function to send to addons module (manage serial apps)
 *
 * \param px_msg  Pointer to command message
 * \param us_len  Length of the serial command message
 *
 */
static void _send_addons_cmd(uint8_t *px_msg, uint16_t us_len)
{
#ifdef ATPL360_ADDONS_ENABLE
	if (sb_component_enabled) {
		atpl360_addon_cmd(px_msg, us_len);
	}

#else
	(void)px_msg;
	(void)us_len;
#endif
}

/**
 * \brief Function to send data to ATPL360 to transmit msg through PLC
 *
 * \param px_msg  Pointer to message struct to transmit
 *
 */
static uint8_t _send_data(tx_msg_t *px_msg)
{
	if (sb_component_enabled) {
		uint16_t us_params_len;

#ifdef ATPL360_ADDONS_ENABLE
		atpl360_addon_stringify_tx(NULL, px_msg);
#endif

		us_params_len = atpl360_comm_stringify(spuc_tx_buffer, (void *)px_msg, sizeof(tx_msg_t));

		/* Send tx message in 2 steps: parameters and data */
		if (us_params_len) {
			atpl360_spi_data_t x_spi_data;

			if (sus_prod_info & ATPL360_G3_MODE_MASK) {
				/* Send tx params */
				x_spi_data.us_len = us_params_len;
				x_spi_data.us_mem_id = atpl360_comm_get_tx_params_id(px_msg);
				x_spi_data.puc_data_buf = spuc_tx_buffer;
				_spi_send_cmd(SPI_WR_CMD, &x_spi_data);

				sx_atpl360_hal_wrapper.plc_delay(DELAY_TREF_US, ATPL360_DELAY_TX_DATA_US);

				if (!spb_cfm_event_enable[0]) {
					/* Send tx data */
					x_spi_data.us_len = px_msg->us_data_len;
					x_spi_data.us_mem_id = atpl360_comm_get_tx_data_id(px_msg);
					x_spi_data.puc_data_buf = spuc_tx_buffer + us_params_len;
					_spi_send_cmd(SPI_WR_CMD, &x_spi_data);
				}
			} else {
				/* Send tx msg */
				x_spi_data.us_len = us_params_len + px_msg->us_data_len;
				x_spi_data.us_mem_id = atpl360_comm_get_tx_params_id(px_msg);
				x_spi_data.puc_data_buf = spuc_tx_buffer;
				_spi_send_cmd(SPI_WR_CMD, &x_spi_data);
			}

			/* Set TX cfm flag */
			suc_waiting_tx_cfm++;

			return TX_RESULT_PROCESS;
		} else {
			return TX_RESULT_INV_LENGTH;
		}
	} else {
		return TX_RESULT_NO_TX;
	}
}

/**
 * \brief Function to set callbacks to call on upper layer
 *
 * \param px_cbs  Pointer to ATPL360 Callback struct
 *
 */
static void _set_callbacks(atpl360_dev_callbacks_t *px_cbs)
{
	_data_confirm_cb_handler = px_cbs->data_confirm;
	_data_indication_cb_handler = px_cbs->data_indication;
	_addons_event_cb_handler = px_cbs->addons_event;
	_exception_event_cb_handler = px_cbs->exception_event;

#ifdef ATPL360_ADDONS_ENABLE
	atpl360_addon_set_event_callback(px_cbs->addons_event);
#endif
}

/**
 * \brief Function to init internal system
 *
 */
static void _init_system(void)
{
	/* Init Callback function pointers */
	_data_confirm_cb_handler = NULL;
	_data_indication_cb_handler = NULL;
	_addons_event_cb_handler = NULL;

	/* Init internal SPI controller */
	atpl360_spi_initialize();

	/* Initialize HAL SPI */
	sx_atpl360_hal_wrapper.plc_init();
	sx_atpl360_hal_wrapper.plc_reset();

	/* Set ATPl360 handler and enable int */
	sx_atpl360_hal_wrapper.plc_set_handler(_handler_atpl360_ext_int);

	/* Init event indicators */
	memset((uint8_t *)spb_cfm_event_enable, false, NUM_TX_BUFFERS);
	sb_data_ind_event_enable = false;
	sb_param_ind_event_enable = false;
	sus_reg_event_len = 0;

	/* Init internal buffers */
	memset(spuc_tx_buffer, 0, sizeof(spuc_tx_buffer));
	memset(spuc_ind_buffer, 0, sizeof(spuc_ind_buffer));
	memset(spuc_cfm_buffer, 0, sizeof(spuc_cfm_buffer));
#ifdef ATPL360_ADDONS_ENABLE
	memset(spuc_addon_buffer, 0, sizeof(spuc_addon_buffer));
#endif
}

/**
 * \brief Function to initialize ATPL360 instance
 *
 * \param descr           Pointer to ATPL360 descriptor
 * \param px_hal_wrapper  Pointer to HAL wrapper (hardware abstraction layer functions)
 *
 */
void atpl360_init(atpl360_descriptor_t *const px_descr, atpl360_hal_wrapper_t *px_hal_wrapper)
{
	/* Fill HAL wrapper functions to access hardware peripherals */
	memcpy(&sx_atpl360_hal_wrapper, px_hal_wrapper, sizeof(atpl360_hal_wrapper_t));

	px_descr->set_callbacks = _set_callbacks;
	px_descr->send_data = _send_data;
	px_descr->get_config = _get_config;
	px_descr->set_config = _set_config;
	px_descr->send_addons_cmd = _send_addons_cmd;

	/* component must be explicitly enabled */
	atpl360_disable();

#ifdef ATPL360_ADDONS_ENABLE
	sx_addon_desc.send_data = _send_data;
	sx_addon_desc.get_config = _get_config;
	sx_addon_desc.set_config = _set_config;
	sx_addon_desc.puc_addon_buffer = &spuc_addon_buffer[0];
	atpl360_addon_init(&sx_addon_desc);
#endif

	/* Init int counter */
	ul_int_rx_data_cnt = 0;
	ul_int_tx_cnt = 0;
	ul_int_reg_cnt = 0;

	/* Init TX cfm flag */
	suc_waiting_tx_cfm = 0;
	sb_report_tx_cfm_error_by_rst = false;

	/* Database for IB initialization */
	atpl360_ib_init();

	/* Get product info */
	atpl360_ib_get_param(ATPL360_HOST_PRODUCT_ID, &sus_prod_info, sizeof(sus_prod_info));

	/* init internal vars */
	_init_system();
}

/**
 * \brief Function to enable ATPL360 instance
 *
 * \param ul_binary_address   Address where binary file for ATPL360 is stored in internal flash memory
 * \param ul_binary_len       Size of binary file for ATPl360
 *
 * \return Result of enable operation
 */
atpl360_res_t atpl360_enable(uint32_t ul_binary_address, uint32_t ul_binary_len)
{
	atpl360_spi_data_t x_spi_data;
	uint8_t puc_int_buffer[ATPL360_EVENT_DATA_LENGTH];

	/* Disable EXT INT */
	sx_atpl360_hal_wrapper.plc_enable_int(false);

	atpl360_boot_init(ul_binary_address, ul_binary_len);

	/* Read Time Ref to get SPI status */
	x_spi_data.us_mem_id = ATPL360_STATUS_INFO_ID;
	x_spi_data.us_len = sizeof(puc_int_buffer);
	x_spi_data.puc_data_buf = puc_int_buffer;
	if (_spi_send_cmd(SPI_RD_CMD, &x_spi_data) == ATPL360_ERROR) {
		return ATPL360_ERROR;
	}

	sb_component_enabled = true;

	/* Enable EXT INT */
	sx_atpl360_hal_wrapper.plc_enable_int(true);

	return ATPL360_SUCCESS;
}

void atpl360_disable(void)
{
	sb_component_enabled = false;

	/* Disable EXT INT by default */
	sx_atpl360_hal_wrapper.plc_enable_int(false);

	/* PL360 reset */
	sx_atpl360_hal_wrapper.plc_reset();
}

/**
 * \brief Function to Check ATPL360 pending events
 *
 */
void atpl360_handle_events(void)
{
	uint8_t uc_i;

	if (sb_component_enabled) {
		/* Check CFM report error due to ATPL360 reset */
		if (sb_report_tx_cfm_error_by_rst) {
			tx_cfm_t x_tx_cfm;

			sb_report_tx_cfm_error_by_rst = false;
			suc_waiting_tx_cfm = 0;

			x_tx_cfm.uc_tx_result = TX_RESULT_NO_TX;
			x_tx_cfm.ul_tx_time = 0;
			x_tx_cfm.ul_rms_calc = 0;

			if (_data_confirm_cb_handler) {
				_data_confirm_cb_handler(&x_tx_cfm);
			}

			return;
		}

		/* Check msg cfm events */
		for (uc_i = 0; uc_i < NUM_TX_BUFFERS; uc_i++) {
			if (spb_cfm_event_enable[uc_i]) {
				tx_cfm_t x_tx_cfm;
				uint8_t uc_ret;

				suc_waiting_tx_cfm--;

				uc_ret = atpl360_comm_parse((void *)&x_tx_cfm, spuc_cfm_buffer[uc_i], sizeof(tx_cfm_t));

#ifdef ATPL360_ADDONS_ENABLE
				/* Check Addons */
				if (_addons_event_cb_handler) {
					if (uc_ret == ATPL360_COMM_SUCCESS) {
						uint16_t us_rsp_len;

						/* report to addon */
						us_rsp_len = atpl360_addon_stringify_cfm(spuc_addon_buffer, &x_tx_cfm);
						/* execute callback */
						if (us_rsp_len) {
							_addons_event_cb_handler(spuc_addon_buffer, us_rsp_len);
						}
					}
				}
#endif

				if (_data_confirm_cb_handler) {
					if (uc_ret == ATPL360_COMM_SUCCESS) {
						_data_confirm_cb_handler(&x_tx_cfm);
					}
				}

				/* Reset event flag */
				spb_cfm_event_enable[uc_i] = false;
			}
		}

		/* Check quality parameters and data msg ind events */
		if (sb_param_ind_event_enable && sb_data_ind_event_enable) {
			rx_msg_t x_rx_msg;
			uint8_t uc_ret;

			uc_ret = atpl360_comm_parse((void *)&x_rx_msg, spuc_ind_buffer, sizeof(rx_msg_t));

#ifdef ATPL360_ADDONS_ENABLE
			/* Check Addons */
			if (_addons_event_cb_handler) {
				if (uc_ret == ATPL360_COMM_SUCCESS) {
					uint16_t us_addon_len;

					/* report to addon */
					us_addon_len = atpl360_addon_stringify_ind(spuc_addon_buffer, &x_rx_msg);
					/* execute callback */
					if (us_addon_len) {
						_addons_event_cb_handler(spuc_addon_buffer, us_addon_len);
					}
				}
			}
#endif

			if (_data_indication_cb_handler) {
				if (uc_ret == ATPL360_COMM_SUCCESS) {
					_data_indication_cb_handler(&x_rx_msg);
				}
			}

			/* Reset events flag */
			sb_param_ind_event_enable = false;
			sb_data_ind_event_enable = false;
		}

#ifdef ATPL360_ADDONS_ENABLE
		/* Check reg events */
		if (sus_reg_event_len) {
			/* Check Addons */
			if (_addons_event_cb_handler) {
				uint16_t us_addon_len;

				/* report to addon */
				us_addon_len = atpl360_addon_stringify_reg(spuc_addon_buffer, spuc_reg_buffer, sus_reg_event_len);
				/* execute callback */
				if (us_addon_len) {
					_addons_event_cb_handler(spuc_addon_buffer, us_addon_len);
				}
			}

			/* Reset event flag */
			sus_reg_event_len = 0;
		}
#endif
	}
}
