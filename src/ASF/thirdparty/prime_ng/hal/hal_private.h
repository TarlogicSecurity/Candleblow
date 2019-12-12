/**
 * \file
 *
 * \brief HAL_PRIVATE: PRIME Hardware Abstraction Layer private headers
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

#ifndef HAL_PRIVATE_H_INCLUDE
#define HAL_PRIVATE_H_INCLUDE

#include "compiler.h"
#include "hal.h"
#include "hal_regions.h"

/* @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* @endcond */

/** \brief AT45DBX Commands */
/* @{ */
/** Status Register Read (Serial/8-bit Mode) */
#define AT45DBX_RD_STATUS_REG        0xD7
/** Manufacturer and Device ID Read (Serial Mode) */
#define AT45DBX_RD_MNFCT_DEV_ID_SM   0x9F
/** Buffer 1 Write (Serial/8-bit Mode). */
#define AT45DBX_WR_BUF1              0x84
/** Buffer 1 Read, Any-Frequency Mode (8-bit Mode) */
#define AT45DBX_RD_BUF1_AF_8M        0x54
/** Main Memory Page Program through Buffer 1 (Serial/8-bit Mode) */
#define AT45DBX_PR_PAGE_TH_BUF1      0x82
/** Main Memory Page Read (Serial/8-bit Mode) */
#define AT45DBX_RD_PAGE              0xD2
/** Main Memory Page to Buffer 1 Transfer (Serial/8-bit Mode) */
#define AT45DBX_XFR_PAGE_TO_BUF1     0x53
/** Buffer 1 to Main Memory Page Program with Built-in Erase (Serial/8-bit Mode) */
#define AT45DBX_PR_BUF1_TO_PAGE_ER   0x83
/** Sector Erase (Serial/8-bit Mode). */
#define AT45DBX_ER_SECTOR            0x7C
/* @} */

/** \brief AT45DB32 Addressing */
/* @{ */
#define AT45DB32_PAGE_ADDR_MASK      0x003FFE00
#define AT45DB32_BUFF_ADDR_MASK      0x000001FF
/* @} */

/** \brief Hardware Abstraction Layer Interface */
/* @{ */
void hal_init(void);
void hal_start(void);
void hal_process(void);

/** System interface */
void hal_restart_system(void);

/** \brief Timer Counters interface */
/* @{ */
void hal_set_gp_timer_handler(hal_gp_timer_t gpt, void (*p_handler)(void));
void hal_clear_gp_timer_handler(hal_gp_timer_t gpt);
uint8_t hal_timer_init(hal_gp_timer_t gpt, hal_timer_mode_t mode, hal_timer_clk_src_t clk_src);
uint32_t hal_timer_count_get(hal_gp_timer_t gpt);
void hal_timer_stop(hal_gp_timer_t gpt, hal_timer_mode_t mode);
void hal_timer_plc_init(uint32_t uc_time_us);
void hal_timer_plc_stop(void);
void hal_set_plc_timer_handler(void (*p_handler)(void));
/* @} */

/** \brief ABSP interface */
/* @{ */
uint32_t hal_pcrc_calc(uint8_t *puc_buf, uint32_t ul_len, uint8_t uc_header_type, uint8_t uc_crc_type, bool b_v14_mode);
uint32_t hal_pcrc_calc_fu(uint8_t *puc_buf, uint32_t ul_len, uint32_t ul_crc_init);
void hal_pcrc_config_sna(uint8_t *puc_sna);
/* @} */

/** \brief GPIO interface */
/* @{ */
void hal_gpio_enable(hal_gpio_bank_id_t uc_gpio_bank_id);
void hal_gpio_disable(hal_gpio_bank_id_t uc_gpio_bank_id);
bool hal_gpio_set_dir(hal_gpio_bank_id_t uc_gpio_bank_id, uint8_t uc_gpio, hal_gpio_dir_t direction);
bool hal_gpio_set_level(hal_gpio_bank_id_t uc_gpio_bank_id, uint8_t uc_gpio, bool b_level);
uint8_t hal_gpio_get_level(hal_gpio_bank_id_t uc_gpio_bank_id, uint8_t uc_gpio);
/* @} */

/** \brief UART/USART interface */
/* @{ */
uint8_t hal_uart_open(uint8_t chn, uint32_t bauds);
uint8_t hal_uart_close(uint8_t chn);
uint16_t hal_uart_read(uint8_t chn, void *buffer, uint16_t len);
uint16_t hal_uart_write(uint8_t chn, const void *buffer, uint16_t len);
int hal_uart_if_rx_char(uint8_t chn);
uint16_t hal_uart_if_tx_char(uint8_t chn, char data);
bool hal_uart_is_free(uint8_t chn);
uint8_t hal_usart_open(uint8_t chn, uint32_t bauds);
uint8_t hal_usart_close(uint8_t chn);
uint16_t hal_usart_read(uint8_t chn, void *buffer, uint16_t len);
uint16_t hal_usart_write(uint8_t chn, const void *buffer, uint16_t len);
int hal_usart_rx_char(uint8_t chn);
uint16_t hal_usart_tx_char(uint8_t chn, char data);
bool hal_usart_is_free(uint8_t chn);
/* @} */

/** \brief Null Device USI interface */
/* @{ */
uint8_t hal_null_dev_open(uint8_t chn, uint32_t bauds);
uint8_t hal_null_dev_close(uint8_t chn);
uint16_t hal_null_dev_read(uint8_t chn, void *buffer, uint16_t len);
uint16_t hal_null_dev_write(uint8_t chn, const void *buffer, uint16_t len);
/* @} */

/** \brief USB USI interface */
/* @{ */
bool hal_usb_cdc_enable(uint8_t port);
void hal_usb_cdc_disable(uint8_t port);
void hal_usb_cdc_sof_action(void);
void hal_usb_cdc_suspend_action(void);
void hal_usb_cdc_resume_action(void);
void hal_usb_cdc_set_dtr(uint8_t port, bool b_enable);
void hal_usb_cdc_rx_notify(uint8_t port);
void hal_usb_cdc_set_coding_ext(uint8_t port, void *cfg);
void hal_usb_udc_start(void);
uint16_t hal_usb_udc_read_buf(uint8_t *puc_buff, uint16_t us_max_len);
uint16_t hal_usb_udc_write_buf(uint8_t *puc_buff, uint16_t us_len);
bool hal_usb_udc_is_tx_ready(void);

/* @} */

/** \brief LED interface */
/* @{ */
void hal_led_set(uint8_t led_id, uint8_t b_value);
bool hal_led_read(uint8_t led_id);
void hal_led_toggle(uint8_t led_id);
/* @} */

/** \brief PLC interface */
/* @{ */
void hal_plc_init(void);
void hal_plc_reset(void);
int8_t hal_plc_cmd_op(uint8_t uc_cmd, uint16_t us_addr, uint16_t us_len, uint8_t *ptr_buf, uint8_t uc_bytes_rep);
void hal_plc_set_handler(void (*p_handler)(void));
void hal_plc_set_tx_signalling_handler(void (*p_handler)(void));
void hal_plc_set_rx_signalling_handler(void (*p_handler)(void));
void hal_plc_tx_signal(void);
void hal_plc_rx_signal(void);

#ifdef HAL_ATPL360_INTERFACE
bool hal_plc_send_boot_cmd(uint16_t us_cmd, uint32_t ul_addr, uint32_t ul_data_len, uint8_t *puc_data_buf, uint8_t *puc_data_read);
bool hal_plc_send_wrrd_cmd(uint8_t uc_cmd, void *px_spi_data, void *px_spi_status_info);
void hal_plc_enable_interrupt(bool enable);
void hal_plc_delay(uint8_t uc_tref, uint32_t ul_delay);
bool hal_plc_get_cd(void);
#endif
/* @} */

/** \brief Permanent store of configuration parameters */
/* @{ */
bool hal_get_config_info(config_info_type_t cfg_type, uint16_t us_size, void *pv_data);
bool hal_set_config_info(config_info_type_t cfg_type, uint16_t us_size, void *pv_data);

#ifdef HAL_NWK_RECOVERY_INTERFACE
uint32_t hal_nwk_recovery_init(void);
void hal_nwk_recovery_read(uint32_t addr, uint8_t *puc_buf, uint16_t us_size);
uint8_t hal_nwk_recovery_write(uint32_t addr, uint8_t *puc_buf, uint16_t us_size);
#endif
/* @} */

/** \brief Firmware Upgrade interface */
/* @{ */
void hal_fu_data_read(uint32_t addr, uint8_t *puc_buf, uint16_t us_size);
uint8_t hal_fu_data_write(uint32_t addr, uint8_t *puc_buf, uint16_t us_size);
void hal_fu_data_cfg_read(void *pv_dst, uint16_t us_size);
uint8_t hal_fu_data_cfg_write(void *pv_src, uint16_t us_size);
void hal_fu_start(hal_fu_info_t *x_fu_info);
void hal_fu_end(hal_fu_result_t uc_hal_res);
void hal_fu_revert(void);
void hal_fu_crc_calculate(void);
void hal_fu_crc_set_callback(void (*p_handler)(uint32_t ul_crc));
void hal_fu_signature_image_check(void);
void hal_fu_signature_image_check_set_callback(void (*p_handler)(hal_fu_verif_result_t uc_result));
void hal_fu_result_set_callback(void (*p_handler)(hal_fu_result_t *uc_result));
void hal_fu_process(void);
uint16_t hal_fu_get_bitmap(uint8_t *puc_bitmap, uint32_t *pus_num_rcv_pages);
void hal_fu_config_regions(uint8_t uc_num_regs, x_fu_region_cfg_t *px_fu_reg);
uint8_t hal_fu_data_copy(uint32_t ul_address_dest, uint32_t ul_address_src, uint16_t us_size);
void hal_fu_init(void);
uint8_t hal_fu_swap(void);
/* @} */

/** \brief Universal Serial Interface */
/* @{ */
void hal_usi_init(void);
usi_status_t hal_usi_set_callback(usi_protocol_t protocol_id, uint8_t (*p_handler)(uint8_t *puc_rx_msg, uint16_t us_len), uint8_t usi_port);
void hal_usi_process(void);
usi_status_t hal_usi_send_cmd(void *msg);
void hal_usi_txrx_block_timer(void);
/* @} */

/** \brief True Random Number Generator */
/* @{ */
void hal_trng_init(void);
uint32_t hal_trng_read(void);
/* @} */

/** \brief AT45DB32 Interface */
/* @{ */
void hal_at45dbx_init(void);
uint8_t hal_at45dbx_send_cmd(uint8_t uc_cmd, uint32_t ul_addr, uint8_t *puc_data, uint16_t uc_len);
void hal_at45dbx_wait_is_ready(void);
/* @} */

/** \brief Debug Interface */
/* @{ */
void hal_debug_report(uint32_t ul_err_type);
/* @} */

/** \brief Network Frequency */
/* @{ */
#ifndef HAL_ATPL360_INTERFACE
void hal_net_freq_init(void);
void hal_net_freq_process(void);
#endif
uint32_t hal_net_get_freq(void);
/* @} */

/** \brief Supply monitor */
/* @{ */
void hal_setup_supply_monitor (uint32_t ul_monitoring_rate, uint32_t ul_monitor_threshold);
/* @} */

#if PRIME_HAL_VERSION == HAL_PRIME_1_4
/** \brief AES Interface */
/* @{ */
void hal_aes_init(void);
void hal_aes_set_callback(void (*p_handler)(void));
void hal_aes_key(uint8_t *puc_key, uint8_t uc_key_len);
void hal_aes_crypt(bool b_crypt_mode, uint8_t *puc_in_text, uint8_t *puc_out_text);
/* @} */
#endif

/** \brief Parameter Information Base Interface */
/* @{ */
void hal_pib_init(void);
void hal_pib_get_request(uint16_t us_pib_attrib);
void hal_pib_get_request_set_callback(void (*p_handler)(uint8_t uc_result, uint16_t us_pib_attrib, void *pv_pib_value, uint8_t uc_pib_size));
void hal_pib_set_request(uint16_t us_pib_attrib, void *pv_pib_value, uint8_t uc_pib_size);
void hal_pib_set_request_set_callback(void (*p_handler)(uint8_t uc_result));
/* @} */

/** \brief Reset Handler Interface */
/* @{ */
void hal_reset_handler_init(void);
void hal_reset_trigger(uint8_t uc_reset_type);
/* @} */

/** \brief Watchdog setup */
/* @{ */
void hal_watchdog_setup(uint32_t ul_watchdog_time);
/* @} */

/* @} */

/* @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* @endcond */
#endif /* HAL_PRIVATE_H_INCLUDE */
