/*
 * init.c: initialize PHY layer
 *
 * Copyright (c) 2020, Tarlogic Security SL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of copyright holders nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS”
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "phy.h"

#include "FreeRTOS.h"
#include "task.h"

#include <boot.h>
#include <ctype.h>
#include <lcd.h>

/************************** Phy layer parameters ****************************/
#define ATPL360_BINARY_ADDRESS                     0x010E0000
#define ATPL360_BINARY_LEN                         0x10000

/* 
 * The following structure is populated by the ATPL360 blob with the
 * addresses of the API.
 */
static atpl360_descriptor_t sx_atpl360_desc; 

/* Commands in GPBR to define behavior of the application at start */
#define PHY_APP_CMD_DEFAULT_MODE       0
#define PHY_APP_CMD_MENU_START_MODE    1
#define PHY_APP_CMD_TX_START_MODE      2

/* FreeRTOS configuration */
/* Tasks priorities */
#define TASK_APP_PHY_PRIO              (tskIDLE_PRIORITY + 1)
#define TASK_APP_GET_TX_RESULT_PRIO    (tskIDLE_PRIORITY + 1)
#define TASK_APP_GET_RX_PRIO           (tskIDLE_PRIORITY + 1)

/* Stack definitions */
#define TASK_APP_PHY_STACK             (configMINIMAL_STACK_SIZE * 5)
#define TASK_APP_GET_TX_RESULT_STACK   (configMINIMAL_STACK_SIZE * 2)
#define TASK_APP_GET_RX_STACK          (configMINIMAL_STACK_SIZE * 2)

/* App Task Period */
#define PRIME_APP_PHY_TIMER_RATE       (5 / portTICK_RATE_MS)

static const uint32_t spul_max_rms_hi[8][NUM_TX_LEVELS] = {
  MAX_RMS_HI_VALUES_CHN_1,
  MAX_RMS_HI_VALUES_CHN_2,
  MAX_RMS_HI_VALUES_CHN_3,
  MAX_RMS_HI_VALUES_CHN_4,
  MAX_RMS_HI_VALUES_CHN_5,
  MAX_RMS_HI_VALUES_CHN_6,
  MAX_RMS_HI_VALUES_CHN_7,
  MAX_RMS_HI_VALUES_CHN_8
};

static const uint32_t spul_max_rms_vlo[8][NUM_TX_LEVELS] = {
  MAX_RMS_VLO_VALUES_CHN_1,
  MAX_RMS_VLO_VALUES_CHN_2,
  MAX_RMS_VLO_VALUES_CHN_3,
  MAX_RMS_VLO_VALUES_CHN_4,
  MAX_RMS_VLO_VALUES_CHN_5,
  MAX_RMS_VLO_VALUES_CHN_6,
  MAX_RMS_VLO_VALUES_CHN_7,
  MAX_RMS_VLO_VALUES_CHN_8
};

static const uint32_t spul_th_hi[8][NUM_TX_LEVELS << 1] = {
  TH_HI_VALUES_CHN_1,
  TH_HI_VALUES_CHN_2,
  TH_HI_VALUES_CHN_3,
  TH_HI_VALUES_CHN_4,
  TH_HI_VALUES_CHN_5,
  TH_HI_VALUES_CHN_6,
  TH_HI_VALUES_CHN_7,
  TH_HI_VALUES_CHN_8
};

static const uint32_t spul_th_vlo[8][NUM_TX_LEVELS << 1] = {
  TH_VLO_VALUES_CHN_1,
  TH_VLO_VALUES_CHN_2,
  TH_VLO_VALUES_CHN_3,
  TH_VLO_VALUES_CHN_4,
  TH_VLO_VALUES_CHN_5,
  TH_VLO_VALUES_CHN_6,
  TH_VLO_VALUES_CHN_7,
  TH_VLO_VALUES_CHN_8
};

static const uint16_t spus_gain_hi_chn_1[3] = {
  IFFT_GAIN_HI_INI_CHN_1, 
  IFFT_GAIN_HI_MIN_CHN_1, 
  IFFT_GAIN_HI_MAX_CHN_1};
  
static const uint16_t spus_gain_vlo_chn_1[3] = {
  IFFT_GAIN_VLO_INI_CHN_1, 
  IFFT_GAIN_VLO_MIN_CHN_1, 
  IFFT_GAIN_VLO_MAX_CHN_1};
  
/* FCC (Channel 3-8) and Channel 2 */
static const uint16_t spus_gain_hi_chn_2_8[3] = {
  IFFT_GAIN_HI_INI_CHN_2_8, 
  IFFT_GAIN_HI_MIN_CHN_2_8, 
  IFFT_GAIN_HI_MAX_CHN_2_8};
  
static const uint16_t spus_gain_vlo_chn_2_8[3] = {
  IFFT_GAIN_VLO_INI_CHN_2_8, 
  IFFT_GAIN_VLO_MIN_CHN_2_8, 
  IFFT_GAIN_VLO_MAX_CHN_2_8};
  
 static const uint16_t spus_equ_hi_chn_1[97] = PREDIST_COEF_HI_VALUES_CHN_1;
 static const uint16_t spus_equ_vlo_chn_1[97] = PREDIST_COEF_VLO_VALUES_CHN_1;
 /* FCC (Channel 3-8) and Channel 2 */
 static const uint16_t spus_equ_hi_chn_2_8[97] = PREDIST_COEF_HI_VALUES_CHN_2_8;
 static const uint16_t spus_equ_vlo_chn_2_8[97] = PREDIST_COEF_VLO_VALUES_CHN_2_8;

 /** DAC configuration. **/
 static const uint32_t spul_dacc_cfg_chn_1[17] = DACC_CFG_TABLE_CHN_1;
 static const uint32_t spul_dacc_cfg_chn_2_8[17] = DACC_CFG_TABLE_CHN_2_8;
 
/******************************* Code start *********************************/
static xTaskHandle phy_task_handle;

/********************************** API *************************************/
extern phy_rx_handler_t rx_handler;
extern void *rx_handler_userdata;
static BOOL sb_pending_except;

BOOL
phy_tx_take_exception(void)
{
  BOOL ret = sb_pending_except;
  
  sb_pending_except = FALSE;
  
  return ret;
}

static void 
_tx_result_handler(tx_cfm_t *px_tx_result)
{
  if (px_tx_result->uc_tx_result != TX_RESULT_SUCCESS)
    lcd_printf(1, "TX ERROR: %d", px_tx_result->uc_tx_result);
}

static void 
_rx_handler(rx_msg_t *x_read_msg)
{
  if (rx_handler != NULL && x_read_msg->us_data_len)
    rx_handler(rx_handler_userdata, x_read_msg->puc_data_buf, x_read_msg->us_data_len);
}

static void 
_error_handler(atpl360_exception_t exception)
{
  if (exception == ATPL360_EXCEPTION_RESET) {
    printf("PL360 Enabled...\n");
    } else {
    printf("ATPL360 Exception code error: %u\n", (unsigned int)exception);
  }

  LED_On(LED1);
  sb_pending_except = TRUE;
}

#if defined (__CC_ARM)
  extern uint8_t atpl_bin_start[];
  extern uint8_t atpl_bin_end[];
#elif defined (__GNUC__)
  extern uint8_t atpl_bin_start;
  extern uint8_t atpl_bin_end;
#elif defined (__ICCARM__)
#  pragma section = "P_atpl_bin"
  extern uint8_t atpl_bin;
#else
#  error This compiler is not supported for now.
#endif

static uint32_t 
_get_pl360_bin_addressing(uint32_t *pul_address)
{
  uint32_t ul_bin_addr;
  uint8_t *puc_bin_start;
  uint8_t *puc_bin_end;

#if defined (__CC_ARM)
  ul_bin_addr = (int)(atpl_bin_start - 1);
  puc_bin_start = atpl_bin_start - 1;
  puc_bin_end = atpl_bin_end;
#elif defined (__GNUC__)
  ul_bin_addr = (uint32_t) &atpl_bin_start;
  puc_bin_start = (uint8_t *) &atpl_bin_start;
  puc_bin_end = (uint8_t *) &atpl_bin_end;
#elif defined (__ICCARM__)
  ul_bin_addr = (int)&atpl_bin;
  puc_bin_start = __section_begin("P_atpl_bin");
  puc_bin_end = __section_end("P_atpl_bin");
#else
  #error This compiler is not supported for now.
#endif

  *pul_address = ul_bin_addr;
  return ((uint32_t)puc_bin_end - (uint32_t)puc_bin_start);
}

static void 
_configure_tx_coup_params(uint8_t uc_chn)
{
  uint16_t *pus_equ_hi, *pus_equ_vlo, *pus_gain_hi, *pus_gain_vlo;
  uint32_t *pul_dacc_table;

  sx_atpl360_desc.set_config(ATPL360_REG_MAX_RMS_TABLE_HI, (uint32_t *)spul_max_rms_hi[uc_chn - 1], NUM_TX_LEVELS << 2);
  sx_atpl360_desc.set_config(ATPL360_REG_MAX_RMS_TABLE_VLO, (uint32_t *)spul_max_rms_vlo[uc_chn - 1], NUM_TX_LEVELS << 2);
  sx_atpl360_desc.set_config(ATPL360_REG_THRESHOLDS_TABLE_HI, (uint32_t *)spul_th_hi[uc_chn - 1], NUM_TX_LEVELS << 3);
  sx_atpl360_desc.set_config(ATPL360_REG_THRESHOLDS_TABLE_VLO, (uint32_t *)spul_th_vlo[uc_chn - 1], NUM_TX_LEVELS << 3);

  if (uc_chn >= 2) {
    /* Channel 2 - 8 */
    pus_equ_hi = (uint16_t *)spus_equ_hi_chn_2_8;
    pus_equ_vlo = (uint16_t *)spus_equ_vlo_chn_2_8;
    pus_gain_hi = (uint16_t *)spus_gain_hi_chn_2_8;
    pus_gain_vlo = (uint16_t *)spus_gain_vlo_chn_2_8;
    pul_dacc_table = (uint32_t *)spul_dacc_cfg_chn_2_8;
    } else {
    /* Channel 1 */
    pus_equ_hi = (uint16_t *)spus_equ_hi_chn_1;
    pus_equ_vlo = (uint16_t *)spus_equ_vlo_chn_1;
    pus_gain_hi = (uint16_t *)spus_gain_hi_chn_1;
    pus_gain_vlo = (uint16_t *)spus_gain_vlo_chn_1;
    pul_dacc_table = (uint32_t *)spul_dacc_cfg_chn_1;
  }

  sx_atpl360_desc.set_config(ATPL360_REG_GAIN_TABLE_HI, pus_gain_hi, 6);
  sx_atpl360_desc.set_config(ATPL360_REG_GAIN_TABLE_VLO, pus_gain_vlo, 6);
  sx_atpl360_desc.set_config(ATPL360_REG_DACC_TABLE_CFG, pul_dacc_table, 17 << 2);
  sx_atpl360_desc.set_config(ATPL360_REG_PREDIST_COEF_TABLE_HI, pus_equ_hi, 97 << 1);
  sx_atpl360_desc.set_config(ATPL360_REG_PREDIST_COEF_TABLE_VLO, pus_equ_vlo, 97 << 1);
}

void
phy_reset_params(void)
{
  uint8_t autodetect = 0;
  uint8_t impedance = 2;
  uint8_t channel = 1;
  
  /* Update channel */
  sx_atpl360_desc.set_config(ATPL360_REG_CHANNEL_CFG, &channel, 1);
  _configure_tx_coup_params(channel); /* TODO: Configure */

  /* Update impedance */
  sx_atpl360_desc.set_config(
    ATPL360_REG_CFG_AUTODETECT_IMPEDANCE, 
    &autodetect, 
    1);
    
  sx_atpl360_desc.set_config(
    ATPL360_REG_CFG_IMPEDANCE, 
    &impedance, 
    1); 
}

static void
atpl360_low_level_init(void)
{
  uint32_t ul_bin_addr;
  uint32_t ul_bin_size;
  atpl360_dev_callbacks_t x_atpl360_cbs;
  atpl360_hal_wrapper_t x_atpl360_hal_wrp;
  uint8_t uc_ret;

  
  /* Init ATPL360 */
  x_atpl360_hal_wrp.plc_init = hal_plc_init;
  x_atpl360_hal_wrp.plc_reset = hal_plc_reset;
  x_atpl360_hal_wrp.plc_set_handler = hal_plc_set_handler;
  x_atpl360_hal_wrp.plc_send_boot_cmd = hal_plc_send_boot_cmd;
  x_atpl360_hal_wrp.plc_write_read_cmd = hal_plc_send_wrrd_cmd;
  x_atpl360_hal_wrp.plc_enable_int = hal_plc_enable_interrupt;
  x_atpl360_hal_wrp.plc_delay = hal_plc_delay;
  atpl360_init(&sx_atpl360_desc, &x_atpl360_hal_wrp);

  /* Callback configuration. Set NULL as Not used */
  x_atpl360_cbs.data_confirm = _tx_result_handler;
  x_atpl360_cbs.data_indication = _rx_handler;
  x_atpl360_cbs.exception_event = _error_handler;
  x_atpl360_cbs.addons_event = NULL;
  sx_atpl360_desc.set_callbacks(&x_atpl360_cbs);

  /* Enable ATPL360 */
  ul_bin_size = _get_pl360_bin_addressing(&ul_bin_addr);
  uc_ret = atpl360_enable(ul_bin_addr, ul_bin_size);
  if (uc_ret == ATPL360_ERROR) {
    printf("main: atpl360_enable call error! (%d)\n", uc_ret);
    hang();
  }

  phy_reset_params();
}

static void
phy_task(void *params)
{
  static portTickType xLastWakeTime;
  static portTickType xPeriod;
  
  xPeriod = PRIME_APP_PHY_TIMER_RATE;
  xLastWakeTime = xTaskGetTickCount();
  
  UNUSED(params);
  
  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
    atpl360_handle_events();
  }
}

BOOL
phy_send_data(tx_msg_t *template, const void *data, size_t size)
{
  template->puc_data_buf = (uint8_t *) data;
  template->us_data_len = (uint16_t) size;
  
  return sx_atpl360_desc.send_data(template) == TX_RESULT_PROCESS;
}

void
phy_init(void)
{
  /* Initialize hardware */
  atpl360_low_level_init();
  
  /* Create task to dispatch events */
  xTaskCreate(
    phy_task, 
    (const signed char *const) "PHY", 
    TASK_APP_PHY_STACK, 
    NULL, 
    TASK_APP_PHY_PRIO, 
    &phy_task_handle);
}
