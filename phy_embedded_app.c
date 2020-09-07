/**
 * \file
 *
 * \brief PHY_EMBEDDED_APP : ATMEL PLC PHY TX Test Console Application
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
#include <stdio.h>
#include "string.h"

/* ASF includes */
#include "asf.h"

#include "hal.h"
#include "hal_private.h"
#include "conf_hal.h"
#if (BOARD == PL360BN)
#include "conf_eth.h"
#endif

/* App includes */
#include "conf_app_example.h"
#include "phy_embedded_app.h"

/* Phy includes */
#include "atpl360.h"
#include "atpl360_comm.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/* Define HAL API interface */
extern const hal_api_t hal_api;

/* Get serial interface */
extern uint32_t serial_read_char(uint8_t *c);

/* Tasks handlers */
xTaskHandle xappPhyHand;
xTaskHandle xappRamHand;
xTaskHandle xappEthHand;
xTaskHandle xappTxResHand;
xTaskHandle xappRxHand;
xTaskHandle xappConsoleHand;

static uint8_t sb_exception_pend;

/* TX configuration parameters */
static txPhyEmbeddedConfig_t xTxPhyCfg;

/* ATPL360 component */
static atpl360_descriptor_t sx_atpl360_desc;

/* Component ATPL360 Binary Information */
#if BOARD == PL360G55CF_EK
#define ATPL360_BINARY_ADDRESS                     0x0044E000
#else
#define ATPL360_BINARY_ADDRESS                     0x010E0000
#endif
#define ATPL360_BINARY_LEN                         0x10000

/* Tx & Coupling parameters for each channel */
/****************************************************************************************************/
/* IMPORTANT!!! : The following values are valid for MCHP EKs. **************************************/
/* For other hardware designs, calibration values should be checked with MCHP PHY Calibration Tool. */
/****************************************************************************************************/
/* ATPL360_REG_MAX_RMS_TABLE_HI. Target RMS_CALC in HI mode value when dynamic gain is enabled (ATPL360_REG_CFG_AUTODETECT_IMPEDANCE = 1 or 2). */
/* For each channel, there are 8 values, corresponding to first 8 Tx attenuation levels (1 dB steps). */
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

/* ATPL360_REG_MAX_RMS_TABLE_VLO. Target RMS_CALC in VLO mode value when dynamic gain is enabled (ATPL360_REG_CFG_AUTODETECT_IMPEDANCE = 1 or 2). */
/* For each channel, there are 8 values, corresponding to first 8 Tx attenuation levels (1 dB steps). */
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

/* ATPL360_REG_THRESHOLDS_TABLE_HI. Thresholds to change impedance mode (ATPL360_REG_CFG_AUTODETECT_IMPEDANCE = 1) from HI mode. */
/* For each channel there are 16 values. */
/* First 8 values (one per Tx level) are thresholds to change from HI to LO (0 to disable). */
/* Next 8 values (one per Tx level) are thresholds to change from HI to VLO. When RMS_CALC is below threshold, impedance mode changes to VLO. */
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

/* ATPL360_REG_THRESHOLDS_TABLE_VLO. Thresholds to change impedance mode (ATPL360_REG_CFG_AUTODETECT_IMPEDANCE = 1) from VLO mode. */
/* For each channel there are 16 values. */
/* First 8 values (one per Tx level) are thresholds to change from VLO to LO (0 to disable). */
/* Next 8 values (one per Tx level) are thresholds to change from VLO to HI. When RMS_CALC is above threshold, impedance mode changes to HI. */
/* For channels 2-8, threshold to change from VLO to HI is disabled (very high value). */
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

/* Gain values for HI and VLO modes. There are three values: {GAIN_INI, GAIN_MIN, GAIN_MAX} */
/* CENELEC-A (Channel 1) */
static const uint16_t spus_gain_hi_chn_1[3] = {IFFT_GAIN_HI_INI_CHN_1, IFFT_GAIN_HI_MIN_CHN_1, IFFT_GAIN_HI_MAX_CHN_1};
static const uint16_t spus_gain_vlo_chn_1[3] = {IFFT_GAIN_VLO_INI_CHN_1, IFFT_GAIN_VLO_MIN_CHN_1, IFFT_GAIN_VLO_MAX_CHN_1};
/* FCC (Channel 3-8) and Channel 2 */
static const uint16_t spus_gain_hi_chn_2_8[3] = {IFFT_GAIN_HI_INI_CHN_2_8, IFFT_GAIN_HI_MIN_CHN_2_8, IFFT_GAIN_HI_MAX_CHN_2_8};
static const uint16_t spus_gain_vlo_chn_2_8[3] = {IFFT_GAIN_VLO_INI_CHN_2_8, IFFT_GAIN_VLO_MIN_CHN_2_8, IFFT_GAIN_VLO_MAX_CHN_2_8};

/* Equalization values for HI and VLO modes. Specific gain for each carrier to equalize transmission and compensate HW filter frequency response. */
/* CENELEC-A (Channel 1). */
static const uint16_t spus_equ_hi_chn_1[97] = PREDIST_COEF_HI_VALUES_CHN_1;
static const uint16_t spus_equ_vlo_chn_1[97] = PREDIST_COEF_VLO_VALUES_CHN_1;
/* FCC (Channel 3-8) and Channel 2 */
static const uint16_t spus_equ_hi_chn_2_8[97] = PREDIST_COEF_HI_VALUES_CHN_2_8;
static const uint16_t spus_equ_vlo_chn_2_8[97] = PREDIST_COEF_VLO_VALUES_CHN_2_8;

/** DAC configuration. **/
static const uint32_t spul_dacc_cfg_chn_1[17] = DACC_CFG_TABLE_CHN_1;
static const uint32_t spul_dacc_cfg_chn_2_8[17] = DACC_CFG_TABLE_CHN_2_8;

static void _set_test_params(void)
{
	xTxPhyCfg.uc_autodetect = 0;
	xTxPhyCfg.uc_impedance = 2;

	printf("Set test params: Imp(%u) Chn(%u)\r\n", (unsigned int)xTxPhyCfg.uc_impedance, (unsigned int)xTxPhyCfg.uc_channel);
}

/**
 * \internal
 * \brief Handler of the result of transmission. Tx result is reported through UART.
 *
 * \param px_tx_result Pointer to result information.
 */
static void _tx_result_handler(tx_cfm_t *px_tx_result)
{
	switch (px_tx_result->uc_tx_result) {
	case TX_RESULT_PROCESS:
		printf("<-TX_RESULT_PROCESS\r\n");
		break;

	case TX_RESULT_SUCCESS:
		printf("<-TX_RESULT_SUCCESS\r\n");
		break;

	case TX_RESULT_INV_LENGTH:
		printf("<-TX_RESULT_INV_LENGTH\r\n");
		break;

	case TX_RESULT_BUSY_CH:
		printf("<-TX_RESULT_BUSY_CH\r\n");
		break;

	case TX_RESULT_BUSY_TX:
		printf("<-TX_RESULT_BUSY_TX\r\n");
		break;

	case TX_RESULT_BUSY_RX:
		printf("<-TX_RESULT_BUSY_RX\r\n");
		break;

	case TX_RESULT_INV_SCHEME:
		printf("<-TX_RESULT_INV_SCHEME\r\n");
		break;

	case TX_RESULT_TIMEOUT:
		printf("<-TX_RESULT_TIMEOUT\r\n");
		break;

	case TX_RESULT_INV_BUFFER:
		printf("<-TX_RESULT_INV_BUFFER\r\n");
		break;

	case TX_RESULT_INV_MODE:
		printf("<-TX_RESULT_INV_MODE\r\n");
		break;

	default:
		printf("<-ERROR: NOT FOUND\r\n");
		break;
	}
}

/**
 * \internal
 * \brief Handler of the reception message. Only blink a led when data message is received.
 *
 * \param x_read_msg Pointer to received data message.
 */
static void _rx_handler(rx_msg_t *x_read_msg)
{
	uint32_t ul_wait_counter;

	/* build response */
	if (x_read_msg->us_data_len) {
		/* blink Reception LED */
#if (BOARD != SAME70_XPLAINED)
		LED_Toggle(LED1);
#else
		LED_Toggle(LED0);
#endif

		ul_wait_counter = 0xFFFF;
		while (ul_wait_counter--) {
		}
	}
}

/**
 * \internal
 * \brief Exceptions Handler
 *
 * \param exception Exception message
 */
static void _error_handler(atpl360_exception_t exception)
{
	if (exception == ATPL360_EXCEPTION_RESET) {
		printf("PL360 Enabled...\r\n");
	} else {
		printf("ATPL360 Exception code error: %u\r\n", (unsigned int)exception);
	}

	LED_On(LED0);
	sb_exception_pend = true;
}

/**
 * \brief Function to configure Tx and coupling parameters for each channel.
 */
static void _configure_tx_coup_params(uint8_t uc_chn)
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

/**
 * \brief Get PL360 binary addressing.
 *
 * \param pul_address   Pointer to store the initial address of PL360 binary data
 *
 * \return Size of PL360 binary file
 */
static uint32_t _get_pl360_bin_addressing(uint32_t *pul_address)
{
	uint32_t ul_bin_addr;
	uint8_t *puc_bin_start;
	uint8_t *puc_bin_end;

#if defined (__CC_ARM)
	extern uint8_t atpl_bin_start[];
	extern uint8_t atpl_bin_end[];
	ul_bin_addr = (int)(atpl_bin_start - 1);
	puc_bin_start = atpl_bin_start - 1;
	puc_bin_end = atpl_bin_end;
#elif defined (__GNUC__)
	extern uint8_t atpl_bin_start;
	extern uint8_t atpl_bin_end;
	ul_bin_addr = (int)&atpl_bin_start;
	puc_bin_start = (int)&atpl_bin_start;
	puc_bin_end = (int)&atpl_bin_end;
#elif defined (__ICCARM__)
	#pragma section = "P_atpl_bin"
	extern uint8_t atpl_bin;
	ul_bin_addr = (int)&atpl_bin;
	puc_bin_start = __section_begin("P_atpl_bin");
	puc_bin_end = __section_end("P_atpl_bin");
#else
	#error This compiler is not supported for now.
#endif

	*pul_address = ul_bin_addr;
	return ((uint32_t)puc_bin_end - (uint32_t)puc_bin_start);
}

static void prvAppPhyTask(void *pvParameters)
{
	uint32_t ul_bin_addr;
	uint32_t ul_bin_size;
	static portTickType xLastWakeTime;
	static portTickType xPeriod;
	atpl360_dev_callbacks_t x_atpl360_cbs;
	atpl360_hal_wrapper_t x_atpl360_hal_wrp;
	uint8_t uc_ret;

	UNUSED(pvParameters);

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
		printf("\r\nmain: atpl360_enable call error!(%d)\r\n", uc_ret);uc_autodetect
		while (1) {
		}
	}

	/* Update channel */
	sx_atpl360_desc.set_config(ATPL360_REG_CHANNEL_CFG, &xTxPhyCfg.uc_channel, 1);
	_configure_tx_coup_params(xTxPhyCfg.uc_channel);

	/* Update impedance */
	sx_atpl360_desc.set_config(ATPL360_REG_CFG_AUTODETECT_IMPEDANCE, &xTxPhyCfg.uc_autodetect, 1);
	sx_atpl360_desc.set_config(ATPL360_REG_CFG_IMPEDANCE, &xTxPhyCfg.uc_impedance, 1);

	xPeriod = PRIME_APP_PHY_TIMER_RATE;
	xLastWakeTime = xTaskGetTickCount();
	for (;;) {
		vTaskDelayUntil(&xLastWakeTime, xPeriod);

		/* Check ATPL360 pending events */
		atpl360_handle_events();
	}
}

static void prvAppPhyTxTask(void *pvParameters)
{
	static portTickType xLastWakeTime;
	static portTickType xPeriod;
	uint32_t ul_gpbr_value;
	uint8_t uc_choice;

	(void)pvParameters;

	xPeriod = xTxPhyCfg.ul_tx_period / portTICK_RATE_MS;
	xLastWakeTime = xTaskGetTickCount();
	for (;;) {
		vTaskDelayUntil(&xLastWakeTime, xPeriod);

		if (sb_exception_pend) {
			sb_exception_pend = false;

			/* Update channel */
			_set_test_params();

			sx_atpl360_desc.set_config(ATPL360_REG_CHANNEL_CFG, &xTxPhyCfg.uc_channel, 1);
			_configure_tx_coup_params(xTxPhyCfg.uc_channel);

			/* Update impedance */
			sx_atpl360_desc.set_config(ATPL360_REG_CFG_AUTODETECT_IMPEDANCE, &xTxPhyCfg.uc_autodetect, 1);
			sx_atpl360_desc.set_config(ATPL360_REG_CFG_IMPEDANCE, &xTxPhyCfg.uc_impedance, 1);
		}

		if (serial_read_char(&uc_choice) == 0) {
			if ((uc_choice == 'x') || (uc_choice == 'X')) {
				ul_gpbr_value = gpbr_read(GPBR0);
				ul_gpbr_value &= 0xFFFFFFF0;
				ul_gpbr_value |= PHY_APP_CMD_MENU_START_MODE;
				gpbr_write(GPBR0, ul_gpbr_value);
				rstc_start_software_reset(RSTC);
			}
		}

		/* Restart watchdog */
		wdt_restart(WDT);
		sx_atpl360_desc.send_data(&xTxPhyCfg.xPhyMsg);
		printf("->Send message\r\n");
		fflush(stdout);
	}
}

#if (BOARD == PL360BN)
static void prvAppSDRAMTask(void *pvParameters)
{
    UNUSED(pvParameters);

    printf("==================================\n");    
    printf("START Memory TEST \r\n");    
    printf("==================================\n"); 
        
    uint32_t *ptr32_ramdom_buffer = (uint32_t *) 0x20410000;
    const uint32_t c_random_len = 0x3FFF;
    const uint32_t c_sdram_base = 0x70000000;
    const uint32_t c_sdram_end  = 0x70800000;
    uint32_t ui_loop = 0;
    uint32_t i = 0;
    
    while(1) {
        uint32_t ui =0;
        printf("==============================Memory TEST Loop %d ==============================\r\n", ui_loop);        
        
        for (i = 0; i < c_random_len; i++)
	  ptr32_ramdom_buffer[i]= rand();
        
        //Copy random buffer into SDRAM
        printf("Copy random buffer to SDRAM\n\r");
        for (ui=c_sdram_base; ui < c_sdram_end; ) {
                *((uint32_t*)ui) = ptr32_ramdom_buffer[ui & c_random_len];			
                ui+=4;
                /*if ( ui % c_random_len == 0) {
                   printf(".");
                }*/
                if ( ui % 0x3FF == 0) {
                    /* Each kb, surrender time */
                    wdt_restart(WDT);
                    vTaskDelay(1 / portTICK_RATE_MS );
                }
        }
        printf(".\r\n");
        
        //Check SDRAM for errors
        printf("Check SDRAM \n\r");
        for (ui=c_sdram_base; ui < c_sdram_end; ) {
                if ( *(uint32_t*)(ui) != ptr32_ramdom_buffer[ui & c_random_len]) {
                        printf("Error! Addr = %X, Value = %X, random = %X\n\r", ui, *(uint32_t*)(ui), ptr32_ramdom_buffer[ui & c_random_len]);
                        while(1);
                }
                ui+=4;
                
                if ( ui % 0x3FF == 0) {
                    /* Each kb, surrender time */
                    wdt_restart(WDT);
                    vTaskDelay(1 / portTICK_RATE_MS );
                }
        }
        ui_loop++;
    }
}

/** The GMAC driver instance */
static gmac_device_t gs_gmac_dev;

/** Buffer for ethernet packets */
uint8_t p_uc_data_wr[GMAC_FRAME_LENTGH_MAX];
uint8_t p_uc_data_rd[GMAC_FRAME_LENTGH_MAX];

/**
 * \brief GMAC interrupt handler.
 */
void GMAC_Handler(void)
{
    gmac_handler(&gs_gmac_dev, GMAC_QUE_0);
}



static void prvAppEthTask(void *pvParameters)
{
    UNUSED(pvParameters);

    printf("==================================\n");    
    printf("START Ethernet  TEST \n");    
    printf("==================================\n"); 
    
    uint8_t uc_rc;
    uint32_t ul_i;
    uint32_t ul_value;
    
    Disable_global_interrupt();
    
    gmac_options_t gmac_option;
    uint8_t gs_uc_mac_address[] =
            { ETHERNET_CONF_ETHADDR0, ETHERNET_CONF_ETHADDR1, ETHERNET_CONF_ETHADDR2,
                    ETHERNET_CONF_ETHADDR3, ETHERNET_CONF_ETHADDR4, ETHERNET_CONF_ETHADDR5
            };
    uint8_t gs_uc_ip_address[] =
            { ETHERNET_CONF_IPADDR0, ETHERNET_CONF_IPADDR1,
                    ETHERNET_CONF_IPADDR2, ETHERNET_CONF_IPADDR3 };
    
    gmac_option.uc_copy_all_frame = 0;
    gmac_option.uc_no_boardcast = 0;

    /* Enable peripheral clock */
    pmc_enable_periph_clk(ID_GMAC);
    
    memcpy(gmac_option.uc_mac_addr, gs_uc_mac_address, sizeof(gs_uc_mac_address));
    
    
    gs_gmac_dev.p_hw = GMAC;

    /* Init GMAC driver structure */
    gmac_dev_init(GMAC, &gs_gmac_dev, &gmac_option);

    /* Enable Interrupt */
    NVIC_EnableIRQ(GMAC_IRQn);

    /* Init MAC PHY driver */
    if (ethernet_phy_init(GMAC, BOARD_GMAC_PHY_ADDR, sysclk_get_cpu_hz())
                                    != GMAC_OK) {
            printf("PHY Initialize ERROR!\r");
    }
    
    /* Set Loopback Mode */
    gmac_enable_management(GMAC, true);
    uc_rc = gmac_phy_read(GMAC, BOARD_GMAC_PHY_ADDR, GMII_BMCR, &ul_value);
    if (uc_rc != GMAC_OK) {
            gmac_enable_management(GMAC, false);
            printf("Error initializing ETH\n");
    }
    
    ul_value  = 0;
    ul_value |=  (uint32_t)GMII_LOOPBACK; /* enable Loopback */
    ul_value |=  (uint32_t)GMII_SPEED_SELECT; /* 100mbps */  
    ul_value &= ~(uint32_t)GMII_AUTONEG; /* Remove auto-negotiation enable */
    ul_value &= ~(uint32_t)GMII_POWER_DOWN; /* power up phy  */
    ul_value |=  (uint32_t)GMII_DUPLEX_MODE; /* Full duplex*/
    
    //ul_value |= (uint32_t)GMII_ISOLATE; /* Electrically isolate PHY */

    uc_rc = gmac_phy_write(GMAC, BOARD_GMAC_PHY_ADDR, GMII_BMCR, ul_value);
    if (uc_rc != GMAC_OK) {
            gmac_enable_management(GMAC, false);
            printf("Error initializing Loopback mode ETH\n");
    }
    
    gmac_set_speed(GMAC, true);
    gmac_enable_full_duplex(GMAC, true);       
    gmac_select_mii_mode(GMAC, ETH_PHY_MODE);
    gmac_enable_transmit(GMAC, true);
    gmac_enable_receive(GMAC, true);
    gmac_enable_management(GMAC, false);
    
    Enable_global_interrupt();
    
    /* create fake msg */
    memset(p_uc_data_wr, 0xff,1500);
    for ( ul_i = 14; ul_i < 1500; ul_i++);
      p_uc_data_wr[ul_i] = 0xaa;
    
    while(1) {
        vTaskDelay(5 / portTICK_RATE_MS );
        /* Write a dummy message */
        
        uint32_t ul_frm_size;
        uint32_t ul_rc;
        
        ul_rc = gmac_dev_write(&gs_gmac_dev, GMAC_QUE_0, p_uc_data_wr, 1500, NULL);
        if (ul_rc == GMAC_OK) {
		printf("Eth send frame\r\n");
	}
        
        
        vTaskDelay(1 / portTICK_RATE_MS );
        if (GMAC_OK == gmac_dev_read(&gs_gmac_dev, GMAC_QUE_0, (uint8_t *) p_uc_data_rd,
						sizeof(p_uc_data_rd), &ul_frm_size)) {
               printf("Eth revc frame\n", ul_rc);                                   
        }

        wdt_restart(WDT);
    }
}
#endif

#if (BOARD == PL360BN) 
extern void prvSetupSdram();
extern void prvEnableEth();
#endif   

void vPhyEmbeddedAppTask(txPhyEmbeddedConfig_t *xAppPhyCfgTx)
{
	/* Store tx configuration parameters */
	memcpy(&xTxPhyCfg, xAppPhyCfgTx, sizeof(txPhyEmbeddedConfig_t));

	/* Create App Phy Control task */
	xTaskCreate(prvAppPhyTask, (const signed char *const)"AppPhy", TASK_APP_PHY_STACK, NULL, TASK_APP_PHY_PRIO, &xappPhyHand);

	/* Create App Phy Transmission task */
	xTaskCreate(prvAppPhyTxTask, (const signed char *const)"AppPhy", TASK_APP_PHY_STACK, NULL, TASK_APP_PHY_PRIO, &xappPhyHand);

#if (BOARD == PL360BN)   
        if (xTxPhyCfg.uc_enable_ram) {
            /* Create SDRAM  test task...*/
            prvSetupSdram();
            xTaskCreate(prvAppSDRAMTask, (const signed char *const)"AppPhy", TASK_APP_PHY_STACK, NULL, TASK_APP_PHY_PRIO-1, &xappRamHand);
        }
        if (xTxPhyCfg.uc_enable_eth) {
            /* Create SDRAM  test task...*/
            prvEnableEth();
            xTaskCreate(prvAppEthTask, (const signed char *const)"AppPhy", TASK_APP_PHY_STACK+4000, NULL, TASK_APP_PHY_PRIO-2, &xappEthHand);
        }
#endif 
}
