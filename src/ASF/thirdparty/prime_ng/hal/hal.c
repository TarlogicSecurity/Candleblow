/**
 * \file
 *
 * \brief HAL: PRIME Hardware Abstraction Layer.
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
#include <stdio.h>
#include <asf.h>

#include "hal.h"
#include "hal_private.h"
#include "flash_efc.h"
#include "wdt.h"

#include "conf_app_example.h"
#ifdef EXAMPLE_LCD_SIGNALLING_ENABLE
#if BOARD == ATPL360ASB
	#include "vim878.h"
#elif (BOARD == ATPL360AMB) || (BOARD == ATPL360MB)
	#include "c0216CiZ.h"
#else
	#include "c42364a.h"
#endif
#endif
#ifdef PRIME_DEBUG_REPORT
#include "modem.h"
#endif

/* @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* @endcond */

/**
 * \weakgroup prime_hal_group
 * @{
 */
const hal_api_t hal_api = {
	hal_restart_system,
	hal_set_gp_timer_handler,
	hal_clear_gp_timer_handler,
	hal_timer_init,
	hal_timer_count_get,
	hal_timer_stop,
	hal_timer_plc_init,
	hal_timer_plc_stop,
	hal_set_plc_timer_handler,
	hal_pcrc_calc,
	hal_pcrc_config_sna,
	hal_fu_data_read,
	hal_fu_data_write,
	hal_fu_data_cfg_read,
	hal_fu_data_cfg_write,
	hal_fu_start,
	hal_fu_end,
	hal_fu_revert,
	hal_fu_crc_calculate,
	hal_fu_crc_set_callback,
	hal_fu_signature_image_check,
	hal_fu_signature_image_check_set_callback,
	hal_fu_get_bitmap,
	hal_plc_init,
	hal_plc_reset,
	hal_plc_cmd_op,
	hal_plc_set_handler,
	hal_plc_tx_signal,
	hal_plc_rx_signal,
	hal_get_config_info,
	hal_set_config_info,
	hal_usi_set_callback,
	hal_usi_send_cmd,
	hal_trng_init,
	hal_trng_read,
	hal_debug_report,
	hal_net_get_freq,
#if PRIME_HAL_VERSION == HAL_PRIME_1_4
	hal_aes_init,
	hal_aes_set_callback,
	hal_aes_key,
	hal_aes_crypt,
#endif
#ifdef HAL_ATPL360_INTERFACE
	hal_plc_send_boot_cmd,
	hal_plc_send_wrrd_cmd,
	hal_plc_enable_interrupt,
	hal_plc_delay,
	hal_plc_get_cd,
#endif
#ifdef HAL_NWK_RECOVERY_INTERFACE
	hal_nwk_recovery_init,
	hal_nwk_recovery_read,
	hal_nwk_recovery_write,
#endif

	hal_pib_get_request,
	hal_pib_get_request_set_callback,
	hal_pib_set_request,
	hal_pib_set_request_set_callback,
};

/**
 * \brief Initalize hardware abstraction layer interface
 *
 */
void hal_init(void)
{
	uint32_t *ptr_nvic_cpr0;
	bool b_is_int_disabled;

	/* Clear pending interrupts */
	ptr_nvic_cpr0 = (uint32_t *)NVIC_ICPR0;
	*ptr_nvic_cpr0 = 0xFFFFFFFF;

#ifndef HAL_ATPL360_INTERFACE
	/* reset hardware MODEM */
	hal_plc_reset();
#endif

	/* set critical region */
	b_is_int_disabled = __get_PRIMASK();
	if (!b_is_int_disabled) {
		Disable_global_interrupt();
	}

#ifndef HAL_ATPL360_INTERFACE
#ifdef SAME70
	hal_net_freq_init();
#endif
#endif

	/* init USI module */
	hal_usi_init();

	/* end critical region */
	if (!b_is_int_disabled) {
		Enable_global_interrupt();
	}

#ifndef DISABLE_RESET_HANDLING
	/* Init reset handler module */
	hal_reset_handler_init();
#endif
#ifndef DISABLE_RESET_HANDLING
	/* Init PIB module */
	hal_pib_init();
#endif
}

/**
 * \brief Process hardware layer interface
 *
 */
void hal_process(void)
{
#ifndef HAL_ATPL360_INTERFACE
#ifdef SAME70
	hal_net_freq_process();
#endif
#endif

	hal_usi_process();
}

/**
 * \brief Restart program
 *
 */
void hal_restart_system(void)
{

	NVIC_SystemReset();
}

/**
 * \brief Report a debug error
 *
 * \param ul_err_type     Type of error
 */
void hal_debug_report(uint32_t ul_err_type)
{
#ifdef EXAMPLE_LCD_SIGNALLING_ENABLE
#if (BOARD == ATPL360AMB) || (BOARD == ATPL360MB)
	char puc_app[16];
	c0216CiZ_set_cursor(C0216CiZ_LINE_DOWN, 0);
	/* Show error */
	sprintf(puc_app, "Error %10u", ul_err_type);
	c0216CiZ_show((const char *)puc_app);
#else
	ul_err_type = ul_err_type % 10000; /* Only four digits in the display */
	c42364a_show_numeric_dec((int32_t)ul_err_type);
#endif
#endif

#ifdef PRIME_DEBUG_REPORT
	modem_debug_report(ul_err_type);
#endif

	(void)ul_err_type;
}

/**
 *  \brief Configure microcontroller supply monitor SUPC and browndown detector.
 */
void hal_setup_supply_monitor (uint32_t ul_monitoring_rate, uint32_t ul_monitor_threshold)
{
	/* Enable sam4c brownout detector */
	supc_enable_brownout_detector(SUPC);
	/* enable sam4c browout detector reset */
	supc_enable_brownout_reset(SUPC);

#if (!SAMG)
	/* enable and configure configure voltage monitor  */
	supc_enable_voltage_regulator(SUPC);
#endif

	/* configure sampling */
	supc_set_monitor_sampling_period(SUPC, ul_monitoring_rate);
	/* Set Monitor Threshold */
	supc_set_monitor_threshold(SUPC, ul_monitor_threshold);
	/* enable reset monitor if voltage monitor is under threshold voltage */
	supc_enable_monitor_reset(SUPC);

	/* Wait 30ms to be sure that voltage is stable */
	delay_ms(30);
}

/**
 * \brief Setup the watchdog
 *
 * \param ul_watchdog_time   Watchdog time in microseconds
 */
void hal_watchdog_setup(uint32_t ul_watchdog_time)
{
	uint32_t timeout_value;
	uint32_t wdt_mode;

	/* get value to init wdog from time in us. */
	timeout_value = wdt_get_timeout_value(ul_watchdog_time, BOARD_FREQ_SLCK_XTAL);

#ifdef WATCHDOG_ENABLE_RESET
	/* Configure WDT to trigger a reset. */
	wdt_mode = WDT_MR_WDRSTEN | /* Enable WDT reset. */
#if (BOARD != ATPL230ABN_V2) && (BOARD != SAME70_XPLAINED) && (BOARD != PL360BN)
			WDT_MR_WDRPROC | /* WDT fault resets processor only. */
#endif
			WDT_MR_WDDBGHLT | /* WDT stops in debug state. */
			WDT_MR_WDIDLEHLT; /* WDT stops in idle state. */
	/* Initialize WDT with the given parameters. */
	wdt_init(WDT, wdt_mode, timeout_value, timeout_value);
#else
	/* Configure WDT to trigger an interruption. */
	wdt_mode = WDT_MR_WDFIEN | /* Enable WDT interrupt. */
#if (BOARD != ATPL230ABN_V2) && (BOARD != SAME70_XPLAINED) && (BOARD != PL360BN)
			WDT_MR_WDRPROC | /* WDT fault resets processor only. */
#endif
			WDT_MR_WDDBGHLT | /* WDT stops in debug state. */
			WDT_MR_WDIDLEHLT; /* WDT stops in idle state. */
	/* Initialize WDT with the given parameters. */
	wdt_init(WDT, wdt_mode, timeout_value, timeout_value);

	/* Configure and enable WDT interrupt. */
	NVIC_DisableIRQ(WDT_IRQn);
	NVIC_ClearPendingIRQ(WDT_IRQn);
	NVIC_SetPriority(WDT_IRQn, 0);
	NVIC_EnableIRQ(WDT_IRQn);
#endif
}

/* @} */

/* @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* @endcond */
