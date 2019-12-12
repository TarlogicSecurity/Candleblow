/**
 * \file
 *
 * \brief HAL_TIMER
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

#include "asf.h"
#include "genclk.h"
#include "hal_private.h"


#define HAL_TC0_PRIO                      0

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

/** General purpose timer callbacks array */
static void(*gp_timer_callbacks[HAL_GP_TIMER_NUM]) (void);

/** General purpose timer block correspondence */
static Tc *puc_tc[HAL_GP_TIMER_NUM] = {TC0, TC0, TC0, TC1, TC1, TC1};

/** General purpose timer channel*/
static uint8_t puc_tc_chn[HAL_GP_TIMER_NUM] = {0, 1, 2, 0, 1, 2};

/** General purpose timer ID */
static uint8_t puc_tc_id[HAL_GP_TIMER_NUM] = {ID_TC0, ID_TC1, ID_TC2, ID_TC3, ID_TC4, ID_TC5};

/** General purpose timer IRQs */
static uint8_t puc_tc_irq_n[HAL_GP_TIMER_NUM] = {TC0_IRQn, TC1_IRQn, TC2_IRQn, TC3_IRQn, TC4_IRQn, TC5_IRQn};

/**
 * \brief This function initializes the general purpose timers
 *
 * \param  gpt        ID of the general purpose timer to configure.
 * \param  mode       Simple (16 bits) or cascade (32 bits) timers.
 *                    NOTE: in SAM4CP16B the cascade mode is only allowed
 *                          with HAL_GP_TIMER_1 (involving implicitly HAL_GP_TIMER_2).
 * \param  clk_src    CLK source for the timer.
 * \return  uc_status  0 tc timer programed for 16 bit counters
 * \return  uc_status  1 tc init failed
 * \return  uc_status  2 tc timer programmed for 32 bit counters
 *
 */
uint8_t hal_timer_init(hal_gp_timer_t gpt, hal_timer_mode_t mode, hal_timer_clk_src_t clk_src)
{
#if (SAMG55)
	struct genclk_config gcfg;
#endif
	uint8_t uc_status = 0;

	/* Configure the PMC to enable the respective TC module */
	sysclk_enable_peripheral_clock(puc_tc_id[gpt]);

#if (SAMG55)
	if (clk_src == HAL_TIMER_CLK_SRC_CLK5) {
		/* Configure PCK3 with SLCK as source*/
		genclk_config_defaults(&gcfg, GENCLK_PCK_3);
		genclk_config_set_source(&gcfg, GENCLK_PCK_SRC_SLCK_XTAL);
		genclk_config_set_divider(&gcfg, GENCLK_PCK_PRES_1);
		genclk_enable(&gcfg, GENCLK_PCK_3);
	}
#endif

	switch (mode) {
	case HAL_TIMER_MODE_32_BITS:
		/* In SAM4CP16B the 32 bits mode is only allowed with */
		/* HAL_GP_TIMER_1 (involving implicitly HAL_GP_TIMER_2) */
		if (gpt == HAL_GP_TIMER_1) {
			/* Configure the PMC to also enable HAL_GP_TIMER_2 module */
			sysclk_enable_peripheral_clock(puc_tc_id[HAL_GP_TIMER_2]);
			/* Init TIMER 2 to waveform mode with CLK source in TIMER 1 overflow line */
			tc_init(puc_tc[HAL_GP_TIMER_2], puc_tc_chn[HAL_GP_TIMER_2],
			(TC_CMR_TCCLKS_XC2 | TC_CMR_WAVSEL_UP | TC_CMR_ACPA_NONE));
			tc_set_block_mode(puc_tc[HAL_GP_TIMER_2], TC_BMR_TC2XC2S_TIOA1);
			/* Enable interrupt on counter overflow */
			tc_enable_interrupt(puc_tc[HAL_GP_TIMER_2], puc_tc_chn[HAL_GP_TIMER_2], TC_IER_COVFS);
			/* Start the timer */
			tc_start(puc_tc[HAL_GP_TIMER_2], puc_tc_chn[HAL_GP_TIMER_2]);
			uc_status = 3;   /* indicates HAL_TIMER_MODE_32_BITS */
		} else {
			/* Error. 32 bit mode can only be used with HAL_GP_TIMER_1. */
			uc_status = 1;
			break;
		}
	/* NOTE: "break" statement intentionally missing here. */
	case HAL_TIMER_MODE_16_BITS:
	  	/* Init TC in waveform mode */
		tc_init(puc_tc[gpt], puc_tc_chn[gpt], ((uint8_t)clk_src | TC_CMR_WAVE | TC_CMR_WAVSEL_UP | TC_CMR_ACPA_NONE | TC_CMR_ACPC_TOGGLE));
		/* Configure compare count to full range */
		tc_write_ra(puc_tc[gpt], puc_tc_chn[gpt], 0x0000);
		tc_write_rc(puc_tc[gpt], puc_tc_chn[gpt], 0xFFFF);
		/* Enable interrupt on counter overflow */
		tc_enable_interrupt(puc_tc[gpt], puc_tc_chn[gpt], TC_IER_COVFS);
		/* Start the timer */
		tc_start(puc_tc[gpt], puc_tc_chn[gpt]);
#if (SAM4E)
		uc_status = 2;
#endif
		break;

	default:
		/* Error. Unknown mode. */
		uc_status = 1;
	}

	return(uc_status);
}

/**
 * \brief This function reads the current count value of the general purpose timers
 *
 * \param  gpt   HAL ID of the general purpose timer to read.
 */
uint32_t hal_timer_count_get(hal_gp_timer_t gpt)
{
	return(tc_read_cv(puc_tc[gpt], puc_tc_chn[gpt]));
}

/**
 * \brief This function stops and turns off the timer
 *
 * \param  gpt        ID of the general purpose timer to switch off.
 * \param  mode       Timer mode
 */
void hal_timer_stop(hal_gp_timer_t gpt, hal_timer_mode_t mode)
{
	switch (mode) {
	case HAL_TIMER_MODE_32_BITS:
		if (gpt == HAL_GP_TIMER_1) {
			tc_stop(puc_tc[HAL_GP_TIMER_2], puc_tc_chn[HAL_GP_TIMER_2]);
			sysclk_disable_peripheral_clock(puc_tc_id[HAL_GP_TIMER_2]);
		}

	/* NOTE: "break" statement intentionally missing here. */
	case HAL_TIMER_MODE_16_BITS:
		tc_stop(puc_tc[gpt], puc_tc_chn[gpt]);
		sysclk_disable_peripheral_clock(puc_tc_id[gpt]);
		break;

	default:
		break;
	}
}

/**
 * \brief This function initializes the PLC timer
 *
 * \param  uc_time_us        Time in microseconds
 */
void hal_timer_plc_init(uint32_t uc_time_us)
{
	uint32_t ul_div, ul_tcclks;
#ifdef SAME70
	uint32_t ul_mck = sysclk_get_peripheral_hz();
#else
	uint32_t ul_mck = sysclk_get_cpu_hz();
#endif
	/* Configure the PMC to enable the respective TC module */
	sysclk_enable_peripheral_clock(puc_tc_id[HAL_GP_PLC_TIMER]);

	/* Configure and enable interrupt on RC compare */
	NVIC_SetPriority((IRQn_Type)TC0_IRQn, HAL_TC0_PRIO);
	NVIC_EnableIRQ((IRQn_Type)TC0_IRQn);

	/* Clear status bit to acknowledge last interrupt */
	tc_get_status(puc_tc[HAL_GP_PLC_TIMER], puc_tc_chn[HAL_GP_PLC_TIMER]);

	/* Disable Int */
	tc_disable_interrupt(puc_tc[HAL_GP_PLC_TIMER], puc_tc_chn[HAL_GP_PLC_TIMER], TC_IER_CPCS);

	/* MCK = 120000000 -> tcclks = 2 : TCLK3 = MCK/32 */
	ul_tcclks = 2;
	ul_div = ((ul_mck / 32000) * uc_time_us) / 1000;

	tc_init(puc_tc[HAL_GP_PLC_TIMER], puc_tc_chn[HAL_GP_PLC_TIMER], ul_tcclks | TC_CMR_CPCTRG);

	tc_write_rc(puc_tc[HAL_GP_PLC_TIMER], puc_tc_chn[HAL_GP_PLC_TIMER], ul_div);

	/** Start the timer. */
	tc_start(puc_tc[HAL_GP_PLC_TIMER], puc_tc_chn[HAL_GP_PLC_TIMER]);

	/* Enable Int */
	tc_enable_interrupt(puc_tc[HAL_GP_PLC_TIMER], puc_tc_chn[HAL_GP_PLC_TIMER], TC_IER_CPCS);
}

/**
 * \brief This function stops the PLC timer
 *
 */
void hal_timer_plc_stop(void)
{
	/* Disable Int */
	tc_disable_interrupt(puc_tc[HAL_GP_PLC_TIMER], puc_tc_chn[HAL_GP_PLC_TIMER], TC_IER_CPCS);

	/** Stop the PLC timer */
	tc_stop(puc_tc[HAL_GP_PLC_TIMER], puc_tc_chn[HAL_GP_PLC_TIMER]);
}

/**
 * \brief This function specifies the callback function for the PLC timer.
 *
 * \param  p_handler  Pointer to the callback function
 */
void hal_set_plc_timer_handler(void (*p_handler)(void))
{
	gp_timer_callbacks[0] = p_handler;
}

/**
 * \brief This function specifies the callback function for the general
 *        purpose hardware timers.
 *
 * \param  gpt        ID of the general purpose timer to configure
 * \param  p_handler  Pointer to the callback function
 */
void hal_set_gp_timer_handler(hal_gp_timer_t gpt, void (*p_handler)(void))
{
	(void) p_handler;

	if (gpt > 0) {
		gp_timer_callbacks[gpt] = NULL;
	}
}

/**
 * \brief This function clears the callback for the given timer. so that its
 *        overflow interrupt is no longer triggered.
 *
 * \param  gpt        ID of the general purpose timer to clear the callback
 */
void hal_clear_gp_timer_handler(hal_gp_timer_t gpt)
{
	if (gpt > 0) {
		gp_timer_callbacks[gpt] = NULL;
		/* Disable NVIC interrupt */
		NVIC_DisableIRQ((IRQn_Type)puc_tc_irq_n[gpt]);
	}
}

/* NOTE: GP Timer 0 is reserved for the PLC PHY driver
 * and therefore, its callback should not be changed here. */
#ifdef HAL_GP_TIMER_PLC_Handler
void HAL_GP_TIMER_PLC_Handler(void)
{
	if (gp_timer_callbacks[0] != NULL) {
		gp_timer_callbacks[0]();
	}
}
#endif

#ifdef HAL_GP_TIMER_1_Handler
void HAL_GP_TIMER_1_Handler(void)
{
	/* Clear status bit to acknowledge last interrupt */
	tc_get_status(puc_tc[HAL_GP_TIMER_1], puc_tc_chn[HAL_GP_TIMER_1]);

	/* Disable counter overflow interrupt */
	tc_disable_interrupt(puc_tc[HAL_GP_TIMER_1], puc_tc_chn[HAL_GP_TIMER_1], TC_IER_COVFS);

	if (gp_timer_callbacks[HAL_GP_TIMER_1] != NULL) {
		gp_timer_callbacks[HAL_GP_TIMER_1]();
	}

	/* Enable counter overflow interrupt */
	tc_enable_interrupt(puc_tc[HAL_GP_TIMER_1], puc_tc_chn[HAL_GP_TIMER_1], TC_IER_COVFS);

	/* Restart timer */
	tc_start(puc_tc[HAL_GP_TIMER_1], puc_tc_chn[HAL_GP_TIMER_1]);
}
#endif

#ifdef HAL_GP_TIMER_2_Handler

void HAL_GP_TIMER_2_Handler(void)
{
	/* Clear status bit to acknowledge last interrupt */
	tc_get_status(puc_tc[HAL_GP_TIMER_2], puc_tc_chn[HAL_GP_TIMER_2]);

	/* Disable counter overflow interrupt */
	tc_disable_interrupt(puc_tc[HAL_GP_TIMER_2], puc_tc_chn[HAL_GP_TIMER_2], TC_IER_COVFS);

	if (gp_timer_callbacks[HAL_GP_TIMER_2] != NULL) {
		gp_timer_callbacks[HAL_GP_TIMER_2]();
	}

	/* Enable counter overflow interrupt */
	tc_enable_interrupt(puc_tc[HAL_GP_TIMER_2], puc_tc_chn[HAL_GP_TIMER_2], TC_IER_COVFS);

	/* Restart timer */
	tc_start(puc_tc[HAL_GP_TIMER_2], puc_tc_chn[HAL_GP_TIMER_2]);
}
#endif

#ifdef HAL_GP_TIMER_3_Handler
void HAL_GP_TIMER_3_Handler(void)
{
	/* Clear status bit to acknowledge last interrupt */
	tc_get_status(puc_tc[HAL_GP_TIMER_3], puc_tc_chn[HAL_GP_TIMER_3]);

	/* Disable counter overflow interrupt */
	tc_disable_interrupt(puc_tc[HAL_GP_TIMER_3], puc_tc_chn[HAL_GP_TIMER_3], TC_IER_COVFS);

	if (gp_timer_callbacks[HAL_GP_TIMER_3] != NULL) {
		gp_timer_callbacks[HAL_GP_TIMER_3]();
	}

	/* Enable counter overflow interrupt */
	tc_enable_interrupt(puc_tc[HAL_GP_TIMER_3], puc_tc_chn[HAL_GP_TIMER_3], TC_IER_COVFS);

	/* Restart timer */
	tc_start(puc_tc[HAL_GP_TIMER_3], puc_tc_chn[HAL_GP_TIMER_3]);
}
#endif

/* NOTE: GP Timer 4 and 5 are reserved for the UART and USART drivers
 * and therefore, their callback should not be changed here. */
/* void HAL_GP_TIMER_4_Handler (void); */
/* void HAL_GP_TIMER_5_Handler (void); */

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */
