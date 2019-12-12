/**
 * \file
 *
 * \brief HAL_reset_handler
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
#include "hal_private.h"
#include "hal.h"

#ifndef DISABLE_RESET_HANDLING
void DumpStack(uint32_t stack[]);

/**
 * \brief Store reset information
 *
 * \param uc_reset_type   Reset type
 */
static void _reset_store_info(uint8_t uc_reset_type)
{
	uint32_t ul_reset_info;
	uint16_t us_num_resets;
		
	/* Read and increase number of resets since start-up */
#if PRIME_MODE == PRIME_SN
	us_num_resets = (uint16_t)(gpbr_read(GPBR5) >> 16);
#else
	us_num_resets = (uint16_t)(gpbr_read(GPBR0) >> 16);
#endif
	++us_num_resets;
	
	/* Store reset information */
	ul_reset_info = (us_num_resets << 16) + uc_reset_type;
#if PRIME_MODE == PRIME_SN
	gpbr_write(GPBR5, ul_reset_info);
#else
	gpbr_write(GPBR0, ul_reset_info);
#endif
}
		
/**
 * \brief Trigger a reset
 *
 * \param uc_reset_type   Reset type
 */
void hal_reset_trigger(uint8_t uc_reset_type)
{
	/* Store reset information */
	_reset_store_info(uc_reset_type);

	/* Trigger reset */
	rstc_start_software_reset(RSTC);
}

void HardFault_Handler(void)
{
	__asm volatile(
		"TST LR, #4	\n"
		"ITE EQ		\n"
		"MRSEQ R0, MSP	\n"
		"MRSNE R0, PSP	\n"
		"B DumpStack	\n");
}

void MemManage_Handler(void)
{
	__asm volatile(
		"TST LR, #4	\n"
		"ITE EQ		\n"
		"MRSEQ R0, MSP	\n"
		"MRSNE R0, PSP	\n"
		"B DumpStack	\n");
}

void BusFault_Handler(void)
{
	__asm volatile(
		"TST LR, #4	\n"
		"ITE EQ		\n"
		"MRSEQ R0, MSP	\n"
		"MRSNE R0, PSP	\n"
		"B DumpStack	\n");
}

void UsageFault_Handler(void)
{
	__asm volatile(
		"TST LR, #4	\n"
		"ITE EQ		\n"
		"MRSEQ R0, MSP	\n"
		"MRSNE R0, PSP	\n"
		"B DumpStack	\n");
}

void WDT_Handler(void)
{
	__asm volatile(
		"TST LR, #4	\n"
		"ITE EQ		\n"
		"MRSEQ R0, MSP	\n"
		"MRSNE R0, PSP	\n"
		"B DumpStack	\n");
}

volatile uint32_t saved_r0;
volatile uint32_t saved_r1;
volatile uint32_t saved_r2;
volatile uint32_t saved_r3;
volatile uint32_t saved_r12;
volatile uint32_t saved_lr;
volatile uint32_t saved_pc;
volatile uint32_t saved_psr;
volatile uint32_t saved_hfsr;
volatile uint32_t saved_cfsr;
void DumpStack(uint32_t stack[])
{
	saved_r0 = stack[0];
    	saved_r1 = stack[1];
    	saved_r2 = stack[2];
    	saved_r3 = stack[3];
    	saved_r12 = stack[4];
    	saved_lr = stack[5];
    	saved_pc = stack[6];
    	saved_psr = stack[7];
	saved_hfsr = SCB->HFSR;
	saved_cfsr = SCB->CFSR;

	/* Store registers */
#if SAM4C || SAM4CP || SAM4CM 
#if PRIME_MODE == PRIME_SN
	gpbr_write(GPBR6, saved_pc);
	gpbr_write(GPBR7, saved_lr);
	gpbr_write(GPBR8, saved_psr);
	gpbr_write(GPBR9, saved_hfsr);
	gpbr_write(GPBR10, saved_cfsr);
	gpbr_write(GPBR11, saved_r0);
	gpbr_write(GPBR12, saved_r1);
	gpbr_write(GPBR13, saved_r2);
	gpbr_write(GPBR14, saved_r3);
#else
	gpbr_write(GPBR1, saved_pc);
	gpbr_write(GPBR2, saved_lr);
	gpbr_write(GPBR3, saved_psr);
	gpbr_write(GPBR4, saved_hfsr);
	gpbr_write(GPBR5, saved_cfsr);
	gpbr_write(GPBR6, saved_r0);
	gpbr_write(GPBR7, saved_r1);
	gpbr_write(GPBR8, saved_r2);
	gpbr_write(GPBR9, saved_r3);
	gpbr_write(GPBR10, saved_r12);
#endif
#endif
#if SAMG55
#if PRIME_MODE == PRIME_SN
	gpbr_write(GPBR6, saved_pc);
	gpbr_write(GPBR7, saved_lr);
#else
	gpbr_write(GPBR1, saved_pc);
	gpbr_write(GPBR2, saved_lr);
	gpbr_write(GPBR3, saved_psr);
	gpbr_write(GPBR4, saved_hfsr);
	gpbr_write(GPBR5, saved_cfsr);
	gpbr_write(GPBR6, saved_r0);
	gpbr_write(GPBR7, saved_r1);
#endif
#endif
#if SAME70
	gpbr_write(GPBR1, saved_pc);
	gpbr_write(GPBR2, saved_lr);
	gpbr_write(GPBR3, saved_psr);
	gpbr_write(GPBR4, saved_hfsr);
	gpbr_write(GPBR5, saved_cfsr);
	gpbr_write(GPBR6, saved_r0);
	gpbr_write(GPBR7, saved_r1);
#endif
#if SAM4E
	gpbr_write(GPBR1, saved_pc);
	gpbr_write(GPBR2, saved_lr);
	gpbr_write(GPBR3, saved_psr);
	gpbr_write(GPBR4, saved_hfsr);
	gpbr_write(GPBR5, saved_cfsr);
	gpbr_write(GPBR6, saved_r0);
	gpbr_write(GPBR7, saved_r1);
	gpbr_write(GPBR8, saved_r2);
	gpbr_write(GPBR9, saved_r3);
	gpbr_write(GPBR10, saved_r12);
#endif

	uint8_t uc_reset_type;

	/* Check forced hard fault */
	if ((saved_hfsr & (1 << 30)) != 0) {
	  	uc_reset_type = HARD_FAULT_RESET;

		/* Check usage error */
		if((saved_cfsr & 0xFFFF0000) != 0) {
			uc_reset_type = USAGE_FAULT_RESET;
		}
		
		/* Check bus fault error */
		if((saved_cfsr & 0xFF00) != 0) {
			uc_reset_type = BUS_FAULT_RESET;
		}
			
		/* Check memory management error */
		if((saved_cfsr & 0xFF) != 0) {
			uc_reset_type = MEM_MANAGE_RESET;
		}      
	/* Check bus fault on a vector table */
	} else if ((saved_hfsr & (1 << 1)) != 0) {
		uc_reset_type = VECTOR_FAULT_RESET;
	} else {
		uc_reset_type = WATCHDOG_RESET;
	}

	/* Trigger reset */
	hal_reset_trigger(uc_reset_type);
}

/**
 * \brief Initialize module
 *
 */
void hal_reset_handler_init(void)
{
	uint8_t uc_reset_type;
	
	/* Read reset type */
	uc_reset_type = (uint8_t)(rstc_get_reset_cause(RSTC) >> RSTC_SR_RSTTYP_Pos);

	/* If it is a software reset, it is from one of these handlers */
	if ((uc_reset_type != SOFTWARE_RESET) && (uc_reset_type != GENERAL_RESET)) {
		/* Store reset information for other reset types */
		_reset_store_info(uc_reset_type);
	}
}
#endif /* #ifndef DISABLE_RESET_HANDLING */
