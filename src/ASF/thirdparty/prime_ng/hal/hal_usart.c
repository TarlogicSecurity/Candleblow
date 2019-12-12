/**
 * \file
 *
 * \brief HAL_USART
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
#include <string.h>
#include "asf.h"
#include "conf_hal.h"
#include "conf_board.h"
#include "hal_private.h"

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

#ifdef CONF_BOARD_USART0
/** Reception Buffer 0 */
static uint8_t rx_usart_buf0[HAL_RX_USART_BUF0_SIZE];
/** Transmission Buffer 0 */
static uint8_t tx_usart_buf0[HAL_TX_USART_BUF0_SIZE];
/** Pointers to Reception Buffer 0 */
uint8_t *const ptr_rx_usart_buf0 = &rx_usart_buf0[0];
/** Pointers to Transmission Buffer 0 */
uint8_t *const ptr_tx_usart_buf0 = &tx_usart_buf0[0];
#endif

#ifdef CONF_BOARD_USART1
/** Reception Buffer 1 */
static uint8_t rx_usart_buf1[HAL_RX_USART_BUF1_SIZE];
/** Transmission Buffer 1 */
static uint8_t tx_usart_buf1[HAL_TX_USART_BUF1_SIZE];
/** Pointers to Reception Buffer 1 */
uint8_t *const ptr_rx_usart_buf1 = &rx_usart_buf1[0];
/** Pointers to Transmission Buffer 1 */
uint8_t *const ptr_tx_usart_buf1 = &tx_usart_buf1[0];
#endif

#ifdef CONF_BOARD_USART2
/** Reception Buffer 2 */
static uint8_t rx_usart_buf2[HAL_RX_USART_BUF2_SIZE];
/** Transmission Buffer 2 */
static uint8_t tx_usart_buf2[HAL_TX_USART_BUF2_SIZE];
/** Pointers to Reception Buffer 2 */
uint8_t *const ptr_rx_usart_buf2 = &rx_usart_buf2[0];
/** Pointers to Transmission Buffer 2 */
uint8_t *const ptr_tx_usart_buf2 = &tx_usart_buf2[0];
#endif

#ifdef CONF_BOARD_USART3
/** Reception Buffer 3 */
static uint8_t rx_usart_buf3[HAL_RX_USART_BUF3_SIZE];
/** Transmission Buffer 3 */
static uint8_t tx_usart_buf3[HAL_TX_USART_BUF3_SIZE];
/** Pointers to Reception Buffer 2 */
uint8_t *const ptr_rx_usart_buf3 = &rx_usart_buf3[0];
/** Pointers to Transmission Buffer 2 */
uint8_t *const ptr_tx_usart_buf3 = &tx_usart_buf3[0];
#endif

#ifdef CONF_BOARD_USART4
/** Reception Buffer 4 */
static uint8_t rx_usart_buf4[HAL_RX_USART_BUF4_SIZE];
/** Transmission Buffer 4 */
static uint8_t tx_usart_buf4[HAL_TX_USART_BUF4_SIZE];
/** Pointers to Reception Buffer 4 */
uint8_t *const ptr_rx_usart_buf4 = &rx_usart_buf4[0];
/** Pointers to Transmission Buffer 4 */
uint8_t *const ptr_tx_usart_buf4 = &tx_usart_buf4[0];
#endif

/** Communications Queue Info */
typedef struct {
	/** Pointer to transmission queue. Buffer */
	uint8_t *puc_tq_buf;
	/** Pointer to reception queue. Buffer */
	uint8_t *puc_rq_buf;
	/** Reception queue. Read index */
	uint16_t us_rq_idx;
	/** Reception queue. Write index */
	uint16_t us_wq_idx;
	/** Reception queue. Occupation count */
	uint16_t us_rq_count;
} busart_comm_data_t;

/** Size of the receive buffer used by the PDC, in bytes */
#define USART_BUFFER_SIZE                       1024

#ifdef CONF_BOARD_USART0
/** Data struct to use with USART0 */
static busart_comm_data_t busart_comm_data_0;
/** Receive buffer use 2 fast buffers */
static uint8_t gs_puc_usart_buf0[USART_BUFFER_SIZE];
/** Current bytes in buffer. */
static uint32_t gs_ul_size_usart_buf0 = USART_BUFFER_SIZE;
/** PDC RX data packet. */
pdc_packet_t g_st_usart_rx_packet0;
/** PDC TX data packet. */
pdc_packet_t g_st_usart_tx_packet0;
/** Pointer to PDC register base. */
Pdc *g_p_usart_pdc0;
/** Number of bytes received in USART0 */
static uint16_t num_bytes_rx_usart0;
#endif

#ifdef CONF_BOARD_USART1
/** Data struct to use with USART0 */
static busart_comm_data_t busart_comm_data_1;
/** Receive buffer use 2 fast buffers */
static uint8_t gs_puc_usart_buf1[USART_BUFFER_SIZE];
/** Current bytes in buffer. */
static uint32_t gs_ul_size_usart_buf1 = USART_BUFFER_SIZE;
/** PDC RX data packet. */
pdc_packet_t g_st_usart_rx_packet1;
/** PDC TX data packet. */
pdc_packet_t g_st_usart_tx_packet1;
/** Pointer to PDC register base. */
Pdc *g_p_usart_pdc1;
/** Number of bytes received in USART1 */
static uint16_t num_bytes_rx_usart1;
#endif

#ifdef CONF_BOARD_USART2
/** Data struct to use with USART2 */
static busart_comm_data_t busart_comm_data_2;
/** Receive buffer use 2 fast buffers */
static uint8_t gs_puc_usart_buf2[USART_BUFFER_SIZE];
/** Current bytes in buffer. */
static uint32_t gs_ul_size_usart_buf2 = USART_BUFFER_SIZE;
/** PDC RX data packet. */
pdc_packet_t g_st_usart_rx_packet2;
/** PDC TX data packet. */
pdc_packet_t g_st_usart_tx_packet2;
/** Pointer to PDC register base. */
Pdc *g_p_usart_pdc2;
/** Number of bytes received in USART2 */
static uint16_t num_bytes_rx_usart2;
#endif

#ifdef CONF_BOARD_USART3
/** Data struct to use with USART3 */
static busart_comm_data_t busart_comm_data_3;
/** Receive buffer use 2 fast buffers */
static uint8_t gs_puc_usart_buf3[USART_BUFFER_SIZE];
/** Current bytes in buffer. */
static uint32_t gs_ul_size_usart_buf3 = USART_BUFFER_SIZE;
/** PDC RX data packet. */
pdc_packet_t g_st_usart_rx_packet3;
/** PDC TX data packet. */
pdc_packet_t g_st_usart_tx_packet3;
/** Pointer to PDC register base. */
Pdc *g_p_usart_pdc3;
/** Number of bytes received in USART3 */
static uint16_t num_bytes_rx_usart3;
#endif

#ifdef CONF_BOARD_USART4
/** Data struct to use with USART3 */
static busart_comm_data_t busart_comm_data_4;
/** Receive buffer use 2 fast buffers */
static uint8_t gs_puc_usart_buf4[USART_BUFFER_SIZE];
/** Current bytes in buffer. */
static uint32_t gs_ul_size_usart_buf4 = USART_BUFFER_SIZE;
/** PDC RX data packet. */
pdc_packet_t g_st_usart_rx_packet4;
/** PDC TX data packet. */
pdc_packet_t g_st_usart_tx_packet4;
/** Pointer to PDC register base. */
Pdc *g_p_usart_pdc4;
/** Number of bytes received in USART4 */
static uint16_t num_bytes_rx_usart4;
#endif

/** Uart channel open / closed */
static uint8_t hal_usart_chn_open[5] = {
	false,
	false,
	false,
	false,
    false
};

#if defined(CONF_BOARD_USART0) || defined(CONF_BOARD_USART1) || defined(CONF_BOARD_USART2) || defined(CONF_BOARD_USART3) || defined(CONF_BOARD_USART4)

/**
 * \brief Configure Timer Counter to generate an interrupt every 10ms.
 * This interrupt will be used to flush USART input and echo back.
 */
static void _configure_TC_usart(void)
{
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk;
	uint32_t ul_frec_hz = (uint32_t)FREQ_TIMER_POLL_USART;

	/* Get system clock. */
	ul_sysclk = sysclk_get_cpu_hz();

	/* Configure PMC. */
	pmc_enable_periph_clk(HAL_ID_TC_USART);

	/* Configure TC for a TC_FREQ frequency and trigger on RC compare. */
	tc_find_mck_divisor(ul_frec_hz, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(HAL_TC_USART, HAL_TC_USART_CHN, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(HAL_TC_USART, HAL_TC_USART_CHN, (ul_sysclk / ul_div) / ul_frec_hz);

	/* Configure and enable interrupt on RC compare. */
	NVIC_SetPriority((IRQn_Type)HAL_ID_TC_USART, TIMER_USART_PRIO);
	NVIC_EnableIRQ((IRQn_Type)HAL_ID_TC_USART);
	tc_enable_interrupt(HAL_TC_USART, HAL_TC_USART_CHN, TC_IER_CPCS);
}

/**
 * \brief Interrupt handler. Record the number of bytes received,
 * and then restart a read transfer on the USART if the transfer was stopped.
 */
void HAL_TC_USART_Handler(void)
{
	uint32_t ul_status;
#if defined(CONF_BOARD_USART0) || defined(CONF_BOARD_USART1) || defined(CONF_BOARD_USART2) || defined(CONF_BOARD_USART3) || defined(CONF_BOARD_USART4)
	uint32_t ul_byte_total = 0;
#endif

	/* Read HAL_TC_USART Status. */
	ul_status = tc_get_status(HAL_TC_USART, HAL_TC_USART_CHN);

	/* RC compare. */
	if ((ul_status & TC_SR_CPCS) == TC_SR_CPCS) {
#ifdef CONF_BOARD_USART0
		if (hal_usart_chn_open[0]) {
			/* Flush PDC buffer. */
			ul_byte_total = USART_BUFFER_SIZE - pdc_read_rx_counter(g_p_usart_pdc0);
			if (ul_byte_total > 0) {
				if (ul_byte_total == num_bytes_rx_usart0) {
					/* Disable timer. */
					tc_stop(HAL_TC_USART, HAL_TC_USART_CHN);

					/* Log current size */
					gs_ul_size_usart_buf0 = ul_byte_total;

					/* Stop DMA USART_RX -> force Uart Handler*/
					g_st_usart_rx_packet0.ul_size = 0;
					pdc_rx_init(g_p_usart_pdc0, &g_st_usart_rx_packet0, NULL);
				} else {
					num_bytes_rx_usart0 = ul_byte_total;
				}
			} else {
				num_bytes_rx_usart0 = 0;
			}
		}
#endif

#ifdef CONF_BOARD_USART1
		if (hal_usart_chn_open[1]) {
			/* Flush PDC buffer. */
			ul_byte_total = USART_BUFFER_SIZE - pdc_read_rx_counter(g_p_usart_pdc1);
			if (ul_byte_total > 0) {
				if (ul_byte_total == num_bytes_rx_usart1) {
					/* Disable timer. */
					tc_stop(HAL_TC_USART, HAL_TC_USART_CHN);

					/* Log current size */
					gs_ul_size_usart_buf1 = ul_byte_total;

					/* Stop DMA USART_RX -> force Uart Handler*/
					g_st_usart_rx_packet1.ul_size = 0;
					pdc_rx_init(g_p_usart_pdc1, &g_st_usart_rx_packet1, NULL);
				} else {
					num_bytes_rx_usart1 = ul_byte_total;
				}
			} else {
				num_bytes_rx_usart1 = 0;
			}
		}
#endif

#ifdef CONF_BOARD_USART2
		if (hal_usart_chn_open[2]) {
			/* Flush PDC buffer. */
			ul_byte_total = USART_BUFFER_SIZE - pdc_read_rx_counter(g_p_usart_pdc2);
			if (ul_byte_total > 0) {
				if (ul_byte_total == num_bytes_rx_usart2) {
					/* Disable timer. */
					tc_stop(HAL_TC_USART, HAL_TC_USART_CHN);

					/* Log current size */
					gs_ul_size_usart_buf2 = ul_byte_total;

					/* Stop DMA USART_RX -> force Uart Handler*/
					g_st_usart_rx_packet2.ul_size = 0;
					pdc_rx_init(g_p_usart_pdc2, &g_st_usart_rx_packet2, NULL);
				} else {
					num_bytes_rx_usart2 = ul_byte_total;
				}
			} else {
				num_bytes_rx_usart2 = 0;
			}
		}
#endif

#ifdef CONF_BOARD_USART3
		if (hal_usart_chn_open[3]) {
			/* Flush PDC buffer. */
			ul_byte_total = USART_BUFFER_SIZE - pdc_read_rx_counter(g_p_usart_pdc3);
			if (ul_byte_total > 0) {
				if (ul_byte_total == num_bytes_rx_usart3) {
					/* Disable timer. */
					tc_stop(HAL_TC_USART, HAL_TC_USART_CHN);

					/* Log current size */
					gs_ul_size_usart_buf3 = ul_byte_total;

					/* Stop DMA USART_RX -> force Uart Handler*/
					g_st_usart_rx_packet3.ul_size = 0;
					pdc_rx_init(g_p_usart_pdc3, &g_st_usart_rx_packet3, NULL);
				} else {
					num_bytes_rx_usart3 = ul_byte_total;
				}
			} else {
				num_bytes_rx_usart3 = 0;
			}
		}
#endif

#ifdef CONF_BOARD_USART4
		if (hal_usart_chn_open[4]) {
			/* Flush PDC buffer. */
			ul_byte_total = USART_BUFFER_SIZE - pdc_read_rx_counter(g_p_usart_pdc4);
			if (ul_byte_total > 0) {
				if (ul_byte_total == num_bytes_rx_usart4) {
					/* Disable timer. */
					tc_stop(HAL_TC_USART, HAL_TC_USART_CHN);

					/* Log current size */
					gs_ul_size_usart_buf4 = ul_byte_total;

					/* Stop DMA USART_RX -> force Uart Handler*/
					g_st_usart_rx_packet4.ul_size = 0;
					pdc_rx_init(g_p_usart_pdc4, &g_st_usart_rx_packet4, NULL);
				} else {
					num_bytes_rx_usart4 = ul_byte_total;
				}
			} else {
				num_bytes_rx_usart4 = 0;
			}
		}
#endif
	}
}

#ifdef CONF_BOARD_USART0

/** @brief	Interruption handler for USART0
 *
 */
void HAL_USART0_Handler(void)
{
	uint32_t ul_status;
	uint16_t us_wr_idx, us_data_count;
	uint16_t us_end_size, us_free_size, us_part_size;

	/* Read USART Status. */
	ul_status = usart_get_status(USART0);

	/* Receive buffer is full. */
	if (ul_status & US_CSR_ENDRX) {
		/* manage data */
		us_wr_idx = busart_comm_data_0.us_wq_idx;
		us_data_count = busart_comm_data_0.us_rq_count;
		us_free_size = HAL_RX_USART_BUF0_SIZE - us_data_count;
		if (gs_ul_size_usart_buf0 <= us_free_size) {
			/* there is enough space to write all data */
			us_end_size = HAL_RX_USART_BUF0_SIZE - us_wr_idx;
			if (us_end_size >= gs_ul_size_usart_buf0) {
				/* there is no overflow of us_wq_idx */
				memcpy(&busart_comm_data_0.puc_rq_buf[us_wr_idx], gs_puc_usart_buf0, gs_ul_size_usart_buf0);
				/* update counters */
				busart_comm_data_0.us_rq_count += gs_ul_size_usart_buf0;
				busart_comm_data_0.us_wq_idx += gs_ul_size_usart_buf0;
			} else {
				/* there is overflow of us_wq_idx -> write in 2 steps	*/
				memcpy(&busart_comm_data_0.puc_rq_buf[us_wr_idx], gs_puc_usart_buf0, us_end_size);
				us_part_size = gs_ul_size_usart_buf0 - us_end_size;
				memcpy(&busart_comm_data_0.puc_rq_buf[0], &gs_puc_usart_buf0[us_end_size], us_part_size);
				/* update counters */
				busart_comm_data_0.us_rq_count += gs_ul_size_usart_buf0;
				busart_comm_data_0.us_wq_idx = us_part_size;
			}
		} else {
			/* there is not enough space to write all data */
			tc_start(HAL_TC_USART, HAL_TC_USART_CHN);
		}

		/* change RX buffer */
		gs_ul_size_usart_buf0 = USART_BUFFER_SIZE;

		/* Restart read on buffer. */
		g_st_usart_rx_packet0.ul_addr = (uint32_t)gs_puc_usart_buf0;
		g_st_usart_rx_packet0.ul_size = USART_BUFFER_SIZE;
		pdc_rx_init(g_p_usart_pdc0, &g_st_usart_rx_packet0, NULL);

		/* Restart timer. */
		tc_start(HAL_TC_USART, HAL_TC_USART_CHN);
	}
}
#endif

#ifdef CONF_BOARD_USART1

/** @brief	Interruption handler for USART1
 *
 */
void HAL_USART1_Handler(void)
{
	uint32_t ul_status;
	uint16_t us_wr_idx, us_data_count;
	uint16_t us_end_size, us_free_size, us_part_size;

	/* Read USART Status. */
	ul_status = usart_get_status(USART1);

	/* Receive buffer is full. */
	if (ul_status & US_CSR_ENDRX) {
		/* manage data */
		us_wr_idx = busart_comm_data_1.us_wq_idx;
		us_data_count = busart_comm_data_1.us_rq_count;
		us_free_size = HAL_RX_USART_BUF1_SIZE - us_data_count;
		if (gs_ul_size_usart_buf1 <= us_free_size) {
			/* there is enough space to write all data */
			us_end_size = HAL_RX_USART_BUF1_SIZE - us_wr_idx;
			if (us_end_size >= gs_ul_size_usart_buf1) {
				/* there is no overflow of us_wq_idx */
				memcpy(&busart_comm_data_1.puc_rq_buf[us_wr_idx], gs_puc_usart_buf1, gs_ul_size_usart_buf1);
				/* update counters */
				busart_comm_data_1.us_rq_count += gs_ul_size_usart_buf1;
				busart_comm_data_1.us_wq_idx += gs_ul_size_usart_buf1;
			} else {
				/* there is overflow of us_wq_idx -> write in 2 steps	*/
				memcpy(&busart_comm_data_1.puc_rq_buf[us_wr_idx], gs_puc_usart_buf1, us_end_size);
				us_part_size = gs_ul_size_usart_buf1 - us_end_size;
				memcpy(&busart_comm_data_1.puc_rq_buf[0], &gs_puc_usart_buf1[us_end_size], us_part_size);
				/* update counters */
				busart_comm_data_1.us_rq_count += gs_ul_size_usart_buf1;
				busart_comm_data_1.us_wq_idx = us_part_size;
			}
		} else {
			/* there is not enough space to write all data */
			tc_start(HAL_TC_USART, HAL_TC_USART_CHN);
		}

		/* change RX buffer */
		gs_ul_size_usart_buf1 = USART_BUFFER_SIZE;

		/* Restart read on buffer. */
		g_st_usart_rx_packet1.ul_addr = (uint32_t)gs_puc_usart_buf1;
		g_st_usart_rx_packet1.ul_size = USART_BUFFER_SIZE;
		pdc_rx_init(g_p_usart_pdc1, &g_st_usart_rx_packet1, NULL);

		/* Restart timer. */
		tc_start(HAL_TC_USART, HAL_TC_USART_CHN);
	}
}
#endif

#ifdef CONF_BOARD_USART2

/** @brief	Interruption handler for USART2
 *
 */
void HAL_USART2_Handler(void)
{
	uint32_t ul_status;
	uint16_t us_wr_idx, us_data_count;
	uint16_t us_end_size, us_free_size, us_part_size;

	/* Read USART Status. */
	ul_status = usart_get_status(USART2);

	/* Receive buffer is full. */
	if (ul_status & US_CSR_ENDRX) {
		/* manage data */
		us_wr_idx = busart_comm_data_2.us_wq_idx;
		us_data_count = busart_comm_data_2.us_rq_count;
		us_free_size = HAL_RX_USART_BUF2_SIZE - us_data_count;
		if (gs_ul_size_usart_buf2 <= us_free_size) {
			/* there is enough space to write all data */
			us_end_size = HAL_RX_USART_BUF2_SIZE - us_wr_idx;
			if (us_end_size >= gs_ul_size_usart_buf2) {
				/* there is no overflow of us_wq_idx */
				memcpy(&busart_comm_data_2.puc_rq_buf[us_wr_idx], gs_puc_usart_buf2, gs_ul_size_usart_buf2);
				/* update counters */
				busart_comm_data_2.us_rq_count += gs_ul_size_usart_buf2;
				busart_comm_data_2.us_wq_idx += gs_ul_size_usart_buf2;
			} else {
				/* there is overflow of us_wq_idx -> write in 2 steps	*/
				memcpy(&busart_comm_data_2.puc_rq_buf[us_wr_idx], gs_puc_usart_buf2, us_end_size);
				us_part_size = gs_ul_size_usart_buf2 - us_end_size;
				memcpy(&busart_comm_data_2.puc_rq_buf[0], &gs_puc_usart_buf2[us_end_size], us_part_size);
				/* update counters */
				busart_comm_data_2.us_rq_count += gs_ul_size_usart_buf2;
				busart_comm_data_2.us_wq_idx = us_part_size;
			}
		} else {
			/* there is not enough space to write all data */
			tc_start(HAL_TC_USART, HAL_TC_USART_CHN);
		}

		/* change RX buffer */
		gs_ul_size_usart_buf2 = USART_BUFFER_SIZE;

		/* Restart read on buffer. */
		g_st_usart_rx_packet2.ul_addr = (uint32_t)gs_puc_usart_buf2;
		g_st_usart_rx_packet2.ul_size = USART_BUFFER_SIZE;
		pdc_rx_init(g_p_usart_pdc2, &g_st_usart_rx_packet2, NULL);

		/* Restart timer. */
		tc_start(HAL_TC_USART, HAL_TC_USART_CHN);
	}
}
#endif

#ifdef CONF_BOARD_USART3

/** @brief	Interruption handler for USART2
 *
 */
void HAL_USART3_Handler(void)
{
	uint32_t ul_status;
	uint16_t us_wr_idx, us_data_count;
	uint16_t us_end_size, us_free_size, us_part_size;

	/* Read USART Status. */
	ul_status = usart_get_status(USART3);

	/* Receive buffer is full. */
	if (ul_status & US_CSR_ENDRX) {
		/* manage data */
		us_wr_idx = busart_comm_data_3.us_wq_idx;
		us_data_count = busart_comm_data_3.us_rq_count;
		us_free_size = HAL_RX_USART_BUF3_SIZE - us_data_count;
		if (gs_ul_size_usart_buf3 <= us_free_size) {
			/* there is enough space to write all data */
			us_end_size = HAL_RX_USART_BUF3_SIZE - us_wr_idx;
			if (us_end_size >= gs_ul_size_usart_buf3) {
				/* there is no overflow of us_wq_idx */
				memcpy(&busart_comm_data_3.puc_rq_buf[us_wr_idx], gs_puc_usart_buf3, gs_ul_size_usart_buf3);
				/* update counters */
				busart_comm_data_3.us_rq_count += gs_ul_size_usart_buf3;
				busart_comm_data_3.us_wq_idx += gs_ul_size_usart_buf3;
			} else {
				/* there is overflow of us_wq_idx -> write in 2 steps	*/
				memcpy(&busart_comm_data_3.puc_rq_buf[us_wr_idx], gs_puc_usart_buf3, us_end_size);
				us_part_size = gs_ul_size_usart_buf3 - us_end_size;
				memcpy(&busart_comm_data_3.puc_rq_buf[0], &gs_puc_usart_buf3[us_end_size], us_part_size);
				/* update counters */
				busart_comm_data_3.us_rq_count += gs_ul_size_usart_buf3;
				busart_comm_data_3.us_wq_idx = us_part_size;
			}
		} else {
			/* there is not enough space to write all data */
			tc_start(HAL_TC_USART, HAL_TC_USART_CHN);
		}

		/* change RX buffer */
		gs_ul_size_usart_buf3 = USART_BUFFER_SIZE;

		/* Restart read on buffer. */
		g_st_usart_rx_packet3.ul_addr = (uint32_t)gs_puc_usart_buf3;
		g_st_usart_rx_packet3.ul_size = USART_BUFFER_SIZE;
		pdc_rx_init(g_p_usart_pdc3, &g_st_usart_rx_packet3, NULL);

		/* Restart timer. */
		tc_start(HAL_TC_USART, HAL_TC_USART_CHN);
	}
}
#endif

#ifdef CONF_BOARD_USART4

/** @brief	Interruption handler for USART4
 *
 */
void HAL_USART4_Handler(void)
{
	uint32_t ul_status;
	uint16_t us_wr_idx, us_data_count;
	uint16_t us_end_size, us_free_size, us_part_size;

	/* Read USART Status. */
	ul_status = usart_get_status(USART4);

	/* Receive buffer is full. */
	if (ul_status & US_CSR_ENDRX) {
		/* manage data */
		us_wr_idx = busart_comm_data_4.us_wq_idx;
		us_data_count = busart_comm_data_4.us_rq_count;
		us_free_size = HAL_RX_USART_BUF4_SIZE - us_data_count;
		if (gs_ul_size_usart_buf4 <= us_free_size) {
			/* there is enough space to write all data */
			us_end_size = HAL_RX_USART_BUF4_SIZE - us_wr_idx;
			if (us_end_size >= gs_ul_size_usart_buf4) {
				/* there is no overflow of us_wq_idx */
				memcpy(&busart_comm_data_4.puc_rq_buf[us_wr_idx], gs_puc_usart_buf4, gs_ul_size_usart_buf4);
				/* update counters */
				busart_comm_data_4.us_rq_count += gs_ul_size_usart_buf4;
				busart_comm_data_4.us_wq_idx += gs_ul_size_usart_buf4;
			} else {
				/* there is overflow of us_wq_idx -> write in 2 steps	*/
				memcpy(&busart_comm_data_4.puc_rq_buf[us_wr_idx], gs_puc_usart_buf4, us_end_size);
				us_part_size = gs_ul_size_usart_buf4 - us_end_size;
				memcpy(&busart_comm_data_4.puc_rq_buf[0], &gs_puc_usart_buf4[us_end_size], us_part_size);
				/* update counters */
				busart_comm_data_4.us_rq_count += gs_ul_size_usart_buf4;
				busart_comm_data_4.us_wq_idx = us_part_size;
			}
		} else {
			/* there is not enough space to write all data */
			tc_start(HAL_TC_USART, HAL_TC_USART_CHN);
		}

		/* change RX buffer */
		gs_ul_size_usart_buf4 = USART_BUFFER_SIZE;

		/* Restart read on buffer. */
		g_st_usart_rx_packet4.ul_addr = (uint32_t)gs_puc_usart_buf4;
		g_st_usart_rx_packet4.ul_size = USART_BUFFER_SIZE;
		pdc_rx_init(g_p_usart_pdc4, &g_st_usart_rx_packet4, NULL);

		/* Restart timer. */
		tc_start(HAL_TC_USART, HAL_TC_USART_CHN);
	}
}
#endif
#endif  /* #if defined(CONF_BOARD_USART0_RXD) || defined(CONF_BOARD_USART1_RXD)  || defined(CONF_BOARD_USART2_RXD)  || defined(CONF_BOARD_USART3_RXD) || defined(CONF_BOARD_USART4_RXD) */

/**
 * \brief This function opens an USART
 *
 * \note Opening of the specified USART implies initializing local variables and
 * opening required hardware with the following configuration:
 * - bauds as specified
 * - 8 bits, no parity, 1 stop bit
 * - enable interrupts
 *
 * \param chn			Communication channel
 * \param bauds			Communication speed in bauds
 *
 * \retval true on success.
 * \retval false on failure.
 */
uint8_t hal_usart_open(uint8_t chn, uint32_t bauds)
{
#if defined(CONF_BOARD_USART0) || defined(CONF_BOARD_USART1) || defined(CONF_BOARD_USART2) || defined(CONF_BOARD_USART3) || defined(CONF_BOARD_USART4)
	sam_usart_opt_t usart_console_settings;

	/* Expected baud rate */
	usart_console_settings.baudrate = bauds;
	/* Configure channel mode (Normal, Automatic, Local_loopback or Remote_loopback) */
	usart_console_settings.channel_mode = US_MR_CHMODE_NORMAL;
	/* Initialize value for USART mode register */
	usart_console_settings.parity_type = US_MR_PAR_NO;
	usart_console_settings.char_length = US_MR_CHRL_8_BIT;
	usart_console_settings.stop_bits = US_MR_NBSTOP_1_BIT;
#else
	UNUSED(bauds);
#endif
	/* check usart and it is close */
	if (chn >= 5) {
		return false;
	}

	if (hal_usart_chn_open[chn] == true) {
		return false;
	}

	switch (chn) {
#ifdef CONF_BOARD_USART0
	case 0:
	{
		/* Configure PMC. */
		pmc_enable_periph_clk(ID_USART0);
		/* Configure USART. */
		usart_init_rs232(USART0, &usart_console_settings, sysclk_get_peripheral_hz());

		/* Assign buffers to pointers */
		busart_comm_data_0.puc_tq_buf = ptr_tx_usart_buf0;
		busart_comm_data_0.puc_rq_buf = ptr_rx_usart_buf0;
		busart_comm_data_0.us_rq_count = 0;
		busart_comm_data_0.us_rq_idx = 0;
		busart_comm_data_0.us_wq_idx = 0;

		/* Get board USART0 PDC base address and enable receiver and transmitter. */
		g_p_usart_pdc0 = usart_get_pdc_base(USART0);
		pdc_enable_transfer(g_p_usart_pdc0, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

		/* Start receiving data and start timer. */
		g_st_usart_rx_packet0.ul_addr = (uint32_t)gs_puc_usart_buf0;
		g_st_usart_rx_packet0.ul_size = USART_BUFFER_SIZE;
		pdc_rx_init(g_p_usart_pdc0, &g_st_usart_rx_packet0, NULL);

		/* Stop transmitting data */
		g_st_usart_tx_packet0.ul_addr = (uint32_t)busart_comm_data_0.puc_tq_buf;
		g_st_usart_tx_packet0.ul_size = 0;
		pdc_tx_init(g_p_usart_pdc0, &g_st_usart_tx_packet0, NULL);

		gs_ul_size_usart_buf0 = USART_BUFFER_SIZE;

		/* Transfer to PDC communication mode, disable RXRDY interrupt
		 * and enable RXBUFF interrupt. */
		usart_disable_interrupt(USART0, US_IDR_RXRDY);
		usart_enable_interrupt(USART0, US_IER_RXBUFF);

		/* Enable the receiver and transmitter. */
		usart_enable_tx(USART0);
		usart_enable_rx(USART0);

		/* Configure and enable interrupt of USART. */
		NVIC_SetPriority((IRQn_Type)USART0_IRQn, USART0_PRIO);
		NVIC_EnableIRQ(USART0_IRQn);

		hal_usart_chn_open[chn] = true;
		num_bytes_rx_usart0 = 0;

		/* Configure TC usart */
		_configure_TC_usart();
		tc_start(HAL_TC_USART, HAL_TC_USART_CHN);

		return true;
	}
	break;
#endif
#ifdef CONF_BOARD_USART1
	case 1:
	{
		/* Configure PMC. */
		pmc_enable_periph_clk(ID_USART1);
		/* Configure USART. */
		usart_init_rs232(USART1, &usart_console_settings, sysclk_get_peripheral_hz());

		/* Assign buffers to pointers */
		busart_comm_data_1.puc_tq_buf = ptr_tx_usart_buf1;
		busart_comm_data_1.puc_rq_buf = ptr_rx_usart_buf1;
		busart_comm_data_1.us_rq_count = 0;
		busart_comm_data_1.us_rq_idx = 0;
		busart_comm_data_1.us_wq_idx = 0;

		/* Get board USART1 PDC base address and enable receiver and
		 * transmitter. */
		g_p_usart_pdc1 = usart_get_pdc_base(USART1);
		pdc_enable_transfer(g_p_usart_pdc1, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

		/* Start receiving data and start timer. */
		g_st_usart_rx_packet1.ul_addr = (uint32_t)gs_puc_usart_buf1;
		g_st_usart_rx_packet1.ul_size = USART_BUFFER_SIZE;
		pdc_rx_init(g_p_usart_pdc1, &g_st_usart_rx_packet1, NULL);

		/* Stop transmitting data */
		g_st_usart_tx_packet1.ul_addr = (uint32_t)busart_comm_data_1.puc_tq_buf;
		g_st_usart_tx_packet1.ul_size = 0;
		pdc_tx_init(g_p_usart_pdc1, &g_st_usart_tx_packet1, NULL);

		gs_ul_size_usart_buf1 = USART_BUFFER_SIZE;

		/* Transfer to PDC communication mode, disable RXRDY interrupt
		 * and enable RXBUFF interrupt. */
		usart_disable_interrupt(USART1, US_IDR_RXRDY);
		usart_enable_interrupt(USART1, US_IER_RXBUFF);

		/* Enable the receiver and transmitter. */
		usart_enable_tx(USART1);
		usart_enable_rx(USART1);

		/* Configure and enable interrupt of USART. */
		NVIC_SetPriority((IRQn_Type)USART1_IRQn, USART1_PRIO);
		NVIC_EnableIRQ(USART1_IRQn);

		hal_usart_chn_open[chn] = true;
		num_bytes_rx_usart1 = 0;

		/* Configure TC usart */
		_configure_TC_usart();
		tc_start(HAL_TC_USART, HAL_TC_USART_CHN);

		return true;
	}
	break;
#endif
#ifdef CONF_BOARD_USART2
	case 2:
	{
		/* Configure PMC. */
		pmc_enable_periph_clk(ID_USART2);
		/* Configure USART. */
		usart_init_rs232(USART2, &usart_console_settings, sysclk_get_peripheral_hz());

		/* Assign buffers to pointers */
		busart_comm_data_2.puc_tq_buf = ptr_tx_usart_buf2;
		busart_comm_data_2.puc_rq_buf = ptr_rx_usart_buf2;
		busart_comm_data_2.us_rq_count = 0;
		busart_comm_data_2.us_rq_idx = 0;
		busart_comm_data_2.us_wq_idx = 0;

		/* Get board USART2 PDC base address and enable receiver and
		 * transmitter. */
		g_p_usart_pdc2 = usart_get_pdc_base(USART2);
		pdc_enable_transfer(g_p_usart_pdc2, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

		/* Start receiving data and start timer. */
		g_st_usart_rx_packet2.ul_addr = (uint32_t)gs_puc_usart_buf2;
		g_st_usart_rx_packet2.ul_size = USART_BUFFER_SIZE;
		pdc_rx_init(g_p_usart_pdc2, &g_st_usart_rx_packet2, NULL);

		/* Stop transmitting data */
		g_st_usart_tx_packet2.ul_addr = (uint32_t)busart_comm_data_2.puc_tq_buf;
		g_st_usart_tx_packet2.ul_size = 0;
		pdc_tx_init(g_p_usart_pdc2, &g_st_usart_tx_packet2, NULL);

		gs_ul_size_usart_buf2 = USART_BUFFER_SIZE;

		/* Transfer to PDC communication mode, disable RXRDY interrupt
		 * and enable RXBUFF interrupt. */
		usart_disable_interrupt(USART2, US_IDR_RXRDY);
		usart_enable_interrupt(USART2, US_IER_RXBUFF);

		/* Enable the receiver and transmitter. */
		usart_enable_tx(USART2);
		usart_enable_rx(USART2);

		/* Configure and enable interrupt of USART. */
		NVIC_SetPriority((IRQn_Type)USART2_IRQn, USART2_PRIO);
		NVIC_EnableIRQ(USART2_IRQn);

		hal_usart_chn_open[chn] = true;
		num_bytes_rx_usart2 = 0;

		/* Configure TC usart */
		_configure_TC_usart();
		tc_start(HAL_TC_USART, HAL_TC_USART_CHN);

		return true;
	}
	break;
#endif
#ifdef CONF_BOARD_USART3
	case 3:
	{
		/* Configure PMC. */
		pmc_enable_periph_clk(ID_USART3);
		/* Configure USART. */
		usart_init_rs232(USART3, &usart_console_settings, sysclk_get_peripheral_hz());

		/* Assign buffers to pointers */
		busart_comm_data_3.puc_tq_buf = ptr_tx_usart_buf3;
		busart_comm_data_3.puc_rq_buf = ptr_rx_usart_buf3;
		busart_comm_data_3.us_rq_count = 0;
		busart_comm_data_3.us_rq_idx = 0;
		busart_comm_data_3.us_wq_idx = 0;

		/* Get board USART3 PDC base address and enable receiver and
		 * transmitter. */
		g_p_usart_pdc3 = usart_get_pdc_base(USART3);
		pdc_enable_transfer(g_p_usart_pdc3, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

		/* Start receiving data and start timer. */
		g_st_usart_rx_packet3.ul_addr = (uint32_t)gs_puc_usart_buf3;
		g_st_usart_rx_packet3.ul_size = USART_BUFFER_SIZE;
		pdc_rx_init(g_p_usart_pdc3, &g_st_usart_rx_packet3, NULL);

		/* Stop transmitting data */
		g_st_usart_tx_packet3.ul_addr = (uint32_t)busart_comm_data_3.puc_tq_buf;
		g_st_usart_tx_packet3.ul_size = 0;
		pdc_tx_init(g_p_usart_pdc3, &g_st_usart_tx_packet3, NULL);

		gs_ul_size_usart_buf3 = USART_BUFFER_SIZE;

		/* Transfer to PDC communication mode, disable RXRDY interrupt
		 * and enable RXBUFF interrupt. */
		usart_disable_interrupt(USART3, US_IDR_RXRDY);
		usart_enable_interrupt(USART3, US_IER_RXBUFF);

		/* Enable the receiver and transmitter. */
		usart_enable_tx(USART3);
		usart_enable_rx(USART3);

		/* Configure and enable interrupt of USART. */
		NVIC_SetPriority((IRQn_Type)USART3_IRQn, USART3_PRIO);
		NVIC_EnableIRQ(USART3_IRQn);

		hal_usart_chn_open[chn] = true;
		num_bytes_rx_usart3 = 0;

		/* Configure TC usart */
		_configure_TC_usart();
		tc_start(HAL_TC_USART, HAL_TC_USART_CHN);

		return true;
	}
	break;
#endif
#ifdef CONF_BOARD_USART4
	case 4:
	{
		/* Configure PMC. */
		pmc_enable_periph_clk(ID_USART4);
		/* Configure USART. */
		usart_init_rs232(USART4, &usart_console_settings, sysclk_get_peripheral_hz());

		/* Assign buffers to pointers */
		busart_comm_data_4.puc_tq_buf = ptr_tx_usart_buf4;
		busart_comm_data_4.puc_rq_buf = ptr_rx_usart_buf4;
		busart_comm_data_4.us_rq_count = 0;
		busart_comm_data_4.us_rq_idx = 0;
		busart_comm_data_4.us_wq_idx = 0;

		/* Get board USART4 PDC base address and enable receiver and
		 * transmitter. */
		g_p_usart_pdc4 = usart_get_pdc_base(USART4);
		pdc_enable_transfer(g_p_usart_pdc4, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

		/* Start receiving data and start timer. */
		g_st_usart_rx_packet4.ul_addr = (uint32_t)gs_puc_usart_buf4;
		g_st_usart_rx_packet4.ul_size = USART_BUFFER_SIZE;
		pdc_rx_init(g_p_usart_pdc4, &g_st_usart_rx_packet4, NULL);

		/* Stop transmitting data */
		g_st_usart_tx_packet4.ul_addr = (uint32_t)busart_comm_data_4.puc_tq_buf;
		g_st_usart_tx_packet4.ul_size = 0;
		pdc_tx_init(g_p_usart_pdc4, &g_st_usart_tx_packet4, NULL);

		gs_ul_size_usart_buf4 = USART_BUFFER_SIZE;

		/* Transfer to PDC communication mode, disable RXRDY interrupt
		 * and enable RXBUFF interrupt. */
		usart_disable_interrupt(USART4, US_IDR_RXRDY);
		usart_enable_interrupt(USART4, US_IER_RXBUFF);

		/* Enable the receiver and transmitter. */
		usart_enable_tx(USART4);
		usart_enable_rx(USART4);

		/* Configure and enable interrupt of USART. */
		NVIC_SetPriority((IRQn_Type)USART4_IRQn, USART4_PRIO);
		NVIC_EnableIRQ(USART4_IRQn);

		hal_usart_chn_open[chn] = true;
		num_bytes_rx_usart4 = 0;

		/* Configure TC usart */
		_configure_TC_usart();
		tc_start(HAL_TC_USART, HAL_TC_USART_CHN);

		return true;
	}
	break;
#endif
	default:
		return false;
	}
}

/**
 * \brief This function closes and disables communication in the specified
 *        USART.
 *
 * \param chn  Communication channel
 *
 * \retval true on success.
 * \retval false on failure.
 *
 * WARNING: There is only one timer to handle all USARTs, so if stopped,
 * all configured USARTs will stop working. However, it is not necessary
 * to stop it since interruptions are disabled just for the closed USART. 
 */
uint8_t hal_usart_close(uint8_t chn)
{
	if (!hal_usart_chn_open[chn]) {
		return false;
	}

	switch (chn) {
#ifdef CONF_BOARD_USART0
	case 0:
	{
		usart_disable_tx(USART0);
		usart_disable_rx(USART0);

		usart_disable_interrupt(USART0, US_IDR_RXRDY);
		usart_disable_interrupt(USART0, US_IER_ENDRX);

		/* Stop TC */
		if (!hal_usart_chn_open[0]) {
			/*tc_stop(HAL_TC_USART, HAL_TC_USART_CHN);*/
		}

		return true;
	}
	break;
#endif

#ifdef CONF_BOARD_USART1
	case 1:
	{
		usart_disable_tx(USART1);
		usart_disable_rx(USART1);

		usart_disable_interrupt(USART1, US_IDR_RXRDY);
		usart_disable_interrupt(USART1, US_IER_ENDRX);

		/* Stop TC */
		if (!hal_usart_chn_open[1]) {
			/*tc_stop(HAL_TC_USART, HAL_TC_USART_CHN);*/
		}

		return true;
	}
	break;
#endif

#ifdef CONF_BOARD_USART2
	case 2:
	{
		usart_disable_tx(USART2);
		usart_disable_rx(USART2);

		usart_disable_interrupt(USART2, US_IDR_RXRDY);
		usart_disable_interrupt(USART2, US_IER_ENDRX);

		/* Stop TC */
		if (!hal_usart_chn_open[2]) {
			/*tc_stop(HAL_TC_USART, HAL_TC_USART_CHN);*/
		}

		return true;
	}
	break;
#endif

#ifdef CONF_BOARD_USART3
	case 3:
	{
		usart_disable_tx(USART3);
		usart_disable_rx(USART3);

		usart_disable_interrupt(USART3, US_IDR_RXRDY);
		usart_disable_interrupt(USART3, US_IER_ENDRX);

		/* Stop TC */
		if (!hal_usart_chn_open[3]) {
			/*tc_stop(HAL_TC_USART, HAL_TC_USART_CHN);*/
		}

		return true;
	}
	break;
#endif

#ifdef CONF_BOARD_USART4
	case 4:
	{
		usart_disable_tx(USART4);
		usart_disable_rx(USART4);

		usart_disable_interrupt(USART4, US_IDR_RXRDY);
		usart_disable_interrupt(USART4, US_IER_ENDRX);

		/* Stop TC */
		if (!hal_usart_chn_open[4]) {
			/*tc_stop(HAL_TC_USART, HAL_TC_USART_CHN);*/
		}

		return true;
	}
	break;
#endif
	default:
		return false;
	}
}

/**
 * \brief This function receives a message.
 *
 * \note This function receives a given number of characters from the specified
 * USART.
 * If so configured, the function waits until all characters are received. In
 * this case, the watchdog timer must be reloaded to avoid a program reset.
 *
 * \param  chn     Communication channel
 * \param  buffer  Pointer to buffer for information
 * \param  len     Number of characters to receive
 *
 * \retval Number of received characters
 */
uint16_t hal_usart_read(uint8_t chn, void *buffer, uint16_t len)
{
#if defined(CONF_BOARD_USART0) || defined(CONF_BOARD_USART1) || defined(CONF_BOARD_USART2) || defined(CONF_BOARD_USART3) || defined(CONF_BOARD_USART4)
	uint16_t us_rd_idx = 0;
	uint16_t us_num_bytes_read, us_num_bytes_to_end, us_num_bytes_to_start;
	uint16_t us_total_pos;
	uint16_t us_buf_size;
	uint8_t *msg = (uint8_t *)buffer;
#else
	UNUSED(buffer);
	UNUSED(len);
#endif

	/* check usart is open */
	if (!hal_usart_chn_open[chn]) {
		return 0;
	}

	switch (chn) {
#ifdef CONF_BOARD_USART0
	case 0:
		us_buf_size = HAL_RX_USART_BUF0_SIZE;
		/* check if there is any byte in rx queue */
		if (busart_comm_data_0.us_rq_count == 0) {
			return 0;
		}

		/* get counters */
		us_rd_idx = busart_comm_data_0.us_rq_idx;
		/* get number of bytes to read */
		if (busart_comm_data_0.us_rq_count >= len) {
			us_num_bytes_read = len;
		} else {
			us_num_bytes_read = busart_comm_data_0.us_rq_count;
		}

		/* check overflow us_rd_idx counter */
		us_total_pos = us_rd_idx + us_num_bytes_read;
		if (us_total_pos <= us_buf_size) {
			/* copy data to buffer */
			memcpy(msg, &busart_comm_data_0.puc_rq_buf[us_rd_idx], us_num_bytes_read);
			/* update counters */
			busart_comm_data_0.us_rq_count -= us_num_bytes_read;
			busart_comm_data_0.us_rq_idx += us_num_bytes_read;
		} else {
			/* copy data to buffer in fragments -> overflow
			 * us_rq_idx counter */
			us_num_bytes_to_start = us_total_pos - us_buf_size;
			us_num_bytes_to_end = us_num_bytes_read - us_num_bytes_to_start;
			memcpy(msg, &busart_comm_data_0.puc_rq_buf[us_rd_idx], us_num_bytes_to_end);
			msg += us_num_bytes_to_end;
			memcpy(msg, &busart_comm_data_0.puc_rq_buf[0], us_num_bytes_to_start);
			/* update counters */
			busart_comm_data_0.us_rq_count -= us_num_bytes_read;
			busart_comm_data_0.us_rq_idx = us_num_bytes_to_start;
		}

		return us_num_bytes_read;
#endif

#ifdef CONF_BOARD_USART1
	case 1:
		us_buf_size = HAL_RX_USART_BUF1_SIZE;
		/* check if there is any byte in rx queue */
		if (busart_comm_data_1.us_rq_count == 0) {
			return 0;
		}

		/* get counters */
		us_rd_idx = busart_comm_data_1.us_rq_idx;
		/* get number of bytes to read */
		if (busart_comm_data_1.us_rq_count >= len) {
			us_num_bytes_read = len;
		} else {
			us_num_bytes_read = busart_comm_data_1.us_rq_count;
		}

		/* check overflow us_rd_idx counter */
		us_total_pos = us_rd_idx + us_num_bytes_read;
		if (us_total_pos <= us_buf_size) {
			/* copy data to buffer */
			memcpy(msg, &busart_comm_data_1.puc_rq_buf[us_rd_idx], us_num_bytes_read);
			/* update counters */
			busart_comm_data_1.us_rq_count -= us_num_bytes_read;
			busart_comm_data_1.us_rq_idx += us_num_bytes_read;
		} else {
			/* copy data to buffer in fragments -> overflow
			 * us_rq_idx counter */
			us_num_bytes_to_start = us_total_pos - us_buf_size;
			us_num_bytes_to_end = us_num_bytes_read - us_num_bytes_to_start;
			memcpy(msg, &busart_comm_data_1.puc_rq_buf[us_rd_idx], us_num_bytes_to_end);
			msg += us_num_bytes_to_end;
			memcpy(msg, &busart_comm_data_1.puc_rq_buf[0], us_num_bytes_to_start);
			/* update counters */
			busart_comm_data_1.us_rq_count -= us_num_bytes_read;
			busart_comm_data_1.us_rq_idx = us_num_bytes_to_start;
		}
		return us_num_bytes_read;
#endif

#ifdef CONF_BOARD_USART2
	case 2:
		us_buf_size = HAL_RX_USART_BUF2_SIZE;
		/* check if there is any byte in rx queue */
		if (busart_comm_data_2.us_rq_count == 0) {
			return 0;
		}

		/* get counters */
		us_rd_idx = busart_comm_data_2.us_rq_idx;
		/* get number of bytes to read */
		if (busart_comm_data_2.us_rq_count >= len) {
			us_num_bytes_read = len;
		} else {
			us_num_bytes_read = busart_comm_data_2.us_rq_count;
		}

		/* check overflow us_rd_idx counter */
		us_total_pos = us_rd_idx + us_num_bytes_read;
		if (us_total_pos <= us_buf_size) {
			/* copy data to buffer */
			memcpy(msg, &busart_comm_data_2.puc_rq_buf[us_rd_idx], us_num_bytes_read);
			/* update counters */
			busart_comm_data_2.us_rq_count -= us_num_bytes_read;
			busart_comm_data_2.us_rq_idx += us_num_bytes_read;
		} else {
			/* copy data to buffer in fragments -> overflow
			 * us_rq_idx counter */
			us_num_bytes_to_start = us_total_pos - us_buf_size;
			us_num_bytes_to_end = us_num_bytes_read - us_num_bytes_to_start;
			memcpy(msg, &busart_comm_data_2.puc_rq_buf[us_rd_idx], us_num_bytes_to_end);
			msg += us_num_bytes_to_end;
			memcpy(msg, &busart_comm_data_2.puc_rq_buf[0], us_num_bytes_to_start);
			/* update counters */
			busart_comm_data_2.us_rq_count -= us_num_bytes_read;
			busart_comm_data_2.us_rq_idx = us_num_bytes_to_start;
		}
		return us_num_bytes_read;
#endif

#ifdef CONF_BOARD_USART3
	case 3:
		us_buf_size = HAL_RX_USART_BUF3_SIZE;
		/* check if there is any byte in rx queue */
		if (busart_comm_data_3.us_rq_count == 0) {
			return 0;
		}

		/* get counters */
		us_rd_idx = busart_comm_data_3.us_rq_idx;
		/* get number of bytes to read */
		if (busart_comm_data_3.us_rq_count >= len) {
			us_num_bytes_read = len;
		} else {
			us_num_bytes_read = busart_comm_data_3.us_rq_count;
		}

		/* check overflow us_rd_idx counter */
		us_total_pos = us_rd_idx + us_num_bytes_read;
		if (us_total_pos <= us_buf_size) {
			/* copy data to buffer */
			memcpy(msg, &busart_comm_data_3.puc_rq_buf[us_rd_idx], us_num_bytes_read);
			/* update counters */
			busart_comm_data_3.us_rq_count -= us_num_bytes_read;
			busart_comm_data_3.us_rq_idx += us_num_bytes_read;
		} else {
			/* copy data to buffer in fragments -> overflow
			 * us_rq_idx counter */
			us_num_bytes_to_start = us_total_pos - us_buf_size;
			us_num_bytes_to_end = us_num_bytes_read - us_num_bytes_to_start;
			memcpy(msg, &busart_comm_data_3.puc_rq_buf[us_rd_idx], us_num_bytes_to_end);
			msg += us_num_bytes_to_end;
			memcpy(msg, &busart_comm_data_3.puc_rq_buf[0], us_num_bytes_to_start);
			/* update counters */
			busart_comm_data_3.us_rq_count -= us_num_bytes_read;
			busart_comm_data_3.us_rq_idx = us_num_bytes_to_start;
		}
		return us_num_bytes_read;
#endif

#ifdef CONF_BOARD_USART4
	case 4:
		us_buf_size = HAL_RX_USART_BUF4_SIZE;
		/* check if there is any byte in rx queue */
		if (busart_comm_data_4.us_rq_count == 0) {
			return 0;
		}

		/* get counters */
		us_rd_idx = busart_comm_data_4.us_rq_idx;
		/* get number of bytes to read */
		if (busart_comm_data_4.us_rq_count >= len) {
			us_num_bytes_read = len;
		} else {
			us_num_bytes_read = busart_comm_data_4.us_rq_count;
		}

		/* check overflow us_rd_idx counter */
		us_total_pos = us_rd_idx + us_num_bytes_read;
		if (us_total_pos <= us_buf_size) {
			/* copy data to buffer */
			memcpy(msg, &busart_comm_data_4.puc_rq_buf[us_rd_idx], us_num_bytes_read);
			/* update counters */
			busart_comm_data_4.us_rq_count -= us_num_bytes_read;
			busart_comm_data_4.us_rq_idx += us_num_bytes_read;
		} else {
			/* copy data to buffer in fragments -> overflow
			 * us_rq_idx counter */
			us_num_bytes_to_start = us_total_pos - us_buf_size;
			us_num_bytes_to_end = us_num_bytes_read - us_num_bytes_to_start;
			memcpy(msg, &busart_comm_data_4.puc_rq_buf[us_rd_idx], us_num_bytes_to_end);
			msg += us_num_bytes_to_end;
			memcpy(msg, &busart_comm_data_4.puc_rq_buf[0], us_num_bytes_to_start);
			/* update counters */
			busart_comm_data_4.us_rq_count -= us_num_bytes_read;
			busart_comm_data_4.us_rq_idx = us_num_bytes_to_start;
		}
		return us_num_bytes_read;
#endif
	default:
		return 0;
	}
}

/**
 * \brief This function transmits a message.
 *
 * \note This function transmits characters via the specified USART.
 * If so configured, the function waits until all characters are inserted
 * in the transmission queue. In this case, the watchdog timer must be
 * reloaded to avoid a program reset.
 *
 * \param  chn     Communication channel
 * \param  buffer  Pointer to information to transmit
 * \param  len     Number of characters to transmit
 *
 * \retval Number of characters sent
 */
uint16_t hal_usart_write(uint8_t chn, const void *buffer, uint16_t len)
{
#if !defined(CONF_BOARD_USART0) && !defined(CONF_BOARD_USART1) && !defined(CONF_BOARD_USART2) && !defined(CONF_BOARD_USART3) && !defined(CONF_BOARD_USART4)
	UNUSED(buffer);
	UNUSED(len);
#endif
	/* check usart is open */
	if (!hal_usart_chn_open[chn]) {
		return 0;
	}

	switch (chn) {
#ifdef CONF_BOARD_USART0
	case 0:
		if (len > HAL_TX_USART_BUF0_SIZE) {
			return 0;
		}

		while (pdc_read_tx_counter(g_p_usart_pdc0) > 0) {
		}
		memcpy(&busart_comm_data_0.puc_tq_buf[0], buffer, len);
		g_st_usart_tx_packet0.ul_addr = (uint32_t)&busart_comm_data_0.puc_tq_buf[0];
		g_st_usart_tx_packet0.ul_size = len;
		pdc_tx_init(g_p_usart_pdc0, &g_st_usart_tx_packet0, NULL);
		return len;
#endif

#ifdef CONF_BOARD_USART1
	case 1:
		if (len > HAL_TX_USART_BUF1_SIZE) {
			return 0;
		}

		while (pdc_read_tx_counter(g_p_usart_pdc1) > 0) {
		}
		memcpy(&busart_comm_data_1.puc_tq_buf[0], buffer, len);
		g_st_usart_tx_packet1.ul_addr = (uint32_t)&busart_comm_data_1.puc_tq_buf[0];
		g_st_usart_tx_packet1.ul_size = len;
		pdc_tx_init(g_p_usart_pdc1, &g_st_usart_tx_packet1, NULL);
		return len;
#endif

#ifdef CONF_BOARD_USART2
	case 2:
		if (len > HAL_TX_USART_BUF2_SIZE) {
			return 0;
		}

		while (pdc_read_tx_counter(g_p_usart_pdc2) > 0) {
		}
		memcpy(&busart_comm_data_2.puc_tq_buf[0], buffer, len);
		g_st_usart_tx_packet2.ul_addr = (uint32_t)&busart_comm_data_2.puc_tq_buf[0];
		g_st_usart_tx_packet2.ul_size = len;
		pdc_tx_init(g_p_usart_pdc2, &g_st_usart_tx_packet2, NULL);
		return len;
#endif

#ifdef CONF_BOARD_USART3
	case 3:
		if (len > HAL_TX_USART_BUF3_SIZE) {
			return 0;
		}

		while (pdc_read_tx_counter(g_p_usart_pdc3) > 0) {
		}
		memcpy(&busart_comm_data_3.puc_tq_buf[0], buffer, len);
		g_st_usart_tx_packet3.ul_addr = (uint32_t)&busart_comm_data_3.puc_tq_buf[0];
		g_st_usart_tx_packet3.ul_size = len;
		pdc_tx_init(g_p_usart_pdc3, &g_st_usart_tx_packet3, NULL);
		return len;
#endif

#ifdef CONF_BOARD_USART4
	case 4:
		if (len > HAL_TX_USART_BUF4_SIZE) {
			return 0;
		}

		while (pdc_read_tx_counter(g_p_usart_pdc4) > 0) {
		}
		memcpy(&busart_comm_data_4.puc_tq_buf[0], buffer, len);
		g_st_usart_tx_packet4.ul_addr = (uint32_t)&busart_comm_data_4.puc_tq_buf[0];
		g_st_usart_tx_packet4.ul_size = len;
		pdc_tx_init(g_p_usart_pdc4, &g_st_usart_tx_packet4, NULL);
		return len;
#endif

	default:
		return 0;
	}
}

/**
 * \brief Get byte from USART.
 *
 * \param  chn  Communication channel
 *
 * \retval Byte received
 * \retval -1 in case of no reception
 */
int hal_usart_rx_char(uint8_t chn)
{
	uint8_t buf[4] = {0, 0, 0, 0};

	if (hal_usart_read(chn, buf, 1) <= 0) {
		return (-1);
	}

	return buf[0];
}

/**
 * \brief Sent byte to USART.
 *
 * \param  chn   Communication channel
 * \param  data  Data to sent
 *
 * \retval Number of characters sent
 */
uint16_t hal_usart_tx_char(uint8_t chn, char data)
{
	return (hal_usart_write(chn, &data, 1));
}

/**
 * \brief Check USART PDC transmission in course.
 *
 * \param  chn   Communication channel
 *
 * \retval true is USART is free to tx, false in otherwise
 */
bool hal_usart_is_free(uint8_t chn)
{
	/* check uart is open */
	if (!hal_usart_chn_open[chn]) {
		return false;
	}

	switch (chn) {
#ifdef CONF_BOARD_USART0
	case 0:
		if (pdc_read_tx_counter(g_p_usart_pdc0)) {
			return false;
		} else {
			return true;
		}
#endif

#ifdef CONF_BOARD_USART1
	case 1:
		if (pdc_read_tx_counter(g_p_usart_pdc1)) {
			return false;
		} else {
			return true;
		}
#endif

#ifdef CONF_BOARD_USART2
	case 2:
		if (pdc_read_tx_counter(g_p_usart_pdc2)) {
			return false;
		} else {
			return true;
		}
#endif

#ifdef CONF_BOARD_USART3
	case 3:
		if (pdc_read_tx_counter(g_p_usart_pdc3)) {
			return false;
		} else {
			return true;
		}
#endif

#ifdef CONF_BOARD_USART4
	case 4:
		if (pdc_read_tx_counter(g_p_usart_pdc4)) {
			return false;
		} else {
			return true;
		}
#endif

	default:
		return false;
	}
}

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */
