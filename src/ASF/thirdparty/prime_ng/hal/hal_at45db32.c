/**
 * \file
 *
 * \brief HAL_AT45DB32
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

#include <sysclk.h>
#include "pdc.h"
#include "usart.h"
#include "hal_private.h"

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

/** Pointer to PDC register base */
static Pdc *g_at45db_pdc;
/** PDC Transmission buffer */
static uint8_t gs_at45db_tx_buffer[520];
/** PDC TX data packet */
static pdc_packet_t g_at45db_tx_packet;
/** PDC Receive buffer */
static uint8_t gs_at45db_rx_buffer[520];
/** PDC RX data packet */
static pdc_packet_t g_at45db_rx_packet;

/**
 * \brief Send Cmd in AT45DB32 data flash memory
 *
 * \param uc_cmd     AT45DB32 command
 * \param ul_addr    Serial Data Address to store/read data
 * \param puc_data   Pointer to data buffer
 * \param uc_len     Length of data in bytes
 *
 */
uint8_t hal_at45dbx_send_cmd(uint8_t uc_cmd, uint32_t ul_addr, uint8_t *puc_data, uint16_t uc_len)
{
	uint16_t i;
	uint8_t uc_dummy_offset;
	uint16_t uc_pkt_size = 0;
	uint8_t *puc_tx_buf;
	uint8_t *puc_rx_buf;
	uint8_t *puc_data_buf;
	bool b_read_en;

	puc_tx_buf = gs_at45db_tx_buffer;
	puc_rx_buf = gs_at45db_rx_buffer;
	puc_data_buf = puc_data;

	switch (uc_cmd) {
	case AT45DBX_RD_MNFCT_DEV_ID_SM:
	case AT45DBX_RD_STATUS_REG:
		uc_dummy_offset = 0;
		b_read_en = true;
		break;

	case AT45DBX_RD_BUF1_AF_8M:
		uc_dummy_offset = 1;
		b_read_en = true;
		break;

	case AT45DBX_RD_PAGE:
		uc_dummy_offset = 4;
		b_read_en = true;
		break;

	case AT45DBX_WR_BUF1:
	case AT45DBX_PR_PAGE_TH_BUF1:
	case AT45DBX_XFR_PAGE_TO_BUF1:
	case AT45DBX_PR_BUF1_TO_PAGE_ER:
	case AT45DBX_ER_SECTOR:
		uc_dummy_offset = 0;
		b_read_en = false;
		break;

	default:
		return 0;
	}

	/* cmd */
	*puc_tx_buf++ = uc_cmd;
	*puc_rx_buf++ = 0;
	/* addr */
	*puc_tx_buf++ = (uint8_t)(ul_addr >> 16);
	*puc_rx_buf++ = 0;
	*puc_tx_buf++ = (uint8_t)(ul_addr >> 8);
	*puc_rx_buf++ = 0;
	*puc_tx_buf++ = (uint8_t)ul_addr;
	*puc_rx_buf++ = 0;

	if (b_read_en) {
		/* READ: TX Dummy data and clear rx data buff */
		uc_pkt_size = uc_len + uc_dummy_offset;
		for (i = 0; i < uc_pkt_size; i++) {
			*puc_tx_buf++ = 0;
			*puc_rx_buf++ = 0;
		}
	} else {
		/* WRITE: TX Copy data and clear rx data buff */
		uc_pkt_size = uc_len;
		for (i = 0; i < uc_pkt_size; i++) {
			*puc_tx_buf++ = *puc_data_buf++;
			*puc_rx_buf++ = 0;
		}
	}

	/* Add header bytes: cmd and address bytes */
	uc_pkt_size += 4;

	g_at45db_rx_packet.ul_addr = (uint32_t)gs_at45db_rx_buffer;
	g_at45db_rx_packet.ul_size = uc_pkt_size;
	pdc_rx_init(g_at45db_pdc, &g_at45db_rx_packet, NULL);

	g_at45db_tx_packet.ul_addr = (uint32_t)gs_at45db_tx_buffer;
	g_at45db_tx_packet.ul_size = uc_pkt_size;
	pdc_tx_init(g_at45db_pdc, &g_at45db_tx_packet, NULL);

	/* Enable the RX and TX PDC transfer requests */
	usart_spi_force_chip_select(USART2);
	pdc_enable_transfer(g_at45db_pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

	/* Waiting transfer done*/
	while (usart_is_rx_buf_full(USART2) == 0) {
	}

	/* Disable the TX PDC transfer requests */
	pdc_disable_transfer(g_at45db_pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);
	usart_spi_release_chip_select(USART2);

	/* Copy data to user buff */
	if (b_read_en) {
		puc_rx_buf = &gs_at45db_rx_buffer[uc_dummy_offset + 4];
		for (i = 0; i < uc_len; i++) {
			*puc_data_buf++ = *puc_rx_buf++;
		}
	}

	return 1;
}

/**
 * \brief Wait to AT45Db32 is ready
 *
 */
void hal_at45dbx_wait_is_ready(void)
{
	uint8_t uc_at45_status;

	do {
		hal_at45dbx_send_cmd(AT45DBX_RD_STATUS_REG, 0, &uc_at45_status, 1);
	} while (!(uc_at45_status & 0x80));
}

/**
 * \brief Init AT45Db32 communication
 *
 */
void hal_at45dbx_init(void)
{
	usart_spi_opt_t at45dbx_opt;

	/* Enable the peripheral clock in the PMC. */
	sysclk_enable_peripheral_clock(ID_USART2);
	/* Configure USART in spi mode. */
	at45dbx_opt.baudrate = 30000000;
	at45dbx_opt.channel_mode = US_MR_CHMODE_NORMAL;
	at45dbx_opt.char_length = US_MR_CHRL_8_BIT;
	at45dbx_opt.spi_mode = SPI_MODE_3;
	usart_init_spi_master(USART2, &at45dbx_opt, sysclk_get_cpu_hz());
	/* Get PDC handler. */
	g_at45db_pdc = usart_get_pdc_base(USART2);
	/* Enable TX & RX function. */
	usart_enable_tx(USART2);
	usart_enable_rx(USART2);
}

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */
