/**
 * \file
 *
 * \brief HAL_PLC
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
#include "string.h"
#include "board.h"
#include "ioport.h"
#include "pmc.h"
#include "pio.h"
#include "pio_handler.h"
#include "pdc.h"
#include "spi.h"
#include "hal_private.h"
#include "conf_hal.h"
#include "delay.h"

#ifdef HAL_ATPL360_INTERFACE
#include "gpio.h"
#include "atpl360_hal_spi.h"
#endif

#ifndef HAL_PLC_POL
#define HAL_PLC_POL           0
#endif

#ifndef HAL_PLC_PHA
#define HAL_PLC_PHA           1
#endif

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

/** PLC clock setting (Hz). */
static uint32_t gs_ul_plc_clock = HAL_PLC_CLOCK;

#ifdef HAL_ATPL360_INTERFACE
/** SPI Header size. */
#define PDC_SPI_HEADER_SIZE            6
/** SPI Max Msg_Data size. */
#define PDC_SPI_MSG_DATA_SIZE          512
/** SPI Max Msg_Data size. */
#define PDC_SPI_MSG_PARAMS_SIZE        116   /* Worst case = 116: sizeof(rx_msg_t) [G3] */
/** PDC buffer us_size. */
#define PDC_PPLC_BUFFER_SIZE           (PDC_SPI_HEADER_SIZE + PDC_SPI_MSG_DATA_SIZE + PDC_SPI_MSG_PARAMS_SIZE)
#else
/** PDC buffer us_size. */
#define PDC_PPLC_BUFFER_SIZE           800
#endif

/** Max number of folowing reads. */
#define MAX_NUM_READ_RX_COUNTER        10

/** PDC Receive buffer */
static uint8_t gs_plc_rx_buffer[PDC_PPLC_BUFFER_SIZE];
/** PDC Transmission buffer */
static uint8_t gs_plc_tx_buffer[PDC_PPLC_BUFFER_SIZE];
/** PDC RX data packet */
pdc_packet_t g_plc_rx_packet;
/** PDC TX data packet */
pdc_packet_t g_plc_tx_packet;
/** Pointer to PDC register base */
Pdc *g_plc_pdc;

/** PLC busy indication */
static uint8_t suc_plc_is_busy;

/** PLC chip select config value */
#define HAL_PLC_PCS    spi_get_pcs(HAL_PLC_CS)

/** \brief PLC interrupt handlers */
/* @{ */
static void (*plc_handler)(void);
static void (*plc_tx_signalling_handler)(void);
static void (*plc_rx_signalling_handler)(void);
/* @} */

/**
 * \brief PLC interruption handler
 *
 * \param ul_id     Identifier
 * \param ul_mask   Mask
 *
 */
static void hal_plc_int_handler(uint32_t ul_id, uint32_t ul_mask)
{
	UNUSED(ul_id);
	UNUSED(ul_mask);
	if (plc_handler != NULL) {
		plc_handler();
	}

	/* Delete level interrupt */
	pio_get_interrupt_status(HAL_PLC_INT_PIO);
}

/**
 * \internal
 * \brief Initialize Proxy PLC controller.
 *
 * This function will change the system clock prescaler configuration to
 * match the parameters.
 *
 * \note The parameters to this function are device-specific.
 *
 */
static void _plc_if_config(void)
{
	uint32_t ul_cpuhz;
	uint8_t uc_div;

	ul_cpuhz = sysclk_get_peripheral_hz();
	uc_div = ul_cpuhz / gs_ul_plc_clock;

	if (ul_cpuhz % gs_ul_plc_clock) {
		uc_div++;
	}

	/* Enable SPI peripheral. */
	spi_enable_clock(HAL_PLC_SPI_MODULE);

	/* Reset SPI */
	spi_disable(HAL_PLC_SPI_MODULE);
	spi_reset(HAL_PLC_SPI_MODULE);

	/* Configure SPI */
	spi_set_master_mode(HAL_PLC_SPI_MODULE);
	spi_disable_mode_fault_detect(HAL_PLC_SPI_MODULE);
	spi_set_peripheral_chip_select_value(HAL_PLC_SPI_MODULE, HAL_PLC_PCS);
	spi_set_clock_polarity(HAL_PLC_SPI_MODULE, HAL_PLC_CS, HAL_PLC_POL);
	spi_set_clock_phase(HAL_PLC_SPI_MODULE, HAL_PLC_CS, HAL_PLC_PHA);
#ifdef HAL_ATPL360_INTERFACE
	spi_set_bits_per_transfer(HAL_PLC_SPI_MODULE, HAL_PLC_CS, SPI_CSR_BITS_16_BIT);
#else
	spi_set_bits_per_transfer(HAL_PLC_SPI_MODULE, HAL_PLC_CS, SPI_CSR_BITS_8_BIT);
#endif
	spi_set_fixed_peripheral_select(HAL_PLC_SPI_MODULE);
	spi_set_baudrate_div(HAL_PLC_SPI_MODULE, HAL_PLC_CS, uc_div);
	spi_set_transfer_delay(HAL_PLC_SPI_MODULE, HAL_PLC_CS, HAL_PLC_DLYBS, HAL_PLC_DLYBCT);
	spi_configure_cs_behavior(HAL_PLC_SPI_MODULE, HAL_PLC_CS, SPI_CS_RISE_NO_TX);

	/* Get board PLC PDC base address and enable receiver and transmitter. */
	g_plc_pdc = spi_get_pdc_base(HAL_PLC_SPI_MODULE);
	spi_enable(HAL_PLC_SPI_MODULE);
}

/**
 * \brief Transmit PLC command to Proxy PLC controller
 *
 *  \param uc_cmd           PLC command to send
 *  \param us_addr          Address where is the data to apply command
 *  \param us_len           Number of bytes in operation
 *  \param ptr_buf          Pointer to data buffer
 *  \param uc_bytes_rep     Number of repetitions(only use in HAL_PLC_CMD_WRITE_REP command)
 *
 * \retval true if there is no error
 * \retval false if there is an error
 */
int8_t hal_plc_cmd_op(uint8_t uc_cmd, uint16_t us_addr, uint16_t us_len, uint8_t *ptr_buf, uint8_t uc_bytes_rep)
{
	uint8_t *ptr_data_buf;
	uint16_t uc_num_tx_bytes;
	uint16_t us_data_len, us_data_len_acc;
	uint16_t us_address;
	bool b_is_int_disabled;

	/* set critical region */
	b_is_int_disabled = __get_PRIMASK();
	if (!b_is_int_disabled) {
		Disable_global_interrupt();
	}

	if (!suc_plc_is_busy) {
		suc_plc_is_busy = true;
		ptr_data_buf = ptr_buf;
		us_data_len = us_len;
		us_data_len_acc = 0;
		us_address = us_addr;

		while (us_data_len) {
			/* protection to data length */
			if (us_data_len > PDC_PPLC_BUFFER_SIZE) {
				us_data_len = PDC_PPLC_BUFFER_SIZE - 4;
			}

			uc_num_tx_bytes = us_data_len + 4;

			/* Configure PLC Tx buffer */
			gs_plc_tx_buffer[0] = uc_cmd;
			gs_plc_tx_buffer[1] = (uint8_t)(us_address >> 8);
			gs_plc_tx_buffer[2] = (uint8_t)(us_address);
			if (uc_cmd == HAL_PLC_CMD_WRITE_REP) {
				gs_plc_tx_buffer[3] = uc_bytes_rep;
			} else {
				gs_plc_tx_buffer[3] = 0;
			}

			/* Fill data */
			if (uc_cmd == HAL_PLC_CMD_READ) {
				memset(&gs_plc_tx_buffer[4], 0, us_data_len);
			} else {
				memcpy(&gs_plc_tx_buffer[4], ptr_data_buf, us_data_len);
			}

			/* Configure DMA channels */
			g_plc_rx_packet.ul_addr = (uint32_t)gs_plc_rx_buffer;
			g_plc_rx_packet.ul_size = uc_num_tx_bytes;
			pdc_rx_init(g_plc_pdc, &g_plc_rx_packet, NULL);

			g_plc_tx_packet.ul_addr = (uint32_t)gs_plc_tx_buffer;
			g_plc_tx_packet.ul_size = uc_num_tx_bytes;
			pdc_tx_init(g_plc_pdc, &g_plc_tx_packet, NULL);

			/* Enable the RX and TX PDC transfer requests */
			pdc_enable_transfer(g_plc_pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

			/* Waiting transfer done*/
			while ((spi_read_status(HAL_PLC_SPI_MODULE) & SPI_SR_RXBUFF) == 0) {
			}

			/* Disable the RX and TX PDC transfer requests */
			pdc_disable_transfer(g_plc_pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);

			/* copy rcv data */
			if (uc_cmd == HAL_PLC_CMD_READ) {
				memcpy(ptr_data_buf, &gs_plc_rx_buffer[4], us_data_len);
			}

			/* update buffer pointers */
			ptr_data_buf += us_data_len;
			us_address += us_data_len;
			/* update data length */
			us_data_len_acc += us_data_len;
			us_data_len = us_len - us_data_len_acc;
		}

		suc_plc_is_busy = false;

		/* end critical region */
		if (!b_is_int_disabled) {
			Enable_global_interrupt();
		}

		return true;
	} else {

		/* end critical region */
		if (!b_is_int_disabled) {
			Enable_global_interrupt();
		}

		return false;
	}
}

/**
 * \brief Reset internal PLC Modem.
 *
 */
void hal_plc_reset(void)
{
#if BOARD == SAM4CP16BMB
	/* Reset on ARST of modem PLC */
	ioport_set_pin_level(HAL_PLC_ARST_GPIO, HAL_PLC_ARST_ACTIVE_LEVEL);
	delay_ms(10);
	/* Clear ARST of modem PLC */
	ioport_set_pin_level(HAL_PLC_ARST_GPIO, HAL_PLC_ARST_INACTIVE_LEVEL);
#elif BOARD == ATPL230AMB
	/* Reset on ARST of modem PLC */
	ioport_set_pin_level(HAL_PLC_ARST_GPIO, HAL_PLC_ARST_ACTIVE_LEVEL);
	delay_ms(10);
	/* Clear ARST of modem PLC */
	ioport_set_pin_level(HAL_PLC_ARST_GPIO, HAL_PLC_ARST_INACTIVE_LEVEL);
#elif BOARD == KINTEX_FPGA
	/* Reset on RST of modem PLC */
	ioport_set_pin_dir(HAL_PLC_RST_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(HAL_PLC_RST_GPIO, HAL_PLC_RST_ACTIVE_LEVEL);
	delay_ms(10);
	/* Clear RST of modem PLC */
	ioport_set_pin_level(HAL_PLC_RST_GPIO, HAL_PLC_RST_INACTIVE_LEVEL);
#elif BOARD == ATPL360ASB
	/* Enable LDO line */
	ioport_set_pin_level(ATPL360_LDO_EN_GPIO, ATPL360_LDO_EN_ACTIVE_LEVEL);
	delay_ms(1);
	/* Reset on RST of modem PLC */
	ioport_set_pin_level(ATPL360_RESET_GPIO, ATPL360_RESET_ACTIVE_LEVEL);
	delay_ms(1);
	/* Clear RST of modem PLC */
	ioport_set_pin_level(ATPL360_RESET_GPIO, ATPL360_RESET_INACTIVE_LEVEL);
#elif (BOARD == ATPL360AMB) || (BOARD == ATPL360MB) || (BOARD == PL360G55CF_EK)
	/* Enable LDO line */
	ioport_set_pin_level(ATPL360_LDO_EN_GPIO, ATPL360_LDO_EN_ACTIVE_LEVEL);
	ioport_set_pin_dir(ATPL360_LDO_EN_GPIO, IOPORT_DIR_OUTPUT);
	delay_ms(1);
	/* Reset on RST of modem PLC */
	ioport_set_pin_level(ATPL360_RESET_GPIO, ATPL360_RESET_ACTIVE_LEVEL);
	delay_ms(1);
	/* Clear RST of modem PLC */
	ioport_set_pin_level(ATPL360_RESET_GPIO, ATPL360_RESET_INACTIVE_LEVEL);
#else
	/* Modify for customer board */;
#endif

	delay_ms(50);
}

/**
 * \brief Initialize PLC interface
 *
 */
void hal_plc_init(void)
{
	/* Init PLC handler */
	plc_handler = NULL;
	/* signalling handler must be set in the application. */
	plc_tx_signalling_handler = NULL;
	plc_rx_signalling_handler = NULL;

	/* Initialize PLC */
	_plc_if_config();

	/* plc free */
	suc_plc_is_busy = false;

	/* Configure PLC reset pins */
#if BOARD == SAM4CP16CMB
	ioport_set_pin_level(HAL_PLC_ARST_GPIO, HAL_PLC_ARST_INACTIVE_LEVEL);
	ioport_set_pin_dir(HAL_PLC_ARST_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(HAL_PLC_SRST_GPIO, HAL_PLC_SRST_INACTIVE_LEVEL);
	ioport_set_pin_dir(HAL_PLC_SRST_GPIO, IOPORT_DIR_OUTPUT);
#elif BOARD == ATPL230AMB
	ioport_set_pin_level(HAL_PLC_ARST_GPIO, HAL_PLC_ARST_INACTIVE_LEVEL);
	ioport_set_pin_dir(HAL_PLC_ARST_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(HAL_PLC_SRST_GPIO, HAL_PLC_SRST_INACTIVE_LEVEL);
	ioport_set_pin_dir(HAL_PLC_SRST_GPIO, IOPORT_DIR_OUTPUT);
#elif BOARD == KINTEX_FPGA
	ioport_set_pin_level(HAL_PLC_RST_GPIO, HAL_PLC_RST_INACTIVE_LEVEL);
	ioport_set_pin_dir(HAL_PLC_RST_GPIO, IOPORT_DIR_OUTPUT);
#elif BOARD == ATPL360ASB
	ioport_set_pin_level(ATPL360_RESET_GPIO, ATPL360_RESET_ACTIVE_LEVEL);
	ioport_set_pin_dir(ATPL360_RESET_GPIO, IOPORT_DIR_OUTPUT);
#elif (BOARD == ATPL360AMB) || (BOARD == ATPL360MB) || (BOARD == PL360G55CF_EK)
	ioport_set_pin_level(ATPL360_RESET_GPIO, ATPL360_RESET_ACTIVE_LEVEL);
	ioport_set_pin_dir(ATPL360_RESET_GPIO, IOPORT_DIR_OUTPUT);
#else
	/* Modify for customer board */;
#endif
}

/**
 * \brief Set an interrupt handler for the specified interrput source.
 */
void hal_plc_set_handler(void (*p_handler)(void))
{
	plc_handler = p_handler;

	/* Configure PLC interruption pin */
	ioport_set_pin_mode(HAL_PLC_INT_GPIO, IOPORT_MODE_PULLDOWN);
	ioport_set_pin_dir(HAL_PLC_INT_GPIO, IOPORT_DIR_INPUT);

	/* Configure PLC Interruption */
	pmc_enable_periph_clk(HAL_PLC_INT_ID);
	pio_handler_set(HAL_PLC_INT_PIO, HAL_PLC_INT_ID, HAL_PLC_INT_MASK, HAL_PLC_INT_ATTR, hal_plc_int_handler);

	NVIC_SetPriority(HAL_PLC_INT_IRQn, HAL_PLC_PRIO);
	NVIC_ClearPendingIRQ(HAL_PLC_INT_IRQn);
	NVIC_EnableIRQ(HAL_PLC_INT_IRQn);
#ifndef HAL_ATPL360_INTERFACE
	pio_enable_interrupt(HAL_PLC_INT_PIO, HAL_PLC_INT_MASK);
#endif
}

/**
 * \brief Set a signalling handler for the PLC transmission.
 */
void hal_plc_set_tx_signalling_handler(void (*p_handler)(void))
{
	plc_tx_signalling_handler = p_handler;
}

/**
 * \brief Set a signalling handler for the PLC reception.
 */
void hal_plc_set_rx_signalling_handler(void (*p_handler)(void))
{
	plc_rx_signalling_handler = p_handler;
}

/**
 * \brief Function called in PLC TX event
 */
void hal_plc_tx_signal(void)
{
	if (plc_tx_signalling_handler) {
		plc_tx_signalling_handler();
	}
}

/**
 * \brief Function called in PLC RX event
 */
void hal_plc_rx_signal(void)
{
	if (plc_rx_signalling_handler) {
		plc_rx_signalling_handler();
	}
}

#ifdef HAL_ATPL360_INTERFACE
bool hal_plc_send_boot_cmd(uint16_t us_cmd, uint32_t ul_addr, uint32_t ul_data_len, uint8_t *puc_data_buf, uint8_t *puc_data_read)
{
	uint8_t *puc_tx_buf;
	uint32_t ul_spi_busy_cnt;
	uint16_t us_tx_size;

	/* Waiting transfer done*/
	ul_spi_busy_cnt = 0;
	while ((spi_read_status(HAL_PLC_SPI_MODULE) & SPI_SR_RXBUFF) == 0) {
		ul_spi_busy_cnt++;
		if (ul_spi_busy_cnt > 5000000) {
			return false;
		}
	}

	NVIC_DisableIRQ(HAL_PLC_INT_IRQn);

	/* Disable the RX and TX PDC transfer requests */
	pdc_disable_transfer(g_plc_pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);

	/* Set 8 bits per transfer */
	spi_set_bits_per_transfer(HAL_PLC_SPI_MODULE, HAL_PLC_CS, SPI_CSR_BITS_8_BIT);

	/* Configure Tx buffer */
	puc_tx_buf = gs_plc_tx_buffer;

	memcpy(puc_tx_buf, &ul_addr, sizeof(uint32_t));
	puc_tx_buf +=  sizeof(uint32_t);
	memcpy(puc_tx_buf, &us_cmd, sizeof(uint16_t));
	puc_tx_buf +=  sizeof(uint16_t);

	memcpy(puc_tx_buf, puc_data_buf, ul_data_len);

	puc_tx_buf += ul_data_len;

	us_tx_size = puc_tx_buf - gs_plc_tx_buffer;

	/* Configure DMA channels */
	g_plc_rx_packet.ul_addr = (uint32_t)gs_plc_rx_buffer;
	g_plc_rx_packet.ul_size = us_tx_size;
	pdc_rx_init(g_plc_pdc, &g_plc_rx_packet, NULL);

	g_plc_tx_packet.ul_addr = (uint32_t)gs_plc_tx_buffer;
	g_plc_tx_packet.ul_size = us_tx_size;
	pdc_tx_init(g_plc_pdc, &g_plc_tx_packet, NULL);

	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(g_plc_pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

	/* Waiting transfer done and read */
	if (puc_data_read) {
		/* while(pdc_read_tx_counter(g_pdc) > 0); */
		ul_spi_busy_cnt = 0;
		while ((spi_read_status(HAL_PLC_SPI_MODULE) & SPI_SR_RXBUFF) == 0) {
			ul_spi_busy_cnt++;
			if (ul_spi_busy_cnt > 5000000) {
				/* Enable PLC interrupt(); */
				NVIC_EnableIRQ(HAL_PLC_INT_IRQn);
				return false;
			}
		}

		memcpy(puc_data_read, &gs_plc_rx_buffer[6], ul_data_len);
	}

	/* Enable PLC interrupt(); */
	NVIC_EnableIRQ(HAL_PLC_INT_IRQn);

	return true;
}

bool hal_plc_send_wrrd_cmd(uint8_t uc_cmd, void *px_spi_data, void *px_spi_status_info)
{
	uint8_t *puc_tx_buf;
	uint32_t ul_spi_busy_cnt;
	uint16_t us_tx_size, us_tx_hdr_size;
	uint16_t us_len_wr_rd;
	spi_data_t *px_data;
	spi_status_info_t *px_status_info;

	px_data = (spi_data_t *)px_spi_data;
	px_status_info = (spi_status_info_t *)px_spi_status_info;

	us_len_wr_rd = (((px_data->us_len + 1) / 2) & HAL_PLC_LEN_MASK) | (uc_cmd << HAL_PLC_WR_RD_POS);

	/* Check length */
	if (!us_len_wr_rd) {
		return false;
	}

	/* Waiting transfer done while(pdc_read_tx_counter(g_pdc) > 0); */
	ul_spi_busy_cnt = 0;
	while ((spi_read_status(HAL_PLC_SPI_MODULE) & SPI_SR_RXBUFF) == 0) {
		ul_spi_busy_cnt++;
		if (ul_spi_busy_cnt > 5000000) {
			return false;
		}
	}

	NVIC_DisableIRQ(HAL_PLC_INT_IRQn);

	/* Disable the RX and TX PDC transfer requests */
	pdc_disable_transfer(g_plc_pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);

	/* Set 16 bits per transfer */
	spi_set_bits_per_transfer(HAL_PLC_SPI_MODULE, HAL_PLC_CS, SPI_CSR_BITS_16_BIT);

	/** Configure PPLC Tx buffer **/
	puc_tx_buf = gs_plc_tx_buffer;
	/* Address */
	*puc_tx_buf++ = (uint8_t)(px_data->us_address);
	*puc_tx_buf++ = (uint8_t)(px_data->us_address >> 8);
	/* Length & read/write */
	*puc_tx_buf++ = (uint8_t)(us_len_wr_rd);
	*puc_tx_buf++ = (uint8_t)(us_len_wr_rd >> 8);

	us_tx_hdr_size = puc_tx_buf - gs_plc_tx_buffer;

	if (uc_cmd == HAL_PLC_CMD_WRITE) {
		memcpy(puc_tx_buf, px_data->puc_data_buf, px_data->us_len);
	} else {
		memset(puc_tx_buf, 0, px_data->us_len);
	}

	puc_tx_buf += px_data->us_len;

	us_tx_size = puc_tx_buf - gs_plc_tx_buffer;
	if (us_tx_size % 2) {
		*puc_tx_buf++ = 0;
		us_tx_size++;
	}

	/* Configure DMA channels */
	g_plc_rx_packet.ul_addr = (uint32_t)gs_plc_rx_buffer;
	g_plc_rx_packet.ul_size = us_tx_size / 2;
	pdc_rx_init(g_plc_pdc, &g_plc_rx_packet, NULL);

	g_plc_tx_packet.ul_addr = (uint32_t)gs_plc_tx_buffer;
	g_plc_tx_packet.ul_size = us_tx_size / 2;
	pdc_tx_init(g_plc_pdc, &g_plc_tx_packet, NULL);

	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(g_plc_pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

	/* Waiting transfer done*/
	/* while(pdc_read_tx_counter(g_pdc) > 0); */
	ul_spi_busy_cnt = 0;
	while ((spi_read_status(HAL_PLC_SPI_MODULE) & SPI_SR_RXBUFF) == 0) {
		ul_spi_busy_cnt++;
		if (ul_spi_busy_cnt > 5000000) {
			/* Enable PLC interrupt(); */
			NVIC_EnableIRQ(HAL_PLC_INT_IRQn);
			return false;
		}
	}

	if (uc_cmd == HAL_PLC_CMD_READ) {
		memcpy(px_data->puc_data_buf, &gs_plc_rx_buffer[us_tx_hdr_size], px_data->us_len);
	}

	px_status_info->us_header_id = HAL_PLC_GET_ID_HEADER(gs_plc_rx_buffer[0], gs_plc_rx_buffer[1]);
	if (HAL_PLC_CHECK_ID_BOOT_HEADER(px_status_info->us_header_id)) {
		px_status_info->ul_flags = HAL_PLC_GET_FLAGS_FROM_BOOT(gs_plc_rx_buffer[0], gs_plc_rx_buffer[2], gs_plc_rx_buffer[3]);
	} else if (HAL_PLC_CHECK_ID_CORTEX_HEADER(px_status_info->us_header_id)) {
		px_status_info->ul_flags = HAL_PLC_GET_FLAGS_FROM_CORTEX(gs_plc_rx_buffer[2], gs_plc_rx_buffer[3]);
	} else {
		px_status_info->ul_flags = 0;
	}

	/* Enable PLC interrupt(); */
	NVIC_EnableIRQ(HAL_PLC_INT_IRQn);

	return true;
}

void hal_plc_enable_interrupt(bool enable)
{
	if (enable) {
		pio_enable_interrupt(HAL_PLC_INT_PIO, HAL_PLC_INT_MASK);
	} else {
		pio_disable_interrupt(HAL_PLC_INT_PIO, HAL_PLC_INT_MASK);
	}
}

void hal_plc_delay(uint8_t uc_tref, uint32_t ul_delay)
{
	if (uc_tref == HAL_TREF_SEC) {
		delay_s(ul_delay);
	} else if (uc_tref == HAL_TREF_MS) {
		delay_ms(ul_delay);
	} else if (uc_tref == HAL_TREF_US) {
		delay_us(ul_delay);
	}
}

bool hal_plc_get_cd(void)
{
	return (!ioport_get_pin_level(ATPL360_CD_EN_GPIO));
}

#endif

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */
