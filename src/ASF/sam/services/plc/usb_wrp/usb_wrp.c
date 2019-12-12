/**
 * \file
 *
 * \brief USB wrapper functions
 *
 * Copyright (c) 2018 Atmel Corporation. All rights reserved.
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

/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include "string.h"
#include "led.h"
#include "udi_cdc.h"
#include "udc.h"
#include "usb_wrp.h"

static volatile bool sb_cdc_dtr = false;

#ifdef USB_WRP_DEBUG
#define USB_WRP_DBG(x)  printf x
#else
#define USB_WRP_DBG(x)
#endif

#define USB_BUF_RX_SIZE        1500

typedef struct x_usb_buffer {
	uint8_t *puc_wr_data;  /* Pointer to write data in buffer */
	uint8_t *puc_rd_data;  /* Pointer to read data from buffer */
	uint8_t *puc_end;
	uint16_t us_size;      /* Size of data pending to be read */
	uint8_t puc_data[USB_BUF_RX_SIZE];
} x_usb_buffer;

static x_usb_buffer sx_rx_data_buf;

static void _init_usb_buffer(x_usb_buffer *px_buff)
{
	px_buff->us_size = 0;
	px_buff->puc_rd_data = px_buff->puc_data;
	px_buff->puc_wr_data = px_buff->puc_data;
	px_buff->puc_end = px_buff->puc_data + sizeof(px_buff->puc_data);
	memset(px_buff->puc_data, 0, sizeof(px_buff->puc_data));
}

static void _upd_rd_info(x_usb_buffer *x_buf, uint16_t us_len)
{
	int i_len_chk;

	if ((x_buf->puc_rd_data + us_len) >= x_buf->puc_end) {
		/* buffer overflow */
		x_buf->puc_rd_data = x_buf->puc_data + (us_len - (x_buf->puc_end - x_buf->puc_rd_data));
	} else {
		x_buf->puc_rd_data += us_len;
	}

	i_len_chk = x_buf->us_size - us_len;
	if (i_len_chk < 0) {
		x_buf->us_size = 0;
		x_buf->us_size = 0;

		/* catch error */
		while (1) {
		}
	} else {
		x_buf->us_size = i_len_chk;
	}
}

static void _upd_wr_info(x_usb_buffer *x_buf, uint16_t us_len)
{
	if ((x_buf->puc_wr_data + us_len) >= x_buf->puc_end) {
		/* buffer overflow */
		x_buf->puc_wr_data = x_buf->puc_data + (us_len - (x_buf->puc_end - x_buf->puc_wr_data));
	} else {
		x_buf->puc_wr_data += us_len;
	}

	x_buf->us_size += us_len;

	if (x_buf->us_size > USB_BUF_RX_SIZE) {
		x_buf->us_size = USB_BUF_RX_SIZE;

		/* catch error */
		while (1) {
		}
	}
}

void usb_wrp_cdc_suspend_action(void)
{
	USB_WRP_DBG(("usb_wrp_cdc_suspend_action\r\n"));
}

void usb_wrp_cdc_resume_action(void)
{
	USB_WRP_DBG(("usb_wrp_cdc_resume_action\r\n"));
}

void usb_wrp_cdc_sof_action(void)
{
	/* In this function, user may perform actions */
	/* when USB packets are exchanged */
	
	/* Example of use: */
	/* uint16_t framenumber;

	framenumber = udd_get_frame_number();
	if (0 == framenumber) {
		LED_On(LED1);
	} else if (1000 == framenumber) {
		LED_Off(LED1);
	} */
}

bool usb_wrp_cdc_enable(uint8_t port)
{
	/* Open communication */
	UNUSED(port);
	USB_WRP_DBG(("usb_wrp_cdc_enable\r\n"));
	return true;
}

void usb_wrp_cdc_disable(uint8_t port)
{
	/* Close communication */
	UNUSED(port);
	USB_WRP_DBG(("usb_wrp_cdc_disable\r\n"));
}

void usb_wrp_cdc_set_dtr(uint8_t port, bool b_enable)
{
	UNUSED(port);
	if (b_enable) {
		/* Host terminal has open COM */
		sb_cdc_dtr = true;
		USB_WRP_DBG(("usb_wrp_cdc_set_dtr 1\r\n"));
	} else {
		/* Host terminal has close COM */
		sb_cdc_dtr = false;
		USB_WRP_DBG(("usb_wrp_cdc_set_dtr 0\r\n"));
	}
}

void usb_wrp_cdc_rx_notify(uint8_t port)
{
	UNUSED(port);

	uint32_t us_len = udi_cdc_get_nb_received_data();

	if (us_len == 0) {
		/* No Data to read */
		return;
	}

	if (us_len > (uint32_t)(USB_BUF_RX_SIZE - sx_rx_data_buf.us_size)) {
		/* There is no free space */
		return;
	}

	if (us_len > (uint32_t)(sx_rx_data_buf.puc_end - sx_rx_data_buf.puc_wr_data)) {
		uint16_t us_size1, us_size2;

		/* copy to circular buffer in two fragments */
		us_size1 = sx_rx_data_buf.puc_end - sx_rx_data_buf.puc_wr_data;
		us_size2 = us_len - us_size1;
		udi_cdc_read_buf(sx_rx_data_buf.puc_wr_data, us_size1);
		udi_cdc_read_buf(sx_rx_data_buf.puc_data, us_size2);
	} else {
		/* copy to circular buffer in only one fragment */
		udi_cdc_read_buf(sx_rx_data_buf.puc_wr_data, us_len);
	}

	USB_WRP_DBG(("usb_wrp_cdc_rx_notify %u\r\n", us_len));

	/* update WR pointer */
	_upd_wr_info(&sx_rx_data_buf, us_len);
}

void usb_wrp_cdc_set_coding_ext(uint8_t port, usb_cdc_line_coding_t *cfg)
{
	UNUSED(port);
	UNUSED(cfg);

	/*	USB_WRP_DBG(("usb_wrp_cdc_set_coding_ext\r\n")); */
}

void usb_wrp_udc_start(void)
{
	_init_usb_buffer(&sx_rx_data_buf);

	/* Start USB stack to authorize VBus monitoring */
	udc_start();
}

uint16_t usb_wrp_udc_read_buf(uint8_t *puc_buff, uint16_t us_max_len)
{
	if (sx_rx_data_buf.us_size) {
		uint16_t us_len;

		/* check max len */
		if (sx_rx_data_buf.us_size > us_max_len) {
			us_len = us_max_len;
		} else {
			us_len = sx_rx_data_buf.us_size;
		}

		/* Copy data */
		if (us_len > (sx_rx_data_buf.puc_end - sx_rx_data_buf.puc_rd_data)) {
			uint16_t us_size1, us_size2;

			/* Extract data in two fragments */
			us_size1 = sx_rx_data_buf.puc_end - sx_rx_data_buf.puc_rd_data;
			us_size2 = us_len - us_size1;
			memcpy(puc_buff, sx_rx_data_buf.puc_rd_data, us_size1);
			memcpy(puc_buff + us_size1, sx_rx_data_buf.puc_data, us_size2);
		} else {
			/* Extract data in only one fragment */
			memcpy(puc_buff, sx_rx_data_buf.puc_rd_data, us_len);
		}

		/* update RD pointer */
		_upd_rd_info(&sx_rx_data_buf, us_len);

		return us_len;
	} else {
		return 0;
	}
}

uint16_t usb_wrp_udc_write_buf(uint8_t *puc_buff, uint16_t us_len)
{
	uint16_t us_len_remaining;
	
	if (sb_cdc_dtr == false) {
		return us_len;
	}

	if (!udi_cdc_is_tx_ready()) {
		/* Fifo full */
		udi_cdc_signal_overrun();
		us_len_remaining = us_len;
		USB_WRP_DBG(("usb_wrp_udc_write_buf : Fifo full\r\n"));
	} else {
		us_len_remaining = udi_cdc_write_buf(puc_buff, us_len);
		USB_WRP_DBG(("usb_wrp_udc_write_buf %u %u\r\n", us_len_remaining, us_len));
	}

	return (us_len - us_len_remaining);
}

bool usb_wrp_udc_is_tx_ready(void)
{
	/* USB_WRP_DBG(("usb_wrp_udc_is_tx_ready\r\n")); */

	return udi_cdc_is_tx_ready();
}
