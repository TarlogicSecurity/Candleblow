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

#ifndef _USB_WRP_H_
#define _USB_WRP_H_

#include "usb_protocol_cdc.h"
#include "wdt.h"

//#define USB_WRP_DEBUG


/*! \brief Opens the communication port
 * This is called by CDC interface when USB Host enable it.
 *
 * \retval true if cdc startup is successfully done
 */
bool usb_wrp_cdc_enable(uint8_t port);

/*! \brief Closes the communication port
 * This is called by CDC interface when USB Host disable it.
 */
void usb_wrp_cdc_disable(uint8_t port);

/*! \brief Manages the leds behaviors
 * Called when a start of frame is received on USB line each 1ms.
 */
void usb_wrp_cdc_sof_action(void);

/*! \brief Enters the application in low power mode
 * Callback called when USB host sets USB line in suspend state
 */
void usb_wrp_cdc_suspend_action(void);

/*! \brief Turn on a led to notify active mode
 * Called when the USB line is resumed from the suspend state
 */
void usb_wrp_cdc_resume_action(void);

/*! \brief Save new DTR state to change led behavior.
 * The DTR notify that the terminal have open or close the communication port.
 */
void usb_wrp_cdc_set_dtr(uint8_t port, bool b_enable);

/*! \brief Called by CDC interface
 * Callback running when CDC device have received data
 */
void usb_wrp_cdc_rx_notify(uint8_t port);

/*! \brief Configures communication line
 *
 * \param cfg      line configuration
 */
void usb_wrp_cdc_set_coding_ext(uint8_t port, usb_cdc_line_coding_t * cfg);

void usb_wrp_udc_start(void);
uint16_t usb_wrp_udc_read_buf(uint8_t *puc_buff, uint16_t us_max_len);
uint16_t usb_wrp_udc_write_buf(uint8_t *puc_buff, uint16_t us_len);
bool usb_wrp_udc_is_tx_ready(void);

/**
 * \brief Waits until a character is received, and returns it.
 *
 * \param p_usart   Base address of the UDC instance.
 * \param data   Data to read
 *
 */
static inline void usb_wrp_udc_getchar(Udp* p_udp, uint8_t *data)
{
	(void)p_udp;
	while (!usb_wrp_udc_read_buf(data, 1)) {
		/* Reset watchdog */
		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;
	}
}

/**
 * \brief Sends a character with the UDC.
 *
 * \param p_usart   Base address of the UDP instance.
 * \param c       Character to write.
 *
 * \return Status.
 *   \retval 1  The character was written.
 *   \retval 0  The function timed out before the USART transmitter became
 * ready to send.
 */
static inline int usb_wrp_udc_putchar(Udp* p_udp, const uint8_t c)
{
	(void)p_udp;
	while (!usb_wrp_udc_write_buf((uint8_t *)&c, 1)) {
		/* Reset watchdog */
		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;
	}
	return 1;
}


#endif // _USB_WRP_H_
