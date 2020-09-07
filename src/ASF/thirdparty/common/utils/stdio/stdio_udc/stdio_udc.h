/**
 *
 * \file
 *
 * \brief Common Standard I/O UDC Management.
 *
 * This file defines a useful set of functions for the Stdio UDC interface on SAM devices.
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
 ******************************************************************************/
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */


#ifndef _STDIO_UDC_H_
#define _STDIO_UDC_H_

/**
 * \defgroup group_common_utils_stdio_stdio_serial Standard UDC I/O (stdio)
 * \ingroup group_common_utils_stdio
 *
 * Common standard UDC I/O management driver that
 * implements a stdio UDC interface on SAM devices.
 *
 * \{
 */

#include <stdio.h>
#include "compiler.h"
#include "usb_wrp.h"

//! Pointer to the base of the USART module instance to use for stdio.
extern volatile void *volatile stdio_base;
//! Pointer to the external low level write function.
extern int (*ptr_put)(void volatile*, char);

//! Pointer to the external low level read function.
extern void (*ptr_get)(void volatile*, char*);

/*! \brief Initializes the stdio in UDC Mode.
 *
 * \param udp       Base address of the UDP instance.
 *
 */
static inline void stdio_udc_init(volatile void *udp)
{
	stdio_base = (void *)udp;
	ptr_put = (int (*)(void volatile*,char))&usb_wrp_udc_putchar;
	ptr_get = (void (*)(void volatile*,char*))&usb_wrp_udc_getchar;

	usb_wrp_udc_start();

# if defined(__GNUC__)
	// For AVR32 and SAM GCC
	// Specify that stdout and stdin should not be buffered.
	setbuf(stdout, NULL);
	setbuf(stdin, NULL);
	// Note: Already the case in IAR's Normal DLIB default configuration
	// and AVR GCC library:
	// - //printf() emits one character at a time.
	// - getchar() requests only 1 byte to exit.
# endif
}

/**
 * \}
 */

#endif  // _STDIO_UDC_H_
