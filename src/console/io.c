/*
 * io.c: UART-based I/O
 *
 * Copyright (c) 2020, Tarlogic Security SL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of copyright holders nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS”
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <console.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Atmel boards includes. */
#include "board.h"

/* Atmel library includes. */
#include "asf.h"
#include "hal_private.h"
#include "math.h"

#define IO_READ_DELAY_MS 10
#define IO_WRITE_DELAY_MS 1

static int
dummy_putchar(volatile void *unused, char c)
{
  return 1;  
}

static void
dummy_getchar(volatile void *unused, char *c)
{
  *c = '\0';
}

BOOL
console_init(void)
{
  #if 0
  const usart_serial_options_t uart_serial_options = {
    .baudrate = CONSOLE_BAUDRATE,
#ifdef CONF_UART_CHAR_LENGTH
    .charlength = CONF_UART_CHAR_LENGTH,
#endif
    .paritytype = CONF_UART_PARITY,
#ifdef CONF_UART_STOP_BITS
    .stopbits = CONF_UART_STOP_BITS,
#endif
  };

  /* Configure console UART. */
  sysclk_enable_peripheral_clock(CONF_UART_ID); 
  stdio_serial_init(CONF_UART, &uart_serial_options);  
  #endif
  
  ptr_put = dummy_putchar;
  ptr_get = dummy_getchar;
  
  hal_uart_open(0, CONSOLE_BAUDRATE);
  
  return TRUE;
}

int
console_getchar(void)
{
  uint8_t c;
  
  while (hal_uart_read(0, &c, 1) == 0)
    taskYIELD();
  
  return c;
}

void
console_putchar(uint8_t c)
{
  while (!hal_uart_is_free(0))
    taskYIELD();
  
  hal_uart_write(0, &c, 1);
}
