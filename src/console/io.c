/*
 * io.c: Console I/O
 *
 * (c) 2019 Tarlogic Security S.L. - All rights reserved
 *
 * Company confidential. Any unauthorized use, disclosure, reproduction or
 * distribution of this file is strictly prohibited.
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

BOOL
console_init(void)
{
  uint32_t waitcnt;
  
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
  
  return TRUE;
}

int
console_getchar(void)
{
  while (!uart_is_rx_ready((Uart *) CONF_UART))
    taskYIELD();
    
  return getchar();
}

void
console_putchar(uint8_t c)
{
  while (uart_write((Uart *) CONF_UART, c))
    taskYIELD();
}
