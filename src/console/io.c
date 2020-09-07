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
