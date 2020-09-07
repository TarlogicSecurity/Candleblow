/*
 * uart.c: UART configuration for SPIP protocol
 *
 * (c) 2019 Tarlogic Security S.L. - All rights reserved
 *
 * Company confidential. Any unauthorized use, disclosure, reproduction or
 * distribution of this file is strictly prohibited.
 */

#include "spip.h"
#include "console.h"
#include <led.h>

#include <string.h>

static BOOL
spip_uart_write_byte(void *unused, uint8_t c)
{
  console_putchar((char) c);

  return TRUE;
}

static BOOL
spip_uart_read_byte(void *unused, uint8_t *c)
{
  *c = console_getchar();
  return TRUE;
}

BOOL
spip_uart_interface_init(spip_iface_t *iface)
{
  iface->write_byte = spip_uart_write_byte;
  iface->read_byte  = spip_uart_read_byte;
  
  return TRUE;  
}
