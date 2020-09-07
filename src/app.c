/*
 * app.c: Application loop
 *
 * (c) 2019 Tarlogic Security S.L. - All rights reserved
 *
 * Company confidential. Any unauthorized use, disclosure, reproduction or
 * distribution of this file is strictly prohibited.
 */

#include "app.h"

#include <console.h>
#include <phy.h>
#include <ctype.h>
#include <boot.h>
#include <lcd.h>
#include <spip.h>

static spip_iface_t si;

static void
on_phy_data(void *data, const uint8_t *buffer, size_t size)
{
  /* Back to master */
  spip_iface_write_frame(&si, buffer, size);
}

void
app_entry(struct tx_task *tt)
{ 
  lcd_puts(0, "UART...");
  lcd_printf(1, "SPIP@%ld bd", CONSOLE_BAUDRATE);
  
  spip_uart_interface_init(&si);
  
  lcd_puts(0, "PHY RX...");
  phy_set_rx_handler(on_phy_data, NULL);
  
  LED_Off(LED0);
  LED_Off(LED1);
  
  lcd_puts(0, "Probe ready! :)");
  spip_iface_board_loop(&si, tt);
  lcd_puts(0, "HANG!!");
  hang();
}