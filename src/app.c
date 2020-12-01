/*
 * app.c: main application loop
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
