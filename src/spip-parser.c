/*
 * spip-parser.c: parse SPIP commands
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

#include "spip.h"
#include <led.h>
#include <lcd.h>
#include <tx-task.h>

static BOOL 
on_pdu(spip_iface_t *iface, struct spip_pdu *pdu, void *userdata)
{
  tx_task_t *tt = (tx_task_t *) userdata;
  
  switch (pdu->command) {
#ifdef SPIP_ATTACK_MODE /* We are not supporting this yet */
    case SPIP_COMMAND_FRAME:
      tx_task_push_frame(tt, pdu->data, pdu->size);
      break;
#endif /* SPIP_ATTACK_MODE */
      
    case SPIP_COMMAND_LCD:
      if (pdu->size == 18) {
        pdu->data[17] = 0;
        lcd_puts(pdu->data[0], (const char *) pdu->data + 1);
      }
      break;
      
    case SPIP_COMMAND_LED_SET_MASK:
      if (pdu->size == 1) {
        if (pdu->data[0] & 1)
          LED_On(LED0);
        else
          LED_Off(LED0);
          
        if (pdu->data[0] & 2)
          LED_On(LED1);
        else
          LED_Off(LED1);
      }
      break;
      
    case SPIP_COMMAND_LED_TOGGLE_MASK:
      if (pdu->size == 1) {
        if (pdu->data[0] & 1)
          LED_Toggle(LED0);
                
        if (pdu->data[0] & 2)
          LED_Toggle(LED1);
      }
      break;
  }
  
  return TRUE;
}

BOOL
spip_iface_board_loop(spip_iface_t *iface, tx_task_t *tt)
{
  return spip_iface_loop(iface, on_pdu, tt);
}
