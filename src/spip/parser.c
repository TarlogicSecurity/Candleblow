/*
 * loop.c: Extract frames from SPIP
 *
 * (c) 2019 Tarlogic Security S.L. - All rights reserved
 *
 * Company confidential. Any unauthorized use, disclosure, reproduction or
 * distribution of this file is strictly prohibited.
 */

#include "spip.h"
#include <led.h>
#include <lcd.h>

static BOOL 
on_pdu(spip_iface_t *iface, struct spip_pdu *pdu, void *userdata)
{
  switch (pdu->command) {
    case SPIP_COMMAND_FRAME:
      /* TODO: Inject frame */
      break;
      
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
spip_iface_board_loop(spip_iface_t *iface)
{
  return spip_iface_loop(iface, on_pdu, NULL);
}
