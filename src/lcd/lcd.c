/*
 * lcd.c: LCD API
 *
 * (c) 2019 Tarlogic Security S.L. - All rights reserved
 *
 * Company confidential. Any unauthorized use, disclosure, reproduction or
 * distribution of this file is strictly prohibited.
 */

#include <lcd.h>
#include <boot.h>
#include <stdarg.h>

static void
lcd_clear(int y)
{
  c0216CiZ_set_cursor(
  y == 0 
    ? C0216CiZ_LINE_UP 
    : C0216CiZ_LINE_DOWN, 
  0);
  c0216CiZ_show("                ");
}

void
lcd_printf(int y, const char *fmt, ...)
{
  va_list ap;
  char fullmsg[17];
  
  va_start(ap, fmt);
  
  vsnprintf(fullmsg, 17, fmt, ap);
  fullmsg[16] = '\0';

  lcd_puts(y, fullmsg);
  
  va_end(ap);
}
void
lcd_puts(int y, const char *message)
{
  lcd_clear(y);
  
  c0216CiZ_set_cursor(
    y == 0
      ? C0216CiZ_LINE_UP
      : C0216CiZ_LINE_DOWN,
    0);
    
  c0216CiZ_show(message);
}

void
lcd_init(void)
{
  if (c0216CiZ_init() != STATUS_OK) {
    printf("Failed to initialize LCD!\n");
    hang();
  }
  
  lcd_clear(0);
  lcd_clear(1);
}