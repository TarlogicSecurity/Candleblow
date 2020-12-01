/*
 * lcd.c: LCD API
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
