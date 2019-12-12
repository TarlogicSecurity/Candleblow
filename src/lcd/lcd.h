/*
 * lcd.h: LCD API
 *
 * (c) 2019 Tarlogic Security S.L. - All rights reserved
 *
 * Company confidential. Any unauthorized use, disclosure, reproduction or
 * distribution of this file is strictly prohibited.
 */


#ifndef LCD_LCD_H
#define LCD_LCD_H

#include <defs.h>

void lcd_init(void);
void lcd_puts(int y, const char *msg);
void lcd_printf(int y, const char *fmt, ...) __attribute__ ((format (gnu_printf, 2, 3)));

#endif /* CONSOLE_CONSOLE_H */