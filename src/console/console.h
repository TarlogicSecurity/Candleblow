/*
 * console.h
 *
 * Created: 18/11/2019 11:38:58
 *  Author: Gonzalo.Carracedo
 */ 


#ifndef CONSOLE_CONSOLE_H
#define CONSOLE_CONSOLE_H

#include <defs.h>

#define CONSOLE_BAUDRATE 921600UL

BOOL console_init(void);
int  console_getchar(void);
void console_putchar(uint8_t);
void console_clear(void);
void console_gotoxy(int x, int y);

#endif /* CONSOLE_CONSOLE_H */