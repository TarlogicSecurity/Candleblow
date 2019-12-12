/*
 * api.c: High-level operations
 *
 * (c) 2019 Tarlogic Security S.L. - All rights reserved
 *
 * Company confidential. Any unauthorized use, disclosure, reproduction or
 * distribution of this file is strictly prohibited.
 */

#include <console.h>

void
console_clear(void)
{
  printf("\033[1;1H\033[2J");
  fflush(stdout);
}  

void
console_gotoxy(int x, int y)
{
  printf("\033[%d;%dH", y + 1, x + 1);
  fflush(stdout);
}