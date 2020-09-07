/*
 * boot.c: Hardware initialization
 *
 * (c) 2019 Tarlogic Security S.L. - All rights reserved
 *
 * Company confidential. Any unauthorized use, disclosure, reproduction or
 * distribution of this file is strictly prohibited.
 */



#ifndef BOOT_H_
#define BOOT_H_

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Atmel boards includes. */
#include "board.h"

/* Atmel library includes. */
#include "asf.h"
#include "hal_private.h"
#include "math.h"

/* PHY includes */
#include "atpl360.h"
#include "conf_atpl360.h"

/* Example Includes */
#include "conf_app_example.h"

void tx_msg_init(void);
void hang(void);

#endif /* BOOT_H_ */