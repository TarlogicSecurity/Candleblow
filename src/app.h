/*
 * app.h: Generic application include file
 *
 * (c) 2019 Tarlogic Security S.L. - All rights reserved
 *
 * Company confidential. Any unauthorized use, disclosure, reproduction or
 * distribution of this file is strictly prohibited.
 */


#ifndef APP_H_
#define APP_H_

#include <defs.h>

struct tx_task;

void app_entry(struct tx_task *);

#endif /* APP_H_ */