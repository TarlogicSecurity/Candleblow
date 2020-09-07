/*
 * tx-task.h: TX Task
 *
 * (c) 2019 Tarlogic Security S.L. - All rights reserved
 *
 * Company confidential. Any unauthorized use, disclosure, reproduction or
 * distribution of this file is strictly prohibited.
 */


#ifndef TX_TASK_H_
#define TX_TASK_H_

#include <FreeRTOS.h>

#include <queue.h>
#include <semphr.h>
#include <defs.h>
#include <task.h>
#include <atpl360_comm.h>

#define BOARD_TX_MESSAGE_COUNT 10
#define BOARD_TX_MESSAGE_SIZE  1020

#define TASK_APP_PHY_PRIO  (tskIDLE_PRIORITY + 1)
#define TASK_APP_PHY_STACK (configMINIMAL_STACK_SIZE * 5)

struct tx_frame {
  size_t size;
  uint8_t data[BOARD_TX_MESSAGE_SIZE];
};

struct tx_task {
  xQueueHandle h_tx_queue;
  xTaskHandle  h_tx_task;
  tx_msg_t     template;
};

typedef struct tx_task tx_task_t;

BOOL tx_task_push_frame(tx_task_t *task, const void *data, size_t size);
BOOL tx_task_init(tx_task_t *task);

#endif /* TX-TASK_H_ */