/*
 * tx-task.h: Transmission task
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
