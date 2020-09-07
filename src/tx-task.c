/*
 * tx-task.c: TX Task
 *
 * (c) 2019 Tarlogic Security S.L. - All rights reserved
 *
 * Company confidential. Any unauthorized use, disclosure, reproduction or
 * distribution of this file is strictly prohibited.
 */

#include <led.h>
#include <lcd.h>
#include <tx-task.h>

#include <boot.h>
#include <phy.h>

static BOOL             g_tx_task_prepared = FALSE;
static struct tx_frame  g_tx_frame_alloc[BOARD_TX_MESSAGE_COUNT];
static uint32_t         g_tx_frame_mask = 0;
static xSemaphoreHandle g_tx_frame_sem;

static struct tx_frame *
tx_frame_take(void)
{
  uint8_t first_free;
  struct tx_frame *ret = NULL;
  
  xSemaphoreTake(g_tx_frame_sem, portMAX_DELAY);
  
  /*
   *  Mask: 00000000011111011
   * ~Mask: 11111111100000100
   * CTZ:   2 (first free)
   */
  
  first_free = ctz(~g_tx_frame_mask);
  
  if (first_free >= BOARD_TX_MESSAGE_COUNT)
    goto fail;
  
  /* Retrieve address and update bitmap */
  ret = g_tx_frame_alloc + first_free;
  ret->size = 0;
  
  g_tx_frame_mask |= 1 << first_free;
    
fail:
  xSemaphoreGive(g_tx_frame_sem);
  
  return ret;
}

static BOOL
tx_frame_return(struct tx_frame *msg)
{
  int ndx = msg - g_tx_frame_alloc;
  BOOL ok = FALSE;
  
  TRY(ndx >= 0 && ndx < BOARD_TX_MESSAGE_COUNT);
  
  xSemaphoreTake(g_tx_frame_sem, portMAX_DELAY);
  
  /* Check if it was reserved */
  TRY((1 << ndx) & g_tx_frame_mask);
    
  g_tx_frame_mask ^= 1 << ndx;
  
  ok = TRUE;
  
fail:
  xSemaphoreGive(g_tx_frame_sem);
  
  return ok;
}

static void
tx_task_prepare(void)
{
  BOOL ok = FALSE;
  
  if (!g_tx_task_prepared) {
    TRY(g_tx_frame_sem = xSemaphoreCreateMutex());
    xSemaphoreGive(g_tx_frame_sem);
    g_tx_task_prepared = TRUE;
    
  }  
  ok = TRUE;
  
fail:
  if (!ok)
    hang();
}

static void
tx_task_func(void *userdata)
{
  struct tx_frame *msg = NULL;
  tx_task_t *task = (tx_task_t *) userdata;
  
  for (;;) {
    xQueueReceive(task->h_tx_queue, &msg, portMAX_DELAY);
    
    if (phy_tx_take_exception())
      phy_reset_params();
      
    if (!phy_send_data(&task->template, msg->data, msg->size))
      lcd_puts(1, "PHY TX ERROR");
      
    if (!tx_frame_return(msg))
      lcd_printf(1, "!RET%p", msg);
  }
}

BOOL
tx_task_push_frame(tx_task_t *task, const void *data, size_t size)
{
  BOOL ok = FALSE;
  struct tx_frame *msg = NULL;
  
  TRY(size <= BOARD_TX_MESSAGE_SIZE);
  TRY(msg = tx_frame_take());
  
  msg->size = size;
  memcpy(msg->data, data, size);
  
  TRY(xQueueSend(task->h_tx_queue, &msg, 0) == pdTRUE);
  
  ok = TRUE;
  
fail:
  return ok;
}
  
BOOL
tx_task_init(tx_task_t *task)
{
  BOOL ok = FALSE;
  
  memset(task, 0, sizeof(tx_task_t));
  
  tx_task_prepare();
  
  TRY(task->h_tx_queue = 
    xQueueCreate(BOARD_TX_MESSAGE_COUNT, sizeof (struct tx_frame *)));
  
  /* Initialize template */
  task->template.uc_disable_rx = true;
  task->template.uc_mod_type   = MODE_TYPE_A;
  task->template.uc_scheme     = MOD_SCHEME_DBPSK_C;
  task->template.uc_buffer_id  = TX_BUFFER_0;
  task->template.uc_tx_mode    = TX_MODE_RELATIVE;
  
  /* Create App Phy Transmission task */
  TRY(
    xTaskCreate(
      tx_task_func, 
      (const signed char *const) "TxTask", 
      TASK_APP_PHY_STACK, 
      task, 
      TASK_APP_PHY_PRIO, 
      &task->h_tx_task) == pdPASS);

  ok = TRUE;
  
fail:
  return ok;
}