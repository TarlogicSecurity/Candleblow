/*
 * boot.c: Hardware initialization
 *
 * (c) 2019 Tarlogic Security S.L. - All rights reserved
 *
 * Company confidential. Any unauthorized use, disclosure, reproduction or
 * distribution of this file is strictly prohibited.
 */


#include <app.h>
#include <console.h>
#include <lcd.h>

#include <phy.h>
#include <tx-task.h>
#include "boot.h"

/****************************** Boot parameters ****************************/
#define WATCHDOG_TIMEOUT_US 5000000 /* 5 seconds */
#define TASK_APP_STACK             (configMINIMAL_STACK_SIZE * 5)

#define mainMONITOR_TIMER_RATE    (500 / portTICK_RATE_MS)
#define mainMONITOR_BLOCK_TIME    (1000 / portTICK_RATE_MS)

static xTaskHandle app_task_handle;

/*************************** Startup code **********************************/
void
hang(void)
{
  unsigned int cntr = 0xffffff;
  volatile unsigned int i;
  
  lcd_puts(0, "panic: hang");
  
  LED_Off(LED0);
  
  for(;;) {
    LED_Toggle(LED1);
    i = cntr;
    while (--i);
    wdt_restart(WDT);
  }
}

#include <sys/time.h>

int _gettimeofday(struct timeval *tv, struct timezone *tz);

int
_gettimeofday(struct timeval *tv, struct timezone *tz)
{
  uint64_t total = xTaskGetTickCount() * portTICK_RATE_MS * 1000u;
  
  tv->tv_sec = total / 1000000u;
  tv->tv_usec = total % 1000000u;
  
  return 0;
}

static void
monitor_timer_cb(xTimerHandle timer)
{
  wdt_restart(WDT); /* I'm that terrible */
}

static void
led_blink(unsigned int times)
{
  uint32_t waitcnt;
  unsigned int i;
  
  LED_Toggle(LED1);
  for (i = 0; i < times; ++i) {
    waitcnt = 0xfffff;
    while (waitcnt--);
    
    LED_Toggle(LED0);
    LED_Toggle(LED1);
  }    
  LED_Toggle(LED1);
}
  
static void
watchdog_init(void)
{
  uint32_t timeout_value;
  uint32_t wdt_mode;
  
  timeout_value = wdt_get_timeout_value(WATCHDOG_TIMEOUT_US, BOARD_FREQ_SLCK_XTAL);
  
  wdt_mode = WDT_MR_WDRSTEN | WDT_MR_WDRPROC | WDT_MR_WDDBGHLT | WDT_MR_WDIDLEHLT;
  
  /* Initialize watchdog */
  wdt_init(WDT, wdt_mode, timeout_value, timeout_value);
}

static void
monitor_init(void)
{
  xTimerHandle xMonitorTimer;
  
  xMonitorTimer = xTimerCreate((const signed char *const)"Monitor timer",
  mainMONITOR_TIMER_RATE,
  pdTRUE,
  NULL,
  monitor_timer_cb);
  
  configASSERT(xMonitorTimer);
  
  xTimerStart(xMonitorTimer, mainMONITOR_BLOCK_TIME);
}

static void
sched_init(void)
{
  vTaskStartScheduler();
}

static void
app_init(void)
{
  static tx_task_t tt;
  
  if (tx_task_init(&tt)) {
    xTaskCreate(
      app_entry,
      (const signed char *const) "APP",
      TASK_APP_STACK,
      &tt,
      tskIDLE_PRIORITY + 1,
      &app_task_handle);
  } else {
    lcd_puts(0, "TX init fail");
    hang();
  }   
}
  
static void
hardware_init(void)
{
  /* Setup system clock */
  sysclk_init();
  
  /* Set interrupt priority */
  NVIC_SetPriorityGrouping(__NVIC_PRIO_BITS);
  
  /* Initialize board */
  board_init();
  
  /* Initialize flash */
  flash_init(FLASH_ACCESS_MODE_128, CHIP_FLASH_WRITE_WAIT_STATE);
  
  /* Initialize console */
  console_init();
  
  /* Notify user with some blinking */
  led_blink(10);
  
  /* Initialize LCD */
  lcd_init();
  
  /* Initialize watchdog */
  watchdog_init();
  
  /* Initialize PHY layer */
  phy_init();
}

int
main(void)
{
  /* Hardware stuff */
  hardware_init();
  
  /* Task monitor */
  monitor_init();
  
  /* Create app task */
  app_init();
  
  /* Init multitasking */
  sched_init();
}