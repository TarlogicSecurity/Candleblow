/*
 * hooks.c: FreeRTOS hooks
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

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

void vApplicationMallocFailedHook(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName);
void vApplicationTickHook(void);

/*-----------------------------------------------------------*/
void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	 * configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	 * function that will get called if a call to pvPortMalloc() fails.
	 * pvPortMalloc() is called internally by the kernel whenever a task, queue,
	 * timer or semaphore is created.  It is also called by various parts of the
	 * demo application.  If heap_1.c or heap_2.c are used, then the size of the
	 * heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	 * FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	 * to query the size of free heap space that remains (although it does not
	 * provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for (;;) {
		while (1) {
		}
	}
}

/*-----------------------------------------------------------*/
void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	 * to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	 * task.  It is essential that code added to this hook function never attempts
	 * to block in any way (for example, call xQueueReceive() with a block time
	 * specified, or call vTaskDelay()).  If the application makes use of the
	 * vTaskDelete() API function (as this demo application does) then it is also
	 * important that vApplicationIdleHook() is permitted to return to its calling
	 * function, because it is the responsibility of the idle task to clean up
	 * memory allocated by the kernel to any task that has since been deleted. */
}

/*-----------------------------------------------------------*/
void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	(void)pcTaskName;
	(void)pxTask;

	/* Run time stack overflow checking is performed if
	 * configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	 * function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for (;;) {
		while (1) {
		}
	}
}

/*-----------------------------------------------------------*/
void vApplicationTickHook( void )
{
	/* This function will be called by each tick interrupt if
	* configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	* added here, but the tick hook is called from an interrupt context, so
	* code must not attempt to block, and only the interrupt safe FreeRTOS API
	* functions can be used (those that end in FromISR()). */
}
