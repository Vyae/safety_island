/*
 * Copyright 2020 ETH Zurich
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Author: Robert Balas (balasr@iis.ee.ethz.ch)
 */

/*
 * A minimal pmsis example. Mainly used to test if we can link against pmsis.
 */

/* FreeRTOS kernel includes. */
#include <FreeRTOS.h>
#include <task.h>

/* c stdlib */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <inttypes.h>

/* system includes */
#include "system.h"
#include "timer_irq.h"
#include "fll.h"
#include "irq.h"
#include "gpio.h"

void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName);
void vApplicationTickHook(void);

void vTask1( void *pvParameters );
void vTask2( void *pvParameters );

/* Program Entry. */
int main(void)
{
	/* Init board hardware. */
	system_init();
	
	printf("\n\n\t *** FreeRTOS HelloWorld *** \n\n");

	/* Create one of the two tasks. Note that a real application should check
	the return value of the xTaskCreate() call to ensure the task was created
	successfully. */
	TaskHandle_t xTask1 = xTaskCreate( vTask1, "Task 1", 100, NULL, 1, NULL );
	//printf("task1 creation: %x\n", xTask1);
	/* Create the other task in exactly the same way and at the same priority. */
	TaskHandle_t xTask2 = xTaskCreate( vTask2, "Task 2", 100, NULL, 1, NULL );
	//printf("task2 creation: %x\n", xTask2);
	/* Start the scheduler so the tasks start executing. */
	vTaskStartScheduler();
	/* If all is well then main() will never reach here as the scheduler will
	now be running the tasks. If main() does reach here then it is likely that
	there was insufficient heap memory available for the idle task to be created.
	Chapter 2 provides more information on heap memory management. */
	for( ;; );
}

void vTask1( void *pvParameters )
{
	const char *pcTaskName = "Task 1 is running\r\n";
	volatile uint32_t ul; /* volatile to ensure ul is not optimized away. */
	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; ) {
		/* Print out the name of this task. */
		printf(pcTaskName);
		/* Delay for a period. */
		for( ul = 0; ul < 1000; ul++ ){
		/* This loop is just a very crude delay implementation. There is
		nothing to do in here. Later examples will replace this crude
		loop with a proper delay/sleep function. */
		}
	}
}

void vTask2( void *pvParameters )
{
	const char *pcTaskName = "Task 2 is running\r\n";
	volatile uint32_t ul; /* volatile to ensure ul is not optimized away. */
	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; ) {
		/* Print out the name of this task. */
		printf(pcTaskName);
		/* Delay for a period. */
		for( ul = 0; ul < 1000; ul++ ){
		/* This loop is just a very crude delay implementation. There is
		nothing to do in here. Later examples will replace this crude
		loop with a proper delay/sleep function. */
		}
	}
}


void vApplicationIdleHook(void)
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
	(void)pcTaskName;
	(void)pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	printf("error: stack overflow\n");
	__asm volatile("ebreak");
	for (;;)
		;
}

void vApplicationTickHook(void)
{
}


