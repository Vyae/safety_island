/*
 * Copyright 2021 ETH Zurich
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
#include "timer.h"
#include "irq.h"

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName);

static int blink_times;

void timer1_handler(void)
{
	blink_times++;
    // After printing interrupts are disabled until we start scheduler
    printf("%d timer interrupts\n", blink_times);
}


int main(void) {

    blink_times = 0;

    /* Init board hardware. */
	system_init();

	/* set high timer interrupt handler */
	irq_set_handler(IRQ_TIMER_HI, timer1_handler);

	/* reset timer (not really necessary in this case) */
	writew(1, (uintptr_t)(PULP_TIMER_ADDR + TIMER_RESET_HI_OFFSET));

	/* set interrupt frequency to 2000 */
	writew(ARCHI_REF_CLOCK / (TickType_t)2000,
	       (uintptr_t)(PULP_TIMER_ADDR + TIMER_CMP_HI_OFFSET));

	/* Enable timer (TIMER_CFG_HI_ENABLE_MASK), use 32khz ref clock as
	 * source (TIMER_CFG_HI_CLKCFG_MASK). Timer will reset automatically
	 * (TIMER_CFG_HI_MODE_MASK) to zero after causing an interrupt
	 * (TIMER_CFG_HI_IRQEN_MASK). Also reset timer to start from a clean
	 * slate (TIMER_CFG_HI_RESET_MASK).
	 */
	writew(TIMER_CFG_HI_ENABLE_MASK | TIMER_CFG_HI_RESET_MASK |
		       TIMER_CFG_HI_CLKCFG_MASK | TIMER_CFG_HI_MODE_MASK |
		       TIMER_CFG_HI_IRQEN_MASK,
	       (uintptr_t)(PULP_TIMER_ADDR + TIMER_CFG_HI_OFFSET));

	irq_enable(IRQ_TIMER_HI);
	
	vTaskStartScheduler();
	/* should never happen */
	for (;;);

}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
	(void)pcTaskName;
	(void)pxTask;

	taskDISABLE_INTERRUPTS();
	printf("error: stack overflow\n");
	__asm volatile("ebreak");
	for (;;)
		;
}