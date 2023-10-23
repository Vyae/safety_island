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
	//printf("blink\n");
	/* terminate after a few blinks */
	blink_times++;
	if (blink_times > 4) {
        printf("4 blinks\n");
    }
    __asm volatile(
		"csrrsi x0, 0x345, 8 \n"
		:::
	);
}


int main(void) {

    blink_times = 0;

    /* Init board hardware. */
	system_init();

    __asm volatile(
        "addi t0, x0, 8\n           \
         csrrs x0, mstatus, t0\n    \
         slli t0, t0, 8\n           \
         csrrs x0, mie, t0\n         "
        :::"t0"
    );

	/* hook up high timer interrupt */
	irq_set_handler(IRQ_FC_EVT_TIMER0_HI, timer1_handler);

	/* reset timer (not really necessary in this case) */
	writew(1, (uintptr_t)(PULP_FC_TIMER_ADDR + TIMER_RESET_HI_OFFSET));

	/* set interrupt frequency to TIMER1_TICK_RATE_HZ */
#define TIMER1_TICK_RATE_HZ ((TickType_t)2000)
	writew(ARCHI_REF_CLOCK / TIMER1_TICK_RATE_HZ,
	       (uintptr_t)(PULP_FC_TIMER_ADDR + TIMER_CMP_HI_OFFSET));

	/* Enable timer (TIMER_CFG_HI_ENABLE_MASK), use 32khz ref clock as
	 * source (TIMER_CFG_HI_CLKCFG_MASK). Timer will reset automatically
	 * (TIMER_CFG_HI_MODE_MASK) to zero after causing an interrupt
	 * (TIMER_CFG_HI_IRQEN_MASK). Also reset timer to start from a clean
	 * slate (TIMER_CFG_HI_RESET_MASK).
	 */
	writew(TIMER_CFG_HI_ENABLE_MASK | TIMER_CFG_HI_RESET_MASK |
		       TIMER_CFG_HI_CLKCFG_MASK | TIMER_CFG_HI_MODE_MASK |
		       TIMER_CFG_HI_IRQEN_MASK,
	       (uintptr_t)(PULP_FC_TIMER_ADDR + TIMER_CFG_HI_OFFSET));

	/* Enable timer1 interrupt. Need to enable this in the CV32E40P and the
	 * apb_interrupt controller. In RI5CY we didn't need to touch the clint
	 * since it was in a kind of "passthrough" mode. */
	irq_enable(IRQ_FC_EVT_TIMER0_HI);
	irq_clint_enable(IRQ_FC_EVT_TIMER0_HI);

	/* start test */
	//printf("Starting timer test\n");

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