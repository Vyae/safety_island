////////////////////////////
// from offload-manager.c //
////////////////////////////

#define DEBUG_LEVEL_OFFLOAD_MANAGER 10

/*
 * Copyright (C) 2018 ETH Zurich and University of Bologna
 *
 * Authors:
 *    Alessandro Capotondi, UNIBO, (alessandro.capotondi@unibo.it)
 */

/* Copyright (C) 2005-2014 Free Software Foundation, Inc.
 *
 * This file is part of the GNU OpenMP Library (libgomp).
 *
 * Libgomp is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * Libgomp is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * Under Section 7 of GPL version 3, you are granted additional
 * permissions described in the GCC Runtime Library Exception, version
 * 3.1, as published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License and
 * a copy of the GCC Runtime Library Exception along with this program;
 * see the files COPYING3 and COPYING.RUNTIME respectively.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "sw_mailbox.h"

#include <stdio.h>
#include <stdint.h>

/* FreeRTOS kernel includes. */
#include <FreeRTOS.h>
#include <task.h>

/* system includes */
#include "system.h"
#include "timer_irq.h"
#include "irq.h"
#include "gpio.h"

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName);

// #include "hero/offload-manager.h"

// FIXME consider to move these variables to L1 (vars it is ususally Empty when
// OpenMP4 Offload is used)
void **offload_func_table;
uint32_t nb_offload_funcs;
void **offload_var_table;
uint32_t nb_offload_vars;

// Struct for offload_task_wrapper arguments
struct offload_wrapper_args {
    void (*offloadFn)(uint64_t);
    void *offloadArgs; 
    int32_t period;
};

void offload_task_wrapper( void *pvParameters )
{
    if(!pvParameters) {
        printf("offload_task_wrapper: Null argument. Abort.\r\n");
        return;
    }

    // Unpack arguments
	struct offload_wrapper_args *offload_params = (struct offload_wrapper_args *) pvParameters;

    TickType_t xLastWakeTime;
    const TickType_t xPeriod = offload_params->period;

    printf("Start of offload_task_wrapper\r\n");
    printf("args: offloadFn: 0x%x, offloadArgs: 0x%x, period: 0x%x\r\n", 
        offload_params->offloadFn, offload_params->offloadArgs, offload_params->period);

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    // Execute offloaded function
    (*offload_params->offloadFn)(offload_params->offloadArgs);

    if(xPeriod > 0) {
        while(1) {
            // Wait for the next cycle.
            vTaskDelayUntil(&xLastWakeTime, xPeriod);

            // Execute offloaded function
            (*offload_params->offloadFn)(offload_params->offloadArgs);
        }
    }
}


void gomp_offload_manager( void *pvParameters ) {
    printf("XXXXXXXXXXXX [GOMP_OFFLOAD_MANAGER]\r\n");

    // FIXME For the momenent we are not using the cmd sended as trigger.
    // It should be used to perform the deactivation of the accelerator,
    // as well as other operations, like local data allocation or movement.
    // FIXME Note that the offload at the moment use several time the mailbox.
    // We should compact the offload descriptor and just sent a pointer to
    // that descriptor.

    uint32_t cmd = (uint32_t)NULL;

    // Offloaded function pointer and arguments
    void (*offloadFn)(uint64_t) = NULL;
    void *offloadArgs = 0x0;
    int32_t priority = 0x0;
    int32_t period = 0x0;
    int nbOffloadRabMissHandlers = 0x0;
    struct offload_wrapper_args *offload_wrapper_args = NULL;

    uint32_t freeHeapSpace = 0;

    //int cycles = 0;

    // Print the addresses of the mboxes for debug purpose
    // Note this printf is uper minimal, we need to wait to avoid
    // loosing chars when the UART is busy
    uint32_t *chs_ctrl_regs = 0x3000000;


    csleep(1024 * 1024 * 2);

    printf("XXXXX mailboxes: 0x%x and 0x%x\r\n", *(chs_ctrl_regs), *(chs_ctrl_regs + 1));
    g_h2a_mbox = *((struct ring_buf **) chs_ctrl_regs);
    g_a2h_mbox = *((struct ring_buf **)(chs_ctrl_regs + 1));

    int task_nr = 0;
    char task_name[] = "Offloaded task XX\0"; 

    while (1) {
        if (DEBUG_LEVEL_OFFLOAD_MANAGER > 0)
            printf("Waiting for command...\r\n");

        // (1) Wait for the offload trigger.
        mailbox_read((uint32_t *)&cmd, 1);
        printf("XXXXX cmd: 0x%x\r\n", cmd);

        if (MBOX_DEVICE_STOP == cmd) {
            if (DEBUG_LEVEL_OFFLOAD_MANAGER > 0)
                printf("Got PULP_STOP from host, stopping execution now.\r\n");
            break;
        }

        // (2) The host sends through the mailbox the pointer to the function
        // that should be executed on the accelerator.
        mailbox_read((uint32_t *)&offloadFn, 1);

        // (3) The host sends through the mailbox the pointer to the arguments
        // that should be used.
        mailbox_read((uint32_t *)&offloadArgs, 1);

        // (3) The host sends through the mailbox the priority of the task to
        // be created.
        mailbox_read((int32_t *)&priority, 1);

        mailbox_read((int32_t *)&period, 1);

        // (3b) The host sends through the mailbox the number of rab misses
        // handlers threads
        mailbox_read((uint32_t *)&nbOffloadRabMissHandlers, 1);

        // if (DEBUG_LEVEL_OFFLOAD_MANAGER > 0)
        //     printf("nbOffloadRabMissHandlers %d\r\n", nbOffloadRabMissHandlers);

        // (3c) Spawning nbOffloadRabMissHandlers
        // Not implemented

        // (4) Ensure access to offloadArgs. It might be in SVM.
        // Not implemented

        if (DEBUG_LEVEL_OFFLOAD_MANAGER > 0)
            printf("Received task to offload: tgt_fn: 0x%x, tgt_vars: 0x%x, priority: 0x%x, period:0x%x\r\n", 
                    (uint32_t)offloadFn, (uint32_t)offloadArgs, (int32_t)priority, (int32_t)period);

        // (5) Execute the offloaded function.
        //reset_timer();
        //start_timer();
        //offloadFn(offloadArgs);

        // Name of task
        task_name[16] = '0' + task_nr % 10;
        task_name[15] = '0' + task_nr / 10;
        task_nr++;

        // Pack arguments for TaskCreate
        offload_wrapper_args = (struct offload_wrapper_args *) pvPortMalloc(sizeof(struct offload_wrapper_args));
        if(!offload_wrapper_args) {
            printf("Malloc returned NULL. Abort.\r\n");
            return;
        }
        offload_wrapper_args->offloadFn = offloadFn;
        offload_wrapper_args->offloadArgs = offloadArgs;
        offload_wrapper_args->period = period;

        freeHeapSpace = xPortGetFreeHeapSize();
        printf("free heap space: %x\r\n", freeHeapSpace);
        if (priority < 0) {
            priority = 2;
        }
        TaskHandle_t xOffloadTask = xTaskCreate( offload_task_wrapper, task_name, 700, (void *) offload_wrapper_args, priority, NULL );
    
        printf("Offloaded task \"%s\" created: %x\r\n", task_name, xOffloadTask);

        //stop_timer();
        //cycles = get_time();

        mailbox_write(MBOX_DEVICE_DONE);
        //mailbox_write(cycles);
        mailbox_write(0x0);

        // if (DEBUG_LEVEL_OFFLOAD_MANAGER > 0)
        //     printf("Kernel execution time [PULP cycles] = %d\r\n", 0x0);

        if (nbOffloadRabMissHandlers) {
        }
    }

    return;
}

void vTask1( void *pvParameters )
{
	const char *pcTaskName = "Task 1 is running\r\n";
	/* As per most tasks, this task is implemented in an infinite loop. */
    TickType_t xLastWakeTime;
    // Before: 1024 x 1024 x 10 cycles
    // Now: 100000 x 100 Cycles? per Tick
    const TickType_t xFrequency = 20 * 1024;

    printf(pcTaskName);

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    volatile int x=0;
    while(1) {
        // Wait for the next cycle.
        //vTaskDelayUntil( &xLastWakeTime, xFrequency );

        for(int i=0; i<1024*20000; i++){
            (void) x;
        }
        /* Print out the name of this task. */
		printf(pcTaskName);
    }
}

////////////////////////////

int done = 0;
int iters = 0;

__attribute__((weak)) int main(int argc, char **argv, char **envp) {
    // FIXME: handle multi-cluster properly
    //csleep(1024 * 1024 * 20);
    printf("\r\n\r\nXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\r\n");
    printf("START OF SAFETY ISLAND MAIN\r\n");

    system_init();

    printf("INITIATED SAFETY ISLAND SYSTEM\r\n");

    TaskHandle_t xOffloadTask = xTaskCreate( gomp_offload_manager, "Offload Manager", 1500, NULL, 2, NULL );
    
    printf("Offload task created: %x\r\n", xOffloadTask);
    
	/* Create the other task in exactly the same way and at the same priority. */
	TaskHandle_t xTask1 = xTaskCreate( vTask1, "Printer Task", 700, NULL, 2, NULL );

    printf("Print task created: %x\r\n", xTask1);

    uint32_t meow = 0;
    asm volatile (
        "addi %0, sp, 0\n"
        :"=r"(meow)
        ::
    );
    printf("sp: 0x%x\r\n", meow);

	/* Start the scheduler so the tasks start executing. */
	vTaskStartScheduler();

    return 0;
}


void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
	(void)pcTaskName;
	(void)pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
    printf("TopOfStack: 0x%08x\r\n", *((unsigned int *) pxTask));
	printf("ERROR: stack overflow from task %s\r\n", pcTaskName);
	//__asm volatile("ebreak");
	for (;;)
		;
}

