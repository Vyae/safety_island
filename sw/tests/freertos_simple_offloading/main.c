////////////////////////////
// from offload-manager.c //
////////////////////////////

//#define DEBUG_LEVEL_OFFLOAD_MANAGER 0 // no debug prints at all
#define DEBUG_LEVEL_OFFLOAD_MANAGER 1 // only errors
//#define DEBUG_LEVEL_OFFLOAD_MANAGER 2 // all debug prints

#define DP(...)                                                                \
  do {                                                                         \
    if (DEBUG_LEVEL_OFFLOAD_MANAGER >= 2) {                                    \
      printf(__VA_ARGS__);                                                     \
    }                                                                          \
  } while (false)


#define ERR(...)                                                                \
  do {                                                                         \
    if (DEBUG_LEVEL_OFFLOAD_MANAGER) {                                         \
      printf(__VA_ARGS__);                                                     \
    }                                                                          \
  } while (false)

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
#include "timer.h"
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
    uint32_t * mb_h2d;
    uint32_t * mb_d2h;
};

// Struct for mailboxes linked list
struct mb {
    TaskHandle_t task_handle;
    volatile struct ring_buf * mb_h2d;
    volatile struct ring_buf * mb_d2h;
    struct mb *next;
};

struct mb *mailboxes = NULL;

uint32_t idle = 0;

// Free own mailbox
// This might break with preemption...
void free_mailbox() {
    // Finding mailbox and prev mailbox (to keep linked list correct)
    TaskHandle_t my_task_handle = xTaskGetCurrentTaskHandle();
    taskENTER_CRITICAL();
    struct mb *mb = mailboxes;
    struct mb *mb_prev = NULL;
    while(mb) {
        if(mb->task_handle == my_task_handle) {
            break;
        } else {
            mb_prev = mb;
            mb = mb->next;
        }
    }

    if(mb) {
        if(mb_prev) {
            mb_prev->next = mb->next;
        } else {
            mailboxes = mb->next;
        }
        taskEXIT_CRITICAL();
        vPortFree(mb);
    } else {
        taskEXIT_CRITICAL();
    }
}

// Search for own mailbox
struct mb *find_mailbox() {
    TaskHandle_t my_task_handle = xTaskGetCurrentTaskHandle();
    DP("[Safety] Searching mailbox for task_handle: 0x%x\r\n", my_task_handle);
    taskENTER_CRITICAL();
    struct mb *mb = mailboxes;
    while(mb) {
        if(mb->task_handle == my_task_handle) {
            break;
        } else {
            mb = mb->next;
        }
    }
    taskEXIT_CRITICAL();
    return mb;
}

// Puts message into d2h mailbox, used in host source code
int send_to_host(uint32_t word) {
    // Search for corresponding mailbox
    struct mb *mb = find_mailbox();
    if(!mb) {
        DP("[Safety] Could not find corresponding mailbox.\r\n");
        return 1;
    }

    // Blocking write
    int ret;
    do {
        ret = rb_device_put(mb->mb_d2h, &word);
        if (ret) {
            csleep(10000);
        }
    } while (ret);

    DP("[Safety] Wrote %d in mailbox 0x%x\r\n", word, mb->mb_d2h);
    return ret;
}

// Puts message into d2h mailbox, used in host source code
int receive_from_host(uint32_t *buffer) {
    // Search for corresponding mailbox
    TaskHandle_t my_task_handle = xTaskGetCurrentTaskHandle();
    DP("[Safety] Callback task_handle: 0x%x\r\n", my_task_handle);
    taskENTER_CRITICAL();
    struct mb *mb = mailboxes;
    while(mb) {
        if(mb->task_handle == my_task_handle) {
            break;
        } else {
            mb = mb->next;
        }
    }
    taskEXIT_CRITICAL();
    if(!mb) {
        DP("[Safety] Could not find corresponding mailbox.\r\n");
        return 1;
    }

    // Blocking read
    int ret;
    do {
        ret = rb_device_get(mb->mb_h2d, 1);
        if (ret) {
            csleep(1000000);
        }
    } while (ret);
    return 0;

    DP("[Safety] Received %d from mailbox 0x%x\r\n", buffer, mb->mb_h2d);
    return 0;
}

void offload_task_wrapper( void *pvParameters )
{
    if(!pvParameters) {
        ERR("[Safety] offload_task_wrapper: Null argument. Abort.\r\n");
        vTaskDelete(NULL);
    }

    // Unpack arguments
	struct offload_wrapper_args *offload_params = (struct offload_wrapper_args *) pvParameters;

    TickType_t xLastWakeTime;
    const TickType_t xPeriod = offload_params->period;

    DP("[Safety] Start of offload_task_wrapper\r\n");
    DP("[Safety] args: offloadFn: 0x%x, offloadArgs: 0x%x, period: 0x%x\r\n", 
        offload_params->offloadFn, offload_params->offloadArgs, offload_params->period);

    // Create Mailbox if needed
    if(offload_params->mb_h2d || offload_params->mb_d2h) {
        DP("[Safety] Filling out mb struct...\r\n");
        struct mb *mb = pvPortMalloc(sizeof(struct mb));
        if(!mb) {
            ERR("[Safety] Failed to malloc mailboxes. Abort.\r\n");
            goto free_params;
        }
        TaskHandle_t taskhandle = xTaskGetCurrentTaskHandle();
        DP("[Safety] Offloaded taskhandle: 0x%x\r\n", taskhandle);
        // Adding mb to linked list
        mb->task_handle = taskhandle;
        mb->mb_h2d = offload_params->mb_h2d;
        mb->mb_d2h = offload_params->mb_d2h;
        taskENTER_CRITICAL();
        mb->next = mailboxes;
        mailboxes = mb;
        taskEXIT_CRITICAL();
    }

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
free_params:
    DP("[Safety] Freeing Structs and deleting task...\r\n");
    free_mailbox();

    // Freeing malloced parameters and deleting this task
    vPortFree(pvParameters);
    vTaskDelete(NULL);
}




void vTask1( void *pvParameters )
{
	const char *pcTaskName = "Task 1 is running\r\n";
    TickType_t xLastWakeTime;

    stop_timer();
    uint32_t cycles = get_time();
    printf("[Safety] Time: %d cycles\r\n", cycles);
    
    printf(pcTaskName);

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t  xPeriod = 0;

    if(xPeriod > 0) {
        while(1) {
            // Wait for the next cycle.
            vTaskDelayUntil(&xLastWakeTime, xPeriod);
            print(pcTaskName);
            
        }
    }

    vTaskDelete(NULL);
}

////////////////////////////

int done = 0;
int iters = 0;

__attribute__((weak)) int main(int argc, char **argv, char **envp) {
    // FIXME: handle multi-cluster properly
    //csleep(1024 * 1024 * 20);
    DP("\r\n\r\nXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\r\n");
    DP("[Safety] START OF SAFETY ISLAND MAIN\r\n");

    system_init();

    DP("[Safety] INITIATED SAFETY ISLAND SYSTEM\r\n");

    //BaseType_t xReturnedOffload = xTaskCreate( gomp_offload_manager, "Offload Manager", 2000, NULL, 4, NULL );
    
    //DP("[Safety] Offload task created: %x\r\n", xReturnedOffload);
    
	/* Create the other task in exactly the same way and at the same priority. */
	BaseType_t xReturnedPrint = xTaskCreate( vTask1, "Printer Task", 700, NULL, 2, NULL );
    DP("Print task created: %x\r\n", xReturnedPrint);

    // uint32_t meow = 0;
    // asm volatile (
    //     "addi %0, sp, 0\n"
    //     :"=r"(meow)
    //     ::
    // );


	/* Start the scheduler so the tasks start executing. */
	vTaskStartScheduler();

    return 0;
}

// Never block this task (i.e. with TaskDelay)
void vApplicationIdleHook( void )
{
    #define STDOUT_FILENO 1
    #define STDERR_FILENO 2

    idle = idle + 1;
    const char *msg = "[Safety] vApplicationIdleHook\r\n";

    if(idle > 5000000) {
        idle = 0;
        //printf("[Safety] vApplicationIdleHook\r\n");
        _write(STDOUT_FILENO, msg, strlen(msg));
    }
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
	(void)pcTaskName;
	(void)pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
    ERR("[Safety] TopOfStack: 0x%08x\r\n", *((unsigned int *) pxTask));
	ERR("[Safety] ERROR: stack overflow from task %s\r\n", pcTaskName);
	//__asm volatile("ebreak");
	for (;;)
		;
}

