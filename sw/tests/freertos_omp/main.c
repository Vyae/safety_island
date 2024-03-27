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
        DP("Freeing mailbox...\r\n");
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
        for(int i=0; i<1; i++) {
        //while(1) {
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


void gomp_offload_manager( void *pvParameters ) {
    DP("[Safety] gomp_offload_manager\r\n");

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
    int32_t mb_h2d = 0x0;
    int32_t mb_d2h = 0x0;
    int nbOffloadRabMissHandlers = 0x0;
    struct offload_wrapper_args *offload_wrapper_args = NULL;

    uint32_t freeHeapSpace = 0;

    // uint32_t cycles0 = 0;
    // uint32_t cycles1 = 0;
    // uint32_t cycles2 = 0;
    // uint32_t cycles3 = 0;
    uint32_t cycles4 = 0;

    // uint32_t cyclesmb0 = 0;
    // uint32_t cyclesmb1 = 0;
    // uint32_t cyclesmb2 = 0;
    // uint32_t cyclesmb3 = 0;
    // uint32_t cyclesmb4 = 0;
    // uint32_t cyclesmb5 = 0;
    // uint32_t cyclesmb6 = 0;
    // uint32_t cyclesmb7 = 0;


    // Print the addresses of the mboxes for debug purpose
    // Note this printf is uper minimal, we need to wait to avoid
    // loosing chars when the UART is busy
    uint32_t *chs_ctrl_regs = 0x3000000;


    DP("[Safety] mailboxes: 0x%x and 0x%x\r\n", *(chs_ctrl_regs), *(chs_ctrl_regs + 1));
    g_h2a_mbox = *((struct ring_buf **) chs_ctrl_regs);
    g_a2h_mbox = *((struct ring_buf **)(chs_ctrl_regs + 1));

    int task_nr = 0;
    char task_name[] = "Offloaded task XX\0"; 

    uint32_t letters = 0;

    while (1) {
        DP("[Safety] Waiting for command...\r\n");

        // (1) Wait for the offload trigger.
        //letters = 0;
        //while(g_h2a_mbox->head - g_h2a_mbox->tail != 8 && g_h2a_mbox->head - g_h2a_mbox->tail != -8);

        while(1) {
            if (mailbox_try_read((uint32_t *)&cmd)) {
                //vTaskDelay(1);
            } else {
                //printf("[Safety] mb dif: %d\r\n", g_h2a_mbox->head - g_h2a_mbox->tail);
                break;
            }
        }

        // stop_timer();
        // cyclesmb0 = get_time();
        // start_timer();

        DP("[Safety] cmd: 0x%x\r\n", cmd);

        if (MBOX_DEVICE_STOP == cmd) {
            DP("[Safety] Got PULP_STOP from host, stopping execution now.\r\n");
            break;
        }


        // (2) The host sends through the mailbox the pointer to the function
        // that should be executed on the accelerator.
        mailbox_read((uint32_t *)&offloadFn, 1);

        // stop_timer();
        // cyclesmb1 = get_time();
        // start_timer();
        // (3) The host sends through the mailbox the pointer to the arguments
        // that should be used.
        mailbox_read((uint32_t *)&offloadArgs, 1);

        // stop_timer();
        // cyclesmb2 = get_time();
        // start_timer();
        // (3) The host sends through the mailbox the priority, period and
        // mailbox pointer of the task to be created. (They can also be -1/0)
        mailbox_read((int32_t *)&priority, 1);
        // stop_timer();
        // cyclesmb3 = get_time();
        // start_timer();
        mailbox_read((int32_t *)&period, 1);
        // stop_timer();
        // cyclesmb4 = get_time();
        // start_timer();
        // mailbox_read((uint32_t *)&mb_h2d, 1);
        // stop_timer();
        // cyclesmb5 = get_time();
        // start_timer();
        mailbox_read((uint32_t *)&mb_d2h, 1);

        // stop_timer();
        // cyclesmb6 = get_time();
        // start_timer();
        // (3b) The host sends through the mailbox the number of rab misses
        // handlers threads
        mailbox_read((uint32_t *)&nbOffloadRabMissHandlers, 1);

        // stop_timer();
        // cyclesmb7 = get_time();
        // start_timer();

        // DP("nbOffloadRabMissHandlers %d\r\n", nbOffloadRabMissHandlers);

        // (3c) Spawning nbOffloadRabMissHandlers
        // Not implemented

        // (4) Ensure access to offloadArgs. It might be in SVM.
        // Not implemented

        DP("[Safety] Received task to offload: tgt_fn: 0x%x, tgt_vars: 0x%x, priority: 0x%x, " "period: 0x%x, mb_h2d: 0x%x, mb_d2h: 0x%x\r\n", 
                    offloadFn, offloadArgs, priority, period, mb_h2d, mb_d2h);

        // (5) Create task that executes the offloaded function.
        //start_timer();

        // Name of task
        task_name[16] = '0' + task_nr % 10;
        task_name[15] = '0' + task_nr / 10;
        task_nr++;

        // stop_timer();
        // cycles1 = get_time();
        // start_timer();

        // Pack arguments for TaskCreate
        offload_wrapper_args = (struct offload_wrapper_args *) pvPortMalloc(sizeof(struct offload_wrapper_args));
        if(!offload_wrapper_args) {
            DP("[Safety] Failed to malloc. Abort.\r\n");
            continue;
        }
        // stop_timer();
        // cycles2 = get_time();
        // start_timer();
        offload_wrapper_args->offloadFn = offloadFn;
        offload_wrapper_args->offloadArgs = offloadArgs;
        offload_wrapper_args->period = period;
        offload_wrapper_args->mb_h2d = mb_h2d;
        offload_wrapper_args->mb_d2h = mb_d2h;

        //printf("offload fn ptr: 0x%x\r\n", offloadFn);
        //printf("offloadArgs ptr: 0x%x\r\n", offloadArgs);
        //printf("offloadArgs[0]]: 0x%d\r\n", *((uint32_t *) offloadArgs));
        //printf("offloadArgs[1]]: 0x%d\r\n", *(((uint32_t *) offloadArgs)+1));

        freeHeapSpace = xPortGetFreeHeapSize();
        DP("[Safety] Free heap space: 0x%x\r\n", freeHeapSpace);
        if (priority < 0) {
            priority = 0;
        }
        TaskHandle_t taskhandle = NULL;
        BaseType_t xReturned = xTaskCreate( offload_task_wrapper, task_name, 700, (void *) offload_wrapper_args, priority, &taskhandle );

        // stop_timer();
        // cycles3 = get_time();
        // start_timer();

        if(xReturned < 0) {
            ERR("[Safety] Creating Task \"%s\" FAILED.\r\n", task_name);
            if(xReturned == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
                ERR("[Safety] Error: errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY\r\n");
        } else {
            DP("[Safety] Offloaded task \"%s\" created: %x\r\n", task_name, xReturned);
        }

        //ticks1 = xTaskGetTickCount();

        stop_timer();
        cycles4 = get_time();
        //start_timer();


        //printf("[Safety] Time MailboxTry: %d cycles\r\n", cycles0);
        // printf("[Safety] Time Reading Args: %d cycles\r\n", cycles1);
        // printf("[Safety] Time Malloc: %d cycles\r\n", cycles2);
        // printf("[Safety] Time TaskCreate: %d cycles\r\n", cycles3);
        
        // printf("[Safety] Time MB 0: %d cycles\r\n", cyclesmb0);
        // printf("[Safety] Time MB 1: %d cycles\r\n", cyclesmb1);
        // printf("[Safety] Time MB 2: %d cycles\r\n", cyclesmb2);
        // printf("[Safety] Time MB 3: %d cycles\r\n", cyclesmb3);
        // printf("[Safety] Time MB 4: %d cycles\r\n", cyclesmb4);
        // printf("[Safety] Time MB 5: %d cycles\r\n", cyclesmb5);
        // printf("[Safety] Time MB 6: %d cycles\r\n", cyclesmb6);
        // printf("[Safety] Time MB 7: %d cycles\r\n", cyclesmb7);

        //uint32_t mb_letters[] = {MBOX_DEVICE_DONE, cycles4};
        //mailbox_write_mult(mb_letters, sizeof(mb_letters)/sizeof(uint32_t));

        mailbox_write(MBOX_DEVICE_DONE);
        mailbox_write(cycles4);

        // DP("Kernel execution time [PULP cycles] = %d\r\n", 0x0);

        vTaskDelay(10);
    }

    vTaskDelete(NULL);
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
        vTaskDelayUntil( &xLastWakeTime, xFrequency );

        //for(int i=0; i<1024*20000; i++){
        //    (void) x;
        //}
        /* Print out the name of this task. */
		printf(pcTaskName);
    }

    vTaskDelete(NULL);
}

////////////////////////////

int done = 0;
int iters = 0;

__attribute__((weak)) int main(int argc, char **argv, char **envp) {
    

    DP("\r\n\r\nXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\r\n");
    DP("[Safety] START OF SAFETY ISLAND MAIN\r\n");

    system_init();

    DP("[Safety] INITIATED SAFETY ISLAND SYSTEM\r\n");

    BaseType_t xReturnedOffload = xTaskCreate( gomp_offload_manager, "Offload Manager", 2000, NULL, 4, NULL );
    
    DP("[Safety] Offload task created: %x\r\n", xReturnedOffload);
    
	/* Create the other task in exactly the same way and at the same priority. */
	//BaseType_t xReturnedPrint = xTaskCreate( vTask1, "Printer Task", 700, NULL, 2, NULL );
    //DP("Print task created: %x\r\n", xReturnedPrint);

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

