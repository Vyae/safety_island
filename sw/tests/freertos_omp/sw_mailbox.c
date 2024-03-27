// Copyright 2020 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "sw_mailbox.h"


/***********************************************************************************
 * MACROS
 ***********************************************************************************/

#define SYS_exit 60
#define SYS_write 64
#define SYS_read 63
#define SYS_wake 1235
#define SYS_cycle 1236

/***********************************************************************************
 * DATA
 ***********************************************************************************/
volatile struct ring_buf *g_a2h_rb;
volatile struct ring_buf *g_a2h_mbox;
volatile struct ring_buf *g_h2a_mbox;

/***********************************************************************************
 * FUNCTIONS
 ***********************************************************************************/
__attribute__((optimize("O0"))) void csleep(uint32_t cycles) {
    for (int i = 0; i < cycles; i++) {
    }
    // uint32_t start = cpu_perf_get(0);
    // while ((cpu_perf_get(0) - start) < cycles)
    //  ;
}

int syscall(uint64_t which, uint64_t arg0, uint64_t arg1, uint64_t arg2,
            uint64_t arg3, uint64_t arg4) {
    uint64_t magic_mem[6];
    int ret;
    uint32_t retries = 0;

    volatile struct ring_buf *rb = g_a2h_rb;

    magic_mem[0] = which;
    magic_mem[1] = arg0;
    magic_mem[2] = arg1;
    magic_mem[3] = arg2;
    magic_mem[4] = arg3;
    magic_mem[5] = arg4;

    do {
        ret = rb_device_put(rb, (void *)magic_mem);
        if (ret) {
            ++retries;
            csleep(1000000);
        }
    } while (ret != 0);
    return retries;
}

void snrt_putchar(char c) { syscall(SYS_write, 1, c, 1, 0, 0); }

void snrt_hero_exit(int code) { syscall(SYS_exit, code, 0, 0, 0, 0); }

/***********************************************************************************
 * MAILBOX
 ***********************************************************************************/
int mailbox_try_read(uint32_t *buffer) {
    return rb_device_get(g_h2a_mbox, buffer);
}

int mailbox_read(uint32_t *buffer, size_t n_words) {
    volatile struct ring_buf *rb = g_h2a_mbox;
    size_t n_words_left = n_words;

    //printf("Reading mailbox 0x%lx...\r\n", g_h2a_mbox);
    while(n_words_left > 0) {
        while(rb->tail != rb->head && n_words_left > 0) {
            buffer[n_words - n_words_left] = *((uint32_t *)rb->data_p + rb->tail);
            n_words_left = n_words_left - 1;
            rb->tail = (rb->tail + 1) % rb->size;
        }
    }

    return 0;
}
int mailbox_write(uint32_t word) {
    int ret;
    do {
        ret = rb_device_put(g_a2h_mbox, &word);
        if (ret) {
            csleep(10000);
        }
    } while (ret);
    return ret;
}

int mailbox_write_mult(uint32_t *words, size_t n_words) {
    int ret, retry = 0;
    volatile struct ring_buf *rb = g_a2h_mbox;
    uint32_t el_free = 0;
    uint32_t next_head = 0;

    while(n_words > 0) {
        next_head = (rb->head + 1) % rb->size;
        while(next_head != rb->tail && n_words > 0) {
            *((uint32_t *)rb->data_p + rb->head) = *words++;
            n_words = n_words - 1;
            rb->head = next_head;
            next_head = (rb->head + 1) % rb->size;
        }
    }

    return 0;
}
