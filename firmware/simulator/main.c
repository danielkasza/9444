/* Copyright (c) 2019, 2020, Daniel Kasza
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "banner.h"

#define KERNEL_START  0x80200000
#define DVTREE_START (KERNEL_START + (16*1024*1024))
#define DVTREE_OURS  (0x40)
#define DVTREE_SIZE  (32*1024)

#define TLB_COUNT 32

#define HTIF_BASE_ADDR 0x40008000

static void printc(char c) {
    *(volatile uint32_t*)(HTIF_BASE_ADDR + 0) = c;
    *(volatile uint32_t*)(HTIF_BASE_ADDR + 4) = 0x01010000;
}

static void printstr(char *str) {
    /* Write data to emulated HTIF. */
    while (*str) {
        printc(*str);
        str++;
    }
}

static char hex_table[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
static void printx32(uint32_t val) {
    unsigned i;
    for (i=0; i<8; i++) {
        printc(hex_table[(val >> (28 - (4*i))) & 0xF]);
    }
}
static void printx64(uint64_t val) {
    unsigned i;
    for (i=0; i<16; i++) {
        printc(hex_table[(val >> (60 - (4*i))) & 0xF]);
    }
}

static void printlx32(char *label, uint32_t val) {
    printstr(label);
    printstr(": ");
    printx32(val);
    printc('\n');
}

static void printlx64(char *label, uint64_t val) {
    printstr(label);
    printstr(": ");
    printx64(val);
    printc('\n');
}


extern void start_supervisor(uint64_t kernel_addr, uint64_t dtb_addr);

int main() {
    printstr(banner);

    /* Copy the DTB closer to the kernel. */
    void *dtb_addr = (void*)DVTREE_START;
    memcpy(
        dtb_addr,
        (void*)DVTREE_OURS,
        DVTREE_SIZE
    );

    start_supervisor(KERNEL_START, (uint64_t)dtb_addr);

    for(;;);

    return 1;
}

static uint64_t get_mepc(void) {
    uint64_t result;
    asm("csrr %[result], mepc" : [result] "=r" (result) ::);
    return result;
}

static void set_mepc(uint64_t mepc) {
    asm("csrw mepc, %[mepc]" :: [mepc] "r" (mepc) :);
}

static void ecall_exception(uint64_t sbi_arg0, uint64_t sbi_arg1, uint64_t sbi_ext) {
    uint64_t mepc = get_mepc();

    if (sbi_ext == 0) {
        /* void sbi_set_timer(uint64_t stime_value) */
        asm("csrw 0x5C0, %[sbi_arg0]" :: [sbi_arg0] "r" (sbi_arg0) :);
    } else if (sbi_ext == 1) {
        /* void sbi_console_putchar(int ch) */
        printc(sbi_arg0);
    } else {
        printstr("M: unexpected call from S-mode\n");
        printlx64("mepc", mepc);
        printlx64("sbi_arg0", sbi_arg0);
        printlx64("sbi_arg1", sbi_arg1);
        printlx64("sbi_ext", sbi_ext);
        for(;;);
    }

    /* Skip over the ecall instruction. */
    set_mepc(get_mepc() + 4);
}

void handle_exception(uint32_t sbi_arg0, uint32_t sbi_arg1, uint32_t sbi_ext, uint32_t mcause) {
    switch (mcause) {
        case 9: /* Environment call from S-mode. */
            ecall_exception(sbi_arg0, sbi_arg1, sbi_ext);
            return;
        
        default:
            printstr("M: unexpected exception\n");
            break;
    }

    for(;;);
}


