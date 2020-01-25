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
#include "spi_driver.h"
#include "sd_driver.h"
#include "bootblock.h"
#include "banner.h"

#define UART_TX_FIFO                                ((volatile uint32_t*)0x100000004)
#define UART_STATUS                                 ((volatile uint32_t*)0x100000008)
#define UART_STATUS_TX_FULL                         (1U<<3)

#define DDR3_MEMORY                                 ((volatile uint64_t*)0x80000000)

#define CLOCK_HZ                                    (25*1000*1000)

#define BOOTBLOCK_NUMBER                            2048

static void printc(char c) {
    /* Wait for space in the TX FIFO. */
    while (*UART_STATUS & UART_STATUS_TX_FULL);
    /* Place character in FIFO to send. */
    *UART_TX_FIFO = c;
}

// TODO: move print functions to their own module

void printstr(char *str) {
    /* Write data to emulated HTIF. */
    while (*str) {
        char c = *str;
        str++;
        
        if (c == '\n') {
            printc('\r');
        }
        
        printc(c);
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
    printstr("\n");
}

static void printlx64(char *label, uint64_t val) {
    printstr(label);
    printstr(": ");
    printx64(val);
    printstr("\n");
}

extern void start_supervisor(uint64_t kernel_addr, uint64_t dtb_addr);

static void check_str_result(char *result) {
    if (result != NULL) {
        printstr("FAILED: ");
        printstr(result);
        printstr("\n");
        for(;;);
    } else {
        printstr("DONE!\n");
    }
}

static uint64_t get_cycle(void) {
    uint64_t result;
    asm("csrr %[result], cycle" : [result] "=r" (result) ::);
    return result;
}

static bootblock_t bootblock;

static void progress_function(uint32_t done) {
    if (done % 1024 == 0) {
        printc('.');
    }
}

int main() {
    char *result;

    printstr(banner);
    
    printstr("Initializing SPI driver...");
    spi_driver_init((void*)0x100010000);
    printstr("DONE!\n");

    printstr("Initializing SD driver...");
    result = sd_driver_init();
    check_str_result(result);

    printstr("Reading boot block from sector 0x");
    printx32(BOOTBLOCK_NUMBER);
    printstr("...");
    result = sd_driver_read_sector(BOOTBLOCK_NUMBER, &bootblock);
    check_str_result(result);

    if (bootblock.magic != BOOTBLOCK_MAGIC) {
        printstr("BOOT FAILED: incorrect bootblock magic number!");
        for(;;);
    }

    printstr("Found boot image: ");
    bootblock.description[255] = '\0';
    printstr(bootblock.description);
    printstr("\n");
    printlx64("sector_count", bootblock.sector_count);
    printlx64("load_addr", bootblock.load_addr);
    printlx64("entry_addr", bootblock.entry_addr);
    printlx64("dtb_addr", bootblock.dtb_addr);

    printstr("Loading image..");
    result = sd_driver_read_sectors(BOOTBLOCK_NUMBER+1, bootblock.sector_count, (void*)bootblock.load_addr, progress_function);
    check_str_result(result);

    printstr("Starting supervisor...\n");

    start_supervisor(bootblock.entry_addr, bootblock.dtb_addr);

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

void handle_exception(uint64_t sbi_arg0, uint64_t sbi_arg1, uint64_t sbi_ext, uint32_t mcause) {
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


