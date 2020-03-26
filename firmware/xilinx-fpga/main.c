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
#include "util.h"
#include "spi_driver.h"
#include "sd_driver.h"
#include "banner.h"
#include "ext2.h"

#define UART_TX_FIFO                                ((volatile uint32_t*)0x100000004)
#define UART_STATUS                                 ((volatile uint32_t*)0x100000008)
#define UART_STATUS_TX_FULL                         (1U<<3)

#define DDR3_MEMORY                                 ((uint64_t)0x80000000)
#define KERNEL_SIZE_MAX                             (32*1024*1024)
#define DTB_SIZE_MAX                                ( 1*1024*1024)

/* Path of kernel image to load. */
const char *kernel_path[]       = { "boot", "Image", NULL };
const char *dtb_override_path[] = { "boot", "override.dtb", NULL };

/* The device tree. */
extern unsigned char genesys2_dtb[];
extern int genesys2_dtb_len;

void printc(char c) {
    /* Wait for space in the TX FIFO. */
    while (*UART_STATUS & UART_STATUS_TX_FULL);
    /* Place character in FIFO to send. */
    *UART_TX_FIFO = c;
}

static void progress_function(uint32_t done) {
    if (done % 1024 == 0) {
        printc('.');
    }
}

static const char *ext2_disk_access(void *context, uint32_t first, uint32_t count, uint8_t *buffer) {
    return sd_driver_read_sectors(first, count, (void*)buffer, progress_function);
}

int main() {
    const char *result;
    ext2_inode_t inode = { 0 };

    printstr(banner);
    
    printstr("Initializing SPI driver...");
    spi_driver_init((void*)0x100010000);
    printstr("DONE!\n");

    printstr("Initializing SD driver...");
    result = sd_driver_init();
    check_str_result(result);

    printstr("Initializing Ext2 filesystem...");
    ext2_fs_t fs = { 0 };
    result = ext2_open_fs(
        ext2_disk_access, NULL,
        (void*)(DDR3_MEMORY + KERNEL_SIZE_MAX + DTB_SIZE_MAX),
        (EXT2_FS_CACHE_BLOCKS_COUNT_MAX * 4096),
        &fs
    );
    check_str_result(result);

    printstr("Opening kernel image...");
    result = ext2_get_inode_by_path(&fs, kernel_path, &inode);
    check_str_result(result);

    /* Check the size of kernel image. */
    if (inode.size > KERNEL_SIZE_MAX) {
        check_str_result("kernel image is too large");
    }

    printstr("Loading kernel image...");
    result = ext2_read(
        &fs, &inode,
        0, inode.size,
        (uint8_t*)DDR3_MEMORY
    );
    check_str_result(result);

    /* Copy the DTB to the cache line following the kernel image. */
    uint64_t dtb_addr = (((DDR3_MEMORY + inode.size + 63) / 64) * 64);
    memcpy((void*)dtb_addr, genesys2_dtb, genesys2_dtb_len);

    /* Check to see if there is an override DTB.
     * This feature exists for development. It is easier to change a file on the card than to rebuild the FPGA bitstream
     * when making device tree changes.
     */
    printstr("Checking for DTB override...");
    result = ext2_get_inode_by_path(&fs, dtb_override_path, &inode);
    if (result == NULL) {
        if (inode.size > DTB_SIZE_MAX) {
            check_str_result("dtb is too large");
        }
        printstr("found it!\n");

        printstr("Loading DTB...");
        result = ext2_read(
            &fs, &inode,
            0, inode.size,
            (uint8_t*)dtb_addr
        );
        check_str_result(result);
    } else {
        printstr("none\n");
    }

    printstr("Starting kernel...\n");
    start_supervisor(DDR3_MEMORY, dtb_addr);

    for(;;);

    return 1;
}



