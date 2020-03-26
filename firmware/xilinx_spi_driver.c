/* Copyright (c) 2019, Daniel Kasza
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

#include "spi_driver.h"
#include <stdint.h>
#include <stdbool.h>

uint64_t spi_base = 0;

/* Software reset register. */
#define SRR         (*(volatile uint32_t*)(spi_base + 0x40))
/* SPI control register. */
#define SPICR       (*(volatile uint32_t*)(spi_base + 0x60))
/* SPI status register. */
#define SPISR       (*(volatile uint32_t*)(spi_base + 0x64))
#define SPISR_RX_EMPTY (1U<<0)
#define SPISR_TX_EMPTY (1U<<2)
/* SPI data transmit register. */
#define SPIDTR      (*(volatile uint32_t*)(spi_base + 0x68))
/* SPI data receive register. */
#define SPIDRR      (*(volatile uint32_t*)(spi_base + 0x6C))
/* SPI Slave select register. */
#define SPISSR      (*(volatile uint32_t*)(spi_base + 0x70))
/* Transmit FIFO occupancy register. */
#define SPITFOR     (*(volatile uint32_t*)(spi_base + 0x74))
/* Receive FIFO occupancy register. */
#define SPIRFOR     (*(volatile uint32_t*)(spi_base + 0x78))
/* Device global interrupt enable register. */
#define DGIER       (*(volatile uint32_t*)(spi_base + 0x1C))
/* IP interrupt status register. */
#define IPISR       (*(volatile uint32_t*)(spi_base + 0x20))
/* IP interrupt enable register. */
#define IPIER       (*(volatile uint32_t*)(spi_base + 0x28))

void spi_driver_init(void *ptr) {
    spi_base = (uint64_t)ptr;

    /* Reset the controller. */
    SRR = 0xa;

    /* Configure control register:
     *  (bit)
     *    0, loopback, 0
     *    1, SPI system enable, 1
     *    2, SPI master mode, 1
     *    3, CPOL, 0
     *    4, CPHA, 0
     *    5, TX FIFO Reset, 1
     *    6, RX FIFO Reset, 1
     *    7, Manual slave selection, 1
     *    8, Master transaction inhibit, 0
     *    9, LSB first, 0
     */
    SPICR = 0xE6;
}

unsigned char spi_driver_xfer(unsigned char out) {
    /* Place data in transmit register. */
    SPIDTR = out;

    /* Wait for transmit FIFO to be empty and receive FIFO to contain data. */
    while ((SPISR & (SPISR_RX_EMPTY | SPISR_TX_EMPTY)) != SPISR_TX_EMPTY);

    /* Return data from receive register. */
    return SPIDRR;
}

void spi_driver_receive_n(void *buffer, unsigned n) {
    unsigned char *b = (unsigned char*)buffer;

    /* This function assumes that the SPI FIFO is enabled, so we can put at least two bytes in the transmit FIFO at a
     * time. The benefit of this is that the controller can handle the second byte while the CPU is pulling out the
     * first byte from the receive FIFO.
     * Note: we know that n is at least two.
     */
    
    /* Start the first two transfers. */
    SPIDTR = 0xFF;
    SPIDTR = 0xFF;

    /* Read the received bytes. */
    while (n) {
        /* Wait for the receive FIFO to contain data. */
        while(SPISR & SPISR_RX_EMPTY);

        /* Get the byte. */
        *b = SPIDRR;
        n--;
        b++;

        /* If there is more than one byte left to receive, start another transfer.
         * If there is only one byte left, the transfer is already in progress.
         */
        if (n > 1) {
            SPIDTR = 0xFF;
        }
    }
}

void spi_driver_select(bool select) {
    /* Assuming only one slave, (un)select it. */
    SPISSR = select ? 0xFFFFFFFE : 0xFFFFFFFF;
}

