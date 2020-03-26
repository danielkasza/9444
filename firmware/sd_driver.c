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

#include "sd_driver.h"
#include "spi_driver.h"
#include <stdlib.h>

/* This driver is designed to be as simple as possible. It only supports SDHC cards. */

/* Receive a byte while sending all 1s. */
static unsigned char just_rx(void) {
    return spi_driver_xfer(0xFF);
}

static void sd_select(bool select) {
    just_rx();
    spi_driver_select(select);
    just_rx();
}

/* Send command, and receive first response byte.
 * The card will be selected at the beginning of the command, but it will not be unselected to allow the caller to
 * retrieve a longer response.
 */
static unsigned char sd_command(unsigned char cmd, uint32_t arg, unsigned char crc) {
    sd_select(true);
    (void)spi_driver_xfer(0x40 | cmd);
    (void)spi_driver_xfer((arg >> 24) & 0xFF);
    (void)spi_driver_xfer((arg >> 16) & 0xFF);
    (void)spi_driver_xfer((arg >>  8) & 0xFF);
    (void)spi_driver_xfer((arg >>  0) & 0xFF);
    (void)spi_driver_xfer(crc);

    unsigned char response;
    do {
        response = just_rx();
    } while (response == 0xFF);

    return response;
}

static uint32_t sd_get_u32_value(void) {
    uint32_t value = 0;
    value |= just_rx();
    value <<= 8;
    value |= just_rx();
    value <<= 8;
    value |= just_rx();
    value <<= 8;
    value |= just_rx();
    return value;
}

char *sd_driver_init(void) {
    uint64_t i;
    unsigned char resp;

    /* Make sure the card is NOT selected, and drive the clock for 100B times to let the card wake up. */
    sd_select(false);
    for (i=0; i<10; i++) {
        (void)just_rx();
    }

    /* CMD0 with CRC. */
    if (sd_command(0, 0, 0x95) != 0x1) {
        sd_select(false);
        return "Unexpected response from card, expected 0x01 (idle) after CMD0";
    }
    sd_select(false);

    /* Send CMD8 to make sure this is a v2 card. */
    if (sd_command(8, 0x1AA, 0x87) != 0x1) {
        sd_select(false);
        return "Unexpected response from card, expected 0x01 (idle) after CMD8";
    }
    if (sd_get_u32_value() != 0x000001AA) {
        sd_select(false);
        return "Unexpected response from card, expected 0x1AA in response to CMD8";
    }
    sd_select(false);

    /* Keep sending ACMD41 until the card is ready. */
    for (;;) {
        /* We have to send CMD55 before ACMD41. */
        if (sd_command(55, 0, 0) != 0x1) {
            sd_select(false);
            return "Unexpected response from card, expected 0x01 (idle) after CMD55";
        }
        sd_select(false);

        /* Send ACMD41 to initialize the card. */
        unsigned char result = sd_command(41, 0x40000000, 0);
        sd_select(false);

        if (result == 0x00) {
            /* Card is ready! */
            break;
        } else if (result == 0x01) {
            /* Card needs more time... */
        } else {
            return "Unexpected response from card, expected 0x00 or 0x01 after ACMD41";
        }
    }

    /* Read OCR with CMD58 and verify that this is a high capacity card. */
    if (sd_command(58, 0, 0) != 0x0) {
        sd_select(false);
        return "Unexpected response from card, expected 0x00 after CMD58";
    }
    if ((sd_get_u32_value() & (1<<30)) == 0) {
        /* Note: it would not take a lot of work to support standard cards, but I don't have any. */
        sd_select(false);
        return "Unexpected card type, only high capacity cards are supported";
    }

    /* Initialization done! */
    return NULL;
}

static char *sd_get_next_block(void *buffer) {
    unsigned char *b = (unsigned char*)buffer;
    unsigned char resp;

    /* Wait for data token. */
    do {
        resp = just_rx();
    } while (resp == 0xFF);
    if (resp != 0xFE) {
        return "Unexpected response from card, expected 0xFE (data token) after CMD17 R1 response";
    }
    
    /* Read the payload. */
    spi_driver_receive_n(buffer, 512);

    /* Skip over 16b CRC. */
    (void)just_rx();
    (void)just_rx();

    return NULL;
}

char *sd_driver_read_sector(uint32_t sector, void *buffer) {
    unsigned char *b = (unsigned char*)buffer;

    /* CMD17 reads a single block. */
    if (sd_command(17, sector, 0) != 0x00) {
        sd_select(false);
        return "Unexpected response from card, expected 0x00 after CMD17";
    }
    
    char *retval = sd_get_next_block(buffer);
    
    sd_select(false);

    return retval;
}

char *sd_driver_read_sectors(
    uint32_t sector, uint32_t n,
    void *buffer,
    void (*progress)(uint32_t n_done)
) {
    uint32_t n_done = 0;

    /* CMD18 reads blocks until CMD12. */
    if (sd_command(18, sector, 0) != 0x00) {
        sd_select(false);
        return "Unexpected response from card, expected 0x00 after CMD19";
    }

    /* Read the blocks we need. */
    if (progress) {
        progress(n_done);
    }
    while(n_done < n) {
        char *r = sd_get_next_block(buffer + (n_done*512));
        if (r != NULL) {
            sd_select(false);
            return r;
        }
        n_done++;
        if (progress) {
            progress(n_done);
        }
    }

    /* Send CMD12. This command is weird, so we don't use sd_command().
     * CMD12 has a stuff byte and it also signals busy by sending 0s.
     */
    (void)spi_driver_xfer(0x40 | 12);
    (void)spi_driver_xfer(0);
    (void)spi_driver_xfer(0);
    (void)spi_driver_xfer(0);
    (void)spi_driver_xfer(0);
    (void)spi_driver_xfer(0);
    (void)just_rx();

    /* Wait for response byte. */
    unsigned char response;
    do {
        response = just_rx();
    } while (response == 0xFF);

    /* Provide clock until the card is no longer busy. */
    while (just_rx() != 0xFF);

    /* Unselect the card and check CMD12 response. */
    sd_select(false);
    if (response != 0x00) {
        return "Unexpected response from card, expected 0x00 after CMD12";
    }

    return NULL;
}