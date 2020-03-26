/* Copyright (c) 2020, Daniel Kasza
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

#include "util.h"
#include <stddef.h>

void printstr(const char *str) {
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

char hex_table[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
void printx32(uint32_t val) {
    unsigned i;
    for (i=0; i<8; i++) {
        printc(hex_table[(val >> (28 - (4*i))) & 0xF]);
    }
}
void printx64(uint64_t val) {
    unsigned i;
    for (i=0; i<16; i++) {
        printc(hex_table[(val >> (60 - (4*i))) & 0xF]);
    }
}

void printlx32(const char *label, uint32_t val) {
    printstr(label);
    printstr(": ");
    printx32(val);
    printstr("\n");
}

 void printlx64(const char *label, uint64_t val) {
    printstr(label);
    printstr(": ");
    printx64(val);
    printstr("\n");
}

void check_str_result(const char *result) {
    if (result != NULL) {
        printstr("FAILED: ");
        printstr(result);
        printstr("\n");
        for(;;);
    } else {
        printstr("DONE!\n");
    }
}