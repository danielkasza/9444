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

#include <stdio.h>

char firmware_data[32*1024] = { 0 };

int main(int argc, char *argv[]) {
    FILE *binfile = fopen("firmware.bin", "r");

    if (binfile == NULL) {
        perror("fopen");
        return 1;
    }

    size_t count = fread(firmware_data, 1, sizeof(firmware_data), binfile);
    if(count == 0) {
        perror("fread");
        return 1;
    }

    printf("memory_initialization_radix=16;\n");
    printf("memory_initialization_vector=\n");

    unsigned i;
    for (i=0; i<sizeof(firmware_data)/32; i++) {
        if (i != 0) {
            printf(",\n");
        }
        unsigned j;
        for (j=0; j<32; j++) {
            unsigned char this_char = firmware_data[(i*32) + (31-j)];
            printf("%02x", this_char);
        }
    }

    printf(";\n");

    return 0;
}