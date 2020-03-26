#!/usr/bin/env python3

# Copyright (c) 2019, Daniel Kasza
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Combine kernel image and dtb and put a header on it, so it can be loaded by the firmware from a memory card.
# Note: this is only needed if the platform does not suport Ext2.
#
# Usage: ./mk9444image.py image-description kernel-image dtb load-address-hex output-file

import sys
import math
import os
from struct import *

descr   = sys.argv[1]
kernel  = open(sys.argv[2], "rb")
dtb     = open(sys.argv[3], "rb")
addr    = int(sys.argv[4], 16)
outfile = open(sys.argv[5], "wb")

def roundup(a, b):
    return a - (a % -b)

# Figure out the final layout.
ks   = os.path.getsize(sys.argv[2])
dtbs = os.path.getsize(sys.argv[3])

kernel_size = roundup(ks, 512)
kernel_pad  = kernel_size - ks
dtb_size    = roundup(dtbs, 512)
dtb_pad     = dtb_size - dtbs
dtb_addr    = addr + kernel_size

sector_count = int((dtb_size + kernel_size)/512)

# Write the top of the bootblock.
magic = 0x000000004B444494
bootblocktop = pack("<qqqqq",
                    magic,
                    sector_count,
                    addr,
                    addr,
                    dtb_addr)
outfile.write(bootblocktop)

# Write reserved fields.
reserved = pack ("q", 0)
for i in range(0, 27):
    outfile.write(reserved)

# Write description string.
descr_len = len(descr)
if descr_len > 255:
    raise Exception("description is too long")
outfile.write(str.encode(descr, "ASCII"))

pad_len = 256-descr_len
pad = pack("B", 0)
for i in range(0, pad_len):
    outfile.write(pad)

# Write kernel
outfile.write(kernel.read())
for i in range(0, kernel_pad):
    outfile.write(pad)

# Write device tree
outfile.write(dtb.read())
for i in range(0, dtb_pad):
    outfile.write(pad)
