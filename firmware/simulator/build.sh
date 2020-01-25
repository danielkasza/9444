#!/bin/bash

set -e
set -x

cd `dirname $0`

riscv64-unknown-elf-gcc                                                         \
    -mcmodel=medany                                                             \
    -Os                                                                         \
    -o "firmware.elf"                                                           \
    -Wl,-Ttext=0x80000100,--section-start=.init=0x80000000                      \
    -ffreestanding                                                              \
    -I..                                                                        \
    ../init.S main.c ../banner.c

riscv64-unknown-elf-objcopy -O binary "firmware.elf" "firmware.bin"
