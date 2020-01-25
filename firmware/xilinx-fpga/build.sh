#!/bin/bash

set -e
set -x

cd `dirname $0`

riscv64-unknown-elf-gcc                                                         \
    -mcmodel=medlow                                                             \
    -Os                                                                         \
    -o "firmware.elf"                                                           \
    -Wl,-Ttext=0x00000100,--section-start=.init=0x00000000                      \
    -ffreestanding                                                              \
    -I..                                                                        \
    ../init.S main.c ../xilinx_spi_driver.c ../sd_driver.c ../banner.c

riscv64-unknown-elf-objcopy -O binary "firmware.elf" "firmware.bin"

gcc ../makecoe.c -o makecoe
./makecoe > firmware.coe

dtc -I dts -O dtb genesys2.dts -o genesys2.dtb
