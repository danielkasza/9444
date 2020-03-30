#!/bin/bash

set -e
set -x

cd `dirname $0`

# Build the DTB
dtc -I dts -O dtb genesys2.dts -o genesys2.dtb

# Convert the DTB to a C array, so it can be embedded in the firmware
xxd -i genesys2.dtb > dtb.c

# Compile the firmware
riscv64-unknown-elf-gcc                                                         \
    -DTARGET_GENESYS2                                                           \
    -mcmodel=medlow                                                             \
    -Os                                                                         \
    -o "firmware.elf"                                                           \
    -Wl,-Ttext=0x00000100,--section-start=.init=0x00000000                      \
    -ffreestanding                                                              \
    -I..                                                                        \
    -I../ext2                                                                   \
    main.c dtb.c                                                                \
    ../init.S ../xilinx_spi_driver.c ../sd_driver.c ../banner.c ../util.c       \
    ../exception.c                                                              \
    ../ext2/ext2.c

# Convert it to a flat binary
riscv64-unknown-elf-objcopy -O binary "firmware.elf" "firmware.bin"

# Convert it to a COE file that Xilinx tools will understand
gcc ../makecoe.c -o makecoe
./makecoe > firmware.coe

