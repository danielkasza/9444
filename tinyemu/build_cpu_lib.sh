#!/bin/bash

set -e
set -x

cd `dirname $0`

verilator                         --Mdir cpu_lib       -Os -CFLAGS "-fno-pic -march=native -mtune=native -flto -O2 -fno-stack-protector" -I../cpu --cc ../cpu/cpu.sv
verilator --trace --trace-depth 2 --Mdir cpu_lib_trace -Os -CFLAGS "-fno-pic -march=native -mtune=native -flto -O2 -fno-stack-protector" -I../cpu --cc ../cpu/cpu.sv

cd cpu_lib
make -f Vcpu.mk

cd ../cpu_lib_trace
make -f Vcpu.mk
