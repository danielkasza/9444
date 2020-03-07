# 9444 Machine Mode Firmware

## Toolchain

The firmware is built using the [RISC-V GNU toolchain](https://github.com/riscv/riscv-gnu-toolchain).

This is how I installed it:

```
sudo apt-get install autoconf automake autotools-dev curl python3 libmpc-dev libmpfr-dev libgmp-dev gawk build-essential bison flex texinfo gperf libtool patchutils bc zlib1g-dev libexpat-dev git
git clone --recursive https://github.com/riscv/riscv-gnu-toolchain
cd riscv-gnu-toolchain/
./configure --with-arch=rv64ima --with-cmodel=medany --prefix=/home/dkasza/9444-tools/
make -j25
```

**NOTE:** You probably want to change the prefix!

Before use:

```
export PATH=$PATH:~/9444-tools/bin
```