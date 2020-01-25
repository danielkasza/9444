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

.section .init
.globl _real_start
_real_start:
.option push
.option norelax
    la      sp, stack_top
.option pop
    /* Install exception handlers. */
    la      t0, exception_handler
    csrw    mtvec, t0
    la      t0, mtlblvec_handler
    csrw    0x7C3, t0 // mtlblvec
    la      t0, mtlbsvec_handler
    csrw    0x7C4, t0 // mtlbsvec
    j       _start

/* Our exception handler entry point. */
.section .text
.align 5
exception_handler:
    // TODO: fast track common exceptions
    /* Swap stack pointers, so we can save the context. */
    csrrw   sp, mscratch, sp
    /* Reserve space for 30 registers. */
    addi    sp, sp, -240
    /* Save registers except zero and sp. */
    sd      x31,   0(sp)
    sd      x30,   8(sp)
    sd      x29,  16(sp)
    sd      x28,  24(sp)
    sd      x27,  32(sp)
    sd      x26,  40(sp)
    sd      x25,  48(sp)
    sd      x24,  56(sp)
    sd      x23,  64(sp)
    sd      x22,  72(sp)
    sd      x21,  80(sp)
    sd      x20,  88(sp)
    sd      x19,  96(sp)
    sd      x18, 104(sp)
    sd      x17, 112(sp)
    sd      x16, 120(sp)
    sd      x15, 128(sp)
    sd      x14, 136(sp)
    sd      x13, 144(sp)
    sd      x12, 152(sp)
    sd      x11, 160(sp)
    sd      x10, 168(sp)
    sd       x9, 176(sp)
    sd       x8, 184(sp)
    sd       x7, 192(sp)
    sd       x6, 200(sp)
    sd       x5, 208(sp)
    sd       x4, 216(sp)
    sd       x3, 224(sp)
    sd       x1, 232(sp)
    /* Restore gp. */
.option push
.option norelax
    la      gp, __global_pointer$
.option pop
    /* Switch to C handler. */
    move    a2, a7
    csrr    a3, mcause
    call    handle_exception
    /* Restore context of supervisor. */
    ld      x31,   0(sp)
    ld      x30,   8(sp)
    ld      x29,  16(sp)
    ld      x28,  24(sp)
    ld      x27,  32(sp)
    ld      x26,  40(sp)
    ld      x25,  48(sp)
    ld      x24,  56(sp)
    ld      x23,  64(sp)
    ld      x22,  72(sp)
    ld      x21,  80(sp)
    ld      x20,  88(sp)
    ld      x19,  96(sp)
    ld      x18, 104(sp)
    ld      x17, 112(sp)
    ld      x16, 120(sp)
    ld      x15, 128(sp)
    ld      x14, 136(sp)
    ld      x13, 144(sp)
    ld      x12, 152(sp)
    ld      x11, 160(sp)
    ld      x10, 168(sp)
    ld       x9, 176(sp)
    ld       x8, 184(sp)
    ld       x7, 192(sp)
    ld       x6, 200(sp)
    ld       x5, 208(sp)
    ld       x4, 216(sp)
    ld       x3, 224(sp)
    ld       x1, 232(sp)
    addi    sp, sp, 240
    csrrw   sp, mscratch, sp
    mret

.align 5
mtlblvec_handler:
#include "mtlbvec.S"

.align 5
mtlbsvec_handler:
#define S_TLB_MISS_HANDLER
#include "mtlbvec.S"

/* Used to drop down to S mode the first time. */
.globl start_supervisor
start_supervisor:
    /* Place our stack pointer in scratch register. */
    la      t0, stack_top
    csrw    mscratch, t0
    /* Set address of first instruction in S mode. */
    csrw    mepc, a0
    li      a0, 0
    /* Set MPP = S. */
    li      t0, 20000
    csrw    mstatus, t0
    /* Delegate all possible exceptions to S mode.
     * Note: TLB exceptions, and ecalls from S and M mode still come back to M mode.
     */
    li      t0, -1
    csrw    mideleg, t0
    li      t0, 0xFFFFF5FF
    csrw    medeleg, t0
    /* Drop down to S mode. */
    mret

/* Our stack. */
.section .data
.align 5
.space 4096
stack_top: