/* Copyright (c) 2019, 2020, Daniel Kasza
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

/* 9444 internal registers.
 * These map to RISC-V CSRs.
 */
`ifndef _IREG_SV
`define _IREG_SV

package ireg;

/* Internal numbers for implemented registers. */
typedef enum logic [5:0] {
    /* (rvpriv.3.1.1) Machine ISA Register.
     * Implemented as read-only, writes ignored.
     */
    misa,
    /* (rvpriv.3.1.2) Machive Vendor ID Register.
     * Read-only.
     */
    mvendorid,
    /* (rvpriv.3.1.3) Machine Architecture ID Register.
     * Read-only.
     */
    marchid,
    /* (rvpriv.3.1.4) Machine Implementation ID Register.
     * Read-only.
     */
    mimpid,
    /* (rvpriv.3.1.5) Hart ID Register.
     * Read-only.
     */
    mhartid,
    /* (rvpriv.3.1.6) Machine Status Register.
     * Read-write.
     */
    mstatus,
    /* (rvpriv.3.1.12) Machine Trap-Vector Base-Address Register.
     * Read-write.
     */
    mtvec,
    /* (rvpriv.3.1.13) Machine Trap Delegation Registers.
     * Read-write.
     */
    medeleg,
    mideleg,
    /* (rvpriv.3.1.14) Machine Interrupt Registers.
     * Read-write.
     */
    mip,
    mie,
    /* (rvpriv.3.1.18) Machine Scratch Register.
     * Read-write.
     */
    mscratch,
    /* (rvpriv.3.1.19) Machine Exception Program Counter.
     * Read-write.
     */
    mepc,
    /* (rvpriv.3.1.20) Machine Cause Register.
     * Read-write.
     */
    mcause,
    /* (rvpriv.3.1.21) Machine Trap Value Register.
     * Read-write.
     */
    mtval,
    /* (rvpriv.4.1.1) Supervisor Status Register.
     * Read-write.
     */
    sstatus,
    /* (rvpriv.4.1.4) Supervisor Trap Vector Base Address Register.
     * Read-write.
     */
    stvec,
    /* (rvpriv.4.1.5) Supervisor Interrupt Registers.
     * Read-write.
     */
    sip,
    sie,
    /* (rvpriv.4.1.8) Supervisor Scratch Register. 
     * Read-write.
     */
    sscratch,
    /* (rvpriv.4.1.9) Supervisor Exception Program Counter.
     * Read-write.
     */
    sepc,
    /* (rvpriv.4.1.10) Supervisor Cause Register.
     * Read-write.
     */
    scause,
    /* (rvpriv.4.1.11) Supervisor Trap Value Register.
     * Read-write.
     */
    stval,
    /* (rvpriv.4.1.12) Supervisor Address Translation and Protection Register.
     * Read-write.
     */
    satp,
    /* 9444 specific.
     * Supervisor next event cycle register.
     */
    snecycle,
    /* (rvpriv.Table 2.2) Cycle counter for RDCYCLE instruction. */
    /* (rvpriv.Table 2.2) Timer for RDTIME instruction. */
    cycle,
    time_,

    /* 9444 TLB registers. *********************************************************************************************
     * These are only accessible in machine mode.
     * Write-only. Reads return unpredictable values.
     */
    /* Set 4KB, 2MB, 1GB page table entries.
     * Only the lower 32b are used.
     * The entry configured is picked pseudo-randomly if the exception was caused by a TLB miss.
     * The entry configured matches the faulting entry if the exception was caused by a missing Dirty flag.
     * Reading back the entries is not possible.
     * The virtual address is picked from mtval.
     */
    mtlbw4k,
    mtlbw2m,
    mtlbw1g,
    /* TLB exception entry points. */
    mtlblvec, /* L_TLB_MISS */
    mtlbsvec, /* S_TLB_MISS, S_TLB_NOT_DIRTY */
    /* 9444 PTE registers **********************************************************************************************
     * These are only accessible in machine mode.
     * These are provided to accelerate TLB reload operations.
     * Read-only.
     */
    /* Address of 1st level page table entry. */
    ml1pteaddr,
    /* Offsets of 2nd, and 3rd level page table entries. */
    ml2pteoff,
    ml3pteoff,

`ifdef WORKAROUND_X_VALUES
    x = 0
`else
    x = 6'bxxxxxx
`endif
} ireg_t;

endpackage

/* RISC-V CSRs.
 * (rvpriv.2.2) CSR Listing.
 */
package csr;

localparam [11:0] cycle      = 12'hC00;
localparam [11:0] time_      = 12'hC01;

localparam [11:0] sstatus    = 12'h100;
localparam [11:0] sedeleg    = 12'h102;
localparam [11:0] sideleg    = 12'h103;
localparam [11:0] sie        = 12'h104;
localparam [11:0] stvec      = 12'h105;
localparam [11:0] scounteren = 12'h106;
localparam [11:0] sscratch   = 12'h140;
localparam [11:0] sepc       = 12'h141;
localparam [11:0] scause     = 12'h142;
localparam [11:0] stval      = 12'h143;
localparam [11:0] sip        = 12'h144;
localparam [11:0] satp       = 12'h180;

localparam [11:0] snecycle   = 12'h5C0;

localparam [11:0] mvendorid  = 12'hF11;
localparam [11:0] marchid    = 12'hF12;
localparam [11:0] mimpid     = 12'hF13;
localparam [11:0] mhartid    = 12'hF14;
localparam [11:0] mstatus    = 12'h300;
localparam [11:0] misa       = 12'h301;
localparam [11:0] medeleg    = 12'h302;
localparam [11:0] mideleg    = 12'h303;
localparam [11:0] mie        = 12'h304;
localparam [11:0] mtvec      = 12'h305;
localparam [11:0] mcounteren = 12'h306;
localparam [11:0] mscratch   = 12'h340;
localparam [11:0] mepc       = 12'h341;
localparam [11:0] mcause     = 12'h342;
localparam [11:0] mtval      = 12'h343;
localparam [11:0] mip        = 12'h344;
localparam [11:0] mcycle     = 12'hB00;
localparam [11:0] minstret   = 12'hB02;

localparam [11:0] mtlbw4k    = 12'h7C0;
localparam [11:0] mtlbw2m    = 12'h7C1;
localparam [11:0] mtlbw1g    = 12'h7C2;
localparam [11:0] mtlblvec   = 12'h7C3;
localparam [11:0] mtlbsvec   = 12'h7C4;

localparam [11:0] ml1pteaddr = 12'hFC0;
localparam [11:0] ml2pteoff  = 12'hFC1;
localparam [11:0] ml3pteoff  = 12'hFC2;


endpackage

`endif /* _IREG_SV */
