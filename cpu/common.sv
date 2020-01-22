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

/* 9444 common types.
 * This is expected to be included by most sources.
 */
`ifndef _COMMON_SV
`define _COMMON_SV

/* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
/* Uncomment the next line if running Xilinx IP packager or other tool that does not like x values.
 * In case of the IP packager, you can comment the line again after you are done with the packaging.
 */
//`define WORKAROUND_X_VALUES 1
/* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

`define XLEN 64

/* Number of bits in virtual address. */
`define VLEN 39

/* Number of bits in physical address. */
`define PLEN 33

package mode;

/* RISC-V execution mode. */
typedef enum logic [1:0] {
    /* User. */
    U,
    /* Supervisor. */
    S,
    /* Reserved. */
    R,
    /* Machine. */
    M
} mode_t;

endpackage

/* MMU interface types. ***********************************************************************************************/
package mmu;

/* Supported page sizes. */
typedef enum [1:0] {
    PSIZE_4KB,
    PSIZE_2MB,
    PSIZE_1GB,
`ifdef WORKAROUND_X_VALUES
    PSIZE_X = 0
`else
    PSIZE_X = 2'bxx
`endif
} psize_t;

endpackage

/* ALU interface types. ***********************************************************************************************/
package alu;

/* Operations supported by the ALU. */
typedef enum logic [2:0] {
    ADD = 3'b000,
    XOR = 3'b001,
    OR  = 3'b010,
    AND = 3'b011,
    SUB = 3'b100
} op_t;
endpackage

/* Divider interface types. *******************************************************************************************/
package div;

/* Operations supported by the divider. */
typedef enum logic [0:0] {
    DIV,
    DIVU
} op_t;
endpackage

/* Multiplier interface types. ****************************************************************************************/
package mul;

/* Operations supported by the multiplier. */
typedef enum logic [1:0] {
    MUL    = 2'b00,
    MULH   = 2'b01,
    MULHSU = 2'b10,
    MULHU  = 2'b11
} op_t;
endpackage

/* Bit-shifter interface types. ***************************************************************************************/
package shifter;

/* Operations supported by the bit-shifter. */
typedef enum logic [1:0] {
    SLL,
    SRL,
    SRA
} op_t;
endpackage

/* Comparator interface types. ****************************************************************************************/
package comparator;

/* Operations supported by the comparator. */
typedef enum logic [2:0] {
    EQ   = 3'b000, /* a == b */
    NE   = 3'b001, /* a != b */
    LT   = 3'b100, /* a <  b */
    LTU  = 3'b110, /* a <  b, unsigned */
    GE   = 3'b101, /* a >= b */
    GEU  = 3'b111, /* a >= b, unsigned */
    TRUE = 3'b011, /* return 1 */ 
`ifdef WORKAROUND_X_VALUES
    X    = 0
`else
    X    = 3'bxxx
`endif
} op_t;
endpackage

/* Execute stage insterface types. ************************************************************************************/
package execute;

/* Selects which output is the "result" that could be saved in a register. */
typedef enum logic [3:0] {
    COMPARATOR,
    SC_RESULT,
    ALU,
    DIV_Q,
    DIV_R,
    MUL,
    SHIFTER,
    MEMORY,
    INTERNAL_REGISTER,
    RETURN_ADDRESS,
`ifdef WORKAROUND_X_VALUES
    RESULT_X = 0
`else
    RESULT_X = 4'bxxxx
`endif
} result_select_t;

/* Memory access type. */
typedef enum logic [3:0] {
    /* 1B read/write. */
    BYTE_READ,
    BYTE_WRITE,
    /* 2B read/write. */
    HWORD_READ,
    HWORD_WRITE,
    /* 4B read/write. */
    WORD_READ,
    WORD_WRITE,
    /* 8B read/write. */
    DWORD_READ,
    DWORD_WRITE,
    /* 32B (cache line) read/write. */
    LINE_READ,
    // TODO: rest of the encoding space here will be used for cache management operations

`ifdef WORKAROUND_X_VALUES
    MEM_X = 0
`else
    MEM_X = 4'bxxxx
`endif
} memory_access_t;

endpackage

/* Internal register file insterface types. ***************************************************************************/
package ireg_file;

/* Operations supported by internal register file. */
typedef enum logic [1:0] {
    /* Do nothing. */
    NOP = 2'b00,
    /* Read/Write. */
    RW  = 2'b01,
    /* Read and set bits. */
    RS  = 2'b10,
    /* Read and clear bits. */
    RC  = 2'b11
} op_t;

endpackage

/* Exception types. ***************************************************************************************************/

package exception;

/* (rvpriv.Table 3.6) Machine cause register (mcause) values after trap.
 * Only the synchronous (interrupt = 0) values are defined here.
 */
typedef enum logic[5:0] {
    /* Instruction address misaligned. */
    I_ADDR_MISALIGNED = 0,
    /* Instruction access fault. */
    I_ACCESS_FAULT = 1,
    /* Illegal instruction. */
    I_ILLEGAL = 2,
    /* Breakpoint. */
    BREAKPOINT = 3,
    /* Load address misaligned. */
    L_ADDR_MISALIGNED = 4,
    /* Load access fault. */
    L_ACCESS_FAULT = 5,
    /* Store/AMO address misaligned. */
    S_ADDR_MISALIGNED = 6,
    /* Store/AMO access fault. */
    S_ACCESS_FAULT = 7,
    /* Environment call from U-mode. */
    U_CALL = 8,
    /* Environment call from S-mode. */
    S_CALL = 9,
    /* Environment call from M-mode. */
    M_CALL = 11,
    /* Instruction page fault. */
    I_PAGE_FAULT = 12,
    /* Load page fault. */
    L_PAGE_FAULT = 13,
    /* Store/AMO page fault. */
    S_PAGE_FAULT = 15,
    /* TLB miss exceptions, not standard.
     * Machine mode is expected to set the accessed bit every time a TLB is filled.
     */
    /* TLB miss on load.
     * Machine mode is expected fill a TLB without modifying the dirty bit.
     */
    L_TLB_MISS = 61,
    /* TLB miss on store.
     * Machine mode is expected to set the dirty bit and fill a TLB.
     */
    S_TLB_MISS = 62,
    /* Store page fault caused by dirty bit that was not set.
     * Machine mode is expected to set the dirty bit and refill the same TLB.
     */
    S_TLB_NOT_DIRTY = 63
} sync_codes_t;

/* (rvpriv.Table 3.6) Machine cause register (mcause) values after trap.
 * Only the asynchronous (interrupt = 1) values are defined here.
 */
typedef enum logic[5:0] {
    /* User software interrupt. */
    U_INT_SW = 0,
    /* Supervisor software interrupt. */
    S_INT_SW = 1,
    /* Machine software interrupt. */
    M_INT_SW = 3,
    /* User timer interrupt. */
    U_INT_TIMER = 4,
    /* Supervisor timer interrupt. */
    S_INT_TIMER = 5,
    /* Machine timer interrupt. */
    M_INT_TIMER = 7,
    /* User external interrupt. */
    U_INT_EXT = 8,
    /* Supervisor external interrupt. */
    S_INT_EXT = 9,
    /* Machine external interrupt. */
    M_INT_EXT = 11
} async_codes_t;

endpackage

`endif /* _COMMON_SV */