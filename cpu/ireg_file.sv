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

/* 9444 internal register file. */
`ifndef _IREG_FILE_SV
`define _IREG_FILE_SV

`include "ireg.sv"
`include "common.sv"

module IRegFile(
    input                   clock,
    input                   stall,
    output mode::mode_t     current_mode = mode::M,

    /* Operation to perform. */
    input ireg_file::op_t   op,
    /* Selected internal register. */
    input ireg::ireg_t      sel,

    /* Input for the operation. */
    input  [`XLEN-1:0]      op_val,

    /* Current value of the register selected. */
    output reg [`XLEN-1:0]  out,

    /** INTERRUPT INTERFACE. **/
    /* Interrupt enables. */
    output                  m_eie,
    output                  m_tie,
    output                  s_eie,
    output                  s_tie,
    /* Interrupt lines. */
    output                  m_timer,
    output reg              s_timer,
    input                   m_interrupt,
    input                   s_interrupt,

    /** EXCEPTION INTERFACE. **/
    input                   s_ret,
    input                   m_ret,
    input  [`XLEN-1:1]      instruction_vaddr,
    input  [31:2]           instruction_word,
    input  [`XLEN-1:0]      add_result,
    input                   exception_pending,
    input                   interrupt_exception,
    input  [5:0]            exception_code,
    output reg [`XLEN-1:2]  exception_vaddr,

    /* MMU INTERFACE. **/
    output                  mmu_sum,
    output                  mmu_u_mode,
    output                  mmu_enabled,
    output [4:0]            mmu_entry,
    output [`VLEN-1:12]     mmu_vpn_in,
    output mmu::psize_t     mmu_psize_in,
    output [31:0]           mmu_pte_in,
    output                  mmu_pte_write,
    input  [ 4:0]           mmu_match_entry
);

/* Output the current value of the selected register. */
always_comb begin
    case (sel)
        ireg::misa:
            out = misa;
        ireg::mvendorid:
            out = 0;
        ireg::marchid:
            out = 0;
        ireg::mimpid:
            out = 0;
        ireg::mhartid:
            out = 0;
        ireg::mstatus:
            out = mstatus;
        ireg::mtvec:
            out = {31'b0, mtvec, 2'b0};
        ireg::medeleg:
            out = medeleg;
        ireg::mideleg:
            out = mideleg;
        ireg::mip:
            out = mip;
        ireg::mie:
            out = mie;
        ireg::mscratch:
            out = mscratch;
        ireg::mepc:
            out = {mepc, 2'b0};
        ireg::mcause:
            out = mcause;
        ireg::mtval:
            out = mtval;
        ireg::sstatus:
            out = sstatus;
        ireg::stvec:
            out = {{25{stvec[`VLEN-1]}},stvec, 2'b0};
        ireg::sip:
            out = sip;
        ireg::sie:
            out = sie;
        ireg::sscratch:
            out = sscratch;
        ireg::sepc:
            out = {sepc, 2'b0};
        ireg::scause:
            out = scause;
        ireg::stval:
            out = stval;
        ireg::satp:
            out = satp;
        ireg::snecycle:
            out = supervisor_next_event_cycle;
        ireg::cycle,
        ireg::time_:
            out = cycle_counter;
        ireg::ml1pteaddr:
            out = ml1pteaddr;
        ireg::ml2pteoff:
            out = ml2pteoff;
        ireg::ml3pteoff:
            out = ml3pteoff;

        /* Reading TLB registers is unpredictable. */

        default:
            /* Bad register selected. Output something. */
            out = 64'hxxxxxxxxxxxxxxxx;
    endcase
end

/* Figure out the new value. */
reg [`XLEN-1:0] new_value;
always_comb begin
    case (op)
        ireg_file::NOP:    new_value = 64'hxxxxxxxxxxxxxxxx;
        ireg_file::RW:     new_value = op_val;
        ireg_file::RS:     new_value = out | op_val;
        ireg_file::RC:     new_value = out & (~op_val);
    endcase
end

always @(posedge clock) begin
    if (!stall) begin
        current_mode <= next_mode;
        if (!exception_pending) begin
            /* Write back the register, if needed and the register is writeable. */
            if (op != ireg_file::NOP) case (sel)
                ireg::mstatus:
                    begin
                        status_sie  <= new_value[1];
                        status_mie  <= new_value[3];
                        status_spie <= new_value[5];
                        status_mpie <= new_value[7];
                        status_spp  <= new_value[8];
                        status_mpp  <= new_value[12:11];
                        status_sum  <= new_value[18];
                    end
                ireg::mtvec:
                    mtvec <= new_value[`PLEN-1:2];
                ireg::medeleg:
                    medeleg_implemented <= new_value[15:0];
                ireg::mideleg:
                    mideleg_implemented <= new_value[11:0];
                ireg::mie:
                    begin
                        stie <= new_value[ 5];
                        mtie <= new_value[ 7];
                        seie <= new_value[ 9];
                        meie <= new_value[11];
                    end
                ireg::mscratch:
                    mscratch <= new_value;
                ireg::mepc:
                    mepc <= new_value[`XLEN-1:2];
                ireg::mcause:
                    begin
                        mcause_code <= new_value[5:0];
                        mcause_interrupt <= new_value[31];
                    end
                ireg::mtval:
                    mtval <= new_value;

                ireg::sstatus:
                    begin
                        status_sie  <= new_value[1];
                        status_spie <= new_value[5];
                        status_spp  <= new_value[8];
                        status_sum  <= new_value[18];
                    end
                ireg::stvec:
                    stvec <= new_value[`VLEN-1:2];
                ireg::sie:
                    begin
                        stie <= new_value[ 5];
                        seie <= new_value[ 9];
                    end
                ireg::sscratch:
                    sscratch <= new_value;
                ireg::sepc:
                    sepc <= new_value[`XLEN-1:2];
                ireg::scause:
                    begin
                        scause_code <= new_value[5:0];
                        scause_interrupt <= new_value[31];
                    end
                ireg::stval:
                    stval <= new_value;
                ireg::satp:
                    begin
                        automatic reg [3:0] new_mode = new_value[63:60];
                        if (new_mode == 8 || new_mode == 0) begin
                            satp_en  <= (new_mode == 8);
                            satp_ppn <= new_value[20:0];
                        end
                    end
                ireg::snecycle:
                    supervisor_next_event_cycle  <= new_value;

                /* TLB exception entry points. */
                ireg::mtlblvec:
                    mtlblvec <= new_value[`PLEN-1:2];
                ireg::mtlbsvec:
                    mtlbsvec <= new_value[`PLEN-1:2];

                /* Other TLB registers are handled by MMU. */

                default: begin /* Do nothing. */ end
            endcase
        end
        /* Exception logic. */
        else if (next_mode==mode::M && !m_ret) begin
            mepc <= instruction_vaddr[`XLEN-1:2];
            status_mie  <= 0;
            status_mpie <= status_mie;
            status_mpp  <= current_mode;
            mcause_interrupt <= interrupt_exception;
            mcause_code      <= exception_code;

            if (exception_code == exception::S_TLB_NOT_DIRTY) begin
                /* Record matching TLB index, so we don't have to search the entire TLB. */
                tlbindex <= mmu_match_entry;
            end
            else begin
                /* Set a random TLB index. */
                tlbindex <= cycle_counter[4:0];
            end

            if (!interrupt_exception) case (exception_code)
                exception::I_ADDR_MISALIGNED:   mtval <= {instruction_vaddr, 1'b0};
                /* Despite the ISA spec allowing the instruction cache to be compressed by dropping the bottom 2 bits,
                 * this will expose this detail to code running on the target.
                 * This should not be an issue in practice though.
                 */
                exception::I_ILLEGAL:           mtval <= {32'b0, instruction_word, 2'b11};

                /* This is not entirely compliant to the spec, but our machine mode is special anyway. */
                default:                        mtval <= add_result;
            endcase
            else begin
                mtval <= 0;
            end
        end
        else if (m_ret) begin
            status_mie  <= status_mpie;
            status_mpie <= 1;
            status_mpp  <= mode::U;
        end
        else if (exception_pending && next_mode==mode::S && !s_ret) begin
            sepc <= instruction_vaddr[`XLEN-1:2];
            status_sie  <= 0;
            status_spie <= status_sie;
            status_spp  <= current_mode[0];
            scause_interrupt <= interrupt_exception;
            scause_code      <= exception_code;

            if (!interrupt_exception) case (exception_code)
                exception::I_ADDR_MISALIGNED:   stval <= {instruction_vaddr, 1'b0};
                /* Despite the ISA spec allowing the instruction cache to be compressed by dropping the bottom 2 bits,
                 * this will expose this detail to code running on the target.
                 * This should not be an issue in practice though.
                 */
                exception::I_ILLEGAL:           stval <= {32'b0, instruction_word, 2'b11};
                exception::L_ADDR_MISALIGNED,
                exception::S_ADDR_MISALIGNED,
                exception::I_PAGE_FAULT,
                exception::L_PAGE_FAULT,
                exception::S_PAGE_FAULT:        stval <= add_result;

                default:                        stval <= 0;
            endcase
            else begin
                stval <= 0;
            end
        end
        else if (s_ret) begin
            status_sie  <= status_spie;
            status_spie <= 1;
            status_spp  <= 0;
        end
    end
end

/* Counter for time and cycle CSRs. */
reg [63:0] cycle_counter = 0;
always @(posedge clock) begin
    cycle_counter <= cycle_counter + 1;
end

/* Supervisor's timer. */
reg [63:0] supervisor_next_event_cycle = 0;
always @(posedge clock) begin
    s_timer <= (cycle_counter >= supervisor_next_event_cycle);
end

// TODO: machine mode timer
assign     m_timer = 0;

/* (rvpriv.3.1.1) Machine ISA Register value.
 * This is read-only.
 */
wire [`XLEN-1:0] misa = {
        2'b10,       /* MXL = 64b */
        36'b0000,    /* Reserved. */
        /* Extensions.
         *  ZYXWVUTSRQPONMLKJIHGFEDCBA */
        26'b00000000000001000100000001
    };

/* (rvpriv.3.1.6) Machine Status Register. */
wire [`XLEN-1:0] mstatus = {
    45'b0,
    status_sum,
    1'b0,
    4'b0,
    status_mpp,
    2'b0,
    status_spp,
    status_mpie,
    1'b0,
    status_spie,
    1'b0,
    status_mie,
    1'b0,
    status_sie,
    1'b0
};
reg             status_sie  = 0;
reg             status_mie  = 0;
reg             status_spie = 0;
reg             status_mpie = 0;
reg             status_spp  = 0;
mode::mode_t    status_mpp  = mode::U;
reg             status_sum  = 0;
/* MPRV, MXR, TVM, TW, TSR, and SD are not implemented. */

/* (rvpriv.3.1.12) Machine Trap-Vector Base-Address Register.
 * MODE field is not implemented, so only "direct" mode is supported.
 */
reg [`PLEN-1:2] mtvec = 0;

/* (rvpriv.3.1.13) Machine Trap Delegation Registers. */
wire [`XLEN-1:0] medeleg = {48'b0, medeleg_implemented};
reg  [15:0] medeleg_implemented = 0;
wire [`XLEN-1:0] mideleg = {52'b0, mideleg_implemented};
reg  [11:0] mideleg_implemented = 0;

/* (rvpriv.3.1.14) Machine Interrupt Registers. */
wire [`XLEN-1:0] mip = {
    52'b0,
    m_interrupt,
    1'b0,
    s_interrupt,
    1'b0,
    m_timer,
    1'b0,
    s_timer,
    5'b0
};
wire [`XLEN-1:0] mie = {
    52'b0,
    meie,
    1'b0,
    seie,
    1'b0,
    mtie,
    1'b0,
    stie,
    5'b0
};
reg meie = 0;
reg seie = 0;
reg mtie = 0;
reg stie = 0;

assign m_eie = meie && status_mie;
assign m_tie = mtie && status_mie;
assign s_eie = seie && status_sie;
assign s_tie = stie && status_sie;

/* (rvpriv.3.1.18) Machine Scratch Register. */
reg [`XLEN-1:0] mscratch = 0;

/* (rvpriv.3.1.19) Machine Exception Program Counter. */
reg [`XLEN-1:2] mepc = 0;

/* (rvpriv.3.1.20) Machine Cause Register. */
wire [`XLEN-1:0] mcause = {mcause_interrupt, 57'b0, mcause_code};
reg              mcause_interrupt = 0;
reg  [5:0]       mcause_code = 0;

/* (rvpriv.3.1.21) Machine Trap Value Register. */
reg [`XLEN-1:0] mtval = 0;

/* (rvpriv.4.1.1) Supervisor Status Register. */
wire [`XLEN-1:0] sstatus = {
    45'b0,
    status_sum,
    9'b0,
    status_spp,
    2'b0,
    status_spie,
    3'b0,
    status_sie,
    1'b0
};

/* (rvpriv.4.1.4) Supervisor Trap Vector Base Address Register. */
reg [`VLEN-1:2] stvec = 0;

/* (rvpriv.4.1.5) Supervisor Interrupt Registers. */
wire [`XLEN-1:0] sip = {
    54'b0,
    s_interrupt,
    3'b0,
    s_timer,
    5'b0
};
wire [`XLEN-1:0] sie = {
    54'b0,
    seie,
    3'b0,
    stie,
    5'b0
};

/* (rvpriv.4.1.8) Supervisor Scratch Register. */
reg [`XLEN-1:0] sscratch = 0;

/* (rvpriv.4.1.9) Supervisor Exception Program Counter. */
reg [`XLEN-1:2] sepc = 0;

/* (rvpriv.4.1.10) Supervisor Cause Register. */
wire [`XLEN-1:0] scause = {scause_interrupt, 57'b0, scause_code};
reg              scause_interrupt = 0;
reg  [5:0]       scause_code = 0;

/* (rvpriv.4.1.11) Supervisor Trap Value Register. */
reg [`XLEN-1:0] stval = 0;

/* (rvpriv.4.1.12) Supervisor Address Translation and Protection Register.
 * ASID is not implemented. Existing operating systems are not using the ASID, so it would be just a waste.
 */
wire [   `XLEN-1: 0] satp = {satp_en ? 4'h8:4'b0, 39'b0, satp_ppn};
reg  [   `PLEN-1:12] satp_ppn  = 0;
reg                  satp_en   = 0;

/* TLB exception entry points. */
reg [`PLEN-1:2] mtlblvec = 0;
reg [`PLEN-1:2] mtlbsvec = 0;

/* EXCEPTION LOGIC. ***************************************************************************************************/

/* Figure out what mode we are switching to. */
mode::mode_t next_mode;
always_comb begin
    next_mode = current_mode;
    if (exception_pending) begin
        if (m_ret) begin
            next_mode = status_mpp;
        end
        else if (s_ret) begin
            next_mode = status_spp ? mode::S : mode::U;
        end
        else if (interrupt_exception) begin
            next_mode = mideleg[exception_code] ? mode::S : mode::M;
        end
        else begin
            next_mode = medeleg[exception_code] ? mode::S : mode::M;
        end
    end
end

/* Figure out what address we should be jumping to. */
always_comb begin
    exception_vaddr = 0;
    if (s_ret) begin
        exception_vaddr = sepc;
    end
    else if (m_ret) begin
        exception_vaddr = mepc;
    end
    else if (next_mode == mode::M) begin
        if (exception_code >= exception::S_TLB_MISS) begin
            exception_vaddr = {31'b0, mtlbsvec};;
        end
        else if (exception_code == exception::L_TLB_MISS) begin
            exception_vaddr = {31'b0, mtlblvec};;
        end
        else begin
            exception_vaddr = {31'b0, mtvec};
        end
    end
    else begin
        exception_vaddr = {{25{stvec[`VLEN-1]}}, stvec};
    end
end

/* MMU INTEGRATION. ***************************************************************************************************/

assign mmu_sum       = status_sum;
assign mmu_u_mode    = (current_mode == mode::U);
assign mmu_enabled   = (current_mode != mode::M) && satp_en;
assign mmu_entry     = tlbindex;
assign mmu_vpn_in    = mtval[`VLEN-1:12];
assign mmu_pte_in    = new_value[31:0];
assign mmu_pte_write  = (!stall) && mtlb_selected && (op != ireg_file::NOP);

reg  [ 4:0] tlbindex = 0;

reg mtlb_selected;
always_comb begin
    mtlb_selected = 1;
    case (sel)
        ireg::mtlbw4k:  mmu_psize_in = mmu::PSIZE_4KB;
        ireg::mtlbw2m:  mmu_psize_in = mmu::PSIZE_2MB;
        ireg::mtlbw1g:  mmu_psize_in = mmu::PSIZE_1GB;
        default:
        begin
            mmu_psize_in = mmu::PSIZE_X;
            mtlb_selected = 0;
        end
    endcase
end

/* PTE REGISTERS. *****************************************************************************************************/

/* (rvpriv.4.4.1 */
wire [8:0] vpn0 = mtval[20:12];
wire [8:0] vpn1 = mtval[29:21];
wire [8:0] vpn2 = mtval[38:30];

wire [`XLEN-1:0] ml1pteaddr = { {`XLEN-`PLEN{1'b0}}, satp_ppn, vpn2, 3'b0};
wire [`XLEN-1:0] ml2pteoff  = { {`XLEN-12{1'b0}},              vpn1, 3'b0};
wire [`XLEN-1:0] ml3pteoff  = { {`XLEN-12{1'b0}},              vpn0, 3'b0};

endmodule

`endif /* _IREG_FILE_SV */
