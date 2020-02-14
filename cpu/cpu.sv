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

/* 9444 CPU top level module. */
`include "common.sv"
`include "decode_stage.sv"
`include "execute_stage.sv"
`include "fetch_stage.sv"
`include "mmu.sv"
`include "register_file.sv"
`include "ireg_file.sv"

module CPU(
    input clock,

    /* Useful for debugging. */
    output mode::mode_t             current_mode,
    output                          mode_m,
    output                          mode_s,
    output                          mode_u,
    output                          waiting_for_interrupt,


    /* Memory interface. */
    output                          mem_cycle,
    output      [`PLEN-1:0]         mem_paddr,
    output execute::memory_access_t mem_access,
    output      [`XLEN-1:0]         mem_data_out,
    input  [3:0][`XLEN-1:0]         mem_data_in,
    input                           mem_ack,

    /* Interrupt interface. */
    input                   m_interrupt,
    input                   s_interrupt
);

assign mode_m = (current_mode == mode::M);
assign mode_s = (current_mode == mode::S);
assign mode_u = (current_mode == mode::U);

assign waiting_for_interrupt = wait_for_interrupt && stall;

/* REGISTER EXTERNAL INTERRUPTS. **************************************************************************************/
/* m_interrupt is not implemented. */

reg s_interrupt_reg;

always @(posedge clock) begin
    s_interrupt_reg <= s_interrupt;
end

/* MMU INSTANCE. ******************************************************************************************************/

wire                    mmu_sum;
wire                    mmu_u_mode;
wire                    mmu_enabled;
wire                    mmu_invalidate;
/* TLB access port. */
wire [ 4:0]             mmu_entry;
wire [`VLEN-1:12]       mmu_vpn_in;
mmu::psize_t            mmu_psize_in;
wire [31:0]             mmu_pte_in;
wire                    mmu_pte_write;
wire [`VLEN-1:12]       mmu_vpn_out;
mmu::psize_t            mmu_psize_out;
wire [31:0]             mmu_pte_out;
/* Access port. */
wire [`VLEN-1:0]        mmu_vaddr;
wire                    mmu_access_r;
wire                    mmu_access_w;
wire                    mmu_access_x;
wire                    mmu_match;
wire [ 4:0]             mmu_match_entry;
wire                    mmu_fault;
wire                    mmu_fault_dns;
wire [`PLEN-1:0]        mmu_paddr;

MMU MMU(
    .clock(clock),

    .sum(mmu_sum),
    .u_mode(mmu_u_mode),
    .enabled(mmu_enabled),
    
    .invalidate(mmu_invalidate),
    .selective_invalidate(decode_is_selective_invalidate),
    .selective_invalidate_vaddr(add_result[`VLEN-1:0]),

    .entry(mmu_entry),
    
    .vpn_in(mmu_vpn_in),
    .psize_in(mmu_psize_in),
    .pte_in(mmu_pte_in),
    .pte_write(mmu_pte_write),

    .vaddr(mmu_vaddr),
    .access_r(mmu_access_r),
    .access_w(mmu_access_w),
    .access_x(mmu_access_x),
    .match(mmu_match),
    .match_entry(mmu_match_entry),
    .fault(mmu_fault),
    .fault_dns(mmu_fault_dns),
    .paddr(mmu_paddr)
);

/* FETCH STAGE INSTANCE. **********************************************************************************************/

/* Connections to decode stage. */
wire                fetch_instruction_addr_misaligned;
wire                fetch_instruction_cache_miss;
wire                fetch_instruction_bubble;
wire [`XLEN-1:1]    fetch_instruction_vaddr;
wire [31:2]         fetch_instruction_word;
wire                fetch_instruction_hold;
/* Cache port. */
wire [`VLEN-1:5]    fetch_cache_port_addr;
wire                fetch_cache_port_set;
wire                fetch_cache_invalidate;

FetchStage FetchStage(
    .clock(clock),
    .mmu_enabled(mmu_enabled),

    .stall(stall),

    .current_mode(current_mode),
    .current_sum(mmu_sum),
    
    .instruction_addr_misaligned(fetch_instruction_addr_misaligned),
    .instruction_cache_miss(fetch_instruction_cache_miss),
    .instruction_bubble(fetch_instruction_bubble),
    .instruction_vaddr(fetch_instruction_vaddr),
    .instruction_word(fetch_instruction_word),
    .instruction_hold(fetch_instruction_hold),

    .redirect(redirect),
    .redirect_vaddr(redirect_vaddr),

    .cache_port_addr(fetch_cache_port_addr),
    .cache_port_data(mem_data_in),
    .cache_port_set(fetch_cache_port_set),
    .cache_invalidate(fetch_cache_invalidate),
    .selective_invalidate(decode_is_selective_invalidate),
    .selective_invalidate_vaddr(add_result[`VLEN-1:`VLEN-1])
);

/* DECODE STAGE INSTANCE. *********************************************************************************************/

wire                            decode_instruction_addr_misaligned;
wire                            decode_instruction_illegal;
wire                            decode_e_call;
wire                            decode_e_break;
wire                            decode_s_ret;
wire                            decode_m_ret;
wire [`XLEN-1:1]                decode_instruction_vaddr;
wire                            decode_is_word_op;
wire [2:0]                      decode_alu_mul_div_shifter_op;
wire comparator::op_t           decode_comparator_op;
wire execute::memory_access_t   decode_memory_access;
wire execute::result_select_t   decode_result_select;
wire                            decode_save_result_to_gpr;
wire [ 4:0]                     decode_rd_num;
wire                            decode_is_div_op;
wire                            decode_is_memory_op;
wire                            decode_is_lr;
wire                            decode_is_sc;
wire                            decode_is_signed_memory_op;
wire                            decode_is_cache_invalidate;
wire                            decode_is_tlb_invalidate;
wire                            decode_is_selective_invalidate;
wire [`XLEN-1:0]                decode_a;
wire [`XLEN-1:0]                decode_b;
wire [`XLEN-1:0]                decode_a_comp;
wire [`XLEN-1:0]                decode_b_comp;
wire                            decode_is_branch;
wire                            wait_for_interrupt;
wire                            suppress_interrupts;

wire [31:2]                     decode_instruction_word;

DecodeStage DecodeStage(
    .clock(clock),
    .stall(stall),
    .current_mode(current_mode),

    .instruction_addr_misaligned_in(fetch_instruction_addr_misaligned),
    .instruction_cache_miss_in(fetch_instruction_cache_miss),
    .instruction_bubble(fetch_instruction_bubble),
    .instruction_vaddr_in(fetch_instruction_vaddr),
    .instruction_word(fetch_instruction_word),
    .instruction_hold(fetch_instruction_hold),

    .instruction_addr_misaligned_out(decode_instruction_addr_misaligned),
    .instruction_illegal_out(decode_instruction_illegal),
    .e_call_out(decode_e_call),
    .e_break_out(decode_e_break),
    .s_ret_out(decode_s_ret),
    .m_ret_out(decode_m_ret),
    .instruction_vaddr_out(decode_instruction_vaddr),
    .is_word_op_out(decode_is_word_op),
    .alu_mul_div_shifter_op_out(decode_alu_mul_div_shifter_op),
    .comparator_op_out(decode_comparator_op),
    .memory_access_out(decode_memory_access),
    .result_select_out(decode_result_select),
    .save_result_to_gpr_out(decode_save_result_to_gpr),
    .rd_num_out(decode_rd_num),
    .is_div_op_out(decode_is_div_op),
    .is_memory_op_out(decode_is_memory_op),
    .is_lr_out(decode_is_lr),
    .is_sc_out(decode_is_sc),
    .is_signed_memory_op_out(decode_is_signed_memory_op),
    .is_cache_invalidate_out(decode_is_cache_invalidate),
    .is_tlb_invalidate_out(decode_is_tlb_invalidate),
    .is_selective_invalidate_out(decode_is_selective_invalidate),
    .a_out(decode_a),
    .b_out(decode_b),
    .a_comp_out(decode_a_comp),
    .b_comp_out(decode_b_comp),
    .is_branch_out(decode_is_branch),
    .wait_for_interrupt_out(wait_for_interrupt),
    .suppress_interrupts(suppress_interrupts),
    .execute_result(execute_result),

    .redirect(redirect),

    .rs1(rs1),
    .rs2(rs2),
    .rs1_value(rs1_value),
    .rs2_value(rs2_value),

    .ireg_op_out(ireg_op),
    .ireg_sel_out(ireg_sel),
    .instruction_word_out(decode_instruction_word)
);

/* EXECUTE STAGE INSTANCE. ********************************************************************************************/

wire                stall;
wire                redirect;
wire [`XLEN-1:1]    redirect_vaddr;

wire                exception_pending;
wire                interrupt_exception;
wire [5:0]          exception_code;
wire [`XLEN-1:2]    exception_vaddr;
wire [`XLEN-1:0]    add_result;
wire [`XLEN-1:0]    execute_result;

ExecuteStage ExecuteStage(
    .clock(clock),
    .stall(stall),
    .current_mode(current_mode),

    .instruction_addr_misaligned(decode_instruction_addr_misaligned),
    .instruction_illegal(decode_instruction_illegal),
    .e_call(decode_e_call),
    .e_break(decode_e_break),
    .s_ret(decode_s_ret),
    .m_ret(decode_m_ret),
    .instruction_vaddr(decode_instruction_vaddr),
    .is_word_op(decode_is_word_op),
    .alu_mul_div_shifter_op(decode_alu_mul_div_shifter_op),
    .comparator_op(decode_comparator_op),
    .memory_access(decode_memory_access),
    .result_select(decode_result_select),
    .save_result_to_gpr(decode_save_result_to_gpr),
    .rd_num(decode_rd_num),
    .is_div_op(decode_is_div_op),
    .is_memory_op(decode_is_memory_op),
    .is_lr(decode_is_lr),
    .is_sc(decode_is_sc),
    .is_signed_memory_op(decode_is_signed_memory_op),
    .is_cache_invalidate(decode_is_cache_invalidate),
    .is_tlb_invalidate(decode_is_tlb_invalidate),
    .a(decode_a),
    .b(decode_b),
    .a_comp(decode_a_comp),
    .b_comp(decode_b_comp),
    .is_branch(decode_is_branch),
    .wait_for_interrupt(wait_for_interrupt),
    .suppress_interrupts(suppress_interrupts),
    .execute_result(execute_result),


    .decode_instruction_vaddr(fetch_instruction_vaddr),
    .redirect(redirect),
    .redirect_vaddr(redirect_vaddr),

    .cache_port_addr(fetch_cache_port_addr),
    .cache_port_set(fetch_cache_port_set),
    .cache_invalidate(fetch_cache_invalidate),

    .rd(rd),
    .rd_value(rd_value),
    .rd_write(rd_write),

    .mmu_vaddr(mmu_vaddr),
    .mmu_access_r(mmu_access_r),
    .mmu_access_w(mmu_access_w),
    .mmu_access_x(mmu_access_x),
    .mmu_invalidate(mmu_invalidate),
    .mmu_match(mmu_match),
    .mmu_fault(mmu_fault),
    .mmu_fault_dns(mmu_fault_dns),
    .mmu_paddr(mmu_paddr),

    .mem_cycle(mem_cycle),
    .mem_paddr(mem_paddr),
    .mem_access(mem_access),
    .mem_data_out(mem_data_out),
    .mem_data_in(mem_data_in[0]),
    .mem_ack(mem_ack),

    .m_eie(m_eie),
    .m_tie(m_tie),
    .s_eie(s_eie),
    .s_tie(s_tie),
    .m_timer(m_timer),
    .s_timer(s_timer),
    .m_interrupt(m_interrupt),
    .s_interrupt(s_interrupt_reg),

    .ireg_out(ireg_out),

    .exception_pending(exception_pending),
    .interrupt_exception(interrupt_exception),
    .exception_code(exception_code),
    .exception_vaddr(exception_vaddr),
    .add_result(add_result)
);

/* REGISTER FILE INSTANCE. ********************************************************************************************/

wire       [4:0]    rs1;
wire       [4:0]    rs2;

wire [`XLEN-1:0]    rs1_value;
wire [`XLEN-1:0]    rs2_value;

wire       [4:0]    rd;
wire                rd_write;

wire [`XLEN-1:0]    rd_value;

RegisterFile RegisterFile(
    .clock(clock),
    .stall(stall),

    .rs1(rs1),
    .rs2(rs2),

    .rs1_value(rs1_value),
    .rs2_value(rs2_value),

    .rd(rd),
    .rd_write(rd_write),

    .rd_value(rd_value)
);

/* INTERNAL REGISTER FILE INSTANCE. ***********************************************************************************/

wire ireg_file::op_t    ireg_op;
wire ireg::ireg_t       ireg_sel;
wire [`XLEN-1:0]        ireg_out;
wire                    m_eie;
wire                    m_tie;
wire                    s_eie;
wire                    s_tie;
wire                    m_timer;
wire                    s_timer;

IRegFile IRegFile(
    .clock(clock),
    .stall(stall),
    .current_mode(current_mode),

    .op(ireg_op),
    .sel(ireg_sel),
    .op_val(decode_a),
    .out(ireg_out),

    .m_eie(m_eie),
    .m_tie(m_tie),
    .s_eie(s_eie),
    .s_tie(s_tie),
    .m_timer(m_timer),
    .s_timer(s_timer),
    .m_interrupt(m_interrupt),
    .s_interrupt(s_interrupt_reg),

    .s_ret(decode_s_ret),
    .m_ret(decode_m_ret),
    .instruction_vaddr(decode_instruction_vaddr),
    .instruction_word(decode_instruction_word),
    .add_result(add_result),
    .exception_pending(exception_pending),
    .interrupt_exception(interrupt_exception),
    .exception_code(exception_code),
    .exception_vaddr(exception_vaddr),

    .mmu_sum(mmu_sum),
    .mmu_u_mode(mmu_u_mode),
    .mmu_enabled(mmu_enabled),
    .mmu_entry(mmu_entry),
    .mmu_vpn_in(mmu_vpn_in),
    .mmu_psize_in(mmu_psize_in),
    .mmu_pte_in(mmu_pte_in),
    .mmu_pte_write(mmu_pte_write),
    .mmu_match_entry(mmu_match_entry)
);

endmodule