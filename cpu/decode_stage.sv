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

/* 9444 instruction decode stage. */
`ifndef _DECODE_STAGE_SV
`define _DECODE_STAGE_SV

`include "common.sv"
`include "opcodes.sv"
`include "ireg.sv"

module DecodeStage(
    input               clock,
    input               stall,
    input mode::mode_t  current_mode,

    /** CONNECTIONS TO FETCH STAGE. **/
    /* Set if the instruction address was misaligned and could not be fetched. */
    input                           instruction_addr_misaligned_in,
    /* Set if the instruction could not be fetched due to a cache miss. */
    input                           instruction_cache_miss_in,
    /* Set if the instruction is not yet ready. */
    input                           instruction_bubble,
    /* Virtual address of the instruction. */
    input  [`XLEN-1:1]              instruction_vaddr_in,
    /* The instruction. */
    input  [31:2]                   instruction_word,
    /* Must be asserted if the current instruction is not done yet. */
    output                          instruction_hold,

    /** CONNECTIONS TO EXECUTE STAGE. **/
    output reg                      instruction_addr_misaligned_out = 0,
    output reg                      instruction_illegal_out = 0,
    output reg                      e_call_out = 0,
    output reg                      e_break_out = 0,
    output reg                      s_ret_out = 0,
    output reg                      m_ret_out = 0,
    output reg [`XLEN-1:1]          instruction_vaddr_out = 0,
    output reg                      is_word_op_out = 0,
    output reg [2:0]                alu_mul_div_shifter_op_out = 0,
    output comparator::op_t         comparator_op_out = comparator::EQ,
    output execute::memory_access_t memory_access_out = execute::BYTE_READ,
    output execute::result_select_t result_select_out = execute::COMPARATOR,
    output reg                      save_result_to_gpr_out = 0,
    output reg [ 4:0]               rd_num_out = 0,
    output reg                      is_div_op_out = 0,
    output reg                      is_memory_op_out = 0,
    output reg                      is_lr_out = 0,
    output reg                      is_sc_out = 0,
    output reg                      is_signed_memory_op_out = 0,
    output reg                      is_cache_invalidate_out = 0,
    output reg                      is_tlb_invalidate_out = 0,
    output reg                      is_selective_invalidate_out = 0,
    output reg [`XLEN-1:0]          a_out = 0,
    output reg [`XLEN-1:0]          b_out = 0,
    output reg [`XLEN-1:0]          a_comp_out = 0,
    output reg [`XLEN-1:0]          b_comp_out = 0,
    output reg                      is_branch_out = 0,
    output reg                      wait_for_interrupt_out = 0,
    output reg                      suppress_interrupts = 0,
    /* Result of current operation.
     * This is useful for multi-cycle instructions.
     */
    input [`XLEN-1:0]               execute_result,
    /* If set, we have to output a NOP. */
    input                           redirect,

    /** CONNECTIONS TO REGISTER FILE. **/
    output [4:0]                    rs1,
    output [4:0]                    rs2,
    input  [`XLEN-1:0]              rs1_value,
    input  [`XLEN-1:0]              rs2_value,

    /** CONNECTIONS TO INTERNAL REGISTER FILE. **/
    output ireg_file::op_t          ireg_op_out,
    output ireg::ireg_t             ireg_sel_out,
    output reg [31:2]               instruction_word_out
);

/* (riscv.2.3) Decode fields of instruction types. */

/* Opcode, all types. */
wire [4:0] opcode = instruction_word[6:2];

/* Rd, RIUJ types. */
wire [4:0] rd = instruction_word[11:7];

/* Funct3, RISB types. */
wire [2:0] funct3 = instruction_word[14:12];

/* Rs1, RISB types. */
assign rs1 = instruction_word[19:15];

/* Rs2, RSB types. */
assign rs2 = instruction_word[24:20];

/* Funct7, R type. */
wire [6:0] funct7 = instruction_word[31:25];

/* I-immediate. */
wire [`XLEN-1:0] i_immediate = {
    {53{instruction_word[31]}},
    instruction_word[30:20]
};

/* S-immediate. */
wire [`XLEN-1:0] s_immediate = {
    {53{instruction_word[31]}},
    instruction_word[30:25],
    instruction_word[11: 7]
};

/* B-immediate. */
wire [`XLEN-1:0] b_immediate = {
    {52{instruction_word[31]}},
    instruction_word[7],
    instruction_word[30:25],
    instruction_word[11: 8],
    1'b0
};

/* U-immediate. */
wire [`XLEN-1:0] u_immediate = {
    {33{instruction_word[31]}},
    instruction_word[30:12],
    12'b0
};

/* J-immediate. */
wire [`XLEN-1:0] j_immediate = {
    {44{instruction_word[31]}},
    instruction_word[19:12],
    instruction_word[20],
    instruction_word[30:21],
    1'b0
};

/* STATE FOR MULTI-CYCLE INSTRUCTIONS. *********************************************************************************
 * Multi-cycle instructions can take exceptions at all steps.
 * Interrupts can only happen on the first step.
 * Use carefully and only when really needed.
 */

/* Set to 0 during decode to continue decoding the same instruction. */
reg instruction_done;
/* Counts how many steps we spent on this instruction. */
reg [2:0] instruction_step = 0;
/* Saved src registers.
 * The source registers are captured during the first cycle because they may be modified by the execute stage.
 * The first cycle must not use these.
 * These must be used during the later cycles.
 */
reg [`XLEN-1:0] srs1 = 0;
reg [`XLEN-1:0] srs2 = 0;

always @(posedge clock) begin
    if (!stall) begin
        if (instruction_done) begin
            instruction_step <= 0;
        end
        else begin
            instruction_step <= instruction_step + 1;
        end

        if (instruction_step == 0) begin
            srs1 <= rs1_value;
            srs2 <= rs2_value;
        end
    end
end

assign instruction_hold = !instruction_done;

/* DECODE NEXT INSTRUCTIONS. ******************************************************************************************/
reg                         instruction_illegal;
reg                         e_call;
reg                         e_break;
reg                         s_ret;
reg                         m_ret;
reg                         is_word_op;
reg [2:0]                   alu_mul_div_shifter_op;
comparator::op_t            comparator_op;
execute::memory_access_t    memory_access;
execute::result_select_t    result_select;
reg                         save_result_to_gpr;
reg [ 4:0]                  rd_num;
reg                         is_div_op;
reg                         is_memory_op;
reg                         is_lr;
reg                         is_sc;
reg                         is_signed_memory_op;
reg                         is_cache_invalidate;
reg                         is_tlb_invalidate;
reg                         is_selective_invalidate;
reg [`XLEN-1:0]             a;
reg [`XLEN-1:0]             b;
reg [`XLEN-1:0]             a_comp;
reg [`XLEN-1:0]             b_comp;
reg                         is_branch;
reg                         wait_for_interrupt;

ireg_file::op_t             ireg_op;
ireg::ireg_t                ireg_sel;

/* Macros for I-type ALU operations. */
`define ALU_IMMEDIATEW(_alu_op_)                                \
            begin                                               \
                alu_mul_div_shifter_op = alu::_alu_op_;         \
                result_select = execute::ALU;                   \
                save_result_to_gpr = 1;                         \
                b = i_immediate;                                \
                is_word_op = 1;                                 \
            end
`define ALU_IMMEDIATED(_alu_op_)                                \
            begin                                               \
                alu_mul_div_shifter_op = alu::_alu_op_;         \
                result_select = execute::ALU;                   \
                save_result_to_gpr = 1;                         \
                b = i_immediate;                                \
                is_word_op = 0;                                 \
            end


/* Macros for R-type ALU operations. */
`define ALU_REGISTERW(_alu_op_)                                 \
            begin                                               \
                alu_mul_div_shifter_op = alu::_alu_op_;         \
                result_select = execute::ALU;                   \
                save_result_to_gpr = 1;                         \
                is_word_op = 1;                                 \
            end
`define ALU_REGISTERD(_alu_op_)                                 \
            begin                                               \
                alu_mul_div_shifter_op = alu::_alu_op_;         \
                result_select = execute::ALU;                   \
                save_result_to_gpr = 1;                         \
                is_word_op = 0;                                 \
            end

/* Macro for I-type comparator operations. */
`define COMPARATOR_IMMEDIATE(_comparator_op_)                   \
            begin                                               \
                comparator_op = comparator::_comparator_op_;    \
                result_select = execute::COMPARATOR;            \
                save_result_to_gpr = 1;                         \
                b_comp = i_immediate;                           \
            end

/* Macro for R-type comparator operations. */
`define COMPARATOR_REGISTER(_comparator_op_)                    \
            begin                                               \
                comparator_op = comparator::_comparator_op_;    \
                result_select = execute::COMPARATOR;            \
                save_result_to_gpr = 1;                         \
            end

/* Macros for I-type shifter operations. */
`define SHIFTER_IMMEDIATEW(_shifter_op_)                                \
            begin                                                       \
                alu_mul_div_shifter_op = {1'bx, shifter::_shifter_op_}; \
                result_select = execute::SHIFTER;                       \
                save_result_to_gpr = 1;                                 \
                b = i_immediate;                                        \
                is_word_op = 1;                                         \
            end
`define SHIFTER_IMMEDIATED(_shifter_op_)                                \
            begin                                                       \
                alu_mul_div_shifter_op = {1'bx, shifter::_shifter_op_}; \
                result_select = execute::SHIFTER;                       \
                save_result_to_gpr = 1;                                 \
                b = i_immediate;                                        \
                is_word_op = 0;                                         \
            end

/* Macros for R-type shifter operations. */
`define SHIFTER_REGISTERW(_shifter_op_)                                 \
            begin                                                       \
                alu_mul_div_shifter_op = {1'bx, shifter::_shifter_op_}; \
                result_select = execute::SHIFTER;                       \
                save_result_to_gpr = 1;                                 \
                is_word_op = 1;                                         \
            end
`define SHIFTER_REGISTERD(_shifter_op_)                                 \
            begin                                                       \
                alu_mul_div_shifter_op = {1'bx, shifter::_shifter_op_}; \
                result_select = execute::SHIFTER;                       \
                save_result_to_gpr = 1;                                 \
                is_word_op = 0;                                         \
            end

/* Invert a condition, assuming the condition is not comparator::TRUE. */
function comparator::op_t invert_condition;
input comparator::op_t op;
case(op)
    comparator::EQ:  invert_condition = comparator::NE;
    comparator::NE:  invert_condition = comparator::EQ;
    comparator::LT:  invert_condition = comparator::GE;
    comparator::LTU: invert_condition = comparator::GEU;
    comparator::GE:  invert_condition = comparator::LT;
    comparator::GEU: invert_condition = comparator::LTU;
    default: invert_condition = comparator::X;
endcase
endfunction

/* Macro for conditional branches. */
`define CONDITIONAL_BRANCH(_comp_op_)                           \
        begin                                                   \
            a = {instruction_vaddr_in, 1'b0};                   \
            is_branch = 1;                                      \
            comparator_op = comparator::_comp_op_;              \
            b = b_immediate;                                    \
        end

/* Macro for load instructions. */
`define MEMORY_LOAD(_memory_access_, _signed_)                  \
            begin                                               \
                result_select = execute::MEMORY;                \
                memory_access = execute::_memory_access_;       \
                save_result_to_gpr = 1;                         \
                is_memory_op = 1;                               \
                is_signed_memory_op = _signed_;                 \
                b = i_immediate;                                \
                is_lr = 0;                                      \
                is_sc = 0;                                      \
            end

/* Macro for store instructions. */
`define MEMORY_STORE(_memory_access_)                           \
            begin                                               \
                memory_access = execute::_memory_access_;       \
                is_memory_op = 1;                               \
                b = s_immediate;                                \
                is_lr = 0;                                      \
                is_sc = 0;                                      \
            end

/* Macros for R-type multiplier operations. */
`define MUL_REGISTERW(_mul_op_)                                 \
            begin                                               \
                alu_mul_div_shifter_op = {1'bx, mul::_mul_op_}; \
                result_select = execute::MUL;                   \
                save_result_to_gpr = 1;                         \
                is_word_op = 1;                                 \
            end
`define MUL_REGISTERD(_mul_op_)                                 \
            begin                                               \
                alu_mul_div_shifter_op = {1'bx, mul::_mul_op_}; \
                result_select = execute::MUL;                   \
                save_result_to_gpr = 1;                         \
                is_word_op = 0;                                 \
            end

/* Macro for CSR instructions. */
`define CSR_INSTRUCTION(_csr_name_)                             \
    /* CSRRW */                                                 \
    {csr::_csr_name_, 5'b?????, 3'b001, 5'b?????, 7'b1110011},  \
    {csr::_csr_name_, 5'b?????, 3'b010, 5'b?????, 7'b1110011},  \
    {csr::_csr_name_, 5'b?????, 3'b011, 5'b?????, 7'b1110011},  \
    {csr::_csr_name_, 5'b?????, 3'b101, 5'b?????, 7'b1110011},  \
    {csr::_csr_name_, 5'b?????, 3'b110, 5'b?????, 7'b1110011},  \
    {csr::_csr_name_, 5'b?????, 3'b111, 5'b?????, 7'b1110011}:  \
        begin                                                   \
            automatic reg [11:0] this_csr = csr::_csr_name_;    \
            ireg_sel = ireg::_csr_name_;                        \
            result_select = execute::INTERNAL_REGISTER;         \
            save_result_to_gpr = 1;                             \
            case (instruction_word[13:12])                      \
                2'b00: ireg_op = ireg_file::NOP;                \
                2'b01: ireg_op = ireg_file::RW;                 \
                2'b10: ireg_op = ireg_file::RS;                 \
                2'b11: ireg_op = ireg_file::RC;                 \
            endcase                                             \
            /* Detect writes to a read-only register. */        \
            if ((rs1 != 0) && (this_csr[11:10] == 2'b11)) begin \
                instruction_illegal = 1;                        \
            end                                                 \
            /* Detect access in the wrong mode. */              \
            if (current_mode < this_csr[9:8]) begin             \
                instruction_illegal = 1;                        \
            end                                                 \
            /* Handle immediate instructions. */                \
            if (instruction_word[14]) begin                     \
                a = {59'b0, rs1};                               \
            end                                                 \
        end

wire [31:0] reconstructed_instruction_word = {instruction_word, 2'b11};
always_comb begin
    /* Set some reasonable defaults. */
    instruction_done = 1;
    instruction_illegal = 0;
    e_call = 0;
    e_break = 0;
    s_ret = 0;
    m_ret = 0;
    is_word_op = 0;
    alu_mul_div_shifter_op = 3'bxxx;
    comparator_op = comparator::X;
    memory_access = execute::MEM_X;
    result_select = execute::RESULT_X;
    save_result_to_gpr = 0;
    rd_num = rd;
    is_div_op = 0;
    is_memory_op = 0;
    /* These must be set if is_memory_op. */
    is_lr = 1'bx;
    is_sc = 1'bx;
    is_signed_memory_op = 1'bx;
    is_cache_invalidate = 0;
    is_tlb_invalidate = 0;
    is_selective_invalidate = 1'bx;
    a = rs1_value;
    b = rs2_value;
    a_comp = rs1_value;
    b_comp = rs2_value;
    is_branch = 0;
    wait_for_interrupt = 0;

    ireg_op  = ireg_file::NOP;
    ireg_sel = ireg::x;

    if (redirect || instruction_bubble)  begin
        /* Nothing to do. The defaults are already a NOP. */
    end
    else if (instruction_cache_miss_in) begin
        /* We have to fetch the instruction cache line. */
        comparator_op = comparator::TRUE;
        memory_access = execute::LINE_READ;
        is_memory_op = 1;
        a = {instruction_vaddr_in[`XLEN-1:5], 5'b0};
        b = 0;
        is_lr = 0;
        is_sc = 0;
        is_branch = 1;
    end
    else casez(reconstructed_instruction_word)
        /* -- ALU instructions. */
        opcodes::ADDI:  `ALU_IMMEDIATED(ADD)
        opcodes::ADDIW: `ALU_IMMEDIATEW(ADD)
        opcodes::ANDI:  `ALU_IMMEDIATED(AND)
        opcodes::ORI:   `ALU_IMMEDIATED(OR)
        opcodes::XORI:  `ALU_IMMEDIATED(XOR)
        opcodes::ADD:   `ALU_REGISTERD(ADD)
        opcodes::ADDW:  `ALU_REGISTERW(ADD)
        opcodes::AND:   `ALU_REGISTERD(AND)
        opcodes::OR:    `ALU_REGISTERD(OR)
        opcodes::XOR:   `ALU_REGISTERD(XOR)
        opcodes::SUB:   `ALU_REGISTERD(SUB)
        opcodes::SUBW:  `ALU_REGISTERW(SUB)
        /* -- comparator instructions. */
        opcodes::SLTI:  `COMPARATOR_IMMEDIATE(LT)
        opcodes::SLTIU: `COMPARATOR_IMMEDIATE(LTU)
        opcodes::SLT:   `COMPARATOR_REGISTER(LT)
        opcodes::SLTU:  `COMPARATOR_REGISTER(LTU)
        /* -- bitshift instructions. */
        opcodes::SLLI:  `SHIFTER_IMMEDIATED(SLL)
        opcodes::SLLIW: `SHIFTER_IMMEDIATEW(SLL)
        opcodes::SRLI:  `SHIFTER_IMMEDIATED(SRL)
        opcodes::SRLIW: `SHIFTER_IMMEDIATEW(SRL)
        opcodes::SRAI:  `SHIFTER_IMMEDIATED(SRA)
        opcodes::SRAIW: `SHIFTER_IMMEDIATEW(SRA)
        opcodes::SLL:   `SHIFTER_REGISTERD(SLL)
        opcodes::SLLW:  `SHIFTER_REGISTERW(SLL)
        opcodes::SRL:   `SHIFTER_REGISTERD(SRL)
        opcodes::SRLW:  `SHIFTER_REGISTERW(SRL)
        opcodes::SRA:   `SHIFTER_REGISTERD(SRA)
        opcodes::SRAW:  `SHIFTER_REGISTERW(SRA)
        /* -- immediate loading instructions. */
        opcodes::LUI:
            begin
                alu_mul_div_shifter_op = alu::ADD;
                result_select = execute::ALU;
                save_result_to_gpr = 1;
                a = 0;
                b = u_immediate;
            end
        opcodes::AUIPC:
            begin
                alu_mul_div_shifter_op = alu::ADD;
                result_select = execute::ALU;
                save_result_to_gpr = 1;
                a = {instruction_vaddr_in, 1'b0};
                b = u_immediate;
            end
        /* -- unconditional jumps. */
        opcodes::JAL:
            begin
                comparator_op = comparator::TRUE;
                result_select = execute::RETURN_ADDRESS;
                save_result_to_gpr = 1;
                a = {instruction_vaddr_in, 1'b0};
                b = j_immediate;
                is_branch = 1;
            end
        opcodes::JALR:
            begin
                comparator_op = comparator::TRUE;
                result_select = execute::RETURN_ADDRESS;
                save_result_to_gpr = 1;
                b = i_immediate;
                is_branch = 1;
            end
        /* -- conditional branches. */
        opcodes::BEQ:   `CONDITIONAL_BRANCH(EQ)
        opcodes::BNE:   `CONDITIONAL_BRANCH(NE)
        opcodes::BLT:   `CONDITIONAL_BRANCH(LT)
        opcodes::BLTU:  `CONDITIONAL_BRANCH(LTU)
        opcodes::BGE:   `CONDITIONAL_BRANCH(GE)
        opcodes::BGEU:  `CONDITIONAL_BRANCH(GEU)
        /* -- load instructions. */
        opcodes::LB:    `MEMORY_LOAD( BYTE_READ,    1)
        opcodes::LBU:   `MEMORY_LOAD( BYTE_READ,    0)
        opcodes::LH:    `MEMORY_LOAD(HWORD_READ,    1)
        opcodes::LHU:   `MEMORY_LOAD(HWORD_READ,    0)
        opcodes::LW:    `MEMORY_LOAD( WORD_READ,    1)
        opcodes::LWU:   `MEMORY_LOAD( WORD_READ,    0)
        opcodes::LD:    `MEMORY_LOAD(DWORD_READ, 1'bx)
        /* -- store instructions. */
        opcodes::SB:    `MEMORY_STORE( BYTE_WRITE)
        opcodes::SH:    `MEMORY_STORE(HWORD_WRITE)
        opcodes::SW:    `MEMORY_STORE( WORD_WRITE)
        opcodes::SD:    `MEMORY_STORE(DWORD_WRITE)
        /* -- memory fence instructions. */
        opcodes::FENCE:
            begin
                /* In-order core. Nothing to do. */
            end
        opcodes::FENCE_I:
            begin
                /* Invalidate instruction cache. */
                is_cache_invalidate = 1;
                is_selective_invalidate = 0;
            end
        /* -- multiplication instructions. */
        opcodes::MUL:   `MUL_REGISTERD(MUL)
        opcodes::MULW:  `MUL_REGISTERW(MUL)
        opcodes::MULH:  `MUL_REGISTERD(MULH)
        opcodes::MULHSU:`MUL_REGISTERD(MULHSU)
        opcodes::MULHU: `MUL_REGISTERD(MULHU)

        /* -- division instructions. */
        opcodes::DIVW,
        opcodes::DIV:
            begin
                alu_mul_div_shifter_op = {2'b0, div::DIV};
                result_select = execute::DIV_Q;
                save_result_to_gpr = 1;
                is_div_op = 1;
                is_word_op = reconstructed_instruction_word[3];
            end
         opcodes::DIVUW,
         opcodes::DIVU:
            begin
                alu_mul_div_shifter_op = {2'b0, div::DIVU};
                result_select = execute::DIV_Q;
                save_result_to_gpr = 1;
                is_div_op = 1;
                is_word_op = reconstructed_instruction_word[3];
            end
        opcodes::REMW,
        opcodes::REM:
            begin
                alu_mul_div_shifter_op = {2'b0, div::DIV};
                result_select = execute::DIV_R;
                save_result_to_gpr = 1;
                is_div_op = 1;
                is_word_op = reconstructed_instruction_word[3];
            end
         opcodes::REMUW,
         opcodes::REMU:
            begin
                alu_mul_div_shifter_op = {2'b0, div::DIVU};
                result_select = execute::DIV_R;
                save_result_to_gpr = 1;
                is_div_op = 1;
                is_word_op = reconstructed_instruction_word[3];
            end

        /* -- atomic instructions. */
        opcodes::LRW:
            begin
                alu_mul_div_shifter_op = alu::ADD;
                result_select = execute::MEMORY;
                memory_access = execute::WORD_READ;
                save_result_to_gpr = 1;
                is_memory_op = 1;
                is_signed_memory_op = 1;
                is_lr = 1;
                is_sc = 0;
            end
        opcodes::SCW:
            begin
                alu_mul_div_shifter_op = alu::ADD;
                result_select = execute::SC_RESULT;
                memory_access = execute::WORD_WRITE;
                b = 0;
                save_result_to_gpr = 1;
                is_memory_op = 1;
                is_lr = 0;
                is_sc = 1;
            end
        opcodes::LRD:
            begin
                alu_mul_div_shifter_op = alu::ADD;
                result_select = execute::MEMORY;
                memory_access = execute::DWORD_READ;
                save_result_to_gpr = 1;
                is_memory_op = 1;
                is_lr = 1;
                is_sc = 0;
            end
        opcodes::SCD:
            begin
                alu_mul_div_shifter_op = alu::ADD;
                result_select = execute::SC_RESULT;
                memory_access = execute::DWORD_WRITE;
                b = 0;
                save_result_to_gpr = 1;
                is_memory_op = 1;
                is_lr = 0;
                is_sc = 1;
            end
        opcodes::AMOMIND,
        opcodes::AMOMINW,
        opcodes::AMOMAXD,
        opcodes::AMOMAXW,
        opcodes::AMOMINUD,
        opcodes::AMOMINUW,
        opcodes::AMOMAXUD,
        opcodes::AMOMAXUW:
            begin
                /* These operations take 3 cycles:
                 *  - load          rd  <= mem[rs1]
                 *  - compare
                 *  - store if matched
                 */
                case (instruction_step)
                    0:
                        begin
                            instruction_done = 0;
                            casez(reconstructed_instruction_word)
                                opcodes::AMOMIND,
                                opcodes::AMOMAXD,
                                opcodes::AMOMINUD,
                                opcodes::AMOMAXUD:
                                    `MEMORY_LOAD(DWORD_READ, 1'bx)
                                opcodes::AMOMINW,
                                opcodes::AMOMAXW:
                                    `MEMORY_LOAD( WORD_READ, 1)
                                opcodes::AMOMINUW,
                                opcodes::AMOMAXUW:
                                    `MEMORY_LOAD( WORD_READ, 0)
                            endcase
                            b = 0;
                        end

                    1:
                        begin
                            instruction_done = 0;
                            result_select = execute::COMPARATOR;
                            casez(reconstructed_instruction_word)
                                opcodes::AMOMIND, opcodes::AMOMINW:     comparator_op = comparator::LT;
                                opcodes::AMOMAXD, opcodes::AMOMAXW:     comparator_op = comparator::GE;
                                opcodes::AMOMINUD, opcodes::AMOMINUW:   comparator_op = comparator::LTU;
                                opcodes::AMOMAXUD, opcodes::AMOMAXUW:   comparator_op = comparator::GEU;
                            endcase
                            a_comp = srs2;
                            b_comp = execute_result;
                        end

                    2:
                        if (execute_result[0]) begin
                            casez(reconstructed_instruction_word)
                                opcodes::AMOMIND,
                                opcodes::AMOMAXD,
                                opcodes::AMOMINUD,
                                opcodes::AMOMAXUD:
                                    `MEMORY_STORE(DWORD_WRITE)
                                opcodes::AMOMINW,
                                opcodes::AMOMAXW,
                                opcodes::AMOMINUW,
                                opcodes::AMOMAXUW:
                                    `MEMORY_STORE( WORD_WRITE)
                            endcase
                            a = srs1;
                            b = 0;
                            b_comp = srs2;
                        end
                endcase
                
            end

        opcodes::AMOADDD,
        opcodes::AMOADDW,
        opcodes::AMOXORD,
        opcodes::AMOXORW,
        opcodes::AMOORD,
        opcodes::AMOORW,
        opcodes::AMOANDD,
        opcodes::AMOANDW,
        opcodes::AMOSWAPD,
        opcodes::AMOSWAPW:
            begin
                /* These operations take 3 cycles:
                 *  - load          rd  <= mem[rs1]
                 *  - ALU operation (not saved) <= srs2 (ALU op) rd
                 *  - store         mem[srs1] <= (not saved)
                 */
                case (instruction_step)
                    0:
                        begin
                            instruction_done = 0;
                            casez(reconstructed_instruction_word)
                                opcodes::AMOADDD,
                                opcodes::AMOXORD,
                                opcodes::AMOORD,
                                opcodes::AMOANDD,
                                opcodes::AMOSWAPD:
                                    `MEMORY_LOAD(DWORD_READ, 1'bx)
                                opcodes::AMOADDW,
                                opcodes::AMOXORW,
                                opcodes::AMOORW,
                                opcodes::AMOANDW,
                                opcodes::AMOSWAPW:
                                    `MEMORY_LOAD( WORD_READ, 1)
                            endcase
                            b = 0;
                        end
                    
                    1:
                        begin
                            instruction_done = 0;
                            a = srs2;
                            b = execute_result;
                            casez(reconstructed_instruction_word)
                                opcodes::AMOADDD:   `ALU_REGISTERD(ADD)
                                opcodes::AMOADDW:   `ALU_REGISTERW(ADD)
                                opcodes::AMOXORD:   `ALU_REGISTERD(XOR)
                                opcodes::AMOXORW:   `ALU_REGISTERW(XOR)
                                opcodes::AMOORD:    `ALU_REGISTERD(OR)
                                opcodes::AMOORW:    `ALU_REGISTERW(OR)
                                opcodes::AMOANDD:   `ALU_REGISTERD(AND)
                                opcodes::AMOANDW:   `ALU_REGISTERW(AND)
                                default:
                                    begin
                                        /* NOP */
                                        `ALU_REGISTERD(ADD)
                                        b = 0;
                                    end
                            endcase
                            save_result_to_gpr = 0;
                        end

                    2:
                        begin
                            casez(reconstructed_instruction_word)
                                opcodes::AMOADDD,
                                opcodes::AMOXORD,
                                opcodes::AMOORD,
                                opcodes::AMOANDD,
                                opcodes::AMOSWAPD:
                                    `MEMORY_STORE(DWORD_WRITE)
                                opcodes::AMOADDW,
                                opcodes::AMOXORW,
                                opcodes::AMOORW,
                                opcodes::AMOANDW,
                                opcodes::AMOSWAPW:
                                    `MEMORY_STORE( WORD_WRITE)
                            endcase
                            a = srs1;
                            b = 0;
                            b_comp = execute_result;
                        end

                endcase
            end

        /* -- CSR instructions. */
        `CSR_INSTRUCTION(misa)
        `CSR_INSTRUCTION(mvendorid)
        `CSR_INSTRUCTION(marchid)
        `CSR_INSTRUCTION(mimpid)
        `CSR_INSTRUCTION(mhartid)
        `CSR_INSTRUCTION(mstatus)
        `CSR_INSTRUCTION(mtvec)
        `CSR_INSTRUCTION(medeleg)
        `CSR_INSTRUCTION(mideleg)
        `CSR_INSTRUCTION(mip)
        `CSR_INSTRUCTION(mie)
        `CSR_INSTRUCTION(mscratch)
        `CSR_INSTRUCTION(mepc)
        `CSR_INSTRUCTION(mcause)
        `CSR_INSTRUCTION(mtval)
        `CSR_INSTRUCTION(sstatus)
        `CSR_INSTRUCTION(stvec)
        `CSR_INSTRUCTION(sip)
        `CSR_INSTRUCTION(sie)
        `CSR_INSTRUCTION(sscratch)
        `CSR_INSTRUCTION(sepc)
        `CSR_INSTRUCTION(scause)
        `CSR_INSTRUCTION(stval)
        `CSR_INSTRUCTION(satp)
        `CSR_INSTRUCTION(snecycle)
        `CSR_INSTRUCTION(cycle)
        `CSR_INSTRUCTION(time_)
        `CSR_INSTRUCTION(mtlbw4k)
        `CSR_INSTRUCTION(mtlbw2m)
        `CSR_INSTRUCTION(mtlbw1g)
        `CSR_INSTRUCTION(mtlblvec)
        `CSR_INSTRUCTION(mtlbsvec)
        `CSR_INSTRUCTION(ml1pteaddr)
        `CSR_INSTRUCTION(ml2pteoff)
        `CSR_INSTRUCTION(ml3pteoff)

        /* -- other system instructions. */
        opcodes::ECALL: e_call = 1;
        opcodes::EBREAK: e_break = 1;
        opcodes::SRET:
            begin
                if (current_mode >= mode::S) begin
                    s_ret = 1;
                end
                else begin
                    instruction_illegal = 1;
                end
            end
        opcodes::MRET:
            begin
                if (current_mode == mode::M) begin
                    m_ret = 1;
                end
                else begin
                    instruction_illegal = 1;
                end
            end
        opcodes::WFI:
            begin
                wait_for_interrupt = 1;
            end
        opcodes::SFENCE_VMA:
            begin
                b = 0;

                /* User mode cannot use this instruction. */
                if (current_mode == mode::U) begin
                    instruction_illegal = 1;
                end
                /* If no vaddr filter is provided, invalidate everything.
                 * The ASID is ignored because ASIDs are not implemented.
                 */
                else if (rs1 == 0) begin
                    is_cache_invalidate = 1;
                    is_tlb_invalidate = 1;
                    is_selective_invalidate = 0;
                end
                /* If the vaddr is provided, we can be selective. */
                else begin
                    is_cache_invalidate = 1;
                    is_tlb_invalidate = 1;
                    is_selective_invalidate = 1;
                end
            end

        /* Nothing matched. */
        default: instruction_illegal = 1;
    endcase
end

/* OUTPUT NEXT INSTRUCTION. *******************************************************************************************/

always @(posedge clock) begin
    if (!stall) begin
        instruction_addr_misaligned_out <= instruction_addr_misaligned_in;
        instruction_illegal_out         <= instruction_illegal;
        e_call_out                      <= e_call;
        e_break_out                     <= e_break;
        s_ret_out                       <= s_ret;
        m_ret_out                       <= m_ret;
        instruction_vaddr_out           <= instruction_vaddr_in;
        is_word_op_out                  <= is_word_op;
        alu_mul_div_shifter_op_out      <= alu_mul_div_shifter_op;
        comparator_op_out               <= comparator_op;
        memory_access_out               <= memory_access;
        result_select_out               <= result_select;
        save_result_to_gpr_out          <= save_result_to_gpr;
        rd_num_out                      <= rd_num;
        is_div_op_out                   <= is_div_op;
        is_memory_op_out                <= is_memory_op;
        is_lr_out                       <= is_lr;
        is_sc_out                       <= is_sc;
        is_signed_memory_op_out         <= is_signed_memory_op;
        is_cache_invalidate_out         <= is_cache_invalidate;
        is_tlb_invalidate_out           <= is_tlb_invalidate;
        is_selective_invalidate_out     <= is_selective_invalidate;
        a_out                           <= a;
        b_out                           <= b;
        a_comp_out                      <= a_comp;
        b_comp_out                      <= b_comp;
        is_branch_out                   <= is_branch;
        wait_for_interrupt_out          <= wait_for_interrupt;
        suppress_interrupts             <= (instruction_step != 0) || redirect || instruction_bubble;

        ireg_op_out                     <= ireg_op;
        ireg_sel_out                    <= ireg_sel;
        instruction_word_out            <= instruction_word[31:2];
    end
end

/* Useful for looking at traces. */
wire [`XLEN-1:0] debug_decode_pc = {instruction_vaddr_in, 1'b0};

endmodule

`endif /* _DECODE_STAGE_SV */
