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

/* 9444 execute stage. */
`ifndef _EXECUTE_STAGE_SV
`define _EXECUTE_STAGE_SV

`include "common.sv"
`include "alu.sv"
`include "comparator.sv"
`include "div.sv"
`include "mul.sv"
`include "shifter.sv"

module ExecuteStage(
    input               clock,
    output              stall,
    input mode::mode_t  current_mode,

    /** CONNECTIONS TO DECODE STAGE. **/
    /* Incoming exception flags. */
    input                           instruction_addr_misaligned,
    input                           instruction_illegal,
    /* ECALL instruction, treated as exception. */
    input                           e_call,
    /* EBREAK instruction, treated as exception. */
    input                           e_break,
    /* Supervisor/machine return, treated as exception.
     * While these are not actually exceptions according to the architecture, they are easier to incorporate in the
     * pipeline as exceptions because they cause a mode switch, control transfer, and CSR changes.
     */
    input                           s_ret,
    input                           m_ret,
    /* Virtual address of the instruction. */
    input [`XLEN-1:1]               instruction_vaddr,
    /* 32b operation. */
    input                           is_word_op,
    /* Selected operations on different modules. */
    input [2:0]                     alu_mul_div_shifter_op,
    input comparator::op_t          comparator_op,
    /* Memory access size. */
    input execute::memory_access_t  memory_access,
    /* Select which value is the "result". */
    input execute::result_select_t  result_select,
    /* Save result to general purpose register. */
    input                           save_result_to_gpr,
    /* Destination register number. */
    input [ 4:0]                    rd_num,
    /* Operation is a divider operation. Must wait for divider to finish. */
    input                           is_div_op,
    /* Operation accesses memory. Must wait for memory access to finish. */
    input                           is_memory_op,
    /* Load-reserve instruction. */
    input                           is_lr,
    /* Store-condition instruction. */
    input                           is_sc,
    /* Memory operation must be sign extended. */
    input                           is_signed_memory_op,
    /* Operation invalidates the instruction cache. */
    input                           is_cache_invalidate,
    /* Operation invalidates the TLB. */
    input                           is_tlb_invalidate,
    /* Input values for modules. */
    input [`XLEN-1:0]               a,
    input [`XLEN-1:0]               b,
    /* Input value for comparator. */
    input [`XLEN-1:0]               a_comp,
    input [`XLEN-1:0]               b_comp,
    /* Jump to output of ALU if comparator output is 1. */
    input                           is_branch,
    input                           wait_for_interrupt,
    input                           suppress_interrupts,
    output [`XLEN-1:0]              execute_result,

    /** CONNECTIONS TO FETCH STAGE. */
    input  [`XLEN-1:1]              decode_instruction_vaddr,
    /* Redirect port. */
    output                          redirect,
    output reg [`XLEN-1:1]          redirect_vaddr,
    /* Cache port. */
    output [`VLEN-1:5]              cache_port_addr,
    output                          cache_port_set,
    output                          cache_invalidate,


    /** CONNECTIONS TO REGISTER FILE. **/
    /* Destination (write) register selector. */
    output [4:0]                    rd,
    /* Destination register value. */
    output [`XLEN-1:0]              rd_value,
    /* If set, the register will be written. */
    output                          rd_write,

    /** CONNECTIONS TO MMU. **/
    output [`VLEN-1:0]              mmu_vaddr,
    output reg                      mmu_access_r,
    output reg                      mmu_access_w,
    output reg                      mmu_access_x,
    output                          mmu_invalidate,
    input                           mmu_match,
    input                           mmu_fault,
    input                           mmu_fault_dns,
    input  [`PLEN-1:0]              mmu_paddr,

    /** MEMORY INTERFACE **/
    output                          mem_cycle,
    output [`PLEN-1:0]              mem_paddr,
    output execute::memory_access_t mem_access,
    output [`XLEN-1:0]              mem_data_out,
    /* Instruction cache fetches do not pass through here. */
    input  [`XLEN-1:0]              mem_data_in,
    /* Has to be asserted for the access to end.
     * Must not be asserted to more than one cycle.
     */
    input                           mem_ack,

    /** INTERRUPT INTERFACE. **/
    /* Interrupt enables. */
    input                           m_eie,
    input                           m_tie,
    input                           s_eie,
    input                           s_tie,
    /* Interrupt lines. */
    input                           m_timer,
    input                           s_timer,
    input                           m_interrupt,
    input                           s_interrupt,

    /** INTERNAL REGISTER FILE INTERFACE. **/
    input [`XLEN-1:0]               ireg_out,

    /** EXCEPTION INTERFACE **
     * Implemented by internal register file.
     */
    /* There is a pending exception. */
    output                          exception_pending,
    /* Pending exception is an interrupt. */
    output reg                      interrupt_exception,
    /* Exception code. */
    output reg [5:0]                exception_code,
    /* Next address to execute when taking an exception. */
    input  [`XLEN-1:2]              exception_vaddr,
    output [`XLEN-1:0]              add_result
);

/* INTERRUPT CONDITIONING. ********************************************************************************************/

/* Set if this is not the first cycle of an instruction. */
reg stalled = 0;
always @(posedge clock) begin
    stalled <= stall;
end

wire m_timer_conditioned     =                              (!suppress_interrupts) && (!stalled) && m_tie && m_timer;
wire s_timer_conditioned     = (current_mode <= mode::S) && (!suppress_interrupts) && (!stalled) && s_tie && s_timer;
wire m_interrupt_conditioned =                              (!suppress_interrupts) && (!stalled) && m_eie && m_interrupt;
wire s_interrupt_conditioned = (current_mode <= mode::S) && (!suppress_interrupts) && (!stalled) && s_eie && s_interrupt;

/* EXCEPTIONS. ********************************************************************************************************/

assign exception_pending =
        /* Incoming exceptions. */
        instruction_addr_misaligned || instruction_illegal || e_call || e_break || s_ret || m_ret
        /* Alignment faults. */
    ||  store_address_misaligned || load_address_misaligned
        /* MMU page faults. */
    ||  instruction_page_fault || store_page_fault || load_page_fault || tlb_miss
        /* Interrupts. */
    ||  m_timer_conditioned || s_timer_conditioned || m_interrupt_conditioned || s_interrupt_conditioned;

always_comb begin
    interrupt_exception = 0;
    exception_code = 0;
    if (m_interrupt_conditioned) begin
        interrupt_exception = 1;
        exception_code = exception::M_INT_EXT;
    end
    else if (s_interrupt_conditioned) begin
        interrupt_exception = 1;
        exception_code = exception::S_INT_EXT;
    end
    else if (m_timer_conditioned) begin
        interrupt_exception = 1;
        exception_code = exception::M_INT_TIMER;
    end
    else if (s_timer_conditioned) begin
        interrupt_exception = 1;
        exception_code = exception::S_INT_TIMER;
    end
    else if (instruction_addr_misaligned) begin
        exception_code = exception::I_ADDR_MISALIGNED;
    end
    else if (instruction_illegal) begin
        exception_code = exception::I_ILLEGAL;
    end
    else if (e_break) begin
        exception_code = exception::BREAKPOINT;
    end
    else if (load_address_misaligned) begin
        exception_code = exception::L_ADDR_MISALIGNED;
    end
    else if (store_address_misaligned) begin
        exception_code = exception::S_ADDR_MISALIGNED;
    end
    else if (e_call) begin
        exception_code = exception::U_CALL + {3'b0, current_mode};
    end
    else if (instruction_page_fault) begin
        exception_code = exception::I_PAGE_FAULT;
    end
    else if (load_page_fault) begin
        exception_code = exception::L_PAGE_FAULT;
    end
    else if (store_page_fault) begin
        if (!mmu_fault_dns) begin
            exception_code = exception::S_PAGE_FAULT;
        end
        else begin
            exception_code = exception::S_TLB_NOT_DIRTY;
        end
    end
    else if (tlb_miss) begin
        if (!mmu_access_w) begin
            exception_code = exception::L_TLB_MISS;
        end
        else begin
            exception_code = exception::S_TLB_MISS;
        end
    end
end

/* ALU INSTANCE. ******************************************************************************************************/

wire [`XLEN-1:0] alu_result;
Alu Alu(
    .op(alu_mul_div_shifter_op),
    .is_word_op(is_word_op),
    .a(a),
    .b(b),
    .result(alu_result),
    .add_result(add_result)
);

/* COMPARATOR INSTANCE. ***********************************************************************************************/

wire comparator_result;

Comparator Comparator(
    .op(comparator_op),
    .a(a_comp),
    .b(b_comp),
    .result(comparator_result)
);

/* DIVIDER INSTANCES. **************************************************************************************************
 *
 * Separate dividers are used for dword and word sized calculations.
 * This may be inefficient, but it keeps the inidividual dividers simple and it allows us to cache two different
 * results at the same time.
 *
 * This may change in the future. I am considering alternative divider design to address this issue and to speed up
 * division for small numbers.
 */

wire            div_done  = div_done_32 && div_done_64;
wire [`XLEN-1:0]    div_q = is_word_op ? {32'b0, div_q_32} : div_q_64;
wire [`XLEN-1:0]    div_r = is_word_op ? {32'b0, div_r_32} : div_r_64;
wire            div_stall = (is_div_op && (!div_done || !stalled));

/* 64b instance. */
wire div_done_64;
wire div_start_64 = (is_div_op && !stalled && !exception_pending && !is_word_op);

wire [`XLEN-1:0] div_q_64;
wire [`XLEN-1:0] div_r_64;

Div #(64) Div64(
    .clock(clock),
    .op(alu_mul_div_shifter_op[0]),
    .start(div_start_64),
    .dividend(a),
    .divisor(b),
    .done(div_done_64),
    .q(div_q_64),
    .r(div_r_64)
);

/* 32b instance. */
wire div_done_32;
wire div_start_32 = (is_div_op && !stalled && !exception_pending && is_word_op);

wire [`XLEN/2-1:0] div_q_32;
wire [`XLEN/2-1:0] div_r_32;

Div #(32) Div32(
    .clock(clock),
    .op(alu_mul_div_shifter_op[0]),
    .start(div_start_32),
    .dividend(a[`XLEN/2-1:0]),
    .divisor(b[`XLEN/2-1:0]),
    .done(div_done_32),
    .q(div_q_32),
    .r(div_r_32)
);

/* MUL INSTANCE. ******************************************************************************************************/

wire [`XLEN-1:0] mul_result;

Mul Mul(
    .clock(clock),
    .op(alu_mul_div_shifter_op[1:0]),
    .is_word_op(is_word_op),
    .a(a),
    .b(b),
    .result(mul_result)
);

/* SHIFTER INSTANCE. **************************************************************************************************/

wire [`XLEN-1:0] shifter_result;

Shifter Shifter(
    .op(alu_mul_div_shifter_op[1:0]),
    .is_word_op(is_word_op),
    .a(a),
    .n(b[5:0]),
    .result(shifter_result)
);

/* LR/SC LOGIC. *******************************************************************************************************/

reg memory_reserve_state = 0;

always @(posedge clock) begin
    /* Clear reservation on exceptions. */
    if (exception_pending) begin
        memory_reserve_state <= 0;
    end
    /* Reserve memory on LR instructions. */
    else if (mem_done && is_lr) begin
        memory_reserve_state <= 1;
    end
    /* Clear it on any other memory access.
     * This is excessive and unneccessary, but it is simple.
     */
    else if (mem_done && is_memory_op) begin
        memory_reserve_state <= 0;
    end
end

/* MEMORY INTERFACE. **************************************************************************************************/

/* Output address to MMU. */
assign mmu_vaddr = add_result[`VLEN-1:0];
wire vaddr_fault = (add_result[`XLEN-1:`VLEN] != {(`XLEN-`VLEN){add_result[`VLEN-1]}});

/* Are we done yet?
 * If this is store-conditional, and the reservation was lost, then we are done without a memory acces.
 * Otherwise, wait for the memory to acknowledge the access.
 */
wire mem_done = mem_ack || (is_sc && !memory_reserve_state);

/* Output memory interface bits.
 * If this is store-conditional, and the reservation was lost, then do not start a memory cycle.
 */
assign mem_cycle = is_memory_op && !exception_pending && (!is_sc || memory_reserve_state);
assign mem_paddr = mmu_paddr;
assign mem_access = memory_access;
assign mem_data_out = b_comp; /* comparator is not used during a store operation. */
assign mmu_invalidate = is_tlb_invalidate && !exception_pending;

/* Output access bits to MMU. */
always_comb begin
    automatic reg [2:0] rwx;
    case (memory_access)
        execute::BYTE_READ, execute::HWORD_READ, execute::WORD_READ, execute::DWORD_READ:
            rwx = 3'b100;
        execute::BYTE_WRITE, execute::HWORD_WRITE, execute::WORD_WRITE, execute::DWORD_WRITE:
            rwx = 3'b010;
        execute::LINE_READ:
            rwx = 3'b001;

        default:
            rwx = 3'b000;
    endcase

    {mmu_access_r, mmu_access_w, mmu_access_x} = rwx;
end

/* Handle sign/zero extension of result. */
reg [`XLEN-1:0] memory_result;
always_comb begin
    /* mem_data_in is expected to be zero extended already. */
    if (!is_signed_memory_op) begin
        case (memory_access)
            execute::BYTE_READ:     memory_result = {56'b0, mem_data_in[ 7:0]};
            execute::HWORD_READ:    memory_result = {48'b0, mem_data_in[15:0]};
            execute::WORD_READ:     memory_result = {32'b0, mem_data_in[31:0]};
            execute::DWORD_READ:    memory_result = mem_data_in;
            
            default:                memory_result = 64'hxxxxxxxxxxxxxxxx;
        endcase
    end
    else begin
        case (memory_access)
            execute::BYTE_READ:     memory_result = {{56{mem_data_in[ 7]}}, mem_data_in[ 7:0]};
            execute::HWORD_READ:    memory_result = {{48{mem_data_in[15]}}, mem_data_in[15:0]};
            execute::WORD_READ:     memory_result = {{32{mem_data_in[31]}}, mem_data_in[31:0]};
            execute::DWORD_READ:    memory_result = mem_data_in;
            
            default:                memory_result = 64'hxxxxxxxxxxxxxxxx;
        endcase
    end
end

/* Handle instruction cache writes. */
assign cache_port_set = mem_done && (memory_access == execute::LINE_READ) && !exception_pending;
assign cache_port_addr = instruction_vaddr[`VLEN-1:5];
assign cache_invalidate = is_cache_invalidate && !exception_pending;

/* Figure exception flags related to memory access. */
reg store_address_misaligned;
reg load_address_misaligned;
reg instruction_page_fault;
reg store_page_fault;
reg load_page_fault;
reg tlb_miss;
always_comb begin
    store_address_misaligned = 0;
    load_address_misaligned  = 0;
    instruction_page_fault   = 0;
    store_page_fault         = 0;
    load_page_fault          = 0;
    tlb_miss                 = 0;

    if (is_memory_op) begin
        automatic reg hword_misaligned = mmu_vaddr[0:0] != 0;
        automatic reg  word_misaligned = mmu_vaddr[1:0] != 0;
        automatic reg dword_misaligned = mmu_vaddr[2:0] != 0;

        /* Alignment checks. */
        case (memory_access)
            execute::HWORD_READ:        load_address_misaligned  = hword_misaligned;
            execute::HWORD_WRITE:       store_address_misaligned = hword_misaligned;
            execute::WORD_READ:         load_address_misaligned  =  word_misaligned;
            execute::WORD_WRITE:        store_address_misaligned =  word_misaligned;
            execute::DWORD_READ:        load_address_misaligned  = dword_misaligned;
            execute::DWORD_WRITE:       store_address_misaligned = dword_misaligned;

            default: begin end
        endcase

        /* Page fault checks. */
        instruction_page_fault = 0;
        store_page_fault = 0;
        load_page_fault = 0;
        if ((mmu_match && mmu_fault) || vaddr_fault) case (memory_access)
            execute::LINE_READ:                                                                     instruction_page_fault = 1;
            execute::BYTE_READ,  execute::HWORD_READ,  execute::WORD_READ,  execute::DWORD_READ:    load_page_fault = 1;
            execute::BYTE_WRITE, execute::HWORD_WRITE, execute::WORD_WRITE, execute::DWORD_WRITE:   store_page_fault = 1;

            default: begin
                /* Should not happen. */
                assert(0);
                instruction_page_fault = 1'bx;
                store_page_fault = 1'bx;
                load_page_fault = 1'bx;
            end
        endcase
        
        /* TLB miss check. */
        tlb_miss = !mmu_match;
    end
end

/* RESULT MUX. ********************************************************************************************************/

reg [`XLEN-1:0] result;

always_comb begin
    case (result_select)
        execute::COMPARATOR:        result = {63'b0, comparator_result};
        execute::SC_RESULT:         result = {63'b0, !memory_reserve_state};
        execute::ALU:               result = alu_result;
        execute::DIV_Q:             result = div_q;
        execute::DIV_R:             result = div_r;
        execute::MUL:               result = mul_result;
        execute::SHIFTER:           result = shifter_result;
        execute::MEMORY:            result = memory_result;
        execute::INTERNAL_REGISTER: result = ireg_out;
        execute::RETURN_ADDRESS:    result = {decode_instruction_vaddr, 1'b0};

        default:                    result = 64'hxxxxxxxxxxxxxxxx;
    endcase
end

assign execute_result = result;

/* SAVE RESULT TO GPR. ************************************************************************************************/

/* Always output the destination register we got from the decode stage.
 * This value is passed through this module just to keep things easier to follow.
 */
assign rd = rd_num;

/* If the instruction writes a GPR, tell the register file. */
assign rd_write = save_result_to_gpr && !exception_pending;
assign rd_value = result;

/* STALL LOGIC. *******************************************************************************************************/

assign stall = !exception_pending && (
            /* Handle stalling on divider. */
            div_stall
            /* Handle stalling on memory interface. */
        ||  (is_memory_op && !mem_done)
            /* Handle waiting for interrupts. */
        ||  (wait_for_interrupt && !(s_interrupt || m_interrupt || s_timer || m_timer))
            /* Multiplier has a 1 clock latency. */
        ||  (result_select == execute::MUL && save_result_to_gpr && !stalled)
    );

/* REDIRECT LOGIC. ****************************************************************************************************/

assign redirect = (is_branch && comparator_result) || exception_pending;

always_comb begin
    if (exception_pending) begin
        redirect_vaddr = {exception_vaddr, 1'b0};
    end
    else if (memory_access == execute::LINE_READ) begin
        redirect_vaddr = instruction_vaddr;
    end
    else begin
        redirect_vaddr = add_result[`XLEN-1:1];
    end
end

endmodule

`endif /* _EXECUTE_STAGE_SV */
