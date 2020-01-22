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

/* 9444 instruction fetch stage.
 *
 * The stage includes the instruction cache.
 * This cache comprises 256 cache lines, each holding 8 instructions. (Effectively, a 4KB cache.)
 * The cache lines are direct mapped.
 * Cache refills are handled by the execute stage through the cache port.
 * The instruction cache can be invalidated in a single cycle.
 *
 * The instruction cache only stores the top 30b of each instruction.
 * During a cache refill, compressed instructions are replaced with an invalid instruction.
 * This is explicitly allowed by (riscv.1.2).
 */
`ifndef _FETCH_STAGE_SV
`define _FETCH_STAGE_SV

`include "common.sv"

module FetchStage#(
    logic  [`XLEN-1:0] reset_vaddr = 0
)(
    input                   clock,
    input                   mmu_enabled,

    /* Set if the pipeline is stalled. */
    input                   stall,

    /* Current execution mode. */
    input mode::mode_t      current_mode,
    /* Set if SUM bit is set.
     * If set, U mode cache lines can be used by S mode.
     */
    input                   current_sum,

    /** CONNECTIONS TO DECODE STAGE. **/
    /* Set if the instruction address was misaligned and could not be fetched. */
    output                  instruction_addr_misaligned,
    /* Set if the instruction could not be fetched due to a cache miss. */
    output reg              instruction_cache_miss,
    /* Virtual address of the instruction. */
    output reg [`XLEN-1:1]  instruction_vaddr = reset_vaddr[`XLEN-1:1],
    /* The instruction. */
    output [31:2]           instruction_word,
    /* Predicted address of next instruction. */
    input  [`XLEN-1:1]      predicted_vaddr,

    /** REDIRECT PORT. **/
    /* If true, the signals below are used to redirect the CPU to a different instruction.
     * If false, the signals below are ignored and the unit fetches the next sequential instruction.
     */
    input                   redirect,
    /* Next instruction to fetch. */
    input  [`XLEN-1:1]      redirect_vaddr,

    /** CACHE PORT. **/
    /* Address of the cache line. */
    input  [`VLEN-1:5]      cache_port_addr,
    /* 8 instructions at this address. */
    input [7:0][31:0]       cache_port_data,
    /* If set, the data will be written in the cache. */
    input                   cache_port_set,
    /* If set, the instruction cache will be invalidated.
     * This takes precedence over cache_port_set.
     */
    input                   cache_invalidate,
    /* If set, invalidate only the entries that match the address.
     * Note that we only check the top bit.
     */
    input                   selective_invalidate,
    input [`VLEN-1:`VLEN-1] selective_invalidate_vaddr
);

/* INSTRUCTION CACHE. *************************************************************************************************/

parameter cache_line_log2count = 9;
localparam int cache_line_count = (1 << cache_line_log2count);
localparam int instructions_in_line = 8;

/* Bitmap of valid cache lines.
 * The valid bit is not part of the tag because we want to be able to clear all valid bits in a single cycle.
 */
reg [(cache_line_count-1):0] valid_lines = 0;

/* Top vaddr bit of each cache line.
 * This is used for selective invalidation.
 */
reg [(cache_line_count-1):0] top_bits = 0;

/* Convert compressed instructions to invalid instructions, and truncate the rest. */
function logic [31:2] preprocess_instruction;
input logic [31:0] instruction;
preprocess_instruction = instruction[1:0] == 2'b11 ? instruction[31:2] : 30'hFFFFFFFF;
endfunction


/* Contents of a cache line. */
typedef struct packed {
    /* Payload, cached instructions. */
    logic [(instructions_in_line-1):0][31:2] instructions;
    /* Mode tag. */
    mode::mode_t                             access_mode;
    /* Virtual address tag. */
    logic [`VLEN-1:5+cache_line_log2count]   vaddr;
    /* MMU status flag.
     * We cannot reuse cache lines filled when the MMU was disabled once the MMU is enabled.
     */
    logic                                    mmu_enabled;
} cache_line_t;

/* Cache lines. */
(* rw_addr_collision = "yes" *) (* ram_style = "block" *) reg [$bits(cache_line_t)-1:0] cache_lines[(cache_line_count-1):0];

/* Index of cache line addressed by cache_port_addr. */
wire [cache_line_log2count-1:0] cache_port_idx = cache_port_addr[5+cache_line_log2count-1:5];

always @(posedge clock) begin
    if (!stall) begin
        if (cache_invalidate) begin
            /* Handle cache invalidation. */

            automatic reg [(cache_line_count-1):0] do_not_invalidate = 0;
            
            if (!selective_invalidate) begin
                /* The entire cache will be invalidated. */
                do_not_invalidate = 0;
            end
            else begin
                /* Only matching entries will be invalidated. */
                do_not_invalidate = selective_invalidate_vaddr ? (~top_bits) : (top_bits);
            end

            valid_lines <= valid_lines & do_not_invalidate;
        end
        else if (cache_port_set) begin
            automatic cache_line_t cache_line;
            /* Save the tag for the cache line. */
            cache_line.access_mode = current_mode;
            cache_line.vaddr       = cache_port_addr[`VLEN-1:5+cache_line_log2count];
            cache_line.mmu_enabled = mmu_enabled;
            /* Save the payload.
             * This is where we convert compressed instructions to invalid instructions.
             */
            cache_line.instructions[0] = preprocess_instruction(cache_port_data[0]);
            cache_line.instructions[1] = preprocess_instruction(cache_port_data[1]);
            cache_line.instructions[2] = preprocess_instruction(cache_port_data[2]);
            cache_line.instructions[3] = preprocess_instruction(cache_port_data[3]);
            cache_line.instructions[4] = preprocess_instruction(cache_port_data[4]);
            cache_line.instructions[5] = preprocess_instruction(cache_port_data[5]);
            cache_line.instructions[6] = preprocess_instruction(cache_port_data[6]);
            cache_line.instructions[7] = preprocess_instruction(cache_port_data[7]);
            /* Mark the cache line as valid. */
            valid_lines[cache_port_idx] <= 1;
            top_bits[cache_port_idx]    <= cache_port_addr[`VLEN-1];
            cache_lines[cache_port_idx] <= cache_line;
        end
        else begin
            /* Nothing to do here. */
        end
    end
end



/* FETCH INSTRUCTION. *************************************************************************************************/

parameter [31:0] reset_address = 0;

/* The address is misaligned if the lowest bit is set. */
assign instruction_addr_misaligned = instruction_vaddr[1:1];

/* Index of cache line addressed by instruction_vaddr. */
wire [cache_line_log2count-1:0] instr_line_idx = instruction_vaddr[5+cache_line_log2count-1:5];
/* Index of instruction within the cache line. */
wire [2:0] instr_idx = instruction_vaddr[4:2];
/* Output the instruction even if there is no hit. */
wire cache_line_t fetch_cache_line = cache_lines[instr_line_idx];
assign instruction_word = fetch_cache_line.instructions[instr_idx];

/* Is this a cache hit? */
always_comb begin
    if (instruction_addr_misaligned) begin
        /* The address is misaligned.
         * Pretend that this was a hit to avoid refilling cache line before the actual exception can be handled.
         */
        instruction_cache_miss = 0;
    end
    else begin
        /* Is the cache line valid? */
        reg valid = valid_lines[instr_line_idx];

        /* Does the virtual address match? */
        reg vaddr_match = (instruction_vaddr[`XLEN-1:5+cache_line_log2count] ==
            {{(`XLEN-`VLEN){fetch_cache_line.vaddr[`VLEN-1]}}, fetch_cache_line.vaddr});

        /* Does the access mode match?
         * Note: S can access U if SUM is set.
         */
        reg mode_match = (current_mode == fetch_cache_line.access_mode)
            || (current_sum && (current_mode == mode::S) && (fetch_cache_line.access_mode == mode::U));
        
        /* Does the MMU status match? */
        reg mmu_status_match = (mmu_enabled == fetch_cache_line.mmu_enabled);
    
        instruction_cache_miss = !(valid && vaddr_match && mode_match && mmu_status_match);
    end
end

/* FIGURE OUT NEXT INSTRUCTION ADDRESS. *******************************************************************************/

always @(posedge clock) begin
    if (!stall) begin
        if (redirect) begin
            /* Execute stage requested redirection. */
            instruction_vaddr <= redirect_vaddr;
        end
        else begin
            /* Continue sequentially. */
            instruction_vaddr <= predicted_vaddr;
        end
    end
end

endmodule

`endif /* _FETCH_STAGE_SV */
