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

/* 9444 CPU Multiplier Unit.
 *
 * Internally, the multiplier always produces a 128b result with a 1 cycle latency.
 * The required 64b of the result are selected after that cycle.
 * While this may look inefficient, it seems to generate the best results with Xilinx tools.
 *
 * Historically, this module was capable of outputting the result combinationally.
 * Unfortunately, multiplication is a relatively slow operation, and this became the critical path in the design.
 * With the 1 cycle latency, this is no longer the critical path.
 */
`ifndef _MUL_SV
`define _MUL_SV

`include "common.sv"

/* Multiplier implementation. */
module Mul(
    input                   clock,
    input                   enable,
    input      mul::op_t    op,
    input                   is_word_op,
    input      [`XLEN-1:0]  a,
    input      [`XLEN-1:0]  b,
    output reg [`XLEN-1:0]  result
);

/* We calculate all results here and then extract the bits we need below.
 * Hopefully, synthesis tools are smart enough to figure out that we don't need all of these.
 */
reg        [(`XLEN*2)-1:0] u_result;
reg signed [(`XLEN*2)-1:0] s_result;
reg signed [(`XLEN*2)-1:0] suresult;

always @(posedge clock) begin
    /* The enable signal was added to improve simulation performance.
     * It is not necessary for the correct operation of the hardware.
     */
    if (enable) begin
        u_result <= a*b;
        s_result <= $signed(a)*$signed(b);
        suresult <= $signed(a)*$signed({1'b0, b});
    end
end

always_comb begin
    reg [`XLEN-1:0] dword_result;
    case(op)
        mul::MUL:    dword_result = s_result[ `XLEN-1   :    0];
        mul::MULH:   dword_result = s_result[(`XLEN*2)-1:`XLEN];
        mul::MULHSU: dword_result = suresult[(`XLEN*2)-1:`XLEN];
        mul::MULHU:  dword_result = u_result[(`XLEN*2)-1:`XLEN];
    endcase

    if (is_word_op) begin
        result = {{32{dword_result[31]}}, dword_result[31:0]};
    end
    else begin
        result = dword_result;
    end
end
endmodule

`endif /* _MUL_SV */
