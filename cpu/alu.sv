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

/* 9444 CPU Arithmetic Logic Unit. */
`ifndef _ALU_SV
`define _ALU_SV

`include "common.sv"

/* ALU implementation. */
module Alu(
    input      alu::op_t    op,
    input                   is_word_op,
    input      [`XLEN-1:0]  a,
    input      [`XLEN-1:0]  b,
    output reg [`XLEN-1:0]  result,
    output reg [`XLEN-1:0]  add_result
);

always_comb begin
    reg [`XLEN-1:0] dword_result;

    add_result = a + b;

    case(op)
        alu::ADD:       dword_result = add_result;
        alu::XOR:       dword_result = a ^ b;
        alu::OR:        dword_result = a | b;
        alu::AND:       dword_result = a & b;
        alu::SUB:       dword_result = a - b;
        default:        dword_result = 64'hxxxxxxxxxxxxxxxx;
    endcase

    if (is_word_op) begin
        result = {{32{dword_result[31]}}, dword_result[31:0]};
    end
    else begin
        result = dword_result;
    end
end
endmodule

`endif /* _ALU_SV */