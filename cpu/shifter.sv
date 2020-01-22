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

/* 9444 CPU Bit-shifter Unit. */
`ifndef _SHIFTER_SV
`define _SHIFTER_SV

`include "common.sv"

/* Bit-shifter implementation. */
module Shifter(
    input  shifter::op_t    op,
    input                   is_word_op,
    input  [`XLEN-1:0]      a,
    input  [ 5:0]           n,
    output reg [`XLEN-1:0]  result
);

always_comb begin
    if (!is_word_op) begin
        case(op)
            shifter::SLL:   result = a << n;
            shifter::SRL:   result = a >> n;
            shifter::SRA:   result = $signed(a) >>> n;

            default:        result = 64'hxxxxxxxxxxxxxxxx;
        endcase
    end
    else begin
        reg [31:0] word_result;
        case(op)
            shifter::SLL:   word_result = a[31:0] << n[4:0];
            shifter::SRL:   word_result = a[31:0] >> n[4:0];
            shifter::SRA:   word_result = $signed(a[31:0]) >>> n[4:0];

            default:        word_result = 32'hxxxxxxxx;
        endcase

        result = {{32{word_result[31]}}, word_result};
    end
end
endmodule

`endif /* _SHIFTER_SV */
