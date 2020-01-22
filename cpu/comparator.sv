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

/* 9444 comparator for conditional instructions. */
`ifndef _COMPARATOR_SV
`define _COMPARATOR_SV

`include "common.sv"

/* Comparator implementation. */
module Comparator(
    input comparator::op_t  op,
    input [`XLEN-1:0]       a,
    input [`XLEN-1:0]       b,
    output reg              result
);

wire eq  = (a == b);
wire lt  = ($signed(a) < $signed(b));
wire ltu = (a < b);

always_comb begin
    case(op)
        comparator::EQ:     result =  eq;
        comparator::NE:     result = !eq;
        comparator::LT:     result =  lt;
        comparator::LTU:    result =  ltu;
        comparator::GE:     result = !lt;
        comparator::GEU:    result = !ltu;
        comparator::TRUE:   result =  1;

        default:            result = 1'bx;
    endcase
end

endmodule

`endif /* _COMPARATOR_SV */