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

/* 9444 CPU Divider Unit.
 *
 * This is a straightforward restoring division implementation.
 * The width of the divider can be configured.
 * It is expected that the CPU will have separate 32b and 64b instances.
 */
`ifndef _DIV_SV
`define _DIV_SV

`include "common.sv"

/* Divider implementation. */
module Div#(
    parameter DIVLEN=32
)(
    input                   clock,
    input      div::op_t    op,
    input                   start,
    input      [DIVLEN-1:0] dividend,
    input      [DIVLEN-1:0] divisor,
    output                  done,
    output reg [DIVLEN-1:0] q,
    output reg [DIVLEN-1:0] r
);

/* We'll use these for the match output and for initializing the state machine. */
reg [DIVLEN-1:0] sr_init; /* Initial value for sr. */
reg [DIVLEN-1:0] d_init;  /* Initial value for d. */
reg q_neg_init;     /* If set, the quotient should be converted to negative when done. */
reg r_neg_init;     /* If set, the remainder should be converted to negative when done. */
always_comb begin
    if (op == div::DIV) begin
        /* Signed division. */
        
        /* If the divisor is negative, make it positive. */
        if ($signed(divisor) < 0) begin
            d_init = -divisor;
        end
        else begin
            d_init =  divisor;
        end

        /* If the dividend is negative, make it positive.
         * Also, the remainder will have to be negative.
         */
        if ($signed(dividend) < 0) begin
            sr_init = -dividend;
            r_neg_init = 1;
        end
        else begin
            sr_init = dividend;
            r_neg_init = 0;
        end

        /* If the signs disagree, the quotient should be negative. */
        if (($signed(dividend) < 0 && $signed(divisor) > 0) || ($signed(dividend) > 0 && $signed(divisor) < 0)) begin
            q_neg_init = 1;
        end
        else begin
            q_neg_init = 0;
        end
    end
    else begin
        /* Unsigned division. */
        d_init = divisor;
        sr_init = dividend;
        r_neg_init = 0;
        q_neg_init = 0;
    end
end

/* Internal state. */
reg [DIVLEN*2-1:0] sr;
reg [DIVLEN-1:0] d;
reg q_neg;
reg r_neg;
reg [$clog2(DIVLEN):0] step; /* Number of steps finished. */
assign done = (step == DIVLEN);

reg [DIVLEN*2-1:0] sr_next;
always_comb begin
    /* We need an extra bit for the sign. */
    reg [DIVLEN*2:0] tmp  = ({1'b0, sr} << 1) - {1'b0, d, {DIVLEN{1'b0}}};
    if ($signed(tmp) < 0) begin
        sr_next = sr << 1;
    end
    else begin
        sr_next = tmp[DIVLEN*2-1:0] | 1;
    end
end

always @(posedge clock) begin
    if (start) begin
        /* start is asserted, load initial values. */
        sr <= {{DIVLEN{1'b0}}, sr_init};
        d  <= d_init;
        q_neg <= q_neg_init;
        r_neg <= r_neg_init;
        step <= 0;
    end
    else if (!done) begin
        /* Keep calculating... */
        sr <= sr_next;
        step <= step + 1;
    end
end

/* Output results. */
always_comb begin
    reg [DIVLEN-1:0] r_unsigned = sr[DIVLEN*2-1:DIVLEN];
    reg [DIVLEN-1:0] q_unsigned = sr[DIVLEN-1:0];

    r = r_neg ? (-r_unsigned) : r_unsigned;
    q = q_neg ? (-q_unsigned) : q_unsigned;
end

endmodule

`endif /* _DIV_SV */
