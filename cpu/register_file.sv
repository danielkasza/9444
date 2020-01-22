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

/* 9444 register file.
 *
 * The register file supports reading two registers, and writing one at the same time.
 * If the register written is also being read, the register file returns the new value.
 * Register 0 always returns 0, as required by RISC-V.
 */
`ifndef _REGISTER_FILE_SV
`define _REGISTER_FILE_SV

`include "common.sv"

module RegisterFile(
    input clock,
    input stall,

    /* Source register selectors. */
    input  [4:0]            rs1,
    input  [4:0]            rs2,

    /* Source register values. */
    output reg [`XLEN-1:0]  rs1_value,
    output reg [`XLEN-1:0]  rs2_value,

    /* Destination (write) register selector. */
    input  [4:0]            rd,
    /* If set, the register will be written. */
    input                   rd_write,

    /* Destination register value. */
    input  [`XLEN-1:0]      rd_value
);

reg [`XLEN-1:0] registers[31:0];

/* Initialize registers to 0. */
initial begin
    int i;
    for (i=0; i<32; i++) begin
        registers[i] = 0;
    end
end

wire rd_valid = (!stall) && rd_write && (rd != 0);

/* Output rs values. */
always_comb begin
    /* rs1 */
    if (rd_valid && (rs1 == rd)) begin
        rs1_value = rd_value;
    end
    else begin
        rs1_value = registers[rs1];
    end

    /* rs2 */
    if (rd_valid && (rs2 == rd)) begin
        rs2_value = rd_value;
    end
    else begin
        rs2_value = registers[rs2];
    end
end

/* Store rd value. */
always @(posedge clock) begin
    if (rd_valid) begin
        registers[rd] <= rd_value;
    end
end

endmodule

`endif /* _REGISTER_FILE_SV */
