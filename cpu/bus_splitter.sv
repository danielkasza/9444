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

/* Bus splitter for 9444 CPU.
 *
 * This module splits the CPU bus into two buses: the Memory bus, an the Peripheral bus.
 * Each bus occupies a 32b chunk of the physical memory bus of the CPU.
 *
 * +---------+
 * |  P-bus  | 1_FFFF_FFFF
 * |         |
 * |  (4GB)  | 1_0000_0000
 * +---------+-------------
 * |  M-bus  | 0_FFFF_FFFF
 * |         |
 * |  (4GB)  | 0_0000_0000
 * +---------+
 *
 * The intent is to use the M-bus width the UnifiedCache and use the P-Bus with the AxiLiteTranslator.
 */

`include "common.sv"

module BusSplitter(
    /* 9444 bus interface. */
    input                           mem_cycle,
    input       [`PLEN-1:0]         mem_paddr,
    input  execute::memory_access_t mem_access,
    input       [`XLEN-1:0]         mem_data_out,
    output [3:0][`XLEN-1:0]         mem_data_in,
    output                          mem_ack,

    /* P-bus interface. */
    output                          p_cycle,
    output      [31:0]              p_paddr,
    output execute::memory_access_t p_access,
    output      [`XLEN-1:0]         p_data_out,
    input  [3:0][`XLEN-1:0]         p_data_in,
    input                           p_ack,

    /* M-bus interface. */
    output                          m_cycle,
    output      [31:0]              m_paddr,
    output execute::memory_access_t m_access,
    output      [`XLEN-1:0]         m_data_out,
    input  [3:0][`XLEN-1:0]         m_data_in,
    input                           m_ack
);

wire p_bus_select = mem_paddr[32];

assign p_cycle = mem_cycle &&  p_bus_select;
assign m_cycle = mem_cycle && !p_bus_select;

assign p_paddr = mem_paddr[31:0];
assign m_paddr = mem_paddr[31:0];

assign p_access = mem_access;
assign m_access = mem_access;

assign p_data_out = mem_data_out;
assign m_data_out = mem_data_out;

assign mem_data_in = p_bus_select ? p_data_in : m_data_in;

assign mem_ack = p_bus_select ? p_ack : m_ack;

endmodule