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

`include "cpu.sv"
`include "bus_splitter.sv"
`include "axi_lite_translator.sv"

/* Uncomment this line to use the simple AXI translator instead of the unified cache for M-bus. */
//`define M_BUS_UNCACHED 1

`ifdef M_BUS_UNCACHED
`include "simple_axi_translator.sv"
`else
`include "unified_cache.sv"
`endif

module Main#(parameter cache_line_log2_count = 13)(
    /* Useful for debugging. */
    output mode::mode_t             current_mode,
    output                          mode_m,
    output                          mode_s,
    output                          mode_u,
    output                          waiting_for_interrupt,

    /* Interrupt interface. */
    input                           s_interrupt,

    /* AXI global signals. */
    input                           ACLK,
    input                           ARESETn,

    /* Full AXI for M-bus. */
    output      [ 2:0]              AWID_M_BUS,
    output      [31:0]              AWADDR_M_BUS,
    output      [ 7:0]              AWLEN_M_BUS,
    output      [ 2:0]              AWSIZE_M_BUS,
    output      [ 3:0]              AWCACHE_M_BUS,
    output      [ 1:0]              AWPROT_M_BUS,
    output                          AWVALID_M_BUS,
    input                           AWREADY_M_BUS,
    output      [255:0]             WDATA_M_BUS,
    output      [ 31:0]             WSTRB_M_BUS,
    output                          WLAST_M_BUS,
    output                          WVALID_M_BUS,
    input                           WREADY_M_BUS,
    input       [ 2:0]              BID_M_BUS,
    input                           BVALID_M_BUS,
    output                          BREADY_M_BUS,
    output      [31:0]              ARADDR_M_BUS,
    output      [ 7:0]              ARLEN_M_BUS,
    output      [ 2:0]              ARSIZE_M_BUS,
    output      [ 3:0]              ARCACHE_M_BUS,
    output      [ 1:0]              ARPROT_M_BUS,
    output                          ARVALID_M_BUS,
    input                           ARREADY_M_BUS,
    input       [255:0]             RDATA_M_BUS,
    input                           RLAST_M_BUS,
    input                           RVALID_M_BUS,
    output                          RREADY_M_BUS,

    /* AXI-Lite for P-bus. */
    output      [31:0]              AWADDR_P_BUS,
    output      [ 1:0]              AWPROT_P_BUS,
    output                          AWVALID_P_BUS,
    input                           AWREADY_P_BUS,
    output       [31:0]             WDATA_P_BUS,
    output       [ 3:0]             WSTRB_P_BUS,
    output                          WVALID_P_BUS,
    input                           WREADY_P_BUS,
    input                           BVALID_P_BUS,
    output                          BREADY_P_BUS,
    output      [31:0]              ARADDR_P_BUS,
    output      [ 1:0]              ARPROT_P_BUS,
    output                          ARVALID_P_BUS,
    input                           ARREADY_P_BUS,
    input       [31:0]              RDATA_P_BUS,
    input                           RVALID_P_BUS,
    output                          RREADY_P_BUS
);

wire                        clock = ACLK;
wire                        m_interrupt = 0;
wire                        mem_cycle;
wire [`PLEN-1:0]            mem_paddr;
execute::memory_access_t    mem_access;
wire [`XLEN-1:0]            mem_data_out;
wire [3:0][`XLEN-1:0]       mem_data_in;
wire                        mem_ack;

CPU CPU(.*);

BusSplitter BusSplitter(.*);

wire                          m_cycle;
wire      [31:0]              m_paddr;
wire execute::memory_access_t m_access;
wire      [`XLEN-1:0]         m_data_out;
wire [3:0][`XLEN-1:0]         m_data_in;
wire                          m_ack;

`ifdef M_BUS_UNCACHED
assign AWID_M_BUS = 0;
SimpleAxiTranslator SimpleAxiTranslator(
`else
UnifiedCache#(.cache_line_log2_count(cache_line_log2_count)) UnifiedCache(
    .AWID(AWID_M_BUS),
    .BID(BID_M_BUS),
`endif
    .AWADDR(AWADDR_M_BUS),
    .AWLEN(AWLEN_M_BUS),
    .AWSIZE(AWSIZE_M_BUS),
    .AWCACHE(AWCACHE_M_BUS),
    .AWPROT(AWPROT_M_BUS),
    .AWVALID(AWVALID_M_BUS),
    .AWREADY(AWREADY_M_BUS),
    .WDATA(WDATA_M_BUS),
    .WSTRB(WSTRB_M_BUS),
    .WLAST(WLAST_M_BUS),
    .WVALID(WVALID_M_BUS),
    .WREADY(WREADY_M_BUS),
    .BVALID(BVALID_M_BUS),
    .BREADY(BREADY_M_BUS),
    .ARADDR(ARADDR_M_BUS),
    .ARLEN(ARLEN_M_BUS),
    .ARSIZE(ARSIZE_M_BUS),
    .ARCACHE(ARCACHE_M_BUS),
    .ARPROT(ARPROT_M_BUS),
    .ARVALID(ARVALID_M_BUS),
    .ARREADY(ARREADY_M_BUS),
    .RDATA(RDATA_M_BUS),
    .RLAST(RLAST_M_BUS),
    .RVALID(RVALID_M_BUS),
    .RREADY(RREADY_M_BUS),
    .*
);

wire                          p_cycle;
wire      [31:0]              p_paddr;
wire execute::memory_access_t p_access;
wire      [`XLEN-1:0]         p_data_out;
wire [3:0][`XLEN-1:0]         p_data_in;
wire                          p_ack;

AxiLiteTranslator AxiLiteTranslator(
    .AWADDR(AWADDR_P_BUS),
    .AWPROT(AWPROT_P_BUS),
    .AWVALID(AWVALID_P_BUS),
    .AWREADY(AWREADY_P_BUS),
    .WDATA(WDATA_P_BUS),
    .WSTRB(WSTRB_P_BUS),
    .WVALID(WVALID_P_BUS),
    .WREADY(WREADY_P_BUS),
    .BVALID(BVALID_P_BUS),
    .BREADY(BREADY_P_BUS),
    .ARADDR(ARADDR_P_BUS),
    .ARPROT(ARPROT_P_BUS),
    .ARVALID(ARVALID_P_BUS),
    .ARREADY(ARREADY_P_BUS),
    .RDATA(RDATA_P_BUS),
    .RVALID(RVALID_P_BUS),
    .RREADY(RREADY_P_BUS),
    .*
);

endmodule
