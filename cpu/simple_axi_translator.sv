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

/* Very small AXI to 9444 bus translator.
 *
 * The module inserts multiple wait states into all transactions, so it is not recommended for real systems.
 * The module provides the same interface as the unified cache, so it can be used for experimentation and for triaging
 * issues that may be cache problems.
 */
`include "common.sv"

module SimpleAxiTranslator(
    /* 9444 bus interface.
     * Note: only 32b because this module is expected to be used with a bus splitter.
     */
    input                           m_cycle,
    input       [31:0]              m_paddr,
    input  execute::memory_access_t m_access,
    input       [`XLEN-1:0]         m_data_out,
    output [3:0][`XLEN-1:0]         m_data_in,
    output reg                      m_ack,

    /* AXI global signals. */
    input                           ACLK,
    input                           ARESETn,

    /* AXI write address channel. */
    output reg  [31:0]              AWADDR,
    output      [ 7:0]              AWLEN,
    output reg  [ 2:0]              AWSIZE,
    output      [ 3:0]              AWCACHE,
    output      [ 1:0]              AWPROT,
    output reg                      AWVALID,
    input                           AWREADY,

    /* AXI write data channel. */
    output reg  [255:0]             WDATA,
    output reg  [ 31:0]             WSTRB,
    output                          WLAST,
    output reg                      WVALID,
    input                           WREADY,

    /* AXI write response channel signals. */
    input                           BVALID,
    output reg                      BREADY,

    /* AXI read address channel signals. */
    output reg  [31:0]              ARADDR,
    output      [ 7:0]              ARLEN,
    output reg  [ 2:0]              ARSIZE,
    output      [ 3:0]              ARCACHE,
    output      [ 1:0]              ARPROT,
    output                          ARVALID,
    input                           ARREADY,

    /* AXI read data signals. */
    input       [255:0]             RDATA,
    input                           RLAST,
    input                           RVALID,
    output                          RREADY
);

/* Some of our signals are fixed. */
assign AWLEN    = 0;
assign ARLEN    = 0;
assign AWCACHE  = 1;
assign ARCACHE  = 1;
assign AWPROT   = 0;
assign ARPROT   = 0;
assign WLAST    = 1;

/* The data read by the CPU is always the read data. */
assign m_data_in = RDATA >> bus_shift;

typedef enum [2:0] {
    /* Idle, no transaction is in progress, or only write transactions are outstanding. */
    STATE__IDLE,

    /* Read address phase. Read address is valid, waiting for ARREADY. */
    STATE__AR,
    /* Read data phase. Waiting for read data. */
    STATE__DR
} state_t;

/* Current state of the module. */
state_t state = STATE__IDLE;

/* Output our valid and ready signals based on current state and reset.
 * Note: *VALID must be low during reset, we do not entirely comply with this.
 */
assign ARVALID = (state == STATE__AR) && ARESETn;
assign BREADY  = 1; /* We can always accept a write response. */
assign RREADY  = (state == STATE__DR);

wire clock = ACLK;

/* Shift for using the correct byte lanes. */
wire [7:0]  bus_shift = m_paddr[4:0] * 8;
wire [4:0] strb_shift = m_paddr[4:0];

/* Number of outstanding writes. */
reg [4:0] outstanding_writes = 0;
wire can_accept_write = (outstanding_writes < 31) && ((!AWVALID) || AWREADY) && ((!WVALID) || WREADY);

/* Figure out next value of AWVALID and WVALID. */
reg awvalid_next = 0;
reg  wvalid_next = 0;
always_comb begin
    /* Having ready signals resets the valid signals. */
    awvalid_next = AWVALID && !AWREADY;
     wvalid_next =  WVALID &&  !WREADY;

    /* But if we are accepting a new write transaction, we have to set them again on the next cycle. */
    if (can_accept_write && m_cycle) begin
        case (m_access)
            execute::BYTE_WRITE,
            execute::HWORD_WRITE,
            execute::WORD_WRITE,
            execute::DWORD_WRITE: begin
                awvalid_next = 1;
                 wvalid_next = 1;
            end
            default: begin /* Nothing to do. */ end
        endcase
    end
end

always @(posedge clock) begin
    if (ARESETn) begin
        /* Normal operation. */
        case (state)
            STATE__IDLE: begin
                /* If the CPU is trying to access memory... */
                if (m_cycle) begin
                    /* Output addresses. */
                    ARADDR <= m_paddr[31:0];
                    if (can_accept_write) begin
                        AWADDR <= m_paddr[31:0];
                    end
                    /* Output access size and strobe. */
                    case (m_access)
                        execute::BYTE_READ,
                        execute::BYTE_WRITE: begin
                                ARSIZE <= 0;
                                if (can_accept_write) begin
                                    AWSIZE <= 0;
                                    WSTRB  <= 32'b1 << strb_shift;
                                end
                            end
                        execute::HWORD_READ,
                        execute::HWORD_WRITE: begin
                                ARSIZE <= 1;
                                if (can_accept_write) begin
                                    WSTRB  <= 32'b11 << strb_shift;
                                    AWSIZE <= 1;
                                end
                            end
                        execute::WORD_READ,
                        execute::WORD_WRITE: begin
                                ARSIZE <= 2;
                                if (can_accept_write) begin
                                    WSTRB  <= 32'b1111 << strb_shift;
                                    AWSIZE <= 2;
                                end
                            end
                        execute::DWORD_READ,
                        execute::DWORD_WRITE: begin
                                ARSIZE <= 3;
                                if (can_accept_write) begin
                                    WSTRB  <= 32'b11111111 << strb_shift;
                                    AWSIZE <= 3;
                                end
                            end
                        execute::LINE_READ: begin
                                ARSIZE <= 5;
                            end
                        default: assert(0);
                    endcase
                    /* Output write data. */
                    if (can_accept_write) begin
                        WDATA <= {192'b0, m_data_out}  << bus_shift;
                    end
                    /* GO to next state. */
                    case (m_access)
                        execute::BYTE_WRITE,
                        execute::HWORD_WRITE,
                        execute::WORD_WRITE,
                        execute::DWORD_WRITE: begin
                                /* Nothing to do. */
                            end
                        execute::BYTE_READ,
                        execute::HWORD_READ,
                        execute::WORD_READ,
                        execute::DWORD_READ,
                        execute::LINE_READ: begin
                                /* Cannot accept read transactions unless all writes are done. */
                                if (outstanding_writes == 0) begin
                                    state <= STATE__AR;
                                end
                            end
                        default: assert(0);
                    endcase
                end
            end

            /* Read */
            STATE__AR: begin
                if (ARREADY) begin
                    state <= STATE__DR;
                end
            end
            STATE__DR: begin
                if (RVALID) begin
                    state <= STATE__IDLE;
                end
            end

            default: assert(0);
        endcase

        /* Count the outstanding write transactions. */
        if (ack_write == BVALID) begin
            /* No change. */
        end
        else if (BVALID) begin
            outstanding_writes <= outstanding_writes - 1;
        end
        else begin
            outstanding_writes <= outstanding_writes + 1;
        end

        /* Update AWVALID, WVALID. */
        AWVALID <= awvalid_next;
         WVALID <=  wvalid_next;
    end
    else begin
        /* We are being reset. Go back to idle state. */
        state <= STATE__IDLE;
        outstanding_writes <= 0;
        AWVALID <= 0;
         WVALID <= 0;
    end

end

/* Acknowledge access from 9444.
 * Note: we can acknowledge writes as soon as we are in the idle state. 9444 does not care about the response, so this
 * works well as a performance improvement.
 */
reg ack_write;
always_comb begin
    m_ack = 0;
    ack_write = 0;

    if ((state == STATE__DR) && RVALID) begin
        /* Read access completed. */
        m_ack = 1;
    end
    else if (state == STATE__IDLE && ARESETn && m_cycle) case (m_access)
        execute::BYTE_WRITE,
        execute::HWORD_WRITE,
        execute::WORD_WRITE,
        execute::DWORD_WRITE:
            if (can_accept_write) begin
                /* We have everything we need to complete this write access. */
                m_ack = 1;
                ack_write = 1;
            end
        default: begin /* Nothing to do. */ end
    endcase
end

endmodule
