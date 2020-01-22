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

/* Unified cache for 9444 CPU.
 *
 * Direct mapped, write-through cache.
 * The cache is a slave of the 9444 CPU bus, and it is an AXI (full) master.
 * The cache lines are 64B long.
 *
 * Read handling:
 *  - 1 clock latency on a cache hit.
 *  - 1+n clock latency if the cache line is not present in the cache. In this case, we have to wait for all write
 *    transactions in the same bucket to complete before we can read in the cache line.
 *
 * Write handling:
 *  - Write transactions always go on the AXI bus.
 *  - We can have up to 15 outstanding write transactions on the AXI bus per bucket.
 *  - There is also a 32 outstanding write limit for the entire bus.
 *  - Cache lines are not allocated on write, but already allocated lines are updated.
 * 
 * Buckets:
 *  - There are 8 transaction "buckets".
 *  - All AXI transactions fall into one of the buckets.
 *  - Outstanding write transactions are counter per bucket.
 *  - Buckets should significantly reduce the cost of fetching a new cache line because there is a good chance that
 *    the transaction bucket corresponding to the read transaction is empty.
 */

`include "common.sv"

module UnifiedCache#(parameter cache_line_log2_count = 13)(
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
    output reg  [ 2:0]              AWID,
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
    input       [ 2:0]              BID,
    input                           BVALID,
    output reg                      BREADY,

    /* AXI read address channel signals. */
    output reg  [31:0]              ARADDR,
    output      [ 7:0]              ARLEN,
    output      [ 2:0]              ARSIZE,
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

wire clock = ACLK;

/* CACHE ARRAY. *******************************************************************************************************/

localparam int cache_line_count = (1 << cache_line_log2_count);
localparam int cache_line_size = 64;

typedef struct packed {
    logic [cache_line_size*8-1:0]                               data;
    logic [31:$clog2(cache_line_size)+cache_line_log2_count]    paddr;
    logic                                                       valid;
} cache_line_t;

(* rw_addr_collision = "yes" *) (* ram_style = "block" *) reg [$bits(cache_line_t)-1:0] cache_lines[(cache_line_count-1):0];

integer i;
initial begin
    for (i=0; i<cache_line_count; i++) begin
        cache_lines[i] = 0;
    end
end

/* 9444 INTERFACE. ****************************************************************************************************/

/* Currently selected cache line. */
reg [cache_line_log2_count-1:0] selector;
cache_line_t cache_line_from_ram = 0;
cache_line_t cache_line_from_reg = 0;
reg use_cache_line_from_ram;
wire cache_line_t cache_line = use_cache_line_from_ram ? cache_line_from_ram : cache_line_from_reg;

/* Desired selector value based on paddr from the CPU. */
wire [cache_line_log2_count-1:0] selector_wanted =
    m_paddr[$clog2(cache_line_size)+cache_line_log2_count-1:$clog2(cache_line_size)];

/* Expected paddr tag. */
wire [31:$clog2(cache_line_size)+cache_line_log2_count] paddr_expected =
    m_paddr_d[31:$clog2(cache_line_size)+cache_line_log2_count];

/* Transaction bucket. */
wire [2:0] bucket_idx = m_paddr_d[$clog2(cache_line_size)+2:$clog2(cache_line_size)];

/* We have a cache hit if the cache line is valid, and the paddr tag is correct. */
wire cache_hit  = (cache_line.valid) && (cache_line.paddr == paddr_expected);

/* Figure out if we can ack the transaction yet.
 * Reads can be acked as soon as we have a cache hit.
 * Writes can be acked if the AXI interface is ready to accept a write transaction, and we have the right cache line
 * selected.
 */
reg ack_write;
always_comb begin
    m_ack = 0;
    ack_write = 0;
    case (m_access_d)
        execute::BYTE_READ,
        execute::HWORD_READ,
        execute::WORD_READ,
        execute::DWORD_READ,
        execute::LINE_READ:
            m_ack = m_cycle_d && cache_hit;
        
        execute::BYTE_WRITE,
        execute::HWORD_WRITE,
        execute::WORD_WRITE,
        execute::DWORD_WRITE: begin
            m_ack = m_cycle_d && can_accept_write;
            ack_write = m_ack;
        end
        
        default: assert(0);
    endcase
end

/* Number of bits to shift the cache line right to get to the requested data. */
wire [8:0] cl_shift = m_paddr_d[5:0] * 8;

/* Output the requested data from the cache. */
wire [511:0] read_shifted_data = cache_line.data >> cl_shift;
assign m_data_in = read_shifted_data[255:0];

/* Register the request from the CPU.
 * Historic note: the cache used to be capable of handling requests with 0 cycle latency, but this made timing closure
 * difficult.
 */
reg                      m_cycle_d;
reg [31:0]               m_paddr_d;
execute::memory_access_t m_access_d;
reg [`XLEN-1:0]          m_data_out_d;
always @(posedge clock) begin
    m_cycle_d <= m_cycle && !m_ack;
    m_paddr_d <= m_paddr;
    m_access_d <= m_access;
    m_data_out_d <= m_data_out;
end

/* AXI INTERFACE. *****************************************************************************************************/

/* Fixed read signals. */
/* Reads always fetch the entire cache line, so the burst length is 2, ARLEN is 1, and ARSIZE is 5. */
assign ARLEN   = 1;
assign ARSIZE  = 5;
/* Write-through, read-allocate. */
assign ARCACHE = 4'b1110;
/* Treating everything as unprivileged secure data access. */
assign ARPROT  = 0;

/* Fixed write signals. */
/* Writes always have a burst length of 1. */
assign AWLEN   = 0;
assign  WLAST  = 1;
/* Write-through, read-allocate. */
assign AWCACHE = 4'b0110;
/* Treating everything as unprivileged secure data access. */
assign AWPROT  = 0;

typedef enum [2:0] {
    /* Idle, no transaction is in progress, or only write transactions are outstanding. */
    STATE__IDLE,

    /* Read address phase. Read address is valid, waiting for ARREADY. */
    STATE__AR,
    /* Read data phase. Waiting for read data. */
    STATE__DR
} state_t;

state_t state = STATE__IDLE;

/* Output our valid and ready signals based on current state and reset.
 * Note: *VALID must be low during reset, we do not entirely comply with this.
 */
assign ARVALID = (state == STATE__AR) && ARESETn;
assign BREADY  = 1; /* We can always accept a write response. */
assign RREADY  = (state == STATE__DR);

/* Shift for using the correct byte lanes. */
wire [7:0]  bus_shift = m_paddr_d[4:0] * 8;
wire [4:0] strb_shift = m_paddr_d[4:0];

/* Number of outstanding writes per bucket. */
reg [7:0][4:0] outstanding_writes_bucket;

/* Number of outstanding writes for this bucket. */
wire [4:0] outstanding_writes = outstanding_writes_bucket[bucket_idx];

/* Total number of outstanding writes. */
reg [5:0] total_outstanding_writes = 0;

/* Can we accept another write?
 * We can, if there are fewer than 15 outstanding writes in this bucket, fewer than 32 outstanding writes total,
 * and the AW, W channels will be available on the next cycle.
 */
wire can_accept_write = (outstanding_writes < 15)
                     && (total_outstanding_writes < 32)
                     && ((!AWVALID) || AWREADY) && ((!WVALID) || WREADY);

/* Figure out next value of AWVALID and WVALID. */
reg awvalid_next;
reg  wvalid_next;
always_comb begin
    /* Having ready signals resets the valid signals. */
    awvalid_next = AWVALID && !AWREADY;
     wvalid_next =  WVALID &&  !WREADY;

    /* But if we are accepting a new write transaction, we have to set them again on the next cycle. */
    if (m_ack) begin
        case (m_access_d)
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
    automatic cache_line_t cl = cache_line;

    if (ARESETn) begin
        /* Normal operation. */
        case (state)
            STATE__IDLE: begin
                /* If the CPU is trying to access memory... */
                if (m_cycle_d) begin
                    /* Output addresses. */
                    ARADDR <= {m_paddr_d[31:$clog2(cache_line_size)], {$clog2(cache_line_size){1'b0}}};
                    if (can_accept_write) begin
                        AWADDR <= m_paddr_d;
                        AWID   <= bucket_idx;
                    end
                    /* Output access size and strobe. */
                    case (m_access_d)
                        execute::BYTE_READ,
                        execute::HWORD_READ,
                        execute::WORD_READ,
                        execute::DWORD_READ,
                        execute::LINE_READ: begin
                                /* Nothing to do. All reads are treated the same. */
                            end

                        execute::BYTE_WRITE: begin
                                if (can_accept_write) begin
                                    AWSIZE <= 0;
                                    WSTRB  <= 32'b1 << strb_shift;
                                end
                            end
                        execute::HWORD_WRITE: begin
                                if (can_accept_write) begin
                                    WSTRB  <= 32'b11 << strb_shift;
                                    AWSIZE <= 1;
                                end
                            end
                        execute::WORD_WRITE: begin
                                if (can_accept_write) begin
                                    WSTRB  <= 32'b1111 << strb_shift;
                                    AWSIZE <= 2;
                                end
                            end
                        execute::DWORD_WRITE: begin
                                if (can_accept_write) begin
                                    WSTRB  <= 32'b11111111 << strb_shift;
                                    AWSIZE <= 3;
                                end
                            end

                        default: assert(0);
                    endcase
                    /* Output write data. */
                    if (can_accept_write) begin
                        WDATA <= {192'b0, m_data_out_d}  << bus_shift;
                    end
                    /* Update cache if this is a write transaction, and we have a copy in the cache. */
                    if (m_ack && cache_hit) begin
                        automatic reg [63:0] write_mask = 0;
                        case (m_access_d)
                            default:
                                begin /* Nothing to do. */ end
                            
                            execute::BYTE_WRITE:    write_mask = 64'h00000000000000FF;
                            execute::HWORD_WRITE:   write_mask = 64'h000000000000FFFF;
                            execute::WORD_WRITE:    write_mask = 64'h00000000FFFFFFFF;
                            execute::DWORD_WRITE:   write_mask = 64'hFFFFFFFFFFFFFFFF;
                        endcase

                        if (write_mask != 0) begin
                            automatic reg [63:0] data_masked = m_data_out_d & write_mask;
                            cl.data &= ~({448'b0, write_mask} << cl_shift);
                            cl.data |= ({448'b0, data_masked} << cl_shift);
                        end
                    end

                    /* GO to next state. */
                    case (m_access_d)
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
                                /* Cannot accept read transactions unless all writes are done, and we do not have to
                                 * fetch the cache line unless there is a cache miss.
                                 */
                                if (outstanding_writes == 0 && (!cache_hit)) begin
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
                    /* We got one half of the cache line data.
                     * If not RLAST, then this is the first half.
                     * If RLAST, this is the second half, and we can go back to idle.
                     */
                    if (!RLAST) begin
                        cl.data[255:0] = RDATA;
                    end 
                    else begin
                        cl.data[511:256] = RDATA;
                        cl.paddr = paddr_expected;
                        cl.valid = 1;
                        state <= STATE__IDLE;
                    end
                end
            end

            default: assert(0);
        endcase


        /* Count the outstanding write transactions in this bucket. */
        if ((ack_write == BVALID) && (BID == bucket_idx) ) begin
            /* No change. */
        end
        else begin
            if (BVALID) begin
                outstanding_writes_bucket[BID] <= outstanding_writes_bucket[BID] - 1;
            end
            if (ack_write) begin
                outstanding_writes_bucket[bucket_idx] <= outstanding_writes + 1;
            end
        end

        /* Count the outstanding write transactions total. */
        if (ack_write == BVALID) begin
            /* No change. */
        end
        else if (BVALID) begin
            total_outstanding_writes <= total_outstanding_writes - 1;
        end
        else begin
            total_outstanding_writes <= total_outstanding_writes + 1;
        end

        /* Update AWVALID, WVALID. */
        AWVALID <= awvalid_next;
         WVALID <=  wvalid_next;
    end
    else begin
        /* We are being reset. Go back to idle state. */
        state <= STATE__IDLE;
        outstanding_writes_bucket <= 0;
        total_outstanding_writes <= 0;
        AWVALID <= 0;
         WVALID <= 0;
    end

    /* Make sure the correct cache line is selected and loaded on the next cycle. 
     * NOTE: This may look odd, but it was very carefully written to allow Vivado to infer block RAM, and to
     * make sure the correct value gets used on the next cycle. If this is not actually needed, please let me know.
     */
    if (selector == selector_wanted) begin
        cache_line_from_reg <= cl;
    end
    if (m_cycle) begin
        selector <= selector_wanted;
        cache_line_from_ram <= cache_lines[selector_wanted];
        use_cache_line_from_ram <= (selector != selector_wanted);
    end

    cache_lines[selector] <= cl;
end

endmodule