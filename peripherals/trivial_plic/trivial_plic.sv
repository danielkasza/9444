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

/* Trivial Platform-Level Interrupt Controller.
 *
 * This module was designed for use in very simple 9444 based systems.
 * The PLIC has an AXI4-Lite interface, not a 9444 native bus interface.
 * All interrupts have the same priority, and nesting is not supported.
 * All interrupts are triggered by high (1) level. Once an interrupt is pending, it will remain pending until it is
 * handled. This should allow interoperability with most peripherals that use edge or level triggered interrupts.
 * Drivers should be prepared to handle spurious interrupts.
 *
 * Only two registers are implemented:
 *  0x00_2000 - ENABLER
 *  0x20_0004 - CLAIMR
 * All other registers read as 0 and ignore writes.
 *
 * ENABLER is a bitmask of the enabled interrupts.
 *
 * CLAIMR returns the lowest pending, but not active interrupt ID and activates the interrupt on read.
 * CLAIMR deactivates the interrupt on write.
 *
 * ID 0 is reserved. Actual interrupt IDs start at 1.
 */

module TrivialPLIC(
    /* Interrupt request lines.
     * These are all level triggered, active high.
     */
    input           irq1,
    input           irq2,
    input           irq3,
    input           irq4,
    input           irq5,
    input           irq6,
    input           irq7,
    input           irq8,
    input           irq9,
    input           irq10,
    input           irq11,
    input           irq12,
    input           irq13,
    input           irq14,
    input           irq15,
    input           irq16,
    input           irq17,
    input           irq18,
    input           irq19,
    input           irq20,
    input           irq21,
    input           irq22,
    input           irq23,
    input           irq24,
    input           irq25,
    input           irq26,
    input           irq27,
    input           irq28,
    input           irq29,
    input           irq30,
    input           irq31,
    input           irq32,

    /* Interrupt request for S-mode. */
    output reg      s_interrupt,

    /* AXI4-Lite interface. */

    /* Global */
    input           ACLK,
    input           ARESETn,

    /* Write address channel. */
    input           AWVALID,
    output          AWREADY,
    input  [23:0]   AWADDR,
    input  [ 1:0]   AWPROT,

    /* Write data channel. */
    input           WVALID,
    output          WREADY,
    input  [31:0]   WDATA,
    input  [ 3:0]   WSTRB,

    /* Write response channel. */
    output          BVALID,
    input           BREADY,
    output [ 2:0]   BRESP,

    /* Read address channel. */
    input           ARVALID,
    output          ARREADY,
    input  [23:0]   ARADDR,
    input  [ 1:0]   ARPROT,

    /* Read data channel. */
    output          RVALID,
    input           RREADY,
    output [31:0]   RDATA,
    output [ 2:0]   RRESP
);

wire [31:0] irqs = {
    irq31, irq30, irq29, irq28, irq27, irq26, irq25, irq24, irq23, irq22, irq21, irq20, irq19, irq18, irq17, irq16,
    irq15, irq14, irq13, irq12, irq11, irq10, irq9,  irq8,  irq7,  irq6,  irq5,  irq4,  irq3,  irq2,  irq1,  1'b0
};

/* Interrupt state machine. *******************************************************************************************/

reg [31:0] pending_interrupts = 0;
reg [31:0] enabled_interrupts = 0;
reg [31:0]  active_interrupts = 0;

/* Find next interrupt to claim. */
reg [4:0] next_to_claim;
always_comb begin
    automatic integer i;

    next_to_claim = 0;
    for (i=1; i<32; i++) begin
        if (pending_interrupts[i]) begin
            next_to_claim = i[4:0];
            break;
        end
    end
end

/* Update active and pending interrupts. */
always @(posedge ACLK) begin
    automatic reg [31:0] new_active_interrupts  = active_interrupts;
    automatic reg [31:0] new_pending_interrupts = pending_interrupts;

    if (WVALID && WREADY && (write_address == 24'h200004)) begin
        /* Write to CLAIMR. */
        new_active_interrupts[WDATA[4:0]] = 0;
    end

    if (ARVALID && ARREADY && (ARADDR == 24'h200004)) begin
        /* Read from CLAIMR. */
        if (next_to_claim != 0) begin
            new_active_interrupts[next_to_claim]  = 1;
            new_pending_interrupts[next_to_claim] = 0;
        end
    end

    /* Register new interrupts. */
    new_pending_interrupts |= (irqs & enabled_interrupts);

    active_interrupts  <= new_active_interrupts;
    pending_interrupts <= new_pending_interrupts;
    s_interrupt <= (new_pending_interrupts != 0);
end

/* Write state machine. ***********************************************************************************************/

typedef enum [1:0] {
    /* Waiting for address on write address channel. */
    WRITE_STATE__IDLE,
    /* Waiting for data on write data channel. */
    WRITE_STATE__DATA,
    /* Waiting for BREADY. */
    WRITE_STATE__RESPONSE
} write_state_t;

write_state_t write_state = WRITE_STATE__IDLE;

/* Output our ready and valid signals. */
assign AWREADY = (write_state == WRITE_STATE__IDLE);
assign  WREADY = (write_state == WRITE_STATE__DATA);
assign BVALID  = (write_state == WRITE_STATE__RESPONSE) && ARESETn;

/* Our response is fixed. */
assign BRESP = 0; /* OKAY */

reg [23:0] write_address = 0;

always @(posedge ACLK) begin
    if (!ARESETn) begin
        write_state <= WRITE_STATE__IDLE;
    end
    else case(write_state)
        WRITE_STATE__IDLE:
            if (AWVALID) begin
                write_address <= AWADDR;
                write_state   <= WRITE_STATE__DATA;
            end
        WRITE_STATE__DATA:
            if (WVALID) begin
                if (write_address == 24'h002000) begin
                    enabled_interrupts <= WDATA;
                end
                /* Note: CLAIMR is handled by the interrupt state machine. */
                write_state <= WRITE_STATE__RESPONSE;
            end
        WRITE_STATE__RESPONSE:
            if (BREADY) begin
                write_state <= WRITE_STATE__IDLE;
            end
        default: assert(0);
    endcase
end

/* Read state machine. ************************************************************************************************/

typedef enum [0:0] {
    /* Waiting for address on read address channel. */
    READ_STATE__IDLE,
    /* Waiting for RREADY. */
    READ_STATE__DATA
} read_state_t;

read_state_t read_state = READ_STATE__IDLE;

/* Output our ready and valid signals. */
assign ARREADY = (read_state == READ_STATE__IDLE);
assign  RVALID = (read_state == READ_STATE__DATA) && ARESETn;

/* Response data. */
reg [31:0] rdata = 0;
assign RDATA = rdata;

/* Our response is fixed. */
assign RRESP = 0; /* OKAY */

always @(posedge ACLK) begin
    if (!ARESETn) begin
        read_state <= READ_STATE__IDLE;
    end
    else case(read_state)
        READ_STATE__IDLE:
            if (ARVALID) begin
                if (ARADDR == 24'h002000) begin
                    rdata <= enabled_interrupts;
                end
                else if (ARADDR == 24'h200004) begin
                    rdata <= {27'b0, next_to_claim};
                end
                else begin
                    rdata <= 0;
                end
                read_state <= READ_STATE__DATA;
            end
        READ_STATE__DATA:
            if (RREADY) begin
                read_state <= READ_STATE__IDLE;
            end
    endcase
end

endmodule