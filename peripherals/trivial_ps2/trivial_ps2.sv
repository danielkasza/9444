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

/* Trivial PS/2 Controller.
 *
 * Register map:
 *  0x0004 - Status Register (RO)
 *  0x0008 - Receive Data (RO)
 *  0x000C - Transmit Data (WO)
 *  0x002C - Global Interrupt Enable (RW)
 *  0x0030 - Interrupt Status (W1C)
 *  0x0038 - Interrupt Enable (RW)
 * The registers are big-endian.
 */

module TrivialPS2(
    output [4:0]    debug_state,

    /* PS/2 interface. */
    input           ps2_data_i,
    output reg      ps2_data_o,
    output reg      ps2_data_t,
    input           ps2_clock_i,
    output reg      ps2_clock_o,
    output reg      ps2_clock_t,

    /* Interrupt request line.
     * This is a level triggered interrupt line.
     */
    output          interrupt_out,

    /* AXI4-Lite interface. */

    /* Global */
    input           ACLK,
    input           ARESETn,

    /* Write address channel. */
    input           AWVALID,
    output          AWREADY,
    input  [15:0]   AWADDR,
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
    input  [15:0]   ARADDR,
    input  [ 1:0]   ARPROT,

    /* Read data channel. */
    output          RVALID,
    input           RREADY,
    output [31:0]   RDATA,
    output [ 2:0]   RRESP
);

reg interrupt_global_enable;
reg interrupt_rx_full_enable;
reg interrupt_tx_complete_enable;

reg [7:0] rx_data = 0;
/* Cleared when the receive register is read. */
reg rx_full = 0;

reg [7:0] tx_data = 0;
wire tx_parity =
    tx_data[0] ^ tx_data[1] ^ tx_data[2] ^ tx_data[3] ^ tx_data[4] ^ tx_data[5] ^ tx_data[6] ^ tx_data[7] ^ 1;
/* Set when the transmit register is written, cleared on tx completetion. */
reg tx_full = 0;
/* Set when the tx_full is cleared, cleared when the corresponding bit in the interrupt status register is written. */
reg tx_complete_interrupt = 0;

wire [31:0]      status_register = {6'b0, tx_full, rx_full, 24'b0};
wire [31:0]     receive_register = {rx_data, 24'b0};
wire [31:0] global_intr_register = {24'b0, interrupt_global_enable, 7'b0};
wire [31:0] intr_status_register = {2'b0, rx_full, 2'b0, tx_complete_interrupt, 2'b0, 24'b0};
wire [31:0] intr_enable_register = {2'b0, interrupt_rx_full_enable, 2'b0, interrupt_tx_complete_enable, 2'b0, 24'b0};

assign interrupt_out = (global_intr_register != 0) && ((intr_status_register & intr_enable_register) != 0);

/* PS/2 interface. ****************************************************************************************************/

/* Last state of the data line.
 * This is used to detect 1 to 0 transitions.
 */
reg last_data = 0;

typedef enum [4:0] {
    /* Bus is idle. No transmit or receive in progress, but the bus may be inhibited. */
    PS2__IDLE,
    
    /* RX states. *****************************************************************************************************/
    /* Waiting for bit 0..7. */
    PS2__RX_D0,
    PS2__RX_D1,
    PS2__RX_D2,
    PS2__RX_D3,
    PS2__RX_D4,
    PS2__RX_D5,
    PS2__RX_D6,
    PS2__RX_D7,
    /* Waiting for parity bit. */
    PS2__RX_P,
    /* Waiting for stop bit. */
    PS2__RX_STOP,

    /* TX states. *****************************************************************************************************/
    /* Sending request to send. */
    PS2__TX_RTS,
    /* Sending start bit. */
    PS2__TX_START,
    /* Sending bit 0..7. */
    PS2__TX_D0,
    PS2__TX_D1,
    PS2__TX_D2,
    PS2__TX_D3,
    PS2__TX_D4,
    PS2__TX_D5,
    PS2__TX_D6,
    PS2__TX_D7,
    /* Sending parity bit. */
    PS2__TX_P,
    /* Sending stop bit. */
    PS2__TX_S,
    /* Waiting for ack bit. */
    PS2__TX_ACK
} ps2_state_t;

ps2_state_t ps2_state;

assign debug_state = ps2_state;

/* If not 0, the bus should be inhibited. */
reg [13:0] inhibit_counter;

/* Delayed clocks.
 * These are used for detecting falling edges.
 */
reg ps2_delay_4_clock = 0;
reg ps2_delay_3_clock = 0;
reg ps2_delay_2_clock = 0;
reg ps2_delay_1_clock = 0;

/* Check for transitions. */
wire ps2_clock_falling = (ps2_delay_4_clock == 1) && (ps2_delay_3_clock == 0);

/* RX buffer used during reception. */
reg [7:0] rx_buff;
/* Expected parity of rx_buff. */
wire rx_buff_xparity =
    rx_buff[0] ^ rx_buff[1] ^ rx_buff[2] ^ rx_buff[3] ^ rx_buff[4] ^ rx_buff[5] ^ rx_buff[6] ^ rx_buff[7] ^ 1;

/* Set state of PS/2 pins combinationally. */
always_comb begin
    ps2_data_o = 0;
    ps2_data_t = 1;
    ps2_clock_o = 0;
    ps2_clock_t = 1;

    /* If the bus is inhibited, drive the clock low. */
    if (inhibit_counter != 0) begin
        ps2_clock_t = 0;
    end

    /* If we are in the TX path, drive the data pin. */
    case (ps2_state)
        PS2__TX_RTS:    ps2_data_t = 0;
        PS2__TX_START:  ps2_data_t = 0;
        PS2__TX_D0:     ps2_data_t = tx_data[0];
        PS2__TX_D1:     ps2_data_t = tx_data[1];
        PS2__TX_D2:     ps2_data_t = tx_data[2];
        PS2__TX_D3:     ps2_data_t = tx_data[3];
        PS2__TX_D4:     ps2_data_t = tx_data[4];
        PS2__TX_D5:     ps2_data_t = tx_data[5];
        PS2__TX_D6:     ps2_data_t = tx_data[6];
        PS2__TX_D7:     ps2_data_t = tx_data[7];
        PS2__TX_P:      ps2_data_t = tx_parity;
        PS2__TX_S:      ps2_data_t = 1;
        default: begin /* nothing to do */ end
    endcase
    case (ps2_state)
        PS2__TX_RTS:    ps2_data_o = 0;
        PS2__TX_START:  ps2_data_o = 0;
        PS2__TX_D0:     ps2_data_o = tx_data[0];
        PS2__TX_D1:     ps2_data_o = tx_data[1];
        PS2__TX_D2:     ps2_data_o = tx_data[2];
        PS2__TX_D3:     ps2_data_o = tx_data[3];
        PS2__TX_D4:     ps2_data_o = tx_data[4];
        PS2__TX_D5:     ps2_data_o = tx_data[5];
        PS2__TX_D6:     ps2_data_o = tx_data[6];
        PS2__TX_D7:     ps2_data_o = tx_data[7];
        PS2__TX_P:      ps2_data_o = tx_parity;
        PS2__TX_S:      ps2_data_o = 1;
        default: begin /* nothing to do */ end
    endcase
end

/* AXI write handling. ************************************************************************************************/

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

reg [15:0] write_address = 0;

/* AXI read handling. *************************************************************************************************/

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

/* Sequential logic. **************************************************************************************************/

always @(posedge ACLK) begin
    automatic reg new_rx_full = rx_full;
    automatic reg new_tx_full = tx_full;
    automatic reg new_tx_complete_interrupt = tx_complete_interrupt;
    automatic reg [13:0] new_inhibit_counter = inhibit_counter;

    /* Update inhibit counter. */
    if (new_inhibit_counter != 0) begin
        new_inhibit_counter--;
    end

    /* Update delayed clocks. */
    ps2_delay_4_clock <= ps2_delay_3_clock;
    ps2_delay_3_clock <= ps2_delay_2_clock;
    ps2_delay_2_clock <= ps2_delay_1_clock;
    ps2_delay_1_clock <= ps2_clock_i;

    /* Handle state transitions. */
    case (ps2_state)
        PS2__IDLE: begin
                /* If there is data in the transmit buffer, start transmitting. */
                if (tx_full) begin
                    /* We also have to inhibit the bus. */
                    new_inhibit_counter = -1;
                    ps2_state <= PS2__TX_RTS;
                end
                /* If there is a start bit, start receiving.
                 * If we are inhibiting the bus, there is no way there could be a real start bit.
                 */
                else if (ps2_clock_falling && !ps2_data_i && (inhibit_counter == 0)) begin
                    ps2_state <= PS2__RX_D0;
                end
            end

        /* RX states. *************************************************************************************************/
        PS2__RX_D0: if (ps2_clock_falling) begin
                ps2_state <= PS2__RX_D1;
                rx_buff[0] <= ps2_data_i;
            end
        PS2__RX_D1: if (ps2_clock_falling) begin
                ps2_state <= PS2__RX_D2;
                rx_buff[1] <= ps2_data_i;
            end
        PS2__RX_D2: if (ps2_clock_falling) begin
                ps2_state <= PS2__RX_D3;
                rx_buff[2] <= ps2_data_i;
            end
        PS2__RX_D3: if (ps2_clock_falling) begin
                ps2_state <= PS2__RX_D4;
                rx_buff[3] <= ps2_data_i;
            end
        PS2__RX_D4: if (ps2_clock_falling) begin
                ps2_state <= PS2__RX_D5;
                rx_buff[4] <= ps2_data_i;
            end
        PS2__RX_D5: if (ps2_clock_falling) begin
                ps2_state <= PS2__RX_D6;
                rx_buff[5] <= ps2_data_i;
            end
        PS2__RX_D6: if (ps2_clock_falling) begin
                ps2_state <= PS2__RX_D7;
                rx_buff[6] <= ps2_data_i;
            end
        PS2__RX_D7: if (ps2_clock_falling) begin
                ps2_state <= PS2__RX_P;
                rx_buff[7] <= ps2_data_i;
            end

        PS2__RX_P: if (ps2_clock_falling) begin
                if (ps2_data_i != rx_buff_xparity) begin
                    /* Parity did not match. Inhibit the bus to force retransmission. */
                    new_inhibit_counter = -1;
                    ps2_state <= PS2__IDLE;
                end
                else if (rx_full) begin
                    /* Software could not keep up with the reception. Inhibit the bus to force retransmission. */
                    new_inhibit_counter = -1;
                    ps2_state <= PS2__IDLE;
                end
                else begin
                    /* Received data. */
                    new_rx_full = 1;
                    rx_data <= rx_buff;
                    ps2_state <= PS2__RX_STOP;
                end
            end
        
        PS2__RX_STOP: if (ps2_clock_falling) begin
                ps2_state <= PS2__IDLE;
            end

        /* TX states. *************************************************************************************************/
        PS2__TX_RTS:   if (inhibit_counter == 0) ps2_state <= PS2__TX_START;
        PS2__TX_START: if (ps2_clock_falling) ps2_state <= PS2__TX_D0;
        PS2__TX_D0:    if (ps2_clock_falling) ps2_state <= PS2__TX_D1;
        PS2__TX_D1:    if (ps2_clock_falling) ps2_state <= PS2__TX_D2;
        PS2__TX_D2:    if (ps2_clock_falling) ps2_state <= PS2__TX_D3;
        PS2__TX_D3:    if (ps2_clock_falling) ps2_state <= PS2__TX_D4;
        PS2__TX_D4:    if (ps2_clock_falling) ps2_state <= PS2__TX_D5;
        PS2__TX_D5:    if (ps2_clock_falling) ps2_state <= PS2__TX_D6;
        PS2__TX_D6:    if (ps2_clock_falling) ps2_state <= PS2__TX_D7;
        PS2__TX_D7:    if (ps2_clock_falling) ps2_state <= PS2__TX_P;
        PS2__TX_P:     if (ps2_clock_falling) ps2_state <= PS2__TX_S;
        PS2__TX_S:     if (ps2_clock_falling) ps2_state <= PS2__TX_ACK;
        PS2__TX_ACK: begin
                if (!ps2_data_i) begin
                    /* Successful transmission. */
                    ps2_state <= PS2__IDLE;
                    new_tx_complete_interrupt = 1;
                    new_tx_full = 0;
                end
                else begin
                    /* Transmission failed. Go back to idle state. We'll transmit again. */
                    ps2_state <= PS2__IDLE;
                end
            end

        default: assert(0);
    endcase

    /* Write state machine. *******************************************************************************************/
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
                if (write_address == 16'h000C) begin
                    tx_data <= WDATA[31:24];
                    new_tx_full = 1;
                end
                else if (write_address == 16'h002C) begin
                    interrupt_global_enable <= WDATA[7];
                end
                else if (write_address == 16'h0030) begin
                    if (WDATA[26]) begin
                        new_tx_complete_interrupt = 0;
                    end
                end
                else if (write_address == 16'h0038) begin
                    interrupt_rx_full_enable <= WDATA[29];
                    interrupt_tx_complete_enable <= WDATA[26];
                end
                write_state <= WRITE_STATE__RESPONSE;
            end
        WRITE_STATE__RESPONSE:
            if (BREADY) begin
                write_state <= WRITE_STATE__IDLE;
            end
        default: assert(0);
    endcase

    /* Read state machine. ********************************************************************************************/
    if (!ARESETn) begin
        read_state <= READ_STATE__IDLE;
    end
    else case(read_state)
        READ_STATE__IDLE:
            if (ARVALID) begin
                if (ARADDR == 16'h0004) begin
                    rdata <= status_register;
                end
                else if (ARADDR == 16'h0008) begin
                    rdata <= receive_register;
                    /* Warning: Technically, this is wrong. If we just received a new byte, we will return the old byte
                     * and clear the flag...
                     * To avoid this, do not read the receive register more than once for a single received byte.
                     */
                    new_rx_full = 0;
                end
                else if (ARADDR == 16'h002C) begin
                    rdata <= global_intr_register;
                end
                else if (ARADDR == 16'h0030) begin
                    rdata <= intr_status_register;
                end
                else if (ARADDR == 16'h0038) begin
                    rdata <= intr_enable_register;
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

    rx_full <= new_rx_full;
    tx_full <= new_tx_full;
    tx_complete_interrupt <= new_tx_complete_interrupt;
    inhibit_counter <= new_inhibit_counter;
end

endmodule