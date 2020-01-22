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

/* Simple VGA device.
 *
 * This module reads a framebuffer over AXI and refreshes a VGA monitor.
 * The only resolution supported is 1024x768.
 * The pixel format is RGB565, little endian.
 */

module SimpleVGA#(
    /* Base address of the framebuffer. */
    logic [31:0] fb_base = 32'hBFE80000,
    /* Pixel clock dividiver.
     * ACLK divided by this should be close to 65MHz.
     */
    logic [ 3:0] pixel_clock_divider = 3
)(
    /* VGA signals. */
    output reg                      VGA_HS,
    output reg                      VGA_VS,
    output reg  [ 4:0]              VGA_R,
    output reg  [ 5:0]              VGA_G,
    output reg  [ 4:0]              VGA_B,

    /* AXI global signals. */
    input                           ACLK,
    input                           ARESETn,

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

/* VGA LOGIC. *********************************************************************************************************/

/* XGA 1024x768 timings. */
/* Whole line, including porches and sync. */
localparam reg [10:0] whole_line  = 1344;
/* Horizontal resolution. */
localparam reg [10:0] h_res       = 1024;
/* Horizontal front porch length. */
localparam reg [10:0] hf_porch    = 24;
/* Horizontal back porch length. */
localparam reg [10:0] hb_porch    = 160;
/* Horizontal sync length. */
localparam reg [10:0] hsync       = 136;
/* Whole frame, including porches and sync. */
localparam reg [10:0] whole_frame = 806;
/* Vertical resolution. */
localparam reg [10:0] v_res       = 768;
/* Vertical front porch length. */
localparam reg [10:0] vf_porch    = 3;
/* Vertical back porch length. */
localparam reg [10:0] vb_porch    = 29;
/* Vertical sync length. */
localparam reg [10:0] vsync       = 6;

/* Data from AXI will be copied here. */
reg [63:0][15:0] vga_data = 0;
/* Current pixel data. */
wire [15:0] vga_px = vga_data[h[5:0]];

/* Asserted when we are in the display area and should output a pixel. */
wire vga_display_area = (h < h_res) && (v < v_res);
/* Asserted on cycles when we have to output a new VGA pixel. */
wire vga_px_change = (c == (pixel_clock_divider-1)) && vga_display_area;
/* Asserted on the cycle when we use the last pixel from VGA data. */
wire vga_data_last = vga_px_change && (h[5:0] == 6'b111111);

/* Asserted when we ran out of pixel data in the VGA side of the logic, and the AXI side of the logic
 * cannot supply new pixels yet.
 */
wire stall = vga_data_last && (state != STATE__IDLE || !ARESETn);

reg [ 3:0] c = 0;
reg [10:0] h = 0;
reg [10:0] v = 0;
always @(posedge clock) begin
    /* Count horizontal and vertical position in frame. */
    if (!stall) begin
        if (c == (pixel_clock_divider-1)) begin
            c <= 0;

            if (h == (whole_line-1)) begin
                h <= 0;
                if (v == (whole_frame-1)) begin
                    v <= 0;
                end
                else begin
                    v <= v + 1;
                end
            end
            else begin
                h <= h + 1;
                v <= v;
            end
        end
        else begin
            c <= c + 1;
            h <= h;
            v <= v;
        end
    end

    /* Output pixel data and generate sync pulses. */
    if (!vga_display_area) begin
        VGA_R <= 0;
        VGA_G <= 0;
        VGA_B <= 0;
    end
    else if (vga_px_change) begin
        VGA_R <= vga_px[15:11];
        VGA_G <= vga_px[10: 5];
        VGA_B <= vga_px[ 4: 0];
    end

    if (vga_data_last) begin
        vga_data <= axi_data;
    end

    if ((h >= (h_res + hf_porch)) && (h < (h_res + hf_porch + hsync))) begin
        VGA_HS <= 0;
    end
    else begin
        VGA_HS <= 1;
    end
    if ((v >= (v_res + vf_porch)) && (v < (v_res + vf_porch + vsync))) begin
        VGA_VS <= 0;
    end
    else begin
        VGA_VS <= 1;
    end
end

/* AXI LOGIC. *********************************************************************************************************/

initial begin
    ARADDR = $unsigned(fb_base)+128;
end

/* We will read 128B of data here. */
reg [1023:0] axi_data;

/* Fixed read signals. */
/* Reads always fetch 128B, so the burst length is 4, ARLEN is 3, and ARSIZE is 5. */
assign ARLEN   = 3;
assign ARSIZE  = 5;
assign ARCACHE = 4'b0000;
/* Treating everything as unprivileged secure data access. */
assign ARPROT  = 0;

typedef enum [2:0] {
    /* Idle, no transaction is in progress. */
    STATE__IDLE,

    /* Read address phase. Read address is valid, waiting for ARREADY. */
    STATE__AR,
    /* Read data phases. Waiting for read data. */
    STATE__DR1,
    STATE__DR2,
    STATE__DR3,
    STATE__DR4
} state_t;

state_t state = STATE__IDLE;

/* Output our valid and ready signals based on current state and reset. */
assign ARVALID = (state == STATE__AR) && ARESETn;
assign RREADY  = 1;

always @(posedge clock) begin
    if (ARESETn) begin
        /* Normal operation. */
        case (state)
            STATE__IDLE: begin
                if (vga_data_last) begin
                    if (ARADDR == ($unsigned(fb_base) + (h_res * v_res * 2) - 128)) begin
                        ARADDR <= $unsigned(fb_base);
                    end
                    else begin
                        ARADDR <= ARADDR + 128;
                    end
                    state <= STATE__AR;
                end
            end

            STATE__AR: begin
                if (ARREADY) begin
                    state <= STATE__DR1;
                end
            end

            STATE__DR1: begin
                if (RVALID) begin
                    axi_data[255:0] <= RDATA;
                    state <= STATE__DR2;
                end
            end
            STATE__DR2: begin
                if (RVALID) begin
                    axi_data[511:256] <= RDATA;
                    state <= STATE__DR3;
                end
            end
            STATE__DR3: begin
                if (RVALID) begin
                    axi_data[767:512] <= RDATA;
                    state <= STATE__DR4;
                end
            end
            STATE__DR4: begin
                if (RVALID) begin
                    axi_data[1023:768] <= RDATA;
                    state <= STATE__IDLE;
                end
            end

            default: assert(0);
        endcase
    end
    else begin
        /* We are being reset. Go back to idle state. */
        state <= STATE__IDLE;
    end
end

endmodule