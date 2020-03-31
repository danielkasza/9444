//Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
//--------------------------------------------------------------------------------
//Tool Version: Vivado v.2019.2 (lin64) Build 2708876 Wed Nov  6 21:39:14 MST 2019
//Date        : Mon Mar 30 20:42:19 2020
//Host        : peacekeeper running 64-bit Ubuntu 18.04.4 LTS
//Command     : generate_target main_wrapper.bd
//Design      : main_wrapper
//Purpose     : IP block netlist
//--------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module main_wrapper
   (VGA_B_0,
    VGA_G_0,
    VGA_HS_0,
    VGA_R_0,
    VGA_VS_0,
    ddr3_sdram_addr,
    ddr3_sdram_ba,
    ddr3_sdram_cas_n,
    ddr3_sdram_ck_n,
    ddr3_sdram_ck_p,
    ddr3_sdram_cke,
    ddr3_sdram_cs_n,
    ddr3_sdram_dm,
    ddr3_sdram_dq,
    ddr3_sdram_dqs_n,
    ddr3_sdram_dqs_p,
    ddr3_sdram_odt,
    ddr3_sdram_ras_n,
    ddr3_sdram_reset_n,
    ddr3_sdram_we_n,
    fan_pwm,
    memory_card_spi_io0_io,
    memory_card_spi_io1_io,
    memory_card_spi_sck_io,
    memory_card_spi_ss_io,
    mode_m,
    mode_s,
    mode_u,
    ps2_clock_JA_tri_io,
    ps2_clock_JB_tri_io,
    ps2_clock_usb_tri_io,
    ps2_data_JA_tri_io,
    ps2_data_JB_tri_io,
    ps2_data_usb_tri_io,
    reset,
    sys_diff_clock_clk_n,
    sys_diff_clock_clk_p,
    usb_uart_rxd,
    usb_uart_txd,
    waiting_for_interrupt);
  output [4:0]VGA_B_0;
  output [5:0]VGA_G_0;
  output VGA_HS_0;
  output [4:0]VGA_R_0;
  output VGA_VS_0;
  output [14:0]ddr3_sdram_addr;
  output [2:0]ddr3_sdram_ba;
  output ddr3_sdram_cas_n;
  output [0:0]ddr3_sdram_ck_n;
  output [0:0]ddr3_sdram_ck_p;
  output [0:0]ddr3_sdram_cke;
  output [0:0]ddr3_sdram_cs_n;
  output [3:0]ddr3_sdram_dm;
  inout [31:0]ddr3_sdram_dq;
  inout [3:0]ddr3_sdram_dqs_n;
  inout [3:0]ddr3_sdram_dqs_p;
  output [0:0]ddr3_sdram_odt;
  output ddr3_sdram_ras_n;
  output ddr3_sdram_reset_n;
  output ddr3_sdram_we_n;
  output [0:0]fan_pwm;
  inout memory_card_spi_io0_io;
  inout memory_card_spi_io1_io;
  inout memory_card_spi_sck_io;
  inout [0:0]memory_card_spi_ss_io;
  output mode_m;
  output mode_s;
  output mode_u;
  inout ps2_clock_JA_tri_io;
  inout ps2_clock_JB_tri_io;
  inout ps2_clock_usb_tri_io;
  inout ps2_data_JA_tri_io;
  inout ps2_data_JB_tri_io;
  inout ps2_data_usb_tri_io;
  input reset;
  input sys_diff_clock_clk_n;
  input sys_diff_clock_clk_p;
  input usb_uart_rxd;
  output usb_uart_txd;
  output waiting_for_interrupt;

  wire [4:0]VGA_B_0;
  wire [5:0]VGA_G_0;
  wire VGA_HS_0;
  wire [4:0]VGA_R_0;
  wire VGA_VS_0;
  wire [14:0]ddr3_sdram_addr;
  wire [2:0]ddr3_sdram_ba;
  wire ddr3_sdram_cas_n;
  wire [0:0]ddr3_sdram_ck_n;
  wire [0:0]ddr3_sdram_ck_p;
  wire [0:0]ddr3_sdram_cke;
  wire [0:0]ddr3_sdram_cs_n;
  wire [3:0]ddr3_sdram_dm;
  wire [31:0]ddr3_sdram_dq;
  wire [3:0]ddr3_sdram_dqs_n;
  wire [3:0]ddr3_sdram_dqs_p;
  wire [0:0]ddr3_sdram_odt;
  wire ddr3_sdram_ras_n;
  wire ddr3_sdram_reset_n;
  wire ddr3_sdram_we_n;
  wire [0:0]fan_pwm;
  wire memory_card_spi_io0_i;
  wire memory_card_spi_io0_io;
  wire memory_card_spi_io0_o;
  wire memory_card_spi_io0_t;
  wire memory_card_spi_io1_i;
  wire memory_card_spi_io1_io;
  wire memory_card_spi_io1_o;
  wire memory_card_spi_io1_t;
  wire memory_card_spi_sck_i;
  wire memory_card_spi_sck_io;
  wire memory_card_spi_sck_o;
  wire memory_card_spi_sck_t;
  wire [0:0]memory_card_spi_ss_i_0;
  wire [0:0]memory_card_spi_ss_io_0;
  wire [0:0]memory_card_spi_ss_o_0;
  wire memory_card_spi_ss_t;
  wire mode_m;
  wire mode_s;
  wire mode_u;
  wire ps2_clock_JA_tri_i;
  wire ps2_clock_JA_tri_io;
  wire ps2_clock_JA_tri_o;
  wire ps2_clock_JA_tri_t;
  wire ps2_clock_JB_tri_i;
  wire ps2_clock_JB_tri_io;
  wire ps2_clock_JB_tri_o;
  wire ps2_clock_JB_tri_t;
  wire ps2_clock_usb_tri_i;
  wire ps2_clock_usb_tri_io;
  wire ps2_clock_usb_tri_o;
  wire ps2_clock_usb_tri_t;
  wire ps2_data_JA_tri_i;
  wire ps2_data_JA_tri_io;
  wire ps2_data_JA_tri_o;
  wire ps2_data_JA_tri_t;
  wire ps2_data_JB_tri_i;
  wire ps2_data_JB_tri_io;
  wire ps2_data_JB_tri_o;
  wire ps2_data_JB_tri_t;
  wire ps2_data_usb_tri_i;
  wire ps2_data_usb_tri_io;
  wire ps2_data_usb_tri_o;
  wire ps2_data_usb_tri_t;
  wire reset;
  wire sys_diff_clock_clk_n;
  wire sys_diff_clock_clk_p;
  wire usb_uart_rxd;
  wire usb_uart_txd;
  wire waiting_for_interrupt;

  main main_i
       (.VGA_B_0(VGA_B_0),
        .VGA_G_0(VGA_G_0),
        .VGA_HS_0(VGA_HS_0),
        .VGA_R_0(VGA_R_0),
        .VGA_VS_0(VGA_VS_0),
        .ddr3_sdram_addr(ddr3_sdram_addr),
        .ddr3_sdram_ba(ddr3_sdram_ba),
        .ddr3_sdram_cas_n(ddr3_sdram_cas_n),
        .ddr3_sdram_ck_n(ddr3_sdram_ck_n),
        .ddr3_sdram_ck_p(ddr3_sdram_ck_p),
        .ddr3_sdram_cke(ddr3_sdram_cke),
        .ddr3_sdram_cs_n(ddr3_sdram_cs_n),
        .ddr3_sdram_dm(ddr3_sdram_dm),
        .ddr3_sdram_dq(ddr3_sdram_dq),
        .ddr3_sdram_dqs_n(ddr3_sdram_dqs_n),
        .ddr3_sdram_dqs_p(ddr3_sdram_dqs_p),
        .ddr3_sdram_odt(ddr3_sdram_odt),
        .ddr3_sdram_ras_n(ddr3_sdram_ras_n),
        .ddr3_sdram_reset_n(ddr3_sdram_reset_n),
        .ddr3_sdram_we_n(ddr3_sdram_we_n),
        .fan_pwm(fan_pwm),
        .memory_card_spi_io0_i(memory_card_spi_io0_i),
        .memory_card_spi_io0_o(memory_card_spi_io0_o),
        .memory_card_spi_io0_t(memory_card_spi_io0_t),
        .memory_card_spi_io1_i(memory_card_spi_io1_i),
        .memory_card_spi_io1_o(memory_card_spi_io1_o),
        .memory_card_spi_io1_t(memory_card_spi_io1_t),
        .memory_card_spi_sck_i(memory_card_spi_sck_i),
        .memory_card_spi_sck_o(memory_card_spi_sck_o),
        .memory_card_spi_sck_t(memory_card_spi_sck_t),
        .memory_card_spi_ss_i(memory_card_spi_ss_i_0),
        .memory_card_spi_ss_o(memory_card_spi_ss_o_0),
        .memory_card_spi_ss_t(memory_card_spi_ss_t),
        .mode_m(mode_m),
        .mode_s(mode_s),
        .mode_u(mode_u),
        .ps2_clock_JA_tri_i(ps2_clock_JA_tri_i),
        .ps2_clock_JA_tri_o(ps2_clock_JA_tri_o),
        .ps2_clock_JA_tri_t(ps2_clock_JA_tri_t),
        .ps2_clock_JB_tri_i(ps2_clock_JB_tri_i),
        .ps2_clock_JB_tri_o(ps2_clock_JB_tri_o),
        .ps2_clock_JB_tri_t(ps2_clock_JB_tri_t),
        .ps2_clock_usb_tri_i(ps2_clock_usb_tri_i),
        .ps2_clock_usb_tri_o(ps2_clock_usb_tri_o),
        .ps2_clock_usb_tri_t(ps2_clock_usb_tri_t),
        .ps2_data_JA_tri_i(ps2_data_JA_tri_i),
        .ps2_data_JA_tri_o(ps2_data_JA_tri_o),
        .ps2_data_JA_tri_t(ps2_data_JA_tri_t),
        .ps2_data_JB_tri_i(ps2_data_JB_tri_i),
        .ps2_data_JB_tri_o(ps2_data_JB_tri_o),
        .ps2_data_JB_tri_t(ps2_data_JB_tri_t),
        .ps2_data_usb_tri_i(ps2_data_usb_tri_i),
        .ps2_data_usb_tri_o(ps2_data_usb_tri_o),
        .ps2_data_usb_tri_t(ps2_data_usb_tri_t),
        .reset(reset),
        .sys_diff_clock_clk_n(sys_diff_clock_clk_n),
        .sys_diff_clock_clk_p(sys_diff_clock_clk_p),
        .usb_uart_rxd(usb_uart_rxd),
        .usb_uart_txd(usb_uart_txd),
        .waiting_for_interrupt(waiting_for_interrupt));
  IOBUF memory_card_spi_io0_iobuf
       (.I(memory_card_spi_io0_o),
        .IO(memory_card_spi_io0_io),
        .O(memory_card_spi_io0_i),
        .T(memory_card_spi_io0_t));
  IOBUF memory_card_spi_io1_iobuf
       (.I(memory_card_spi_io1_o),
        .IO(memory_card_spi_io1_io),
        .O(memory_card_spi_io1_i),
        .T(memory_card_spi_io1_t));
  IOBUF memory_card_spi_sck_iobuf
       (.I(memory_card_spi_sck_o),
        .IO(memory_card_spi_sck_io),
        .O(memory_card_spi_sck_i),
        .T(memory_card_spi_sck_t));
  IOBUF memory_card_spi_ss_iobuf_0
       (.I(memory_card_spi_ss_o_0),
        .IO(memory_card_spi_ss_io[0]),
        .O(memory_card_spi_ss_i_0),
        .T(memory_card_spi_ss_t));
  IOBUF ps2_clock_JA_tri_iobuf
       (.I(ps2_clock_JA_tri_o),
        .IO(ps2_clock_JA_tri_io),
        .O(ps2_clock_JA_tri_i),
        .T(ps2_clock_JA_tri_t));
  IOBUF ps2_clock_JB_tri_iobuf
       (.I(ps2_clock_JB_tri_o),
        .IO(ps2_clock_JB_tri_io),
        .O(ps2_clock_JB_tri_i),
        .T(ps2_clock_JB_tri_t));
  IOBUF ps2_clock_usb_tri_iobuf
       (.I(ps2_clock_usb_tri_o),
        .IO(ps2_clock_usb_tri_io),
        .O(ps2_clock_usb_tri_i),
        .T(ps2_clock_usb_tri_t));
  IOBUF ps2_data_JA_tri_iobuf
       (.I(ps2_data_JA_tri_o),
        .IO(ps2_data_JA_tri_io),
        .O(ps2_data_JA_tri_i),
        .T(ps2_data_JA_tri_t));
  IOBUF ps2_data_JB_tri_iobuf
       (.I(ps2_data_JB_tri_o),
        .IO(ps2_data_JB_tri_io),
        .O(ps2_data_JB_tri_i),
        .T(ps2_data_JB_tri_t));
  IOBUF ps2_data_usb_tri_iobuf
       (.I(ps2_data_usb_tri_o),
        .IO(ps2_data_usb_tri_io),
        .O(ps2_data_usb_tri_i),
        .T(ps2_data_usb_tri_t));
endmodule
