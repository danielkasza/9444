set_property BITSTREAM.CONFIG.CONFIGRATE 33 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]

## VGA Connector
set_property -dict {PACKAGE_PIN AH20 IOSTANDARD LVCMOS33} [get_ports {VGA_B_0[0]}]
set_property -dict {PACKAGE_PIN AG20 IOSTANDARD LVCMOS33} [get_ports {VGA_B_0[1]}]
set_property -dict {PACKAGE_PIN AF21 IOSTANDARD LVCMOS33} [get_ports {VGA_B_0[2]}]
set_property -dict {PACKAGE_PIN AK20 IOSTANDARD LVCMOS33} [get_ports {VGA_B_0[3]}]
set_property -dict {PACKAGE_PIN AG22 IOSTANDARD LVCMOS33} [get_ports {VGA_B_0[4]}]

set_property -dict {PACKAGE_PIN AJ23 IOSTANDARD LVCMOS33} [get_ports {VGA_G_0[0]}]
set_property -dict {PACKAGE_PIN AJ22 IOSTANDARD LVCMOS33} [get_ports {VGA_G_0[1]}]
set_property -dict {PACKAGE_PIN AH22 IOSTANDARD LVCMOS33} [get_ports {VGA_G_0[2]}]
set_property -dict {PACKAGE_PIN AK21 IOSTANDARD LVCMOS33} [get_ports {VGA_G_0[3]}]
set_property -dict {PACKAGE_PIN AJ21 IOSTANDARD LVCMOS33} [get_ports {VGA_G_0[4]}]
set_property -dict {PACKAGE_PIN AK23 IOSTANDARD LVCMOS33} [get_ports {VGA_G_0[5]}]

set_property -dict {PACKAGE_PIN AK25 IOSTANDARD LVCMOS33} [get_ports {VGA_R_0[0]}]
set_property -dict {PACKAGE_PIN AG25 IOSTANDARD LVCMOS33} [get_ports {VGA_R_0[1]}]
set_property -dict {PACKAGE_PIN AH25 IOSTANDARD LVCMOS33} [get_ports {VGA_R_0[2]}]
set_property -dict {PACKAGE_PIN AK24 IOSTANDARD LVCMOS33} [get_ports {VGA_R_0[3]}]
set_property -dict {PACKAGE_PIN AJ24 IOSTANDARD LVCMOS33} [get_ports {VGA_R_0[4]}]

set_property -dict {PACKAGE_PIN AF20 IOSTANDARD LVCMOS33} [get_ports VGA_HS_0]
set_property -dict {PACKAGE_PIN AG23 IOSTANDARD LVCMOS33} [get_ports VGA_VS_0]

## LEDs
set_property -dict {PACKAGE_PIN T28 IOSTANDARD LVCMOS33} [get_ports mode_m]
set_property -dict {PACKAGE_PIN V19 IOSTANDARD LVCMOS33} [get_ports mode_s]
set_property -dict {PACKAGE_PIN U30 IOSTANDARD LVCMOS33} [get_ports mode_u]
set_property -dict {PACKAGE_PIN W23 IOSTANDARD LVCMOS33} [get_ports waiting_for_interrupt]

## PMOD PS2 (JA)
set_property -dict {PACKAGE_PIN T22 IOSTANDARD LVCMOS33} [get_ports ps2_data_JA_tri_io]
set_property -dict {PACKAGE_PIN T20 IOSTANDARD LVCMOS33} [get_ports ps2_clock_JA_tri_io]

## PMOD PS2 (JB)
set_property -dict {PACKAGE_PIN T25 IOSTANDARD LVCMOS33} [get_ports ps2_data_JB_tri_io]
set_property -dict {PACKAGE_PIN U22 IOSTANDARD LVCMOS33} [get_ports ps2_clock_JB_tri_io]

## USB to PS2
set_property -dict {PACKAGE_PIN AE20 IOSTANDARD LVCMOS33} [get_ports ps2_data_usb_tri_io]
set_property -dict {PACKAGE_PIN AD23 IOSTANDARD LVCMOS33} [get_ports ps2_clock_usb_tri_io]

## Fan control
set_property -dict {PACKAGE_PIN W19 IOSTANDARD LVCMOS33} [get_ports {fan_pwm[0]}]

## Memory card
set_property -dict {PACKAGE_PIN R29 IOSTANDARD LVCMOS33} [get_ports memory_card_spi_io0_io]
set_property -dict {PACKAGE_PIN R26 IOSTANDARD LVCMOS33} [get_ports memory_card_spi_io1_io]
set_property -dict {PACKAGE_PIN R28 IOSTANDARD LVCMOS33} [get_ports memory_card_spi_sck_io]
set_property -dict {PACKAGE_PIN T30 IOSTANDARD LVCMOS33} [get_ports {memory_card_spi_ss_io[0]}]
