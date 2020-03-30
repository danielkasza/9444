# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  ipgui::add_param $IPINST -name "Component_Name"
  #Adding Page
  set Page_0 [ipgui::add_page $IPINST -name "Page 0"]
  ipgui::add_param $IPINST -name "fb_base" -parent ${Page_0}
  ipgui::add_param $IPINST -name "pixel_clock_divider" -parent ${Page_0}


}

proc update_PARAM_VALUE.fb_base { PARAM_VALUE.fb_base } {
	# Procedure called to update fb_base when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.fb_base { PARAM_VALUE.fb_base } {
	# Procedure called to validate fb_base
	return true
}

proc update_PARAM_VALUE.pixel_clock_divider { PARAM_VALUE.pixel_clock_divider } {
	# Procedure called to update pixel_clock_divider when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.pixel_clock_divider { PARAM_VALUE.pixel_clock_divider } {
	# Procedure called to validate pixel_clock_divider
	return true
}


proc update_MODELPARAM_VALUE.fb_base { MODELPARAM_VALUE.fb_base PARAM_VALUE.fb_base } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.fb_base}] ${MODELPARAM_VALUE.fb_base}
}

proc update_MODELPARAM_VALUE.pixel_clock_divider { MODELPARAM_VALUE.pixel_clock_divider PARAM_VALUE.pixel_clock_divider } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.pixel_clock_divider}] ${MODELPARAM_VALUE.pixel_clock_divider}
}

