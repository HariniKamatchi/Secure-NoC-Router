######################################################################
# ZedBoard - Secure NoC Router Hardware Constraints
# Device: XC7Z020-CLG484-1
######################################################################

# 1. 100 MHz Single-Ended Clock (Pin Y9 - Bank 13)
set_property PACKAGE_PIN Y9 [get_ports clk_100]
set_property IOSTANDARD LVCMOS33 [get_ports clk_100]
create_clock -name clk_100 -period 10.000 -waveform {0.000 5.000} [get_ports clk_100]

# 2. Onboard User LEDs (LD0 to LD7 - Bank 33)
# LD0 -> T22 : Clock Heartbeat (~0.75 Hz)
set_property PACKAGE_PIN T22 [get_ports {led[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[0]}]

# LD1 -> T21 : Port 0 (North) Active
set_property PACKAGE_PIN T21 [get_ports {led[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[1]}]

# LD2 -> U22 : Port 1 (South) Active
set_property PACKAGE_PIN U22 [get_ports {led[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[2]}]

# LD3 -> U21 : Port 2 (East) Active
set_property PACKAGE_PIN U21 [get_ports {led[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[3]}]

# LD4 -> V22 : Port 3 (West) Active
set_property PACKAGE_PIN V22 [get_ports {led[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[4]}]

# LD5 -> W22 : Security ACL Drop Flag
set_property PACKAGE_PIN W22 [get_ports {led[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[5]}]

# LD6 -> U19 : Parity Error Detected
set_property PACKAGE_PIN U19 [get_ports {led[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[6]}]

# LD7 -> U14 : Test PASS Indicator
set_property PACKAGE_PIN U14 [get_ports {led[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[7]}]

# 3. Timing Exceptions
set_false_path -to [get_ports {led[*]}]