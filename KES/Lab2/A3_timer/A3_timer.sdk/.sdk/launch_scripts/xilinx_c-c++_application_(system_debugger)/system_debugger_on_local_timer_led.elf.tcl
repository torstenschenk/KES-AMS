connect -url tcp:127.0.0.1:3121
source C:/Users/Misca/Documents/KES-AMS/KES/Lab2/A3_timer/A3_timer.sdk/design_1_wrapper_hw_platform_0/ps7_init.tcl
targets -set -filter {name =~"APU" && jtag_cable_name =~ "Digilent Zed 210248A49949"} -index 0
loadhw C:/Users/Misca/Documents/KES-AMS/KES/Lab2/A3_timer/A3_timer.sdk/design_1_wrapper_hw_platform_0/system.hdf
targets -set -filter {name =~"APU" && jtag_cable_name =~ "Digilent Zed 210248A49949"} -index 0
stop
ps7_init
ps7_post_config
targets -set -nocase -filter {name =~ "ARM*#0" && jtag_cable_name =~ "Digilent Zed 210248A49949"} -index 0
rst -processor
targets -set -nocase -filter {name =~ "ARM*#0" && jtag_cable_name =~ "Digilent Zed 210248A49949"} -index 0
dow C:/Users/Misca/Documents/KES-AMS/KES/Lab2/A3_timer/A3_timer.sdk/Timer_LED/Release/Timer_LED.elf
targets -set -nocase -filter {name =~ "ARM*#0" && jtag_cable_name =~ "Digilent Zed 210248A49949"} -index 0
con
