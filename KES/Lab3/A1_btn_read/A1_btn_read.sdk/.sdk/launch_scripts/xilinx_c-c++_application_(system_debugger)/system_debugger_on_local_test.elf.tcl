connect -url tcp:127.0.0.1:3121
source C:/Users/omid/Documents/GitHub/KES-AMS/KES/Lab3/A1_btn_read/A1_btn_read.sdk/design_1_wrapper_hw_platform_0/ps7_init.tcl
targets -set -filter {name =~"APU" && jtag_cable_name =~ "Digilent Zed 210248A49949"} -index 0
loadhw C:/Users/omid/Documents/GitHub/KES-AMS/KES/Lab3/A1_btn_read/A1_btn_read.sdk/design_1_wrapper_hw_platform_0/system.hdf
targets -set -filter {name =~"APU" && jtag_cable_name =~ "Digilent Zed 210248A49949"} -index 0
stop
ps7_init
ps7_post_config
targets -set -nocase -filter {name =~ "ARM*#0" && jtag_cable_name =~ "Digilent Zed 210248A49949"} -index 0
rst -processor
targets -set -nocase -filter {name =~ "ARM*#0" && jtag_cable_name =~ "Digilent Zed 210248A49949"} -index 0
dow C:/Users/omid/Documents/GitHub/KES-AMS/KES/Lab3/A1_btn_read/A1_btn_read.sdk/test/Release/test.elf
targets -set -nocase -filter {name =~ "ARM*#0" && jtag_cable_name =~ "Digilent Zed 210248A49949"} -index 0
con
