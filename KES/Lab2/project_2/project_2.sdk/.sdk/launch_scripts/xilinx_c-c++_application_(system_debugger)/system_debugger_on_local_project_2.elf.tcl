connect -url tcp:127.0.0.1:3121
source H:/CES/project_2/project_2.sdk/design_2_wrapper_hw_platform_0/ps7_init.tcl
targets -set -filter {name =~"APU" && jtag_cable_name =~ "Digilent Zed 210248492445"} -index 0
loadhw H:/CES/project_2/project_2.sdk/design_2_wrapper_hw_platform_0/system.hdf
targets -set -filter {name =~"APU" && jtag_cable_name =~ "Digilent Zed 210248492445"} -index 0
stop
ps7_init
ps7_post_config
targets -set -nocase -filter {name =~ "ARM*#0" && jtag_cable_name =~ "Digilent Zed 210248492445"} -index 0
rst -processor
targets -set -nocase -filter {name =~ "ARM*#0" && jtag_cable_name =~ "Digilent Zed 210248492445"} -index 0
dow H:/CES/project_2/project_2.sdk/project_2/Debug/project_2.elf
targets -set -nocase -filter {name =~ "ARM*#0" && jtag_cable_name =~ "Digilent Zed 210248492445"} -index 0
con
