connect -url tcp:127.0.0.1:3121
source C:/Users/Misca/Beuth_Embedded_Design/Master/GITrepo/CES_AMS/KES/Lab1/A3/A1_A2.sdk/design_1_wrapper_hw_platform_0/ps7_init.tcl
targets -set -nocase -filter {name =~"APU*" && jtag_cable_name =~ "Digilent Zed 210248A49949"} -index 0
loadhw -hw C:/Users/Misca/Beuth_Embedded_Design/Master/GITrepo/CES_AMS/KES/Lab1/A3/A1_A2.sdk/design_1_wrapper_hw_platform_0/system.hdf -mem-ranges [list {0x40000000 0xbfffffff}]
configparams force-mem-access 1
targets -set -nocase -filter {name =~"APU*" && jtag_cable_name =~ "Digilent Zed 210248A49949"} -index 0
stop
ps7_init
ps7_post_config
targets -set -nocase -filter {name =~ "ARM*#0" && jtag_cable_name =~ "Digilent Zed 210248A49949"} -index 0
rst -processor
targets -set -nocase -filter {name =~ "ARM*#0" && jtag_cable_name =~ "Digilent Zed 210248A49949"} -index 0
dow C:/Users/Misca/Beuth_Embedded_Design/Master/GITrepo/CES_AMS/KES/Lab1/A3/A1_A2.sdk/led_n_buttons/Release/led_n_buttons.elf
configparams force-mem-access 0
targets -set -nocase -filter {name =~ "ARM*#0" && jtag_cable_name =~ "Digilent Zed 210248A49949"} -index 0
con
