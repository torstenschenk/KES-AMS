# 
# Synthesis run script generated by Vivado
# 

set_msg_config -id {HDL 9-1061} -limit 100000
set_msg_config -id {HDL 9-1654} -limit 100000
set_msg_config -id {Synth 8-256} -limit 10000
set_msg_config -id {Synth 8-638} -limit 10000
create_project -in_memory -part xc7z020clg484-1

set_param project.singleFileAddWarning.threshold 0
set_param project.compositeFile.enableAutoGeneration 0
set_param synth.vivado.isSynthRun true
set_property webtalk.parent_dir C:/Users/omid/Documents/GitHub/KES-AMS/KES/Lab3/A1_testbench/A1_testbench.cache/wt [current_project]
set_property parent.project_path C:/Users/omid/Documents/GitHub/KES-AMS/KES/Lab3/A1_testbench/A1_testbench.xpr [current_project]
set_property default_lib xil_defaultlib [current_project]
set_property target_language Verilog [current_project]
set_property board_part em.avnet.com:zed:part0:1.3 [current_project]
read_vhdl -library xil_defaultlib {
  C:/Users/omid/Documents/GitHub/KES-AMS/KES/Lab3/A1_testbench/simple_logic.vhd
  C:/Users/omid/Documents/GitHub/KES-AMS/KES/Lab3/A1_testbench/logic_demo.vhd
}
foreach dcp [get_files -quiet -all *.dcp] {
  set_property used_in_implementation false $dcp
}

synth_design -top logic_demo -part xc7z020clg484-1


write_checkpoint -force -noxdef logic_demo.dcp

catch { report_utilization -file logic_demo_utilization_synth.rpt -pb logic_demo_utilization_synth.pb }
