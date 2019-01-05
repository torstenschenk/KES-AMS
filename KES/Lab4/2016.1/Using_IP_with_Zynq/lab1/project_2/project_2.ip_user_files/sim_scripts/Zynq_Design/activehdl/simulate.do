onbreak {quit -force}
onerror {quit -force}

asim -t 1ps +access +r +m+Zynq_Design -pli "C:/Xilinx/Vivado/2016.1/lib/win64.o/libxil_vsim.dll" -L unisims_ver -L unimacro_ver -L secureip -L processing_system7_bfm_v2_0_5 -L xil_defaultlib -L lib_cdc_v1_0_2 -L proc_sys_reset_v5_0_8 -L generic_baseblocks_v2_1_0 -L fifo_generator_v13_1_0 -L axi_data_fifo_v2_1_7 -L axi_infrastructure_v1_1_0 -L axi_register_slice_v2_1_8 -L axi_protocol_converter_v2_1_8 -O5 xil_defaultlib.Zynq_Design xil_defaultlib.glbl

do {wave.do}

view wave
view structure
view signals

do {Zynq_Design.udo}

run -all

endsim

quit -force
