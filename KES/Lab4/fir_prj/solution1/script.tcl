############################################################
## This file is generated automatically by Vivado HLS.
## Please DO NOT edit it.
## Copyright (C) 1986-2016 Xilinx, Inc. All Rights Reserved.
############################################################
open_project fir_prj
set_top fir
add_files 2016.1/Introduction/lab2/fir.c
add_files -tb 2016.1/Introduction/lab2/out.gold.dat
add_files -tb 2016.1/Introduction/lab2/fir_test.c
open_solution "solution1"
set_part {xc7z020clg484-1}
create_clock -period 10 -name default
#source "./fir_prj/solution1/directives.tcl"
csim_design
csynth_design
cosim_design -rtl vhdl -tool xsim
export_design -format ip_catalog
