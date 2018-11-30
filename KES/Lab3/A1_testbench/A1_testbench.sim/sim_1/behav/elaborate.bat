@echo off
set xv_path=C:\\Xilinx\\Vivado\\2016.2\\bin
call %xv_path%/xelab  -wto ad2212a73a0a4318bf369f9ed87a11b3 -m64 --debug typical --relax --mt 2 -L xil_defaultlib -L secureip --snapshot logic_demo_tb_behav xil_defaultlib.logic_demo_tb -log elaborate.log
if "%errorlevel%"=="0" goto SUCCESS
if "%errorlevel%"=="1" goto END
:END
exit 1
:SUCCESS
exit 0
