
�
@Attempting to get a license for feature '%s' and/or device '%s'
308*common2"
Implementation2default:default2
xc7z0202default:defaultZ17-347h px� 
�
0Got license for feature '%s' and/or device '%s'
310*common2"
Implementation2default:default2
xc7z0202default:defaultZ17-349h px� 
x
,Running DRC as a precondition to command %s
1349*	planAhead2#
write_bitstream2default:defaultZ12-1349h px� 
P
Running DRC with %s threads
24*drc2
22default:defaultZ23-27h px� 
�
Rule violation (%s) %s - %s
20*drc2
DPIP-12default:default2$
Input pipelining2default:default2�
�DSP design_1_i/lab3_a1_vga_v2_0/U0/lab3_a1_vga_v2_v1_0_S00_AXI_inst/p_0_in input design_1_i/lab3_a1_vga_v2_0/U0/lab3_a1_vga_v2_v1_0_S00_AXI_inst/p_0_in/A[29:0] is not pipelined. Pipelining DSP48 input will improve performance.2default:defaultZ23-20h px� 
�
Rule violation (%s) %s - %s
20*drc2
DPIP-12default:default2$
Input pipelining2default:default2�
�DSP design_1_i/lab3_a1_vga_v2_0/U0/lab3_a1_vga_v2_v1_0_S00_AXI_inst/p_0_in input design_1_i/lab3_a1_vga_v2_0/U0/lab3_a1_vga_v2_v1_0_S00_AXI_inst/p_0_in/C[47:0] is not pipelined. Pipelining DSP48 input will improve performance.2default:defaultZ23-20h px� 
�
Rule violation (%s) %s - %s
20*drc2
DPOP-12default:default2*
PREG Output pipelining2default:default2�
�DSP design_1_i/lab3_a1_vga_v2_0/U0/lab3_a1_vga_v2_v1_0_S00_AXI_inst/RAM_address0 output design_1_i/lab3_a1_vga_v2_0/U0/lab3_a1_vga_v2_v1_0_S00_AXI_inst/RAM_address0/P[47:0] is not pipelined (PREG=0). Pipelining the DSP48 output will improve performance and often saves power so it is suggested whenever possible to fully pipeline this function.  If this DSP48 function was inferred, it is suggested to describe an additional register stage after this function.  If the DSP48 was instantiated in the design, it is suggested to set the PREG attribute to 1.2default:defaultZ23-20h px� 
�
Rule violation (%s) %s - %s
20*drc2
DPOP-12default:default2*
PREG Output pipelining2default:default2�
�DSP design_1_i/lab3_a1_vga_v2_0/U0/lab3_a1_vga_v2_v1_0_S00_AXI_inst/p_0_in output design_1_i/lab3_a1_vga_v2_0/U0/lab3_a1_vga_v2_v1_0_S00_AXI_inst/p_0_in/P[47:0] is not pipelined (PREG=0). Pipelining the DSP48 output will improve performance and often saves power so it is suggested whenever possible to fully pipeline this function.  If this DSP48 function was inferred, it is suggested to describe an additional register stage after this function.  If the DSP48 was instantiated in the design, it is suggested to set the PREG attribute to 1.2default:defaultZ23-20h px� 
�
Rule violation (%s) %s - %s
20*drc2
DPOP-22default:default2*
MREG Output pipelining2default:default2�
�DSP design_1_i/lab3_a1_vga_v2_0/U0/lab3_a1_vga_v2_v1_0_S00_AXI_inst/RAM_address0 multiplier stage design_1_i/lab3_a1_vga_v2_0/U0/lab3_a1_vga_v2_v1_0_S00_AXI_inst/RAM_address0/P[47:0] is not pipelined (MREG=0). Pipelining the multiplier function will improve performance and will save significant power so it is suggested whenever possible to fully pipeline this function.  If this multiplier was inferred, it is suggested to describe an additional register stage after this function.  If there is no registered adder/accumulator following the multiply function, two pipeline stages are suggested to allow both the MREG and PREG registers to be used.  If the DSP48 was instantiated in the design, it is suggested to set both the MREG and PREG attributes to 1 when performing multiply functions.2default:defaultZ23-20h px� 
�
Rule violation (%s) %s - %s
20*drc2
DPOP-22default:default2*
MREG Output pipelining2default:default2�
�DSP design_1_i/lab3_a1_vga_v2_0/U0/lab3_a1_vga_v2_v1_0_S00_AXI_inst/p_0_in multiplier stage design_1_i/lab3_a1_vga_v2_0/U0/lab3_a1_vga_v2_v1_0_S00_AXI_inst/p_0_in/P[47:0] is not pipelined (MREG=0). Pipelining the multiplier function will improve performance and will save significant power so it is suggested whenever possible to fully pipeline this function.  If this multiplier was inferred, it is suggested to describe an additional register stage after this function.  If there is no registered adder/accumulator following the multiply function, two pipeline stages are suggested to allow both the MREG and PREG registers to be used.  If the DSP48 was instantiated in the design, it is suggested to set both the MREG and PREG attributes to 1 when performing multiply functions.2default:defaultZ23-20h px� 
f
DRC finished with %s
1905*	planAhead2(
0 Errors, 6 Warnings2default:defaultZ12-3199h px� 
i
BPlease refer to the DRC report (report_drc) for more information.
1906*	planAheadZ12-3200h px� 
i
)Running write_bitstream with %s threads.
1750*designutils2
22default:defaultZ20-2272h px� 
?
Loading data files...
1271*designutilsZ12-1165h px� 
>
Loading site data...
1273*designutilsZ12-1167h px� 
?
Loading route data...
1272*designutilsZ12-1166h px� 
?
Processing options...
1362*designutilsZ12-1514h px� 
<
Creating bitmap...
1249*designutilsZ12-1141h px� 
7
Creating bitstream...
7*	bitstreamZ40-7h px� 
g
Writing bitstream %s...
11*	bitstream2*
./design_1_wrapper.bit2default:defaultZ40-11h px� 
F
Bitgen Completed Successfully.
1606*	planAheadZ12-1842h px� 
�
�WebTalk data collection is mandatory when using a WebPACK part without a full Vivado license. To see the specific WebTalk data collected for your design, open the usage_statistics_webtalk.html or usage_statistics_webtalk.xml file in the implementation directory.
120*projectZ1-120h px� 
Z
Releasing license: %s
83*common2"
Implementation2default:defaultZ17-83h px� 
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2%
write_bitstream: 2default:default2
00:01:242default:default2
00:01:462default:default2
1512.4692default:default2
214.4062default:defaultZ17-268h px� 
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2"
write_sysdef: 2default:default2
00:00:002default:default2
00:00:092default:default2
1512.4692default:default2
0.0002default:defaultZ17-268h px� 


End Record