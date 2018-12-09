----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 02.12.2018 14:42:55
-- Design Name: 
-- Module Name: vga_tb - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity vga_tb is
 --- emtpy testbench
end vga_tb;

architecture Behavioral of vga_tb is

 component vga_clk_wiz_0_0 is
 port (
   resetn   : in STD_LOGIC;
   clk_in1  : in STD_LOGIC;
   
   clk_out1 : out STD_LOGIC;
   locked   : out STD_LOGIC
 );
 
 -- signal reset_rtl_sim : std_logic;
 signal sys_clock_sim : std_logic := '0';
 signal clk_out_sim   : std_logic;
 signal locked_sim    : std_logic;
 
begin
    vga_i: component vga_clk_wiz_0_0
     port map (
        resetn   => '1',
        clk_in1  => sys_clock_sim,
        clk_out1 => clk_out_sim,
        locked   => locked_sim
    );

    


end Behavioral;
