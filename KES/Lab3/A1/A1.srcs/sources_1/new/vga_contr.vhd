----------------------------------------------------------------------------------
-- Group name: Index 
-- Engineer: 
-- 
-- Create Date: 25.11.2018 23:47:44
-- Design Name: 
-- Module Name: vga_contr - Behavioral
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

entity vga_contr is
    Port ( clk : in STD_LOGIC;
           ext_datacontrol : in STD_LOGIC;
           r : out std_logic_vector (3 downto 0);
           g : out std_logic_vector (3 downto 0);
           b : out std_logic_vector (3 downto 0);
           hsync : out std_logic;
           vsync : out std_logic);
end vga_contr;

architecture Behavioral of vga_contr is





begin


end Behavioral;
