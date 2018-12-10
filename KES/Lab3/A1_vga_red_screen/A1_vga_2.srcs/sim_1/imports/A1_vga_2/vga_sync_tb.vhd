----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 09.12.2018 13:00:30
-- Design Name: 
-- Module Name: vga_sync_tb - impl
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

entity vga_sync_tb is
    --  Port ( );
end vga_sync_tb;

architecture impl of vga_sync_tb is

    -- CLOCK
    --  component design_1_clk_wiz_0_0 
    --  Port ( -- sys-clk Clock in ports
    --         clk_in1 : in STD_LOGIC;
    --         -- vga-clk Clock out ports
    --         clk_out1 : out STD_LOGIC);
    --  end component;

    -- VGA-SYNC
    component vga_sync
    Port ( vga_clk  : in std_logic;
           reset    : in std_logic;
           pixel_x  : out std_logic_vector (9 downto 0);
           pixel_y  : out std_logic_vector (9 downto 0);
           video_on : out std_logic;
           hSync    : out std_logic;
           vSync    : out std_logic);
    end component;
    
    signal clk_sim      : std_logic;
    signal reset_sig    : std_logic := '1';
    signal pixel_x_sig  : std_logic_vector (9 downto 0);
    signal pixel_y_sig  : std_logic_vector (9 downto 0);
    signal video_on_sig : std_logic;
    signal hsynct_sig   : std_logic;
    signal vsynct_sig   : std_logic;

    constant clk_period : time := 39ns;
begin
    -- clk_wiz_comp: design_1_clk_wiz_0_0 port map (
    --     clk_in1  => sys_clk,
    --     clk_out1 => clk_sim );

    vga_sync_comp: vga_sync port map (
        -- in
        vga_clk	 => clk_sim,
        reset    => reset_sig,
        -- out
        pixel_x  => pixel_x_sig,
        pixel_y  => pixel_y_sig,
        video_on => video_on_sig,
        hSync    => hsynct_sig,
        vSync    => vsynct_sig );

   clk_gen : process
   begin
       clk_sim <= '1';
       wait for clk_period/2;
       clk_sim <= '0';
       wait for clk_period/2;
   end process clk_gen;

    sim_proc : process
    begin
        wait for 90ns;
        reset_sig <= '0';
        wait;
    end process sim_proc;

end impl;
