----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 09.12.2018 12:48:58
-- Design Name: 
-- Module Name: vga_sync - impl
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


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity vga_sync is
    Port ( vga_clk   : in  std_logic;
           reset     : in  std_logic;
           pixel_x   : out std_logic_vector (9 downto 0);
           pixel_y   : out std_logic_vector (9 downto 0);
           video_on  : out std_logic;
           hSync     : out std_logic;
           vSync     : out std_logic);
end vga_sync;

architecture impl of vga_sync is
    CONSTANT h_active_sig    : integer := 799; -- full horizontal
    CONSTANT h_active_video  : integer := 639; -- horizontal active video
    -- CONSTANT h_front_porch   : integer := 16; -- horizontal front porch
    -- CONSTANT h_retrace       : integer := 96; -- horizontal retrace / sync pulse length
    -- CONSTANT h_back_porch    : integer := 48; -- horizontal back porch
    CONSTANT h_retrace       : integer := 655; -- h_active_video 639 + 16
    CONSTANT h_back_porch    : integer := 752; -- h_active_video 639 + 16 + 96 ( +1 for < check)
    
    CONSTANT v_active_sig    : integer := 524; -- full vertical
    CONSTANT v_active_video  : integer := 479; -- vertical active video
    -- CONSTANT v_front_porch   : integer := 10; -- vertical front porch
    -- CONSTANT v_retrace       : integer := 2; -- vertical retrace / sync pulse length
    -- CONSTANT v_back_porch    : integer := 33; -- vertical back porch
    CONSTANT v_retrace       : integer := 489; -- v_active_video 479 + 10
    CONSTANT v_back_porch    : integer := 492; -- v_active_video 479 + 10 + 2 ( +1 for < check)
   
    -- CONSTANT h_active_sig    : integer := 29; -- full horizontal
    -- CONSTANT h_active_video  : integer := 19; -- horizontal active video
    -- -- CONSTANT h_front_porch   : integer := 16; -- horizontal front porch
    -- -- CONSTANT h_retrace       : integer := 96; -- horizontal retrace / sync pulse length
    -- -- CONSTANT h_back_porch    : integer := 48; -- horizontal back porch
    -- CONSTANT h_retrace       : integer := 24; -- h_active_video 639 + 16
    -- CONSTANT h_back_porch    : integer := 27; -- h_active_video 639 + 16 + 96 ( +1 for < check)
    -- 
    -- CONSTANT v_active_sig    : integer := 19; -- full vertical
    -- CONSTANT v_active_video  : integer := 9; -- vertical active video
    -- -- CONSTANT v_front_porch   : integer := 10; -- vertical front porch
    -- -- CONSTANT v_retrace       : integer := 2; -- vertical retrace / sync pulse length
    -- -- CONSTANT v_back_porch    : integer := 33; -- vertical back porch
    -- CONSTANT v_retrace       : integer := 13; -- v_active_video 479 + 10
    -- CONSTANT v_back_porch    : integer := 16; -- v_active_video 479 + 10 + 2 ( +1 for < check)

   signal px_reg   : unsigned(9 downto 0) := (others => '0'); -- x full 0 to 799
   signal px_next  : unsigned(9 downto 0);
   signal py_reg   : unsigned(9 downto 0) := (others => '0'); -- y full 0 to 524
   signal py_next  : unsigned(9 downto 0);
   
   signal hsync_reg  : std_logic := '0';
   signal hsync_next : std_logic;
   
   signal vsync_reg  : std_logic := '0';
   signal vsync_next : std_logic;
   
   signal video_on_reg  : std_logic := '0';
   signal video_on_next : std_logic;
   
begin

    vga_count : process (vga_clk)
    begin
        if (rising_edge(vga_clk)) then
            if (reset = '1') then
                px_reg    <= (others => '0');
                py_reg    <= (others => '0');
               
                hsync_reg <= '1';
                vsync_reg <= '1';
            else
            
                -- full size counter including porches
                px_reg  <= px_next;
                py_reg  <= py_next;
                
                -- sync signalss
                hsync_reg <= hsync_next;
                vsync_reg <= vsync_next;
                
                -- video active - inactive
                video_on_reg <= video_on_next;
      
            end if;
        end if;
    end process vga_count;
    
    px_next <= (others => '0') when (px_reg = h_active_sig) else px_reg + 1; -- modulo 799
    
    py_next <= (others => '0') when ((px_reg = h_active_sig) and (py_reg = v_active_sig)) else -- reset on h=799 and v=524 (max rows)
                py_reg + 1     when (px_reg = h_active_sig) else py_reg; -- add 1 if row finished 
    
    video_on_next <= '0' when ((px_reg > h_active_video) or (py_reg > v_active_video)) else '1'; -- modulo 639 in col, 479 in row
    
    hsync_next <= '0' when ((px_reg > h_retrace) and (px_reg < h_back_porch)) else '1'; -- '0' from 656 to 751 (96 px)
    vsync_next <= '0' when ((py_reg > v_retrace) and (py_reg < v_back_porch)) else '1'; -- '0' from 490 to 491 (2 px) 
    
    -- set output from registers
    pixel_x <= std_logic_vector(px_reg);
    pixel_y <= std_logic_vector(py_reg);
    video_on <= video_on_reg;
    hSync    <= hsync_reg;
    vSync    <= vsync_reg;

end impl;
