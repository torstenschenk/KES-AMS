library ieee;
use ieee.std_logic_1164.all;

entity logic_demo_tb is 
end logic_demo_tb;

architecture sim of logic_demo_tb is 
	component logic_demo
		port 
		(
			SW0 : in std_logic;
			SW1 : in std_logic;
			LD4 : out std_logic;
			LD5 : out std_logic
		);
	end component;
	
	signal SW0_sim : std_logic;
	signal SW1_sim : std_logic;
	signal LD4_sim : std_logic;
	signal LD5_sim : std_logic;
	
begin
	dut : logic_demo
	port map
	(
		SW0 => SW0_sim,
		SW1 => SW1_sim,
		LD4 => LD4_sim,
		LD5 => LD5_sim
	);

	process
	begin
		SW0_sim <= '0';
		SW1_sim <= '0';
		
		wait for 50 ns;
		SW0_sim <= '1';
		SW1_sim <= '0';

		wait for 50 ns;
		SW0_sim <= '0';
		SW1_sim <= '1';		
		
		wait for 50 ns;
		SW0_sim <= '1';
		SW1_sim <= '1';
		
		wait;
	end process;
end sim;