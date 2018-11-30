library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity logic_demo is 
	port(
		SW0 : in STD_LOGIC;
		SW1 : in STD_LOGIC;
		LD4 : out STD_LOGIC;
		LD5 : out STD_LOGIC
	);
end logic_demo;

architecture impl of logic_demo is 
component simple_logic
	port(
		in_1 : in std_logic;
		in_2 : in std_logic;
		out_and : out std_logic;
		out_or : out std_logic
	);
end component;

begin 
my_logic : simple_logic
	port map
	(
		in_1 => SW0,
		in_2 => SW1,
		out_and => LD4,
		out_or => LD5
	);
end impl;