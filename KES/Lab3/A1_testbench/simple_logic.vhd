library ieee;
use ieee.std_logic_1164.all;

entity simple_logic is 
	port
	(
		in_1 : in std_logic;
		in_2 : in std_logic;
		out_and : out std_logic;
		out_or : out std_logic
	);end simple_logic;
	
	architecture impl of simple_logic is 
	
	begin
		out_and <= in_1 and in_2;
		out_or <= in_1 or in_2;
	end impl;
