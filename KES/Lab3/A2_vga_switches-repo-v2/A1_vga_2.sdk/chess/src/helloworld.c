/******************************************************************************
*
* Copyright (C) 2009 - 2014 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
******************************************************************************/

/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */
#include  <stdio.h>
#include "platform.h"
//  Include  Files
#include "xparameters.h"
#include "xgpio.h"
#include "xstatus.h"
#include "xil_printf.h"

//  Definitions
//#define  LEDS_DEVICE_ID  XPAR_AXI_GPIO_0_DEVICE_ID
#define  SW_DEVICE_ID  XPAR_AXI_GPIO_0_DEVICE_ID

#define  LED_DELAY  10000000
#define  LED_CHANNEL 1
#define  printf  xil_printf

XGpio  SWInst; // LEDInst
static int sw_value ;

/*
int  LEDOutputExample( VOID ) {
	volatile  int  Delay;

	// loop  forever  blinking  the  LED.
	while( 42 ) {
		// read buttons
		btn_value = XGpio_DiscreteRead (&BTNInst , 1);
		//  write  button value  to LEDs
		XGpio_DiscreteWrite( &LEDInst , LED_CHANNEL , btn_value );

		for ( Delay = 0;  Delay  < LED_DELAY; Delay++ );
	}
	return  XST_SUCCESS;
}
*/

int SWReadExample () {
	volatile  int  Delay;

	// loop  forever  blinking  the  LED.
	while( 1 ) {
		// read buttons
		sw_value = XGpio_DiscreteRead (&SWInst , 1);
		xil_printf ("Switch: %x\n\r",sw_value);
		sleep(1);
		//for ( Delay = 0;  Delay  < LED_DELAY; Delay++ );
	}
	return  XST_SUCCESS;
}

int  main()
{
	init_platform ();

	int status ;
	// ----------------------------------------------------
	// INITIALIZE THE PERIPHERALS & SET DIRECTIONS OF GPIO
	// ----------------------------------------------------
	// Initialise LEDs
	//status = XGpio_Initialize (&LEDInst , LEDS_DEVICE_ID );
	//if ( status != XST_SUCCESS )
	//	return XST_FAILURE ;

	// Initialise Push Buttons
	status = XGpio_Initialize (&SWInst , SW_DEVICE_ID );
	if ( status != XST_SUCCESS )
		return XST_FAILURE ;

	// Set LEDs direction to outputs
	//XGpio_SetDataDirection (&LEDInst,1,0x00);
	// Set all switches direction to inputs
	XGpio_SetDataDirection (&SWInst,1,0xFF);

	print("Hello world\n\r");

	SWReadExample ();

	//LEDOutputExample ();
	cleanup_platform ();
	return  0;
}

// how to write to ram
#if 0
   -- Write RAM-Data from AXI-Stream
   -- Get  from AXI-Stream slv_reg0, with the information:
   -- X Position
   -- Y Position
   -- Colour
   -- and write it into the RAM.
   process(S_AXI_ACLK)
       variable x_position   : integer;
       variable y_position   : integer;
       variable RAM_position : integer;

       begin
           if rising_edge(S_AXI_ACLK) then
                   x_position := to_integer(ieee.NUMERIC_STD.unsigned(  slv_reg0(20 downto 11) ) );
                   y_position := to_integer(ieee.NUMERIC_STD.unsigned(  slv_reg0(10 downto 2 ) ) );

                   RAM_position :=  (x_position + (y_position * 640 ));

                   RAM(RAM_position) <= slv_reg0(1 downto 0);
           end if;
   end process;
#endif
