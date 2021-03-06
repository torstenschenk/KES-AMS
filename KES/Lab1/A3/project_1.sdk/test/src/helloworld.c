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
#define  LEDS_DEVICE_ID  XPAR_AXI_GPIO_0_DEVICE_ID
#define  BTNS_DEVICE_ID  XPAR_AXI_GPIO_1_DEVICE_ID

#define  LED_DELAY  1000000
#define  LED_CHANNEL 1
#define  printf  xil_printf

XGpio  Gpio;
XGpio LEDInst , BTNInst ;
static int btn_value ;

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

int  main()
{
	init_platform ();

	int status ;
	// ----------------------------------------------------
	// INITIALIZE THE PERIPHERALS & SET DIRECTIONS OF GPIO
	// ----------------------------------------------------
	// Initialise LEDs
	status = XGpio_Initialize (&LEDInst , LEDS_DEVICE_ID );
	if ( status != XST_SUCCESS )
		return XST_FAILURE ;

	// Initialise Push Buttons
	status = XGpio_Initialize (&BTNInst , BTNS_DEVICE_ID );
	if ( status != XST_SUCCESS )
		return XST_FAILURE ;

	// Set LEDs direction to outputs
	XGpio_SetDataDirection (&LEDInst,1,0x00);
	// Set all buttons direction to inputs
	XGpio_SetDataDirection (&BTNInst,1,0xFF);

	print("Hello  meine  LED\n\r");

	LEDOutputExample ();
	cleanup_platform ();
	return  0;
}
