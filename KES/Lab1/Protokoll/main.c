#INCLUDE  <stdio.h>
#INCLUDE "platform.h"
//  Include  Files
#INCLUDE "xparameters.h"
#INCLUDE "xgpio.h"
#INCLUDE "xstatus.h"
#INCLUDE "xil_printf.h"
//  Definitions
#DEFINE  GPIO_DEVICE_ID  XPAR_AXI_GPIO_0_DEVICE_ID
#DEFINE  LED 0x55//0xC3
#DEFINE  LED_DELAY  10000000
#DEFINE  LED_CHANNEL 1
#DEFINE  printf  xil_printf

XGpio  Gpio;
// void  print( char *str );

int  LEDOutputExample( VOID ) {
	VOLATILE  int  Delay;
	int  Status;
	int  led = LED;
	// GPIO  driver  initialization
	Status = XGpio_Initialize( &Gpio , GPIO_DEVICE_ID );
	if ( Status  !=  XST_SUCCESS ) {
		return  XST_FAILURE;
	}
	// Set  the  direction  for  the  LEDs to  output
	XGpio_SetDataDirection( &Gpio , LED_CHANNEL , 0x00 );
	// loop  forever  blinking  the  LED.
	while( 42 ) {
		//  write  output  to LEDs
		XGpio_DiscreteWrite( &Gpio , LED_CHANNEL , led );
		// flip  LEDs
		led = ~led;
		for ( Delay = 0;  Delay  < LED_DELAY; Delay++ );
	}
	return  XST_SUCCESS;
}

int  main()
{
	init_platform ();
	print("Hello  meine  LED\n\r");
	LEDOutputExample ();
	cleanup_platform ();
	return  0;
}