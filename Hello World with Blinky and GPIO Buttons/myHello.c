//*****************************************************************************
//
// hello.c - Simple hello world example.
//
// Copyright (c) 2011-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the DK-TM4C123G Firmware Package.
//
//*****************************************************************************

/******************************************************************************
* Name: Aidan Rutherford
* The purpose of this program is to print a personal in a box on the display
* and print my name in the middle of the display. The buttons were also 
* given the functionality of printing the direction of the associated button
* on the display
******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "grlib/grlib.h"
#include "drivers/cfal96x64x16.h"
#include "utils/uartstdio.h"
#include "driverlib/gpio.h"
//
// Defines the duration of the duty cycle
//
#define DUTY 2000000
//
// The error routine that is called if the driver library encounters an error.
//
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

int main(void)
{
	/***************************************************************************/
	//
	// The configuration code for the OLDED from the hello.c example
	// in the TivaWare PDL. I twas used to draw to the OLED
	//
    tContext sContext;
    tRectangle sRect;
    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPULazyStackingEnable();
    //
    // Set the clocking to run directly from the crystal.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);
    //
    // Initialize the display driver.
    //
    CFAL96x64x16Init();
    //
    // Initialize the graphics context.
    //
    GrContextInit(&sContext, &g_sCFAL96x64x16);
    //
    // Fill the top 24 rows of the screen with blue to create the banner.
    //
    sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect.i16YMax = 23;
    GrContextForegroundSet(&sContext, ClrDarkBlue);
    GrRectFill(&sContext, &sRect);
    //
    // Put a white box around the banner.
    //
    GrContextForegroundSet(&sContext, ClrWhite);
    GrRectDraw(&sContext, &sRect);
    //
    // Print a personal message in the middle of the banner.
    //
    GrContextFontSet(&sContext, g_psFontCm12);
    GrStringDrawCentered(&sContext, "How's life?", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 10, 0);
    //
    // Print my name using the Computer Modern 40 point font in the middle of the display
    //
    GrContextFontSet(&sContext, g_psFontCm12);
    GrStringDrawCentered(&sContext, "Aidan Rutherford", -1,
                         GrContextDpyWidthGet(&sContext) / 2,
                         ((GrContextDpyHeightGet(&sContext) - 24) / 2) + 24,
                         0);
    //
    // Flush any cached drawing operations.
    //
    GrFlush(&sContext);
	/**************************************************************************************/
	/**************************************************************************************/
	// This section is the configuration code for the LED from the 
	// blinky.c example in the TivaWare PDL
    //
    //With volatile: The duty cycle is respected and the light blinks at a lower frequency
    //Without volatile: The duty cycle is ignored and the LED does not blink, it stays solid
    // 
    volatile uint32_t ui32Loop;
    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    //
    // Check if the peripheral access is enabled.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG))
    {
    }
    //
    // Enable the GPIO pin for the LED (PG2).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_2);
	/***************************************************************************************/
	//
    // Enable the GPIO port for the buttons.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM); 
	//
    // Check if the peripheral access is enabled.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM))
    {
    }
    //
	// Enable the GPIO pins for the buttons. Set the direction as inputs
	//
    GPIOPinTypeGPIOInput(GPIO_PORTM_BASE,GPIO_PIN_0); 
    GPIOPinTypeGPIOInput(GPIO_PORTM_BASE,GPIO_PIN_1);
    GPIOPinTypeGPIOInput(GPIO_PORTM_BASE,GPIO_PIN_2);
    GPIOPinTypeGPIOInput(GPIO_PORTM_BASE,GPIO_PIN_3);
    GPIOPinTypeGPIOInput(GPIO_PORTM_BASE,GPIO_PIN_4);
    //
	// Define the button press logic for each button as a weak pull up
	//
    GPIOPadConfigSet(GPIO_PORTM_BASE,GPIO_PIN_0,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTM_BASE,GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTM_BASE,GPIO_PIN_2,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTM_BASE,GPIO_PIN_3,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTM_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    //
    // We are finished. Hang around, watch the LED, and push some buttons.
    //  
    while(1)
    {
		//
		// Clear the middle of the screen
		//
        GrStringDrawCentered(&sContext, "                     ", -1,
                         GrContextDpyWidthGet(&sContext) / 2,
                         ((GrContextDpyHeightGet(&sContext) - 24) / 2) + 24,
                         1);
		//
		// Wait for each button press and print a command associated with each button
		//
        if(!GPIOPinRead(GPIO_PORTM_BASE,GPIO_PIN_0))
        {
          GrStringDrawCentered(&sContext, "UP", -1,
                         GrContextDpyWidthGet(&sContext) / 2,
                         ((GrContextDpyHeightGet(&sContext) - 24) / 2) + 24,
                         1);
        }
        if(!GPIOPinRead(GPIO_PORTM_BASE,GPIO_PIN_1)) 
		{
                GrStringDrawCentered(&sContext, "Down", -1,
                         GrContextDpyWidthGet(&sContext) / 2,
                         ((GrContextDpyHeightGet(&sContext) - 24) / 2) + 24,
                         1);
        }
        if(!GPIOPinRead(GPIO_PORTM_BASE,GPIO_PIN_3))
        {
          GrStringDrawCentered(&sContext, "Right", -1,
                         GrContextDpyWidthGet(&sContext) / 2,
                         ((GrContextDpyHeightGet(&sContext) - 24) / 2) + 24,
                         1);
        }
        if(!GPIOPinRead(GPIO_PORTM_BASE,GPIO_PIN_2))
        {
          GrStringDrawCentered(&sContext, "Left", -1,
                         GrContextDpyWidthGet(&sContext) / 2,
                         ((GrContextDpyHeightGet(&sContext) - 24) / 2) + 24,
                         1);
        }
        if(!GPIOPinRead(GPIO_PORTM_BASE,GPIO_PIN_4))
        {
          GrStringDrawCentered(&sContext, "select", -1,
                         GrContextDpyWidthGet(&sContext) / 2,
                         ((GrContextDpyHeightGet(&sContext) - 24) / 2) + 24,
                         1);
        }
		/*****This section is from the blinky.c example from the TivaWare PDL*****/
		//
        // Turn on the LED.
		//
        GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, GPIO_PIN_2);
        //
        // Delay for a bit.
        //
        for(ui32Loop = 0; ui32Loop < DUTY; ui32Loop++)
        {
        }
        //
        // Turn off the LED.
        //
        GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, 0);
        //
        // Delay for a bit.
        //
        for(ui32Loop = 0; ui32Loop < DUTY; ui32Loop++)
        {
        }
		/**************************************************************************/
    }
}


    


