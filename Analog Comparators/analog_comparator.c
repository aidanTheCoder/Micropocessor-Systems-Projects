//*****************************************************************************
//
// Aidan Rutherford
//
// File: analog_comparator.c
//
// This project intorudces the use of analog comparators and continues the
// use of interrupts, speciifcally different types, static (those manipulated
// in startup_ewarm.c) and dynamic (configuring the interrupt call by
// registering it with a function), making it unnecessary to manually adjust
// adjust the interrupt vector table. Interrupt-driven peripherals were UART,
// the analog comparator, and GPIO switches.
//
//*****************************************************************************

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/rom.h"
#include "grlib/grlib.h"
#include "drivers/cfal96x64x16.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "driverlib/comp.h"

#define BAUD 115200 //baud rate for configuring UART
#define HOLD 1000000 //LED duty cyle
#define TIME 25000000 //splash screen timer

#ifdef DEBUG //error routine if library fails to compile
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

void configureUART0(void);             //
void configureAnalogComparator0(void); //each of these functions configure
void configureGraphicsAndLED(void);    //the peripheral in their name.
void configureSwitches(void);          //
void splashScreen();                   //prints cool graphics upon initialization
void toggleLED(uint32_t flagLED);      //acts as heartbeat of program
void process_menu(uint32_t choice);    //directs user input to cause an event
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count); //transmits characters to PuTTY
void printMenu(void);       //does what it sounds like
void UARTIntHandler(void);  //manges the UART interrupt when a message is received
void GPIOMIntHandler(void); //reguisters button presses and counts
void COMP0IntHandler(void); //dispays state returned by analog comparator

uint8_t ledToggle = 0; //LED state
tRectangle sRect;      //graphics var
tContext g_sContext;   //graphics var
uint32_t voltageThreshold = 0; //flag for state of comparator

uint8_t checkComparator = 0; //used to debounce the comparator
uint8_t checkSwitches = 0;   //used to debounce the switches

uint32_t button0 = 0; //
uint32_t button1 = 0; //button counters
uint32_t button2 = 0; //
uint32_t button3 = 0;
uint32_t button4 = 0;

int main(void)
{  
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    FPULazyStackingEnable();
    // Set the clocking to run directly from the crystal.
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    IntMasterDisable(); // disable all interrupts
    configureGraphicsAndLED();    //does what it sounds like
    configureUART0();		      //does what it sounds like
    configureSwitches();	   	  //does what it sounds like
    configureAnalogComparator0(); //does what it sounds like
    splashScreen(); //show neat graphics
    IntMasterEnable(); //now that graphics are done, enable interrupts
    printMenu(); //does what it sounds like
    while(1)
    {
      toggleLED('\0'); //heartbeat
    }
}

void configureUART0(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //enable GPIO pins for UART and configure
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) //enable GPIO pins for UART and Configure
  {
  } 
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);       //enable UART peripheral
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)) //wait for peripheral
  {
  }
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1); //assign PA0 and PA1 to be UART pins
  UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), BAUD,         //configure UART clock, set baud rate to 115,200,
                     (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | //8 bit message with 1 stop bit and no parity bit
                      UART_CONFIG_PAR_NONE));
  IntEnable(INT_UART0); //enable UART interrupts
  UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT); //enable UART interrupts
  UARTIntRegister(UART0_BASE,UARTIntHandler);
}

void configureSwitches(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM))
  {
  }
  GPIOPinTypeGPIOInput(GPIO_PORTM_BASE,GPIO_PIN_0); //
  GPIOPinTypeGPIOInput(GPIO_PORTM_BASE,GPIO_PIN_1); //set pin type of each
  GPIOPinTypeGPIOInput(GPIO_PORTM_BASE,GPIO_PIN_2); //GPIO pin switch to
  GPIOPinTypeGPIOInput(GPIO_PORTM_BASE,GPIO_PIN_3); //be an input 
  GPIOPinTypeGPIOInput(GPIO_PORTM_BASE,GPIO_PIN_4); //
  GPIOPadConfigSet(GPIO_PORTM_BASE,GPIO_PIN_0,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU); //
  GPIOPadConfigSet(GPIO_PORTM_BASE,GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU); //set the strength and
  GPIOPadConfigSet(GPIO_PORTM_BASE,GPIO_PIN_2,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU); //type of button press
  GPIOPadConfigSet(GPIO_PORTM_BASE,GPIO_PIN_3,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU); //logic for each switch
  GPIOPadConfigSet(GPIO_PORTM_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU); //
  GPIOIntTypeSet(GPIO_PORTM_BASE,GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |      //set an interrupt for each switch
                                 GPIO_PIN_3 | GPIO_PIN_4,GPIO_FALLING_EDGE); //trigger the interrupt on button press
  IntEnable(INT_GPIOM); //enable GPIO interrupt
  GPIOIntEnable(GPIO_PORTM_BASE,GPIO_INT_PIN_0 | GPIO_INT_PIN_1 | GPIO_INT_PIN_2 | //enable the interrupt for
                                GPIO_INT_PIN_3 | GPIO_INT_PIN_4);                  //each switch
  GPIOIntRegister(GPIO_PORTM_BASE,GPIOMIntHandler); //configure interruot dynamically
}

void configureGraphicsAndLED(void)
{
  CFAL96x64x16Init();                           // Initialize the display driver.
  GrContextInit(&g_sContext, &g_sCFAL96x64x16); // Initialize the graphics context and find the middle X coordinate.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);  //enable GPIO pins for LED and configure
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG))
  {
  }
  GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE,GPIO_PIN_2); //set PG2 for LED to output
}  
void configureAnalogComparator0(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); //enbale GPIO Port C peripheral
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC))
  {
  }
  GPIOPinTypeGPIOInput(GPIO_PORTC_BASE,GPIO_PIN_7); //set pin C7 to be input
  SysCtlPeripheralEnable(SYSCTL_PERIPH_COMP0);      //enable comparator peripheral
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_COMP0))
  {
  }
  ComparatorRefSet(COMP_BASE,COMP_REF_1_65V);     //set the threshold voltage
  ComparatorConfigure(COMP_BASE,0,COMP_INT_BOTH); //set direction of that the voltage
  IntEnable(INT_COMP0);                           //that causes the trigger
  ComparatorIntEnable(COMP_BASE,0);               //configure interrupts and make dynamic
  ComparatorIntRegister(COMP_BASE,0,COMP0IntHandler);
}

void splashScreen(void)
{
  uint32_t clockRate = SysCtlClockGet();
  char clockBuffer[7];
  sprintf(clockBuffer,"CLK: %d",clockRate);
  GrContextFontSet(&g_sContext, g_psFontCm12);
  //draw a black rectangle on the entire OLED
  sRect.i16XMin = 0; 
  sRect.i16YMin = 0;
  sRect.i16XMax = GrContextDpyWidthGet(&g_sContext);
  sRect.i16YMax = GrContextDpyHeightGet(&g_sContext);
  GrContextForegroundSet(&g_sContext, ClrBlack);
  GrRectFill(&g_sContext, &sRect);
  //draw a red rectangle across the top of the OLED
  sRect.i16XMin = 0;
  sRect.i16YMin = 0;
  sRect.i16XMax = GrContextDpyWidthGet(&g_sContext);
  sRect.i16YMax = 8;
  GrContextForegroundSet(&g_sContext, ClrRed);
  GrRectFill(&g_sContext, &sRect);
  //draw a green rectangle on the right side of the OLED
  sRect.i16XMin = GrContextDpyWidthGet(&g_sContext) - 8;
  sRect.i16YMin = 0;
  sRect.i16XMax = GrContextDpyWidthGet(&g_sContext);
  sRect.i16YMax = GrContextDpyHeightGet(&g_sContext);
  GrContextForegroundSet(&g_sContext, ClrGreen);
  GrRectFill(&g_sContext, &sRect);
  //draw a blue rectangle on the bottom of the OLED
  sRect.i16XMin = 0;
  sRect.i16YMin = GrContextDpyHeightGet(&g_sContext) - 8;
  sRect.i16XMax = GrContextDpyWidthGet(&g_sContext);
  sRect.i16YMax = GrContextDpyHeightGet(&g_sContext);
  GrContextForegroundSet(&g_sContext, ClrBlue);
  GrRectFill(&g_sContext, &sRect);
  //draw a yellow rectangle on the left of the OLED
  sRect.i16XMin = 0;
  sRect.i16YMin = 0;
  sRect.i16XMax = 8;
  sRect.i16YMax = GrContextDpyHeightGet(&g_sContext);
  GrContextForegroundSet(&g_sContext, ClrYellow);
  GrRectFill(&g_sContext, &sRect);
  //print a message in the  middle of the screen
  GrContextForegroundSet(&g_sContext,ClrWhite);
  GrStringDrawCentered(&g_sContext,clockBuffer, -1,
                       GrContextDpyWidthGet(&g_sContext) / 2,
                       (GrContextDpyHeightGet(&g_sContext) / 2),0);
  SysCtlDelay(TIME); //give splash screen time to show
  sRect.i16XMin = 0; //cover entire OLED with black rectangle
  sRect.i16YMin = 0;
  sRect.i16XMax = GrContextDpyWidthGet(&g_sContext);
  sRect.i16YMax = GrContextDpyHeightGet(&g_sContext);
  GrContextForegroundSet(&g_sContext, ClrBlack);
  GrRectFill(&g_sContext, &sRect);
}

void toggleLED(uint32_t flagLED)
{ //toggle LED
  if((ledToggle == 0) && (flagLED == 'd'))
  { //if LED is enabled and user wnats to to toggle => disable
    GPIOPinWrite(GPIO_PORTG_BASE,GPIO_PIN_2,GPIO_PIN_2); //turn on LED
    SysCtlDelay(HOLD);
    GPIOPinWrite(GPIO_PORTG_BASE,GPIO_PIN_2,0); //turn off LED
    SysCtlDelay(HOLD);
    ledToggle = 1; //disable LED
  }
  else if((ledToggle == 1) && (flagLED == 'd'))
  { //LED disabled and user wants to adjust to adjust => enable
    GPIOPinWrite(GPIO_PORTG_BASE,GPIO_PIN_2,GPIO_PIN_2); //turn on LED
    SysCtlDelay(HOLD);
    GPIOPinWrite(GPIO_PORTG_BASE,GPIO_PIN_2,0); //turn off LED
    SysCtlDelay(HOLD);
    ledToggle = 0; //enable LED
  }
  else if((ledToggle == 0) && (flagLED != 'd'))
  { //LED is enabled, continue to flash
    GPIOPinWrite(GPIO_PORTG_BASE,GPIO_PIN_2,GPIO_PIN_2); //turn on LED
    SysCtlDelay(HOLD);
    GPIOPinWrite(GPIO_PORTG_BASE,GPIO_PIN_2,0); //turn off LED
    SysCtlDelay(HOLD);
  }
  else
  { //LED disabled, do nothing
    GPIOPinWrite(GPIO_PORTG_BASE,GPIO_PIN_2,0); //turn off LED
  }
}

void UARTIntHandler(void)
{
    uint32_t ui32Status;
    uint32_t select;
    ui32Status = UARTIntStatus(UART0_BASE, true); // Get the interrrupt status.
    UARTIntClear(UART0_BASE, ui32Status); // Clear the asserted interrupts.
    select = UARTCharGetNonBlocking(UART0_BASE); //retrieve message
    if (select != -1) //if message is valid
    {
      process_menu(select); //take users input to execute menu option
    }
}

void GPIOMIntHandler(void)
{
  char buttonBuff[4];
  GPIOIntClear(GPIO_PORTM_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1 | //clear every
               GPIO_INT_PIN_2 | GPIO_INT_PIN_3 | GPIO_INT_PIN_4); //button interrupt
  GrContextFontSet(&g_sContext, g_psFontFixed6x8); //set graphics attributes
  GrContextForegroundSet(&g_sContext,ClrWhite);  					
  IntMasterDisable();                              				
  if(!GPIOPinRead(GPIO_PORTM_BASE,GPIO_PIN_0))    					
  { //if button pressed, all comments in this if block are the same for every other if block             													
        GrStringDrawCentered(&g_sContext,"  Up  ", -1,	//draw name of button for neatness
                         GrContextDpyWidthGet(&g_sContext)/2,
                         GrContextDpyHeightGet(&g_sContext)/4,1);
            sprintf(buttonBuff," %d ",button0); //convert count to string         

    GrStringDrawCentered(&g_sContext,buttonBuff, -1, //show count on OLED
                         GrContextDpyWidthGet(&g_sContext)/2,
                         GrContextDpyHeightGet(&g_sContext)/2,1);
    while(!GPIOPinRead(GPIO_PORTM_BASE,GPIO_PIN_0)) //while button prssed, debounces buttons
    {                                              
    }
    button0++; //increment count                                      
    sprintf(buttonBuff," %d ",button0); //reconvert count     

    GrStringDrawCentered(&g_sContext,buttonBuff, -1, //print new value of buffer
                         GrContextDpyWidthGet(&g_sContext)/2,
                         GrContextDpyHeightGet(&g_sContext)/2,1);
  }
  if(!GPIOPinRead(GPIO_PORTM_BASE,GPIO_PIN_1))
  {
        GrStringDrawCentered(&g_sContext," Down ", -1,
                         GrContextDpyWidthGet(&g_sContext)/2,
                         GrContextDpyHeightGet(&g_sContext)/4,1);
            sprintf(buttonBuff," %d ",button1);

    GrStringDrawCentered(&g_sContext,buttonBuff, -1,
                         GrContextDpyWidthGet(&g_sContext)/2,
                         GrContextDpyHeightGet(&g_sContext)/2,1);
    while(!GPIOPinRead(GPIO_PORTM_BASE,GPIO_PIN_1))
    {
    }
    button1++;
    sprintf(buttonBuff," %d ",button1);

    GrStringDrawCentered(&g_sContext,buttonBuff, -1,
                         GrContextDpyWidthGet(&g_sContext)/2,
                         GrContextDpyHeightGet(&g_sContext)/2,1);
  }
  if(!GPIOPinRead(GPIO_PORTM_BASE,GPIO_PIN_3))
  {
        GrStringDrawCentered(&g_sContext," Right ", -1,
                         GrContextDpyWidthGet(&g_sContext)/2,
                         GrContextDpyHeightGet(&g_sContext)/4,1);
            sprintf(buttonBuff," %d ",button3);

    GrStringDrawCentered(&g_sContext,buttonBuff, -1,
                         GrContextDpyWidthGet(&g_sContext)/2,
                         GrContextDpyHeightGet(&g_sContext)/2,1);
    while(!GPIOPinRead(GPIO_PORTM_BASE,GPIO_PIN_3))
    {
    }
    button3++;
    sprintf(buttonBuff," %d ",button3);

    GrStringDrawCentered(&g_sContext,buttonBuff, -1,
                         GrContextDpyWidthGet(&g_sContext)/2,
                         GrContextDpyHeightGet(&g_sContext)/2,1);
  }
  if(!GPIOPinRead(GPIO_PORTM_BASE,GPIO_PIN_2))
  {
        GrStringDrawCentered(&g_sContext," Left ", -1,
                         GrContextDpyWidthGet(&g_sContext)/2,
                         GrContextDpyHeightGet(&g_sContext)/4,1);
        sprintf(buttonBuff," %d ",button2);

    GrStringDrawCentered(&g_sContext,buttonBuff, -1,
                         GrContextDpyWidthGet(&g_sContext)/2,
                         GrContextDpyHeightGet(&g_sContext)/2,1);
    while(!GPIOPinRead(GPIO_PORTM_BASE,GPIO_PIN_2))
    {
    }
    button2++;
    sprintf(buttonBuff," %d ",button2);

    GrStringDrawCentered(&g_sContext,buttonBuff, -1,
                         GrContextDpyWidthGet(&g_sContext)/2,
                         GrContextDpyHeightGet(&g_sContext)/2,1);
  }
  if(!GPIOPinRead(GPIO_PORTM_BASE,GPIO_PIN_4))
  { 
        GrStringDrawCentered(&g_sContext,"Select", -1,
                         GrContextDpyWidthGet(&g_sContext)/2,
                         GrContextDpyHeightGet(&g_sContext)/4, 1);
            sprintf(buttonBuff," %d ",button4);

    GrStringDrawCentered(&g_sContext,buttonBuff, -1,
                         GrContextDpyWidthGet(&g_sContext)/2,
                         GrContextDpyHeightGet(&g_sContext)/2, 1);
    while((!GPIOPinRead(GPIO_PORTM_BASE,GPIO_PIN_4)))
    {
    }
    button4++;
    sprintf(buttonBuff," %d ",button4);

    GrStringDrawCentered(&g_sContext,buttonBuff, -1,
                         GrContextDpyWidthGet(&g_sContext)/2,
                         GrContextDpyHeightGet(&g_sContext)/2, 1);
  }
  IntMasterEnable();
}

void COMP0IntHandler(void)
{
  char voltageBuffer[10]; //buffer for number of voltage counts
  ComparatorIntClear(COMP_BASE,0); //clear comparator interrupt
  IntMasterDisable();
 
  GrContextForegroundSet(&g_sContext,ClrWhite);
 
  if((ComparatorValueGet(COMP_BASE,0)) && (!checkComparator)) //get comparator value (0 or 1) and flush 
  {                                   //state to buffer using word above or below
    voltageThreshold++; //increment number of times interrupt occurs
    sprintf(voltageBuffer," %d ",voltageThreshold);
    GrStringDrawCentered(&g_sContext," Above ",-1,
                         GrContextDpyWidthGet(&g_sContext)/2,
                         GrContextDpyHeightGet(&g_sContext)/4,1);
    GrStringDrawCentered(&g_sContext,voltageBuffer,-1,          //display voltage 
                         GrContextDpyWidthGet(&g_sContext)/2,   //threshold to OLED
                         GrContextDpyHeightGet(&g_sContext)/2,1);
    checkComparator = 1;
  }
  else if((!ComparatorValueGet(COMP_BASE,0)) && (checkComparator))
  {
    voltageThreshold++; //increment number of times interrupt occurs
    sprintf(voltageBuffer," %d ",voltageThreshold);
    GrStringDrawCentered(&g_sContext," Below ",-1,
                         GrContextDpyWidthGet(&g_sContext)/2,
                         GrContextDpyHeightGet(&g_sContext)/4,1);
    GrStringDrawCentered(&g_sContext,voltageBuffer,-1,          //display voltage 
                         GrContextDpyWidthGet(&g_sContext)/2,   //threshold to OLED
                         GrContextDpyHeightGet(&g_sContext)/2,1);
    checkComparator = 0;
  }
  IntMasterEnable(); 
}

void process_menu(uint32_t choice)
{ //evaluate user input to decide with function to call
  if(choice == 'm')
  { //print menu
    UARTSend((uint8_t *)"Print menu\r\n",14);
    printMenu();
  }
  else if(choice == 'd')
  { //enable or disable the LED
    UARTSend((uint8_t *)"Toggle LED\r\n",14);
    toggleLED(choice);
  }
  else if(choice == 's')
  { //show the splash screen
    UARTSend((uint8_t *)"Display splash screen\r\n",25);
    IntMasterDisable();
    splashScreen();
    IntMasterEnable();
  }
  else if(choice == 'q')
  { //quit program
    UARTSend((uint8_t *)"Quitting Program...\r\n",23);
    GPIOPinWrite(GPIO_PORTG_BASE,GPIO_PIN_2,0); //turn off LED
    IntMasterDisable(); //disbale interrupts forever
    sRect.i16XMin = 0; //cover entire OLED with black rectangle
    sRect.i16YMin = 0;
    sRect.i16XMax = GrContextDpyWidthGet(&g_sContext);
    sRect.i16YMax = GrContextDpyHeightGet(&g_sContext);
    GrContextForegroundSet(&g_sContext, ClrBlack);
    GrRectFill(&g_sContext, &sRect);
    while(1)
    {
    }
  }
  else
  { //make user self-conscious
    UARTSend((uint8_t *)"invalid input, dummy\r\n",24); 
  }
}

void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
  while(ui32Count--) //while chars in FIFO
  {
    UARTCharPut(UART0_BASE,*pui8Buffer++); //send
  }
}

void printMenu(void)
{ //does exactly what is sounds like, or it should
  IntMasterDisable();
  UARTSend((uint8_t *)"Menu selection:\r\n",17);
  UARTSend((uint8_t *)"m - print this menu\r\n",21);
  UARTSend((uint8_t *)"d - toggle LED\r\n",16);
  UARTSend((uint8_t *)"s - show splash screen\r\n",24);
  UARTSend((uint8_t *)"q - quit program\r\n",20);
  IntMasterEnable();
}