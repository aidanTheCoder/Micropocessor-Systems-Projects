//*****************************************************************************
//
// Aidan Rutherford
// This example application is form the timers.c example from the 
// DK-TM4C123G Firmware Package, version 2.1.4.178. It demonstrates the use of 
// the timers to generate periodic interrupts.  One timer is set up to interrupt 
// once per second and. The other to interrupt depending on a value from an ADC. 
// Each interrupt has counters. Timer1 for the the number of requests, and actual 
// service routines. Timer1 has counter for RAW amount of service routines, both 
// serviced and unserviced.
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
#include "driverlib/adc.h"
#include "driverlib/gpio.h"

#define BAUD 115200 //baud rate for configuring UART
#define HOLD 1000000 //LED duty cyle
#define TIME 2500000 //splash screen timer

#ifdef DEBUG //error routine if library fails to compile
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

void configureUART0(void);  //
void configureADC0(void);   //each function that starts with 'configure' does
void configureTimer0(void); //exactly what it sounds like
void configureTimer1(void); //
void splashScreen(); //prints cool graphics to screen
void toggleLED(uint32_t flagLED); //program heartbeat, togglable
void UARTIntHandler(void); //manages UART interrupts
void Timer0IntHandler(void); //manages Timer0 interrupts
void Timer1IntHandler(void); //manages Timer1 interrupts
uint32_t ADC0GetValue(void); //does what it sounds like
void process_menu(uint32_t choice); //executes function based off user input
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count); //sends characters to PuTTY
void printMenu(void); //does what it sounds like
extern void isr_asm_start(void);

uint8_t ledToggle = 0; //LED state
tRectangle sRect;      //graphics var
tContext g_sContext;
uint32_t g_ui32Flags;  //interrupt flags
uint32_t IntReqCount = 0; //count number of ISR
uint32_t IntSvcCount = 0; //count interrupts actually serviced
char IntSvcBuffer[10]; //buffer for int counts
uint8_t toggleCount = 0; //number of svc counts
uint8_t ADCFlag = 0; //flag to toggle ADC count on UART
uint8_t timerLoadFlag = 0; //flag to toggle timer load set on UART

int main(void)
{  
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    FPULazyStackingEnable();
    // Set the clocking to run at 80 MHz
    /*SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);*/
    //set clock to run at 50 MHz
    /*SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);*/
    //set clock to run at 16 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
    
    IntMasterDisable(); // disable all interrupts
    CFAL96x64x16Init(); // Initialize the display driver.
    GrContextInit(&g_sContext, &g_sCFAL96x64x16); // Initialize the graphics context and find the middle X coordinate.
    configureTimer0(); // Enable timer 1 and timer 0 peripheral
    configureTimer1();
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //enable GPIO pins for UART and configure
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) //enable GPIO pins for UART and Configure
    {
    } 
    configureUART0();
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); //enable GPIO pins for ADC and Configure
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
    {
    }
    configureADC0();
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);     //enable GPIO pins for LED and configure
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG))
    {
    }
    GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE,GPIO_PIN_2); //set PG2 for LED to output

    ADCIntClear(ADC0_BASE,3); //clear the ADC interrupt flag
    ADCProcessorTrigger(ADC0_BASE,3); //trigger cause of ADC to get reading  
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

void configureADC0(void)
{
  GPIOPinTypeADC(GPIO_PORTD_BASE,GPIO_PIN_5); //set pin type to ADC
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); //enable ADC0 peripheral
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
  {
  }
  ADCSequenceDisable(ADC0_BASE,3); //stop sequence from working
  ADCSequenceConfigure(ADC0_BASE,3,ADC_TRIGGER_PROCESSOR,0); //set ADC trigger
  ADCSequenceStepConfigure(ADC0_BASE,3,0,ADC_CTL_CH6 | ADC_CTL_IE | ADC_CTL_END);
  ADCSequenceEnable(ADC0_BASE,3); //enable to sequence
}

void configureTimer0(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); //enable Timer0 peripheral
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0))
  {
  }
  TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); //configure timer
  TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()); //set the duration
  IntEnable(INT_TIMER0A); //enable the interrupt
  TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  TimerEnable(TIMER0_BASE, TIMER_A); //enable timer
  TimerIntRegister(TIMER0_BASE,TIMER_A,Timer0IntHandler); //register interrupt dynamically
  
}

void configureTimer1(void)
{
   SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); //enable Timer1 peripheral
   while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1))
   {
   }
   TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC); //configure timer
   TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() / 2); //set the duration
   IntEnable(INT_TIMER1A); //enable the interrupt
   TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
   TimerEnable(TIMER1_BASE, TIMER_A); //enable timer
   TimerIntRegister(TIMER1_BASE, TIMER_A,isr_asm_start); //register interrupt dynamically

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
  
  GrContextForegroundSet(&g_sContext,ClrWhite); //print req amd svc for optimization
  GrStringDrawCentered(&g_sContext,"req:",-1,GrContextDpyWidthGet(&g_sContext)/4,GrContextDpyHeightGet(&g_sContext)/10,1);
  GrStringDrawCentered(&g_sContext,"svc:",-1,GrContextDpyWidthGet(&g_sContext)/4,GrContextDpyHeightGet(&g_sContext)/4,1);
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
	//although this is never called, program won't compile without it
    while(UARTCharsAvail(UART0_BASE))
    { // Loop while there are characters in the receive FIFO.
      // Read the next character from the UART and write it back to the UART.
      select = UARTCharGetNonBlocking(UART0_BASE); //retrieve message
      if (select != -1) //if message is valid
      {
        process_menu(select); //take users input to execute menu option
      }
    }
}

void Timer0IntHandler(void)
{
    char ADCBuffer[5];
    char timerLoadBuffer[10];
    char IntReqBuffer[10];
    uint32_t readADCValue;
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); //clear interrupt
    readADCValue = ADC0GetValue(); //get the ADC value
    GrContextForegroundSet(&g_sContext,ClrWhite);
    IntMasterDisable(); //disable interrupts
    IntReqCount = (readADCValue*100)+1;
    TimerDisable(TIMER1_BASE, TIMER_A); //reset timer duration for new ADC value
    TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet()/IntReqCount);
    TimerEnable(TIMER1_BASE, TIMER_A);
    if(ADCFlag)
    { //choose to toggle the reading produced by the ADC
      sprintf(ADCBuffer,"  %d  ",readADCValue);
      GrStringDrawCentered(&g_sContext,ADCBuffer,-1,GrContextDpyWidthGet(&g_sContext)-32,(GrContextDpyHeightGet(&g_sContext)/2)-4,1);
    }
    if(timerLoadFlag)
    { //choose to print the timer load for timer1
      sprintf(timerLoadBuffer," %d ",SysCtlClockGet()/IntReqCount);
      GrStringDrawCentered(&g_sContext,"             ",-1,GrContextDpyWidthGet(&g_sContext)/2,GrContextDpyHeightGet(&g_sContext)-8,1);
      GrStringDrawCentered(&g_sContext,timerLoadBuffer,-1,GrContextDpyWidthGet(&g_sContext)/2,GrContextDpyHeightGet(&g_sContext)-8,1);
    }
    //prints the interrupt service and request count
    sprintf(IntReqBuffer,"%d",IntReqCount);
    sprintf(IntSvcBuffer,"%d",IntSvcCount);
    GrStringDrawCentered(&g_sContext,"          ",-1,GrContextDpyWidthGet(&g_sContext)-32,GrContextDpyHeightGet(&g_sContext)/10,1);
    GrStringDrawCentered(&g_sContext,IntReqBuffer,-1,GrContextDpyWidthGet(&g_sContext)-32,GrContextDpyHeightGet(&g_sContext)/10,1);
    GrStringDrawCentered(&g_sContext,"          ",-1,GrContextDpyWidthGet(&g_sContext)-32,GrContextDpyHeightGet(&g_sContext)/4,1);
    GrStringDrawCentered(&g_sContext,IntSvcBuffer,-1,GrContextDpyWidthGet(&g_sContext)-32,GrContextDpyHeightGet(&g_sContext)/4,1);
    IntMasterEnable();
    IntSvcCount = 0; //reset interrupt service count
}

void Timer1IntHandler(void)
{ 
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT); //clear interrupt
    IntSvcCount++; //increment number of svc rountines
}

uint32_t ADC0GetValue(void)
{
  uint32_t ADCBuffer[1];
  ADCProcessorTrigger(ADC0_BASE,3); //trigger cause of ADC to get reading   
  while(!ADCIntStatus(ADC0_BASE,3,false)) //gets interrput status
  {
  }
  ADCIntClear(ADC0_BASE,3); //clear the interrupt
  ADCSequenceDataGet(ADC0_BASE,3,ADCBuffer); //retrive data from ADC
  return ADCBuffer[0];
}

void process_menu(uint32_t choice)
{ //evaluate user input to decide with function to call
  if(choice == 'm')
  { //print menu
    UARTSend((uint8_t *)"print menu\r\n",14);
    printMenu();
  }
  else if(choice == 'd')
  { //enable or disable the LED
    UARTSend((uint8_t *)"toggle LED\r\n",14);
    toggleLED(choice);
  }
  else if(choice == 's')
  { //show the splash screen
    UARTSend((uint8_t *)"show splash screen\r\n",22);
    IntMasterDisable();
    splashScreen();
    IntMasterEnable();
  }
  else if(choice == 'a')
  { //toggle ADC reading shown on OLED
     UARTSend((uint8_t *)"toggle ADC value\r\n",20);
     if(ADCFlag)
     { //make the flag false
      ADCFlag = 0;
      sRect.i16XMin = 0; //cover ADC data with a black rectangle
      sRect.i16YMin = (GrContextDpyHeightGet(&g_sContext)/2)-8;
      sRect.i16XMax = GrContextDpyWidthGet(&g_sContext);
      sRect.i16YMax = GrContextDpyHeightGet(&g_sContext)/2;
      GrContextForegroundSet(&g_sContext, ClrBlack);
      GrRectFill(&g_sContext, &sRect);
     }
     else
     { //show the ADC stuff
       ADCFlag = 1;
       GrStringDrawCentered(&g_sContext,"adc:",-1,GrContextDpyWidthGet(&g_sContext)/4,(GrContextDpyHeightGet(&g_sContext)/2)-4,1);
     }
  }
  else if(choice == 't')
  { //toggle timer load value
    IntMasterDisable();
    UARTSend((uint8_t *)"toggle timer load\r\n",21);
     if(timerLoadFlag)
     {
      timerLoadFlag = 0; //make the flag false
      sRect.i16XMin = 0; //cover the timer load infro with rectangle
      sRect.i16YMin = GrContextDpyHeightGet(&g_sContext)-25;
      sRect.i16XMax = GrContextDpyWidthGet(&g_sContext);
      sRect.i16YMax = GrContextDpyHeightGet(&g_sContext);
      GrContextForegroundSet(&g_sContext, ClrBlack);
      GrRectFill(&g_sContext, &sRect);
     }
     else
     { //show the timer load set
       timerLoadFlag = 1;
       GrStringDrawCentered(&g_sContext,"tls:",-1,GrContextDpyWidthGet(&g_sContext)/2,GrContextDpyHeightGet(&g_sContext)-20,1);
       
     }
     IntMasterEnable();
  }
  else if(choice == 'y')
  { //switch to assembly interrupt routine
    UARTSend((uint8_t *)"Switch to Assembly Routine\r\n",30);
    TimerIntUnregister(TIMER1_BASE,TIMER_A);
    TimerIntRegister(TIMER1_BASE,TIMER_A,isr_asm_start);              
  }
  else if(choice == 'c')
  { //switch to C routine
    UARTSend((uint8_t *)"Switch to C routine\r\n",23);
    TimerIntUnregister(TIMER1_BASE,TIMER_A);
    TimerIntRegister(TIMER1_BASE,TIMER_A,Timer1IntHandler);
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
  UARTSend((uint8_t *)"a - toggle ADC value\r\n",24);
  UARTSend((uint8_t *)"t - toggle timer load\r\n",25);
  UARTSend((uint8_t *)"y - switch to assembly routine\r\n",34);
  UARTSend((uint8_t *)"c - switch to C routine\r\n",27);
  IntMasterEnable();
}