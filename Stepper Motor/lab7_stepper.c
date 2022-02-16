//*****************************************************************************
//
// Aidan Rutherford
//
// File: lab7_stepper.c
//
// This project uses timers and UART to control a stepper motor. Sveral drive
// modes are implemented such as wave full, and half step. Each mode can 
// operate in a constant velocity mode and follower mode as controlled by an
// analog to digital converter.
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
#include "driverlib/adc.h"

#define BAUD 115200 //baud rate for configuring UART
#define HOLD 1000000 //LED duty cyle
#define TIME 25000000 //splash screen timer

#ifdef DEBUG //error routine if library fails to compile
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

void configureADC0(void);            //
void configureUART0(void);           //peripheral specific
void configureGraphicsAndLED(void);  //configurations
void configureTimer0(void);          //
void configureGPIOPortN(void);  
void splashScreen();                 //prints cool graphics upon initialization
void toggleLED(uint32_t flagLED);    //acts as heartbeat of program
void process_menu(uint32_t choice);  //directs user input to cause an event
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count); //transmits characters to PuTTY
void printMenu(void);           //does what it sounds like
void UARTIntHandler(void);      //manges the UART interrupt when a message is received
void Timer0IntHandler(void);    //controls different modes of stepper motor
uint32_t ADC0GetValue(void);    //obtains ADC value

const uint8_t halfStep[8] = {0x0C,0x04,0x06,0x02,0x03,0x01,0x09,0x08}; //halfstep mode
const uint8_t fullStep[4] = {0x0C,0x06,0x03,0x09};                     //fullstep modes
const uint8_t waveDrive[4] = {0x08,0x04,0x02,0x01};                    //wave drive mode
#define INIT_RPM 60 //starting RPM
#define RPM_MIN 1   //minimum RPM
#define RPM_MAX 150 //maximum RPM

uint8_t ledToggle = 0;  //LED state
tRectangle sRect;       //graphics var
tContext g_sContext;    //graphics var
uint8_t stepperMode = 1;//sets stepper mode
uint8_t stepIndex = 0;  //array index to switch between data
uint8_t reverse = 0;    //flag to reverse direction
uint32_t stepperRPM = INIT_RPM; //initialize the beginning RPM to 60
uint32_t previousValue = 0;     //have a past ADC reading
uint8_t index = 0;              //counts the number of steps
int steps = 0;                  //# of steps motor needs to move 

int main(void)
{  
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    FPULazyStackingEnable();
    // Set the clocking to run directly from the crystal.
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    IntMasterDisable(); // disable all interrupts
    configureGraphicsAndLED();  //does what it sounds like
    configureUART0();           //does what it sounds like
    configureADC0();            //does what it sounds like     
    configureTimer0();          //does what it sounds like      
    configureGPIOPortN();       //does what it sounds like
    splashScreen(); //show neat graphics
    IntMasterEnable(); //now that graphics are done, enable interrupts
    printMenu(); //does what it sounds like
    while(1)
    {
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

void configureTimer0(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); //enable Timer0 peripheral
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0))
  {
  }
  TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); //configure timer
  TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/3); //set the duration
  IntEnable(INT_TIMER0A); //enable the interrupt
  TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  TimerEnable(TIMER0_BASE, TIMER_A); //enable timer
  TimerIntRegister(TIMER0_BASE,TIMER_A,Timer0IntHandler); //register interrupt dynamically
}

void configureGPIOPortN(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);  //configure a peripheral for
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)) //GPIO outputs
  {
  }
  GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE,GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
}

void configureADC0(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); //enable ADC
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD))
  {
  }
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
  GrContextForegroundSet(&g_sContext,ClrWhite);
  GrStringDrawCentered(&g_sContext,"count",-1,
                       GrContextDpyWidthGet(&g_sContext)/4,
                       GrContextDpyHeightGet(&g_sContext)/4,0);
  GrStringDrawCentered(&g_sContext,"mode",-1,
                       (GrContextDpyWidthGet(&g_sContext)/4)-4,
                       GrContextDpyHeightGet(&g_sContext)/2,0);
  GrStringDrawCentered(&g_sContext,"RPM",-1,
                       (GrContextDpyWidthGet(&g_sContext)/4),
                       GrContextDpyHeightGet(&g_sContext)-15,0);
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

void Timer0IntHandler()
{
  
  uint32_t stepperPosition;
  char buffer[10];
  GrContextForegroundSet(&g_sContext,ClrWhite);
  TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); //clear interrupt
  TimerLoadSet(TIMER0_BASE,TIMER_A,(SysCtlClockGet()*60)/(200*stepperRPM)); //reset the timer load
  IntMasterDisable();
  sprintf(buffer," %d ",stepperRPM); //print the RPM to the OLED
  GrStringDrawCentered(&g_sContext,buffer,-1,
                       (GrContextDpyWidthGet(&g_sContext)/2),
                       GrContextDpyHeightGet(&g_sContext)-15,0);
  if((stepperMode == 1) && (!reverse))
  { //full drive mode
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 ,
                 fullStep[stepIndex]); //write data
    stepIndex = (1 + stepIndex)%4; //increment the array index
    sprintf(buffer," %d ",stepIndex); //draw some stuff about the mode and data
    GrStringDrawCentered(&g_sContext,buffer, -1,
                         GrContextDpyWidthGet(&g_sContext)/2,
                         GrContextDpyHeightGet(&g_sContext)/4,1);
    GrStringDrawCentered(&g_sContext," full ", -1,
                         (GrContextDpyWidthGet(&g_sContext)/2)+4,
                         GrContextDpyHeightGet(&g_sContext)/2,1);
  }
  else if((stepperMode == 2) && (!reverse))
  { //half drive mode
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
                 halfStep[stepIndex]);  //write data
    stepIndex = (1 + stepIndex)%8; //increment the array index
    sprintf(buffer," %d ",stepIndex); //draw some stuff about the mode and data
    GrStringDrawCentered(&g_sContext,buffer, -1,
                         GrContextDpyWidthGet(&g_sContext)/2,
                         GrContextDpyHeightGet(&g_sContext)/4,1);
    GrStringDrawCentered(&g_sContext," half ", -1,
                         (GrContextDpyWidthGet(&g_sContext)/2)+4,
                         GrContextDpyHeightGet(&g_sContext)/2,1);
  }
  else if((stepperMode == 3) && (!reverse))
  { //wave drive mode
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
                 waveDrive[stepIndex]); //write data
    stepIndex = (1 + stepIndex)%4; //increment the array index
    sprintf(buffer," %d ",stepIndex); //draw some stuff about the mode and data
    GrStringDrawCentered(&g_sContext,buffer, -1, 
                         GrContextDpyWidthGet(&g_sContext)/2, 
                         GrContextDpyHeightGet(&g_sContext)/4,1);
    GrStringDrawCentered(&g_sContext,"wave", -1, 
                         (GrContextDpyWidthGet(&g_sContext)/2)+4, 
                         GrContextDpyHeightGet(&g_sContext)/2,1);
  }
  else if((stepperMode == 1) && (reverse))
  { //full drive reverse mode
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 
                 waveDrive[stepIndex]); //write data
    stepIndex = ((stepIndex - 1) + 4) % 4; //increment the array index
    GrStringDrawCentered(&g_sContext," full ", -1, 
                         (GrContextDpyWidthGet(&g_sContext)/2) +4,
                         GrContextDpyHeightGet(&g_sContext)/2,1);
    sprintf(buffer," %d ",stepIndex); //draw some stuff about the mode and data
    GrStringDrawCentered(&g_sContext,buffer, -1, 
                         (GrContextDpyWidthGet(&g_sContext)/2), 
                         GrContextDpyHeightGet(&g_sContext)/4,1);
  }
  else if((stepperMode == 2) && (reverse))
  { //halfstep reverse mode
     GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
                  halfStep[stepIndex]);  //write data
     stepIndex = ((stepIndex - 1) + 8) % 8; //increment the array index
     GrStringDrawCentered(&g_sContext," half ", -1, 
                          (GrContextDpyWidthGet(&g_sContext)/2)+4,
                          GrContextDpyHeightGet(&g_sContext)/2,1);
     sprintf(buffer," %d ",stepIndex); //draw some stuff about the mode and data
     GrStringDrawCentered(&g_sContext,buffer, -1,
                          GrContextDpyWidthGet(&g_sContext)/2, 
                          GrContextDpyHeightGet(&g_sContext)/4,1);
  }
  else if((stepperMode == 3) && (reverse))
  { //wave drive reverse
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 
                 waveDrive[stepIndex]);  //write data
    stepIndex = ((stepIndex - 1) + 4) % 4; //increment the array index
    GrStringDrawCentered(&g_sContext,"wave", -1, 
                         (GrContextDpyWidthGet(&g_sContext)/2) + 4, 
                         GrContextDpyHeightGet(&g_sContext)/2,1);
    sprintf(buffer," %d ",stepIndex); //draw some stuff about the mode and data
    GrStringDrawCentered(&g_sContext,buffer, -1, 
                         GrContextDpyWidthGet(&g_sContext)/2, 
                         GrContextDpyHeightGet(&g_sContext)/4,1);
  }
  else if(stepperMode == 4)
  { //follower mode
    stepperRPM = RPM_MAX; //set the max RPM during follow mode
    stepperPosition = (ADC0GetValue()*200)/4095; //find the position that the stepper needs to go to
    if((stepperPosition > previousValue))
    { //if stepper needs to turn to the right
      steps = stepperPosition - previousValue; //find the number of steps to make
      stepIndex = (1 + stepIndex)%4; //increment the array index
    }
    else if((stepperPosition < previousValue))// && (index < steps))
    { //if stepper needs to turn to the right
      steps = previousValue - stepperPosition; //find the position that the stepper needs to go to
      stepIndex = ((stepIndex - 1) + 4) % 4; //increment the array index
    }
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 
                 waveDrive[stepIndex]); //write data
    index++; //increment count of steps made
    if(index == steps)
    { //if all the steps have been made
      index = 0; //reset the count
      stepperPosition = (ADC0GetValue()*200)/4095; //find new position
    }
    previousValue = stepperPosition; //set the new to the previous
  }
  else
  {
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 
                 0x00); //turn off motor and draw stuff
    GrStringDrawCentered(&g_sContext," off ", -1, 
                         (GrContextDpyWidthGet(&g_sContext)/2) + 4, 
                         GrContextDpyHeightGet(&g_sContext)/2,1);
    GrStringDrawCentered(&g_sContext,"0x00", -1, 
                         (GrContextDpyWidthGet(&g_sContext)/2)+8, 
                         GrContextDpyHeightGet(&g_sContext)/4,1);
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
  else if(choice == 't')
  { //move to the next stepper mode
    UARTSend((uint8_t *)"toggle stepper mode\r\n",24);
    GrStringDrawCentered(&g_sContext,"     ", -1, 
                         (GrContextDpyWidthGet(&g_sContext)/2)+8, 
                         GrContextDpyHeightGet(&g_sContext)/4,1);
    stepperMode = (1+stepperMode)%4;
    if(stepperMode == 0)
    {
      stepperMode = 1;
    }
  }
  else if(choice == 'r')
  { //reverse the direction of the stepper motor
    UARTSend((uint8_t *)"reverse stepper direction\r\n",29);
    reverse = !reverse;
    if((stepperMode == 2) && (reverse))
    { //reconfigure the array index to decrement
      stepIndex = 7;
      GrStringDrawCentered(&g_sContext,"rev 1", -1, 
                           GrContextDpyWidthGet(&g_sContext)-20, 
                           GrContextDpyHeightGet(&g_sContext)/4,1);
    }
    else if(reverse)
    { //reconfigure the array index to decrement
      stepIndex = 3;
      GrStringDrawCentered(&g_sContext,"rev 1", -1,
                           GrContextDpyWidthGet(&g_sContext)-20, 
                           GrContextDpyHeightGet(&g_sContext)/4,1);
    }
    else
    { //reconfigure the array index to decrement
      stepIndex = 0;
      GrStringDrawCentered(&g_sContext,"rev 0", -1, 
                           GrContextDpyWidthGet(&g_sContext)-20, 
                           GrContextDpyHeightGet(&g_sContext)/4,1);
    }
  }
  else if(choice == 'f')
  { //switch to follower mode
    UARTSend((uint8_t *)"switch to follower mode\r\n",27);
    stepperMode = 4;
  }
  else if(choice == 'o')
  { //turj off the motor
    stepperMode = 0;
  }
  else if(choice == '+')
  { //increase the RPM
    stepperRPM++;
    UARTSend((uint8_t *)"increase RPM\r\n",16);
    sRect.i16XMin =(GrContextDpyWidthGet(&g_sContext)/2) - 7;
    sRect.i16YMin = GrContextDpyHeightGet(&g_sContext)-18;
    sRect.i16XMax = (GrContextDpyWidthGet(&g_sContext)/2) +4;
    sRect.i16YMax = GrContextDpyHeightGet(&g_sContext)-10;
    GrContextForegroundSet(&g_sContext, ClrBlack);
    GrRectFill(&g_sContext, &sRect);
    if (stepperRPM > 150)
    { //restricts RPM to 150
      UARTSend((uint8_t *)"Maximum RPM reached\r\n",23);
      stepperRPM = 150;
    }
  }  
  else if(choice == '-')
  { //decrease the RPM
    UARTSend((uint8_t *)"reduce RPM\r\n",14);
    stepperRPM--;
    sRect.i16XMin =(GrContextDpyWidthGet(&g_sContext)/2) - 7;
    sRect.i16YMin = GrContextDpyHeightGet(&g_sContext)-18;
    sRect.i16XMax = (GrContextDpyWidthGet(&g_sContext)/2) +4;
    sRect.i16YMax = GrContextDpyHeightGet(&g_sContext)-10;
    GrContextForegroundSet(&g_sContext, ClrBlack);
    GrRectFill(&g_sContext, &sRect);
    GrContextForegroundSet(&g_sContext, ClrWhite);
    if (stepperRPM < 1)
    { //keep the RPM at at least 1
      UARTSend((uint8_t *)"Minimum RPM reached\r\n",23);
      stepperRPM = 1;
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
  UARTSend((uint8_t *)"s - show splash screen\r\n",26);
  UARTSend((uint8_t *)"t - toggle stepper mode\r\n",27);
  UARTSend((uint8_t *)"q - quit program\r\n",20);
  UARTSend((uint8_t *)"+ - increase motor RPM\r\n",26);
  UARTSend((uint8_t *)"- - decrease motor RPM\r\n",26);
  UARTSend((uint8_t *)"r - reverse stepper direction\r\n",33); 
  UARTSend((uint8_t *)"o - turn off motor\r\n",22);
  IntMasterEnable();
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