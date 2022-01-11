/****************************************************************
* Aidan Rutherford
* Send a menu via UART to prompt user for a menu option
* options include printing the menu, toggling the LED, showing a
* neat splash screen, and toggling between viewing and hiding 
* graphs and numerical values recorded by the ADC

* This project was derived form the uart_echo example project in
  the TivaWare peripheral driver library, part of the part of
  revision 2.1.4.178 of the DK-TM4C123G Firmware Package
****************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "grlib/grlib.h"
#include "drivers/cfal96x64x16.h"
#include "driverlib/adc.h"

#define BAUD 115200 //baud rate for configuring UART
#define HOLD 1000000 //LED duty cyle
#define TIME 50000000 //splash screen timer

#ifdef DEBUG //error routine if driver library encounters error
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

void controlADC0(void);
void graphADCValues(void);
void printADCValues(void);
void getADC0Value(void);
void splash_screen(void); //makes cool dislay on OLED
void process_menu(uint32_t choice); //selects option from menu
void print_menu(void); //prints menu to UART terminal
void UARTIntHandler(void); //manages interrupts for uart_echo
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count); //sends data via UART
void toggle_LED();

uint32_t ADCBuffer[3];
uint8_t ledToggle = 0;
tContext sContext; //used as pointer to draw graphics to OLED
tRectangle sRect; //used as pointer to draw graphics to OLED

int main()
{
  uint32_t select; //message to be set to select menu option
  //floating-point instructions can be used in interrupt handlers
  FPULazyStackingEnable();
  // Set the clocking to run directly from the crystal.
  SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                 SYSCTL_XTAL_16MHZ);

  CFAL96x64x16Init(); // Initialize the graphics context.
  GrContextInit(&sContext, &g_sCFAL96x64x16);
  GrContextFontSet(&sContext, g_psFontCm12);

  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);       //enable pin group A
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) //wait for peripheral
  {
  }
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);       //enable UART peripheral
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) //wait for peripheral
  {
  }
  //assign PA0 and PA1 to be UART pins
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  //
  //configure UART clock, set baud rate to 115,200,
  //8 bit message with 1 stop bit and no parity bit
  //
  UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), BAUD,
                     (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                      UART_CONFIG_PAR_NONE));

  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);       //enable group G pins
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG)) //wait for peripheral
  {
  }
  GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE,GPIO_PIN_2); //set PG2 for LED to output
  
  //enable pins and peripheral for ADC
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD))
  {
  }
  GPIOPinTypeADC(GPIO_PORTD_BASE,GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
  {
  }
  
  ADCSequenceDisable(ADC0_BASE,1); //stop sequence from working
  ADCSequenceConfigure(ADC0_BASE,1,ADC_TRIGGER_PROCESSOR,0); //set ADC trigger
  ADCSequenceStepConfigure(ADC0_BASE,1,0,ADC_CTL_CH6); //configure each
  ADCSequenceStepConfigure(ADC0_BASE,1,1,ADC_CTL_CH5); //ADC channel, 3 times
  ADCSequenceStepConfigure(ADC0_BASE,1,2,ADC_CTL_CH4 | ADC_CTL_IE | ADC_CTL_END);
  ADCSequenceEnable(ADC0_BASE,1); //enable to sequence

  splash_screen(); //do what they sound like
  print_menu();
  while(1)
  {
    toggle_LED('\0'); //indicate heartbeat of program
    while(UARTCharsAvail(UART0_BASE))
    { //wait for a UART message to be queued up the receive
      select = UARTCharGetNonBlocking(UART0_BASE); //retrieve message
      if (select != -1) //if message is valid
      {
        process_menu(select); //take users input to execute menu option
      }
    }
  }
}

void getADC0Value(void)
{
  ADCProcessorTrigger(ADC0_BASE,1); //trigger cause of ADC to get reading   
  while(!ADCIntStatus(ADC0_BASE,1,false)) //gets interrput status
  {
  }
  ADCIntClear(ADC0_BASE,1); //clear the interrupt
  ADCSequenceDataGet(ADC0_BASE,1,ADCBuffer); //retrive data from ADC
}

void graphADCValues(void)
{  
  uint32_t sizeRect; //each width size per graph
  uint32_t graphSize;
  uint32_t pictureSize;
  uint8_t watchFlag = 1; //en/disable user ability to graph
  int32_t quitGraph; //check to quit
  UARTSend((uint8_t *)"Press 'g' to stop\r\n",21);
  while(watchFlag == 1)
  {
    toggle_LED('\0'); //indicate heartbeat
    getADC0Value(); //retrieve ADC value
    pictureSize = (ADCBuffer[2]*95)/4095; //scale graph size
    graphSize = (ADCBuffer[1]*95)/4095;
    sizeRect = (ADCBuffer[0]*95)/4095;
    sRect.i16XMin = 0; //initialize 3 rectangles for graphs
    sRect.i16YMin = GrContextDpyHeightGet(&sContext) - 8;
    sRect.i16XMax = sizeRect;
    sRect.i16YMax = GrContextDpyHeightGet(&sContext);
    //clear screen
    GrContextForegroundSet(&sContext,ClrDarkSlateGray);
    GrStringDrawCentered(&sContext,"                              ", -1,  
                         GrContextDpyWidthGet(&sContext) / 2, 
                         GrContextDpyHeightGet(&sContext) - 4, 1);
    GrContextForegroundSet(&sContext,ClrRed); //print red bar for AIN4
    GrRectFill(&sContext,&sRect);
      
    sRect.i16XMin = 0;
    sRect.i16YMin = GrContextDpyHeightGet(&sContext) - 16;
    sRect.i16XMax = graphSize;
    sRect.i16YMax = GrContextDpyHeightGet(&sContext) - 8;
    //clear screen
    GrContextForegroundSet(&sContext,ClrDodgerBlue); 
    GrStringDrawCentered(&sContext,"                              ", -1,  
                         GrContextDpyWidthGet(&sContext) / 2, 
                         GrContextDpyHeightGet(&sContext) - 12, true);
    //print AIN5 graph in green
    GrContextForegroundSet(&sContext,ClrGreen);
    GrRectFill(&sContext,&sRect);
      
    sRect.i16XMin = 0;
    sRect.i16YMin = GrContextDpyHeightGet(&sContext) - 24;
    sRect.i16XMax = pictureSize;
    sRect.i16YMax = GrContextDpyHeightGet(&sContext) - 16;
    //clear screen
    GrContextForegroundSet(&sContext,ClrLightSteelBlue);
    GrStringDrawCentered(&sContext,"                              ", -1,  
                         GrContextDpyWidthGet(&sContext) / 2, 
                         GrContextDpyHeightGet(&sContext) - 20, true);
    //print AIN6 graph in blue
    GrContextForegroundSet(&sContext,ClrBlue);
    GrRectFill(&sContext,&sRect);
    while(UARTCharsAvail(UART0_BASE))
    { //retrieve flag to stop adjusting graphs
      quitGraph = UARTCharGetNonBlocking(UART0_BASE);
      if(quitGraph == 'g')
      {
        watchFlag = 0;
      }
    }
  }
}

void printADCValues(void)
{
  char ADCNumCh1[15]; //buffers for ADC values
  char ADCNumCh2[15];
  char ADCNumCh3[15];
  uint8_t controlFlag = 1; //en/disables ability to priunt ADC values
  int32_t stop; //check to disable
  UARTSend((uint8_t *)"Press 'n' to stop\r\n",21);
  while(controlFlag == 1)
  { //while ADC enabled for use
    toggle_LED('\0'); // show heartbeat
    getADC0Value(); //retrieve value in ADC
    sprintf(ADCNumCh1,"AIN4 %d",ADCBuffer[0]); //convert each ADC reading to
    sprintf(ADCNumCh2,"AIN5 %d",ADCBuffer[1]); //a string
    sprintf(ADCNumCh3,"AIN6 %d",ADCBuffer[2]);
    GrContextForegroundSet(&sContext,ClrMagenta); //make it pretty
    GrStringDrawCentered(&sContext,"                               ", -1, //clear screen
                         GrContextDpyWidthGet(&sContext) / 2, 4, true);
    GrStringDrawCentered(&sContext,ADCNumCh3, -1,  //flush buffer
                         GrContextDpyWidthGet(&sContext) / 2, 4, true);			 
    GrStringDrawCentered(&sContext,"                               ", -1, //clear screen
                         GrContextDpyWidthGet(&sContext) / 2, 16, true);
    GrStringDrawCentered(&sContext,ADCNumCh2, -1, //flush buffer
                         GrContextDpyWidthGet(&sContext) / 2, 16, true);
    GrStringDrawCentered(&sContext,"                               ", -1, //clear screen 
                         GrContextDpyWidthGet(&sContext) / 2, 28, true);
    GrStringDrawCentered(&sContext,ADCNumCh1, -1,  //flush buffer
                         GrContextDpyWidthGet(&sContext) / 2, 28, true);
    while(UARTCharsAvail(UART0_BASE))
    { //retrieve character to stop printing ADC values
      stop = UARTCharGetNonBlocking(UART0_BASE);
      if(stop == 'n')
      {
        UARTSend((uint8_t *)"You chose 'n'\r\n",17);
        controlFlag = 0;
      }
    }
  }
}

void UARTIntHandler(void)
{
  uint32_t ui32Status;
  ui32Status = UARTIntStatus(UART0_BASE, true); // Get the interrrupt status.
  UARTIntClear(UART0_BASE, ui32Status); // Clear the asserted interrupts.
  //although this is never called, program won't compile without it
  while(UARTCharsAvail(UART0_BASE))
  { // Loop while there are characters in the receive FIFO.
    // Read the next character from the UART and write it back to the UART.
    UARTCharPutNonBlocking(UART0_BASE,
                           UARTCharGetNonBlocking(UART0_BASE));
  }
}

void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
  while(ui32Count--)
  { //while characters are still queued up to be sent
    UARTCharPut(UART0_BASE, *pui8Buffer++); //send them
  }
}

void splash_screen(void)
{
  //draw a black rectangle on the entire OLED
  sRect.i16XMin = 0; 
  sRect.i16YMin = 0;
  sRect.i16XMax = GrContextDpyWidthGet(&sContext);
  sRect.i16YMax = GrContextDpyHeightGet(&sContext);
  GrContextForegroundSet(&sContext, ClrBlack);
  GrRectFill(&sContext, &sRect);
  //draw a red rectangle across the top of the OLED
  sRect.i16XMin = 0;
  sRect.i16YMin = 0;
  sRect.i16XMax = GrContextDpyWidthGet(&sContext);
  sRect.i16YMax = 8;
  GrContextForegroundSet(&sContext, ClrRed);
  GrRectFill(&sContext, &sRect);
  //draw a green rectangle on the right side of the OLED
  sRect.i16XMin = GrContextDpyWidthGet(&sContext) - 8;
  sRect.i16YMin = 0;
  sRect.i16XMax = GrContextDpyWidthGet(&sContext);
  sRect.i16YMax = GrContextDpyHeightGet(&sContext);
  GrContextForegroundSet(&sContext, ClrGreen);
  GrRectFill(&sContext, &sRect);
  //draw a blue rectangle on the bottom of the OLED
  sRect.i16XMin = 0;
  sRect.i16YMin = GrContextDpyHeightGet(&sContext) - 8;
  sRect.i16XMax = GrContextDpyWidthGet(&sContext);
  sRect.i16YMax = GrContextDpyHeightGet(&sContext);
  GrContextForegroundSet(&sContext, ClrBlue);
  GrRectFill(&sContext, &sRect);
  //draw a yellow rectangle on the left of the OLED
  sRect.i16XMin = 0;
  sRect.i16YMin = 0;
  sRect.i16XMax = 8;
  sRect.i16YMax = GrContextDpyHeightGet(&sContext);
  GrContextForegroundSet(&sContext, ClrYellow);
  GrRectFill(&sContext, &sRect);
  //print a message in the  middle of the screen
  GrContextForegroundSet(&sContext,ClrWhite);
  GrStringDrawCentered(&sContext, "Lab3 ADC", -1,
                       GrContextDpyWidthGet(&sContext) / 2,
                       (GrContextDpyHeightGet(&sContext) / 2),0);
  SysCtlDelay(TIME); //give splash screen time to show
  sRect.i16XMin = 0; //cover entire OLED with black rectangle
  sRect.i16YMin = 0;
  sRect.i16XMax = GrContextDpyWidthGet(&sContext);
  sRect.i16YMax = GrContextDpyHeightGet(&sContext);
  GrContextForegroundSet(&sContext, ClrBlack);
  GrRectFill(&sContext, &sRect);
}

void process_menu(uint32_t choice)
{ //evaluate user input to decide with function to call
  if(choice == 'g')
  { //draw ADC graphs
    UARTSend((uint8_t *)"You selected 'g'\r\n",20);
    graphADCValues();
  }
  else if(choice == 'q')
  { //quit program
    UARTSend((uint8_t *)"You selected 'q'\r\nGoodbye world\r\n",37);
    while(1); //enter infinite loop so nobody can do nothin'
  }
  else if(choice == 'm')
  { //print the menu
    UARTSend((uint8_t *)"You selected 'm'\r\n",20);
    print_menu();
  }
  else if(choice == 's')
  { //show the splash screen
	UARTSend((uint8_t *)"You selected 's'\r\n",20);
	splash_screen();
  }
  else if(choice == 'd')
  { //enable or disable the LED
    UARTSend((uint8_t *)"You selected 'd'\r\n",20);
    toggle_LED(choice);
  }
  else if(choice == 'n')
  { //print ADC numerical values
    UARTSend((uint8_t *)"You selected 'n'\r\n",20);
    printADCValues();
  }
  else if(choice == 'j')
  { //hide ADC graphs
	UARTSend((uint8_t *)"You selected 'j'\r\n",20);
	sRect.i16XMin = 0;
	sRect.i16YMin = 33;
	sRect.i16XMax = GrContextDpyWidthGet(&sContext);
	sRect.i16YMax = GrContextDpyHeightGet(&sContext);
	GrContextForegroundSet(&sContext,ClrBlack);
	GrRectFill(&sContext,&sRect);
  }
  else if(choice == 'h')
  { //hide ADC numerical values
	UARTSend((uint8_t *)"You selected 'h'\r\n",20);
	sRect.i16XMin = 0;
	sRect.i16YMin = 0;
	sRect.i16XMax = GrContextDpyWidthGet(&sContext);
	sRect.i16YMax = 32;
	GrContextForegroundSet(&sContext,ClrBlack);
	GrRectFill(&sContext,&sRect);
  }
  else
  {
    UARTSend((uint8_t *)"invalid input, dummy\r\n",24); //make user self-conscious
  }
}

void print_menu(void)
{ //does exactly what is sounds like
  UARTSend((uint8_t *)"Menu selection:\r\n",19);
  UARTSend((uint8_t *)"m - print this menu\r\n",23);
  UARTSend((uint8_t *)"q - quit the program\r\n",24);
  UARTSend((uint8_t *)"d - toggle LED\r\n",18);
  UARTSend((uint8_t *)"s - show splash screen\r\n",26);
  UARTSend((uint8_t *)"h - hide ADC values\r\n",23);
  UARTSend((uint8_t *)"n - print ADC values\r\n",24);
  UARTSend((uint8_t *)"g - Show ADC graphs\r\n",23);
  UARTSend((uint8_t *)"j - hide ADC graphs\r\n",23);
}

void toggle_LED(uint32_t flagLED)
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