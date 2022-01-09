/****************************************************************
* Aidan Rutherford
* Send a menu via UART to prompt user for a menu option
* options include printing the menu, print number of while(1)
  loop executions, printing the number of received and sent
  messages, clearing an flooding the UART terminal, and toggling
  between the LED and the buttons

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

#define BAUD 115200 //baud rate for configuring UART
#define HOLD 2000000//duration of the duty cycle for LED and flood
#define CLEAR 100 //number of time times a new line is printed to clear UART
#define MENU 245 //size of the menu

#ifdef DEBUG //error routine if driver library encounters error
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

void clear_OLED(void); //clears OLED of any data
void splash_screen(void); //makes cool dislay on OLED
void process_menu(uint32_t choice); //selects option from menu
void print_menu(void); //prints menu to UART terminal
void print_loop_count(void); //prints number of times while(1) loop executes
void print_UART_count(void); //prints number of transmitted or received messages
void toggle_LED(void); //allows user to turn LED on and off
void clear_rx_terminal(void); //clears UART terminal
void toggle_flood(void); //prints a single character to UART temrinal on timer
void toggle_buttons(void); //displays which button ad how many times it was pushed
void UARTIntHandler(void); //manages interrupts for uart_echo
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count); //sends data via UART

uint32_t loopCount = 0; //counts total executions of while(1) loop
uint32_t xmitCount = 0; //counts total number of messages sent
uint32_t recvCount = 0; //counts total number of messages received

tContext sContext; //used as pointer to draw graphics to OLED
tRectangle sRect; //used as pointer to draw graphics to OLED
int main()
{

  //floating-point instructions can be used in interrupt handlers
  FPULazyStackingEnable();
  // Set the clocking to run directly from the crystal.
  SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                 SYSCTL_XTAL_16MHZ);

  CFAL96x64x16Init(); //initialize display driver
  GrContextInit(&sContext, &g_sCFAL96x64x16); //initialize the graphics context

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

  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);        //enable group M pins
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)) //wait for peripheral
  {
  }
  GPIOPinTypeGPIOInput(GPIO_PORTM_BASE,GPIO_PIN_0); //configuring M pins for buttons
  GPIOPinTypeGPIOInput(GPIO_PORTM_BASE,GPIO_PIN_1); //to be inputs
  GPIOPinTypeGPIOInput(GPIO_PORTM_BASE,GPIO_PIN_2); //define button press logic to be
  GPIOPinTypeGPIOInput(GPIO_PORTM_BASE,GPIO_PIN_3); //weak pull resistor
  GPIOPinTypeGPIOInput(GPIO_PORTM_BASE,GPIO_PIN_4); //pressed button = 1 released = 0
  GPIOPadConfigSet(GPIO_PORTM_BASE,GPIO_PIN_0,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
  GPIOPadConfigSet(GPIO_PORTM_BASE,GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
  GPIOPadConfigSet(GPIO_PORTM_BASE,GPIO_PIN_2,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
  GPIOPadConfigSet(GPIO_PORTM_BASE,GPIO_PIN_3,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
  GPIOPadConfigSet(GPIO_PORTM_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);

  uint32_t select; //message to be set to select menu option
  volatile uint32_t dutyCycle; //counter for LED duty cycle

  splash_screen(); // draw neat graphic to screen
  print_menu(); //print menu options to UART
  while(1)
  { //hang around and send some messages
    GPIOPinWrite(GPIO_PORTG_BASE,GPIO_PIN_2,GPIO_PIN_2); //turn on LED
    SysCtlDelay(HOLD);
    GPIOPinWrite(GPIO_PORTG_BASE,GPIO_PIN_2,0); //turn off LED
    SysCtlDelay(HOLD);
    loopCount++; //increment number of times loop executes
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

void clear_OLED(void)
{ //draw blank rectangles over text in the middle of the OLED
  GrStringDrawCentered(&sContext, "                              ", -1,
                       GrContextDpyWidthGet(&sContext) / 2, 20, false);
  GrStringDrawCentered(&sContext,"                               ", -1,
                       GrContextDpyWidthGet(&sContext) / 2, 30, false);
}

void splash_screen(void)
{
  sRect.i16XMin = 0; //print a red background on the entire OLED
  sRect.i16YMin = 0;
  sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
  sRect.i16YMax = GrContextDpyHeightGet(&sContext) - 1;
  GrContextForegroundSet(&sContext,ClrDarkRed);
  GrRectFill(&sContext,&sRect);

  sRect.i16XMin = 0; //print a blue banner with a catchy title on the
  sRect.i16YMin = 0; //top of the OLED
  sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
  sRect.i16YMax = 9;
  GrContextForegroundSet(&sContext, ClrDarkBlue);
  GrRectFill(&sContext, &sRect);
  GrContextForegroundSet(&sContext, ClrWhite);
  GrContextFontSet(&sContext, g_psFontFixed6x8);
  GrStringDrawCentered(&sContext, "Lab 2 UART Menu", -1,
                       GrContextDpyWidthGet(&sContext) / 2, 4, 0);
}

void process_menu(uint32_t choice)
{ //evaluate user input to decide with function to call
  if(choice == 'M')
  {
    recvCount++; //indicate received message
        UARTSend((uint8_t *)choice,1);
    print_menu(); //display menu to terminal
  }
  else if(choice == 'Q')
  {
    recvCount++; //indicate received message
        UARTSend((uint8_t *)choice,1);
        UARTSend((uint8_t *)"Goodbye world\r\n",17);
    while(1) //enter infinite loop so nobody can do nothin'
    {
    }
  }
  else if(choice == 'L')
  {
    recvCount++; //indicate received message
        UARTSend((uint8_t *)choice,1);
    clear_OLED(); //wipe the OLED
    print_loop_count(); //print number of times while(1) executes
  }
  else if(choice == 'U')
  {
    recvCount++; //indicate received message
        UARTSend((uint8_t *)choice,1);
    clear_OLED(); //wipe the OLED
    print_UART_count(); //prints number of xmitted or recved messages
  }
  else if(choice == 'B')
  {
    recvCount++; //indicate received message
    clear_OLED(); //wipe the OLED
        UARTSend((uint8_t *)choice,1);
    toggle_buttons(); //allow user to see button press info
  }
  else if(choice == 'D')
  {
    recvCount++; //indicate received message
        UARTSend((uint8_t *)choice,1);
    toggle_LED(); // allows user ot toggle LED
  }
  else if(choice == 'C')
  {
    recvCount++; //indicate received message
        UARTSend((uint8_t *)choice,1);
    clear_rx_terminal(); //wipes Rx terminal clean
  }
  else if(choice == 'F')
  {
    recvCount++; //indicate received message
        UARTSend((uint8_t *)choice,1);
    toggle_flood(); //prints character to terminal on a timer
  }
  else
  {
    recvCount++; //indicate received message
    UARTSend((uint8_t *)"invalid input, dummy\r\n",24); //make user self-conscious
    xmitCount += 24; //record messages sent
  }
}
void print_menu(void)
{ //print user menu
  UARTSend((uint8_t *)"Menu selection:\r\n",19);
  UARTSend((uint8_t *)"M - print this menu\r\n",23);
  UARTSend((uint8_t *)"Q - quit the program\r\n",24);
  UARTSend((uint8_t *)"L - toggle loop count display\r\n",33);
  UARTSend((uint8_t *)"U - toggle UART recv/xmit display\r\n",37);
  UARTSend((uint8_t *)"B - toggle button display\r\n",29);
  UARTSend((uint8_t *)"D - toggle flashing LED\r\n",27);
  UARTSend((uint8_t *)"C - clear terminal menu\r\n",27);
  UARTSend((uint8_t *)"F - flood the terminal\r\n",26);
  xmitCount += MENU; //record messages sent
}

void print_loop_count(void)
{
  char loopBuffer[100]; //buffer for integer
  //parse integer to string
  snprintf(loopBuffer,sizeof(loopBuffer),"loop count %d",loopCount); 
  GrStringDrawCentered(&sContext,loopBuffer, -1, //print value to OLED
                       GrContextDpyWidthGet(&sContext) / 2, 20, true);
}

void print_UART_count(void)
{
  uint32_t decision; //type of data to display
  char xmitBuffer[50]; //integer buffers
  char recvBuffer[50];
  UARTSend((uint8_t *)"xmit(x) or recv(R)\r\n",22); //prompt user for data type
  xmitCount += 22; //record messages sent
    decision = UARTCharGet(UART0_BASE);
    if(decision == 'X')
    {
    recvCount++; //indicate received message
	//parse integer to string
    snprintf(xmitBuffer,sizeof(xmitBuffer),"xmit count %d",xmitCount);
    GrStringDrawCentered(&sContext,xmitBuffer, -1, //show number of xmits
                       GrContextDpyWidthGet(&sContext) / 2, 20, true);
    }
    else if(decision == 'R')
    {
    recvCount++; //indicate received message
	//parse integer to string
    snprintf(recvBuffer,sizeof(recvBuffer),"recv count %d",recvCount); 
    GrStringDrawCentered(&sContext,recvBuffer, -1, //show number of recvs
                       GrContextDpyWidthGet(&sContext) / 2, 20, true);
    }
    else
    {
    recvCount++; //indicate received message
    UARTSend((uint8_t *)"invalid input, dummy\r\n",24); //make user self-conscious
    xmitCount += 24; //record messages sent
    }
}

void toggle_LED(void)
{
  if(GPIOPinRead(GPIO_PORTG_BASE,GPIO_PIN_2))
  { //if LED is on
    GPIOPinWrite(GPIO_PORTG_BASE,GPIO_PIN_2,0); //turn off LED
    SysCtlDelay(HOLD); //run duty cycle to notice blink
  }
  else
  { //if LED is off
    GPIOPinWrite(GPIO_PORTG_BASE,GPIO_PIN_2,GPIO_PIN_2); //turn on LED
    SysCtlDelay(HOLD); //run duty cycle to notice blink
  }
}

void clear_rx_terminal(void)
{ //clear UART terminal
  volatile uint32_t clear; 
  for(clear = 0; clear < CLEAR; clear++)
  { //print 100 blank lines to clear screen
    UARTSend((uint8_t *)"\r\n",4); //make space
    xmitCount += 4; //record messages sent
  }
}

void toggle_flood()
{
    uint32_t decision;
    uint8_t floodFlag = 1;
    while(floodFlag == 1)
    { //while charcetr to stop flood is not entered
        //UARTSend((uint8_t *)"$ ",2); //flood screen with unique symbol
        print_menu();
        xmitCount++; //record messages sent
        SysCtlDelay(HOLD); //prevent mega flood of doom
        while(UARTCharsAvail(UART0_BASE))
        { //poll the user to stop flood
            decision = UARTCharGetNonBlocking(UART0_BASE);
            recvCount++; //indicate received message
        }
        if(decision == 'F')
        { //check to stop
            floodFlag = 0;
        }
    }
}

void toggle_buttons(void)
{
    uint8_t buttonFlag = 1;
    uint32_t stopButtons;
    uint32_t button1 = 0; //button counters
    uint32_t button2 = 0;
    uint32_t button3 = 0;
    uint32_t button4 = 0;
    uint32_t button5 = 0;
    char b1Buff[50]; //button buffers
    char b2Buff[50];
    char b3Buff[50];
    char b4Buff[50];
    char b5Buff[50];
    UARTSend((uint8_t *)"Press 'B' to return to menu\r\n",31);
    xmitCount += 31;
    while(buttonFlag == 1)
    { //while user wants wants to push buttons
        //clear screen
        GrStringDrawCentered(&sContext, "                              ", -1,
                             GrContextDpyWidthGet(&sContext) / 2, 30, 1);
        //read which button is pushed, display its action and incrment count
        if(!GPIOPinRead(GPIO_PORTM_BASE,GPIO_PIN_0))
        {
            button1++; //count button press
            sprintf(b1Buff,"Up %d",button1); //place counter in buffer
            GrStringDrawCentered(&sContext,b1Buff, -1,  //flush buffer
                                 GrContextDpyWidthGet(&sContext) / 2, 30, true);
            SysCtlDelay(HOLD); //used to slow button presses
        }
        if(!GPIOPinRead(GPIO_PORTM_BASE,GPIO_PIN_1))
        {
            button2++; //count button press
            sprintf(b2Buff,"Down %d",button2); //place counter in buffer
            GrStringDrawCentered(&sContext,b2Buff, -1,  //flush buffer
                                     GrContextDpyWidthGet(&sContext) / 2, 30, false);
            SysCtlDelay(HOLD);
        }
        if(!GPIOPinRead(GPIO_PORTM_BASE,GPIO_PIN_2))
        {
            button3++; //count button press
            sprintf(b3Buff,"Left %d",button3); //place counter in buffer
            GrStringDrawCentered(&sContext,b3Buff, -1, //flush buffer
                                     GrContextDpyWidthGet(&sContext) / 2, 30, false);
            SysCtlDelay(HOLD);
        }
        if(!GPIOPinRead(GPIO_PORTM_BASE,GPIO_PIN_3))
        {
            button4++; //count button press
            sprintf(b4Buff,"Right %d",button4); //place counter in buffer
            GrStringDrawCentered(&sContext,b4Buff, -1,  //flush buffer
                                     GrContextDpyWidthGet(&sContext) / 2, 30, false);
            SysCtlDelay(HOLD);
        }
        if(!GPIOPinRead(GPIO_PORTM_BASE,GPIO_PIN_4))
        {
            button5++; //count button press
            sprintf(b5Buff,"Select %d",button5); //place counter in buffer
            GrStringDrawCentered(&sContext,b5Buff, -1,  //flush buffer
                                     GrContextDpyWidthGet(&sContext) / 2, 30, false);
            SysCtlDelay(HOLD);
        }
        while(UARTCharsAvail(UART0_BASE))
        { //poll user to stop buttons
            stopButtons = UARTCharGetNonBlocking(UART0_BASE);
            recvCount++; //indicate received message
        }
        if(stopButtons == 'B')
        {
            buttonFlag = 0;
        }
    }
}

