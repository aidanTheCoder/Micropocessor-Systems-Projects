#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
  name timer1_assembly
  section .text:CODE
  extern IntSvcCount
  public isr_asm_start
isr_asm_start
;clear the TIMER1 interrupt flag
  push {lr}            ;push link register onto stack
  mov r1,#1            ;place a value in the reguster to indicate the interrupt has been serviced
  mov32 r0,#0x40031000 ;move the address of TIMER1 into R0
  str r1, [r0,#0x24]   ; the value in R1, assign it to the address in R0 + 0x24
;increment the interrupt service count
  ldr r0, =IntSvcCount ;place the address of the global ISR counter in r0
  ldr r1, [r0]         ;defreference r0 to obtain the value inside and put it in r1
  add r1, r1, #1       ;increment the value of the count being stored in R1
  str r1, [r0]	       ;store the new count from R1 into R0 to assign a value to intSvcCount
  pop {pc}             ;return the program counter from the stack to countinue execution
  end		       ;stop