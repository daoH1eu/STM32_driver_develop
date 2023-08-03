/**
 ******************************************************************************
 * @file   : core_cm4.h
 * @author : daoH1eu
 * @brief  : CMSIS Cortex-M4 Core Peripheral Access Layer Header File
 ******************************************************************************
 */

#ifndef INC_CORE_CM4_H
#define INC_CORE_CM4_H

/**
  \brief  Enum type to define the TRQ number based on the vector table.
 */
typedef enum IRQn
{
/* Processor Exceptions Numbers */
  NonMaskableInt_IRQn   = -14, /*  2 Non Maskable Interrupt */
  HardFault_IRQn    	  = -13, /*  3 HardFault Interrupt */
  MemoryManagement_IRQn	= -12, /*  4 Memory Management Interrupt */
  BusFault_IRQn     	  = -11, /*  5 Bus Fault Interrupt */
  UsageFault_IRQn   	  = -10, /*  6 Usage Fault Interrupt */
  SVCall_IRQn       	  =  -5, /* 11 SV Call Interrupt */
  DebugMonitor_IRQn     =  -4, /* 12 Debug Monitor Interrupt */
  PendSV_IRQn       	  =  -2, /* 14 Pend SV Interrupt */
  SysTick_IRQn      	  =  -1, /* 15 System Tick Interrupt */

/* Processor Interrupt Numbers */
  EXTI0_IRQn		  = 	6,
  EXTI1_IRQn   		= 	7,
  EXTI2_IRQn   		= 	8,
  EXTI3_IRQn   		= 	9,
  EXTI4_IRQn   		= 	10,
  EXTI9_5_IRQn   	= 	23,
  EXTI15_10_IRQn 	= 	40,

  /* Interrupts 10 .. 223 are left out */
  Interrupt224_IRQn     =   224
} IRQn_Type;

/******************************************************************************
 * 
 * ---------------  Processor Cortex-M4 Register Abstraction  -----------------
 *
******************************************************************************/

/* bypass waring in core_cm4.h*/
#define __CHECK_DEVICE_DEFINES    1
#define __FPU_PRESENT             1

/* reference CMSIS Version 4*/
#include "core_cm4.h"

#endif /* INC_CORE_CM4_H */
