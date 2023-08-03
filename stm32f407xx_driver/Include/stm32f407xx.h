/**
 ******************************************************************************
 * @file   : stm32f407xx.h
 * @author : daoH1eu
 * @brief  : Peripherals Abstraction header file
 ******************************************************************************
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include "cortex_m4.h"

/******************************************************************************
 * 
 * -------------------  Peripherals Register Abstraction  ---------------------
 *
******************************************************************************/
/* Memomry Base Address */
#define FLASH_BASEADDR		0x08000000u
#define SRAM1_BASEADDR		0x20000000u
#define SRAM2_BASEADDR		0x2001C000u
#define ROM_BASEADDR		0x1FFF0000u
#define SRAM				SRAM1_BASEADDR

/* Bus Base Address */
#define APB1_BASEADDR		0x40000000u
#define APB2_BASEADDR		0x40010000u
#define AHB1_BASEADDR		0x40020000u
#define AHB2_BASEADDR		0x50000000u

/* APB1 peripherals */
#define TIM2_BASEADDR		APB1_BASEADDR
#define TIM3_BASEADDR		(APB1_BASEADDR + 0x0400u)
#define TIM4_BASEADDR		(APB1_BASEADDR + 0x0800u)
#define TIM5_BASEADDR		(APB1_BASEADDR + 0x0C00u)
#define TIM6_BASEADDR		(APB1_BASEADDR + 0x1000u)
#define TIM7_BASEADDR		(APB1_BASEADDR + 0x1400u)
#define TIM12_BASEADDR		(APB1_BASEADDR + 0x1800u)
#define TIM13_BASEADDR		(APB1_BASEADDR + 0x1C00u)
#define TIM14_BASEADDR		(APB1_BASEADDR + 0x2000u)
#define I2C2_BASEADDR		(APB1_BASEADDR + 0x5800u)
#define I2C3_BASEADDR		(APB1_BASEADDR + 0x5C00u)
#define I2C1_BASEADDR		(APB1_BASEADDR + 0x5400u)
#define UART4_BASEADDR		(APB1_BASEADDR + 0x4C00u)
#define UART5_BASEADDR		(APB1_BASEADDR + 0x5000u)
#define USART2_BASEADDR		(APB1_BASEADDR + 0x4400u)
#define USART3_BASEADDR		(APB1_BASEADDR + 0x4800u)
#define SPI3_BASEADDR		(APB1_BASEADDR + 0x3C00u)
#define SPI2_BASEADDR		(APB1_BASEADDR + 0x3800u)

/* APB2 peripherals */
#define SPI1_BASEADDR		(APB2_BASEADDR + 0x3000u)
#define USART1_BASEADDR		(APB2_BASEADDR + 0x1000u)
#define USART6_BASEADDR		(APB2_BASEADDR + 0x1400u)
#define EXTI_BASEADDR		(APB2_BASEADDR + 0x3C00u)
#define SYSCFG_BASEADDR		(APB2_BASEADDR + 0x3800u)

/* AHB1 peripherals */
#define GPIOA_BASEADDR		AHB1_BASEADDR
#define GPIOB_BASEADDR		(AHB1_BASEADDR + 0x0400u)
#define GPIOC_BASEADDR		(AHB1_BASEADDR + 0x00800u)
#define GPIOD_BASEADDR		(AHB1_BASEADDR + 0x0C00u)
#define GPIOE_BASEADDR		(AHB1_BASEADDR + 0x1000u) 
#define RCC_BASEADDR		(AHB1_BASEADDR + 0x3800u)

/* AHB2 peripherals are leftout */
//to be updated

/* AHB3 peripherals are leftout*/
//to be updated

/**
  \brief  Structure type to access GPIO registers.
 */
typedef struct GPIOx_Register{
	volatile uint32_t MODER; 		/*0x00 GPIO port mode register*/
	volatile uint32_t OTYPER;		/*0x04 GPIO port output type register*/
	volatile uint32_t OSPEEDR;		/*0x08 GPIO port output speed register*/
	volatile uint32_t PUPDR;		/*0x0C GPIO port pull-up/pull-down register*/
	volatile uint32_t IDR;			/*0x10 GPIO port input data register*/
	volatile uint32_t ODR;			/*0x14 GPIO port output data register*/
	volatile uint32_t BSRR;			/*0x18 GPIO port bit set/reset register*/
	volatile uint32_t LCKR;			/*0x1C GPIO port configuration lock register*/
	volatile uint32_t AFRL;			/*0x20 GPIO alternate function low register*/
	volatile uint32_t AFRH;			/*0x24 GPIO alternate function high register*/
}GPIOx_RegDef_t;

#define GPIOA	((GPIOx_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB	((GPIOx_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC	((GPIOx_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD	((GPIOx_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE	((GPIOx_RegDef_t*) GPIOE_BASEADDR)

/**
  \brief  Structure type to access RCC registers.
 */
typedef struct  RCC_Register{
	volatile uint32_t CR;				/*0x00 RCC clock control register*/
	volatile uint32_t PLLCFGR;			/*0x04 RCC PLL configuration register*/
	volatile uint32_t CFGR;				/*0x08 RCC clock configuration register*/
	volatile uint32_t CIR;				/*0x0C RCC clock interrupt register*/
	volatile uint32_t AHB1RSTR;			/*0x10 RCC AHB1 peripheral reset register*/
	volatile uint32_t AHB2RSTR;			/*0x14 RCC AHB2 peripheral reset register*/
	volatile uint32_t AHB3RSTR;			/*0x18 RCC AHB3 peripheral reset register*/
	volatile uint32_t Reserved_1;		/*0x1C*/
	volatile uint32_t APB1RSTR;			/*0x20 RCC APB1 peripheral reset register*/
	volatile uint32_t APB2RSTR;			/*0x24 RCC APB2 peripheral reset register*/
	volatile uint32_t Reserved_2;		/*0x28*/
	volatile uint32_t Reserved_3;		/*0x2C*/
	volatile uint32_t AHB1ENR;			/*0x30 RCC AHB1 peripheral clock enable register*/
	volatile uint32_t AHB2ENR;			/*0x34 RCC AHB2 peripheral clock enable register*/
	volatile uint32_t AHB3ENR;			/*0x38 RCC AHB3 peripheral clock enable register*/
	volatile uint32_t Reserved_4;		/*0x3C*/
	volatile uint32_t APB1ENR;			/*0x40 RCC APB1 peripheral clock enable register*/
	volatile uint32_t APB2ENR;			/*0x44 RCC APB2 peripheral clock enable register*/
	volatile uint32_t Reserved_5;		/*0x48 */
	volatile uint32_t Reserved_6;		/*0x4C */
	volatile uint32_t AHB1LPENR;		/*0x50 RCC AHB1 peripheral clock enable in low power mode register*/
	volatile uint32_t AHB2LPENR;		/*0x54 RCC AHB2 peripheral clock enable in low power mode register*/
	volatile uint32_t AHB3LPENR;		/*0x58 RCC AHB3 peripheral clock enable in low power mode register*/
	volatile uint32_t Reserved_7;		/*0x5C */
	volatile uint32_t APB1LPENR;		/*0x60 RCC APB1 peripheral clock enable in low power mode register*/
	volatile uint32_t APB2LPENR;		/*0x64 RCC APB2 peripheral clock enable in low power mode register*/
	volatile uint32_t Reserved_8;		/*0x68 */
	volatile uint32_t Reserved_9;		/*0x6C */
	volatile uint32_t BDCR;				/*0x70 RCC Backup domain control register*/
	volatile uint32_t CSR;				/*0x74 RCC clock control & status register*/
	volatile uint32_t Reserved_10;		/*0x78 */
	volatile uint32_t Reserved_11;		/*0x7C */
	volatile uint32_t SSCGR;			/*0x80 RCC spread spectrum clock generation register*/
	volatile uint32_t PLLI2SCFGR;		/*0x84 RCC PLLI2S configuration register*/
}RCC_RegDef_t;

#define RCC		((RCC_RegDef_t*) RCC_BASEADDR)

/**
  \brief  Structure type to access the System configuration controller (SYSCFG).
	It manage the external interrupt line connection to the GPIOs.
 */
typedef struct 
{
	volatile uint32_t SYSCFG_MEMRMP;	/* SYSCFG memory remap register */
	volatile uint32_t SYSCFG_PMC;		/* SYSCFG peripheral mode configuration register */
	volatile uint32_t SYSCFG_EXTICR[3];	/* SYSCFG external interrupt configuration register 1 to 4 */
	volatile uint32_t SYSCFG_CMPCR;		/* SYSCFG Compensation cell control register */
}SYSCFG_RegDef_t;

#define SYSCFG		((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

/**
  \brief  Structure type to access External interrupt/event controller (EXTI).
 */
typedef struct 
{
	volatile uint32_t EXTI_IMR;		/* Interrupt mask register (EXTI_IMR) */
	volatile uint32_t EXTI_EMR;		/* Event mask register (EXTI_EMR) */
	volatile uint32_t EXTI_RTSR;	/* Rising trigger selection register (EXTI_RTSR) */
	volatile uint32_t EXTI_FTSR;	/* Falling trigger selection register (EXTI_FTSR) */
	volatile uint32_t EXTI_SWIER;	/* Software interrupt event register (EXTI_SWIER) */
	volatile uint32_t EXTI_PR;		/* Pending register (EXTI_PR) */
}EXTI_RegDef_t;

#define EXTI		((EXTI_RegDef_t*) EXTI_BASEADDR)


/* -------------------  General macro  ------------------- */
#define ENABLE		1
#define DISABLE		0
#define SET			ENABLE
#define RESET		DISABLE

#include "stm32f407xx_gpio.h"

#endif /* INC_STM32F407XX_H_ */
