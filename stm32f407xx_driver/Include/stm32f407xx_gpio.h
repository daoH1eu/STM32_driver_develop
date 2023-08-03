/**
 ******************************************************************************
 * @file           : stm32f407xx_gpio.h
 * @author         : daoH1eu
 * @brief          : GPIO driver header file
 ******************************************************************************
 */

#ifndef INC_STM32F407XX_GPIO_H_
#define INC_STM32F407XX_GPIO_H_

#include "stm32f407xx.h"

/* Clock enable for GPIO*/
#define RCC_AHB1ENR			(RCC->AHB1ENR)
#define GPIOA_PCLK_ENABLE 	(RCC_AHB1ENR |= (1<<0))
#define GPIOB_PCLK_ENABLE 	(RCC_AHB1ENR |= (1<<1))
#define GPIOC_PCLK_ENABLE 	(RCC_AHB1ENR |= (1<<2))
#define GPIOD_PCLK_ENABLE 	(RCC_AHB1ENR |= (1<<3))
#define GPIOE_PCLK_ENABLE 	(RCC_AHB1ENR |= (1<<4))

/* Clock disable for GPIO*/
#define GPIOA_PCLK_DISABLE 	(RCC_AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DISABLE 	(RCC_AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DISABLE 	(RCC_AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DISABLE 	(RCC_AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DISABLE 	(RCC_AHB1ENR &= ~(1<<4))

#define GPIOA_RESET_REG		do{ /*Set bit to reset*/(RCC->APB1RSTR |= 1<<0); /*Clear bit set*/(RCC->APB1RSTR &= ~(1<<0)); } while (0)
#define GPIOB_RESET_REG		do{ /*Set bit to reset*/(RCC->APB1RSTR |= 1<<1); /*Clear bit set*/(RCC->APB1RSTR &= ~(1<<1)); } while (0)
#define GPIOC_RESET_REG		do{ /*Set bit to reset*/(RCC->APB1RSTR |= 1<<2); /*Clear bit set*/(RCC->APB1RSTR &= ~(1<<2)); } while (0)
#define GPIOD_RESET_REG		do{ /*Set bit to reset*/(RCC->APB1RSTR |= 1<<3); /*Clear bit set*/(RCC->APB1RSTR &= ~(1<<3)); } while (0)
#define GPIOE_RESET_REG		do{ /*Set bit to reset*/(RCC->APB1RSTR |= 1<<4); /*Clear bit set*/(RCC->APB1RSTR &= ~(1<<4)); } while (0)

/* Clock enable for SYSCFG*/
#define SYSCFG_PCLK_ENABLE 	(RCC->APB2ENR |= (1<<14))

/* Return the corresponding port code to config SYSCFG_EXTICR (x is GPIO base address) */
#define GPIO_TO_EXTI_PORT_CODE(x)	(  	(x == GPIOA_BASEADDR) ? 0 :\
										(x == GPIOB_BASEADDR) ? 1 :\
										(x == GPIOC_BASEADDR) ? 2 :\
										(x == GPIOD_BASEADDR) ? 3 :\
										(x == GPIOE_BASEADDR) ? 4 : 0)

/* Clear pending register after trigger an interrupt (x is corresponding EXTI line)*/
#define EXTI_PENDING_BIT_CLEAR(x)	(EXTI->EXTI_PR |= 1<<(x))

/* GPIO Pin Number */
#define AVAILABLE_PIN	15 /*Avaliable pins of each port*/
#define GPIO_PIN_0		0
#define GPIO_PIN_1		1
#define GPIO_PIN_2		2
#define GPIO_PIN_3		3
#define GPIO_PIN_4		4
#define GPIO_PIN_5		5
#define GPIO_PIN_6		6
#define GPIO_PIN_7		7
#define GPIO_PIN_8		8
#define GPIO_PIN_9		9
#define GPIO_PIN_10		10
#define GPIO_PIN_11 	11
#define GPIO_PIN_12 	12
#define GPIO_PIN_13		13
#define GPIO_PIN_14		14
#define GPIO_PIN_15		15

/* GPIO port mode */
#define INPUT_MODE			0	/* 00: Input (reset state) */
#define OUTPUT_MODE			1	/* 01: General purpose output mode */
#define ALT_MODE			2	/* 10: Alternate function mode */
#define ANALOG_MODE			3	/* 11: Analog mode */
#define INTR_RISING_MODE	4	/* Interrupt with rising detection */
#define INTR_FALLING_MODE	5	/* Interrupt with falling detection */
#define INTR_BOTH_RF_MODE		6	/* Interrupt with both rising and falling detection */

/* GPIO output type */
#define PUSHPULL 		0
#define OPENDRAIN		1

/* GPIO port output speed */
#define LOWSPD			0	/* 00: Low speed */
#define MEDIUMSPD		1	/* 01: Medium speed */
#define HIGHSPD			2	/* 10: High speed */
#define VERYHIGHSPD		3	/* 11: Very high speed */

/* GPIO port pull-up,pull-down */
#define NOPULL			0
#define PULLUP			1
#define PULLDOWN		2

/**
  \brief  Structure type for configuration GPIO.
 */
typedef struct GPIO_Config{
	uint8_t pinNumber;	/* GPIO Pin Number*/
	uint8_t pinMode; 	/* GPIO port mode*/
	uint8_t pinSpeed;	/* GPIO port output speed*/
	uint8_t ouputType;	/* GPIO output type*/
	uint8_t pullUpDown; /* GPIO port pull-up, pull-down*/
	uint8_t altMode;
}GPIO_Config_t;

/**
  \brief  Structure type for configuration GPIO.
 */
typedef struct GPIO_Handle{
	GPIOx_RegDef_t *pGPIOx; 	/* GPIO register (Pointer to GPIOx address)*/
	GPIO_Config_t pinConfig; 	/* GPIO configuration (pin number, mode...)*/
}GPIO_Handle_t;

/*******************************************************************************
 * 
 * ----------------------------  GPIO Functions  ------------------------------ 
 *
******************************************************************************/
/**
	* GPIO_ClockControl
	* @brief  -	Enable/Disable clock for a port (RCC register)
	* @param  -	*pGPIOx : GPIO Port Base address marco (GPIOA -> GPIOE marco)
	* @param  -	IsEnable : ENABLE(1) or DISABLE(0) marco
	* @retval -	void
	*/
void GPIO_ClockControl(GPIOx_RegDef_t *pGPIOx, uint8_t IsEnable);

/**
	* GPIO_Init
	* @brief  -	Config specific mode for a pin (mode, speed, output type, input, altmode etc..)
	* @param  -	*pGPIOHandle : specify GPIO Port Base address
	* @retval -	void
	*/
void GPIO_Init(GPIO_Handle_t* pGPIOHandle);

/**
	* GPIO_DeInit
	* @brief  -	Reset peripheral using RCC AHB1 peripheral reset register
	* @param  -	*pGPIOHandle : specify GPIO Port Base address
	* @retval -	void
	*/
void GPIO_DeInit(GPIOx_RegDef_t *pGPIOx);

/**
	* GPIO_ReadFromPin
	* @brief  -	Read input value of a pin
	* @param  -	*pGPIOx : point to GPIO Port Base address
	* @param  -	pinNumber : specify a pin number	
	* @retval -	value of the corresponding I/O port (return a bit of GPIOx_IDR - 1 or 0 only)
	*/
uint8_t GPIO_ReadFromPin(GPIOx_RegDef_t *pGPIOx, uint8_t pinNumber);

/**
	* GPIO_ReadFromPort
	* @brief  -	Read input value of all pin in a port
	* @param  -	*pGPIOx : point to GPIO Port Base address
	* @param  -	pinNumber : specify a pin number	
	* @retval -	value of the corresponding port (return entire GPIOx_IDR - 16 bit)
	*/
uint16_t GPIO_ReadFromPort(GPIOx_RegDef_t *pGPIOx);

/**
	* GPIO_WriteToPin
	* @brief  -	Set/reset bit of a pin
	* @param  -	*pGPIOx : point to GPIO Port Base address
	* @param  -	pinNumber : specify a pin number	
	* @param  -	inVal : SET/RESET value		
	* @retval -	void
	*/
void GPIO_WriteToPin(GPIOx_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t inVal);

/**
	* GPIO_WriteToPort
	* @brief  -	Set/reset bit of entire a port
	* @param  -	*pGPIOx : point to GPIO Port Base address	
	* @param  -	inVal : SET/RESET value		
	* @retval -	void
	*/
void GPIO_WriteToPort(GPIOx_RegDef_t *pGPIOx, uint8_t inVal);

/**
	* GPIO_TogglePin
	* @brief  -	Toggle bit a pin
	* @param  -	*pGPIOx : point to GPIO Port Base address
	* @param  -	pinNumber : specify a pin number	
	* @retval -	void
	*/
void GPIO_TogglePin(GPIOx_RegDef_t *pGPIOx, uint8_t pinNumber);

/**
	* GPIO_IRQConfig
	* @brief  -	Enable or Disable interrupt for a GPIO pin
	* @param  -	IRQnumber : refer to the vector table
	* @retval -	void
	*/
void GPIO_IRQConfig(IRQn_Type IRQnumber, uint8_t IsEnable);

// void GPIO_IRQHandling (IRQn_Type IRQnumber);

void delay(uint32_t delay);

#endif /* INC_STM32F407XX_GPIO_H_ */
