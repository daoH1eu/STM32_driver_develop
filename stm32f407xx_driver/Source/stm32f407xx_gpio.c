/**
 ******************************************************************************
 * @file           : stm32f407xx_gpio.c
 * @author         : daoH1eu
 * @brief          : GPIO driver API
 ******************************************************************************
 */

#include "stm32f407xx_gpio.h"

/**
	* GPIO_ClockControl
	* @brief  -	Enable/Disable clock for a port (RCC register)
	* @param  -	*pGPIOx : GPIO Port Base address marco (GPIOA -> GPIOE marco)
	* @param  -	IsEnable : ENABLE(1) or DISABLE(0) marco
	* @retval -	void
	*/
void GPIO_ClockControl(GPIOx_RegDef_t *pGPIOx, uint8_t IsEnable)
{
	/*casting to integer value for base address comparsion*/
	uint32_t baseAddr = (uint32_t) pGPIOx; 

	switch (baseAddr)
	{
	case GPIOA_BASEADDR:
		(DISABLE == IsEnable) ? GPIOA_PCLK_DISABLE : GPIOA_PCLK_ENABLE;
		break;
	case GPIOB_BASEADDR:
		(DISABLE == IsEnable) ? GPIOB_PCLK_DISABLE : GPIOB_PCLK_ENABLE;
		break;
	case GPIOC_BASEADDR:
		(DISABLE == IsEnable) ? GPIOC_PCLK_DISABLE : GPIOC_PCLK_ENABLE;
		break;
	case GPIOD_BASEADDR:
		(DISABLE == IsEnable) ? GPIOD_PCLK_DISABLE : GPIOD_PCLK_ENABLE;
		break;
	case GPIOE_BASEADDR:
		(DISABLE == IsEnable) ? GPIOE_PCLK_DISABLE : GPIOE_PCLK_ENABLE;
		break;
	}
}

/**
	* GPIO_Init
	* @brief  -	Config specific mode for a pin (mode, speed, output type, input, altmode etc..)
	* @param  -	*pGPIOHandle : specify GPIO Port Base address & Pin number
	* @retval -	void
	*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t modeRegVal = 0;   	/*mode register value*/
	uint32_t spdRegVal = 0;	   	/*speed register value*/
	uint32_t pupdRegVal = 0;  	/*pull-up/pull-down register*/
	uint32_t outputRegVal = 0; 	/*output type register*/

	/*1. Config modes*/
	if (ANALOG_MODE == pGPIOHandle->pinConfig.pinMode)
	{
		/*Analog mode*/
		modeRegVal = (pGPIOHandle->pinConfig.pinMode << (2 * pGPIOHandle->pinConfig.pinNumber));
		/*Clear bits fiddle before assign new value*/
		pGPIOHandle->pGPIOx->MODER &= ~(3 << (2 * pGPIOHandle->pinConfig.pinNumber));
		/*Assign new value*/
		pGPIOHandle->pGPIOx->MODER |= modeRegVal;
	}
	else if (OUTPUT_MODE == pGPIOHandle->pinConfig.pinMode)
	{
		/*Output mode*/
		modeRegVal = (pGPIOHandle->pinConfig.pinMode << (2 * pGPIOHandle->pinConfig.pinNumber));
		/*Clear bits fiddle before assign new value*/
		pGPIOHandle->pGPIOx->MODER &= ~(3 << (2 * pGPIOHandle->pinConfig.pinNumber));
		/*Assign new value*/
		pGPIOHandle->pGPIOx->MODER |= modeRegVal;
		/*config the output type if mode is PushPull or OpenDrain*/
		outputRegVal = (pGPIOHandle->pinConfig.ouputType << pGPIOHandle->pinConfig.pinNumber);
		pGPIOHandle->pGPIOx->OTYPER |= outputRegVal;
		/*config the pull mode (pull up, pull down...)*/
		pupdRegVal = (pGPIOHandle->pinConfig.pullUpDown << (2 * pGPIOHandle->pinConfig.pinNumber));
		pGPIOHandle->pGPIOx->PUPDR |= pupdRegVal;
	}
	else if (INPUT_MODE == pGPIOHandle->pinConfig.pinMode)
	{
		/*Input mode*/
		modeRegVal = (pGPIOHandle->pinConfig.pinMode << (2 * pGPIOHandle->pinConfig.pinNumber));
		/*Clear bits fiddle before assign new value*/
		pGPIOHandle->pGPIOx->MODER &= ~(3 << (2 * pGPIOHandle->pinConfig.pinNumber));
		/*Assign new value*/
		pGPIOHandle->pGPIOx->MODER |= modeRegVal;
		/*config the pull mode (pull up, pull down...)*/
		pupdRegVal = (pGPIOHandle->pinConfig.pullUpDown << (2 * pGPIOHandle->pinConfig.pinNumber));
		pGPIOHandle->pGPIOx->PUPDR |= pupdRegVal;
	}
	else if (ALT_MODE == pGPIOHandle->pinConfig.pinMode)
	{
		/*Config the alt function if mode is ALT mode*/
	}
	else
	{
		/*Handle interrupt mode*/
		if (INTR_RISING_MODE == pGPIOHandle->pinConfig.pinMode)
		{
			//config based on EXT registers
		}
		else if(INTR_FALLING_MODE == pGPIOHandle->pinConfig.pinMode)
		{

		}
		else if(INTR_RF_MODE == pGPIOHandle->pinConfig.pinMode)
		{

		}
	}

	/*2. Config the speed*/
	spdRegVal = (pGPIOHandle->pinConfig.pinSpeed << (2 * pGPIOHandle->pinConfig.pinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= spdRegVal;
}

/**
	* GPIO_DeInit
	* @brief  -	Reset peripheral using RCC AHB1 peripheral reset register
	* @param  -	*pGPIOHandle : specify GPIO Port Base address
	* @retval -	void
	*/
void GPIO_DeInit(GPIOx_RegDef_t *pGPIOx)
{
	/*casting to integer value for base address comparsion*/
	uint32_t baseAddr = (uint32_t) pGPIOx;

	switch (baseAddr)
	{
	case GPIOA_BASEADDR:
		GPIOA_RESET_REG;
		break;
	case GPIOB_BASEADDR:
		GPIOB_RESET_REG;
		break;
	case GPIOC_BASEADDR:
		GPIOC_RESET_REG;
		break;
	case GPIOD_BASEADDR:
		GPIOD_RESET_REG;
		break;
	case GPIOE_BASEADDR:
		GPIOA_RESET_REG;
		break;
	}
}

/**
	* GPIO_ReadFromPin
	* @brief  -	Read input value of a pin
	* @param  -	*pGPIOx : point to GPIO Port Base address
	* @param  -	pinNumber : specify a pin number	
	* @retval -	value of the corresponding I/O port (return a bit of GPIOx_IDR - 1 or 0 only)
	*/
uint8_t GPIO_ReadFromPin(GPIOx_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	uint8_t val = 0;

	if (AVAILABLE_PIN >= pinNumber)
	{
		val = (pGPIOx->IDR) & (1 << pinNumber);
	}
	else
	{
		/*Do nothing*/
	}

	return val;
}

/**
	* GPIO_ReadFromPort
	* @brief  -	Read input value of all pin in a port
	* @param  -	*pGPIOx : point to GPIO Port Base address
	* @param  -	pinNumber : specify a pin number	
	* @retval -	value of the corresponding port (return entire GPIOx_IDR - 16 bit)
	*/
uint16_t GPIO_ReadFromPort(GPIOx_RegDef_t *pGPIOx)
{
	uint16_t val = 0;

	val = (pGPIOx->IDR);

	return val;
}

/**
	* GPIO_WriteToPin
	* @brief  -	Set/reset bit of a pin
	* @param  -	*pGPIOx : point to GPIO Port Base address
	* @param  -	pinNumber : specify a pin number	
	* @param  -	inVal : SET/RESET value		
	* @retval -	void
	*/
void GPIO_WriteToPin(GPIOx_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t inVal)
{
	if (AVAILABLE_PIN >= pinNumber)
	{
		if(RESET == inVal)
		{
			pGPIOx->BSRR |= (1 << (pinNumber + AVAILABLE_PIN + 1));
		}
		else
		{
			pGPIOx->BSRR |= (1 << pinNumber);
		}
	}
	else{
		/*Do nothing*/
	}
}

/**
	* GPIO_WriteToPort
	* @brief  -	Set/reset bit of entire a port
	* @param  -	*pGPIOx : point to GPIO Port Base address	
	* @param  -	inVal : SET/RESET value		
	* @retval -	void
	*/
void GPIO_WriteToPort(GPIOx_RegDef_t *pGPIOx, uint8_t inVal)
{
	if(RESET == inVal)
	{
		pGPIOx->ODR &= ~0xFF;
	}
	else
	{
		pGPIOx->ODR |= 0xFF;
	}
}

/**
	* GPIO_TogglePin
	* @brief  -	Toggle bit a pin
	* @param  -	*pGPIOx : point to GPIO Port Base address
	* @param  -	pinNumber : specify a pin number	
	* @retval -	void
	*/
void GPIO_TogglePin(GPIOx_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	if (AVAILABLE_PIN >= pinNumber)
	{
		pGPIOx->ODR ^= (1 << pinNumber);
	}
	else
	{
		/*Do nothing*/	
	}
}

void GPIO_IRQConfig()
{
}

void GPIO_IRQHandling()
{
}

void delay(uint32_t delay){
	for(int i = 0; i <= 1000000; i++);
}
