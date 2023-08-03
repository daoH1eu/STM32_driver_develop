/**
 ******************************************************************************
 * @file           : stm32f407xx_gpio.h
 * @author         : daoH1eu
 * @brief          : GPIO driver source file
 ******************************************************************************
 */

#include "stm32f407xx_gpio.h"

void GPIO_ClockControl(GPIOx_RegDef_t *pGPIOx, uint8_t IsEnable)
{
	/*casting to integer value for base address comparsion*/
	uint32_t baseAddr = (uint32_t) pGPIOx; 

	if(DISABLE == IsEnable)
	{
		if(GPIOA_BASEADDR == baseAddr){GPIOA_PCLK_DISABLE;}
		if(GPIOB_BASEADDR == baseAddr){GPIOB_PCLK_DISABLE;}
		if(GPIOC_BASEADDR == baseAddr){GPIOC_PCLK_DISABLE;}
		if(GPIOD_BASEADDR == baseAddr){GPIOD_PCLK_DISABLE;}
		if(GPIOE_BASEADDR == baseAddr){GPIOE_PCLK_DISABLE;}
	}
	else if(ENABLE == IsEnable)
	{
		if(GPIOA_BASEADDR == baseAddr){GPIOA_PCLK_ENABLE;}
		if(GPIOB_BASEADDR == baseAddr){GPIOB_PCLK_ENABLE;}
		if(GPIOC_BASEADDR == baseAddr){GPIOC_PCLK_ENABLE;}
		if(GPIOD_BASEADDR == baseAddr){GPIOD_PCLK_ENABLE;}
		if(GPIOE_BASEADDR == baseAddr){GPIOE_PCLK_ENABLE;}
	}

}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	if ((AVAILABLE_PIN >= pGPIOHandle->pinConfig.pinNumber) && (pGPIOHandle->pinConfig.pinNumber >= 0))
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
			/* Input interrupt mode, manage GPIO mapping to 16 external interrupt/event lines */
			if ((INTR_RISING_MODE == pGPIOHandle->pinConfig.pinMode) || (INTR_FALLING_MODE == pGPIOHandle->pinConfig.pinMode) || (INTR_BOTH_RF_MODE == pGPIOHandle->pinConfig.pinMode))
			{
				/* Setting the connection of GPIOs to EXTI line by setting SYSCFG registers */
				uint32_t baseAddr = (uint32_t) pGPIOHandle->pGPIOx; 
				/* Finding corresponding index for register SYSCFG_EXTICRx (x = 0 to 3)*/
				uint8_t EXTICR_index = pGPIOHandle->pinConfig.pinNumber / 4;
				/* Finding corresponding index for EXTIx[3:0]: EXTI x configuration (x = 0 to 3)*/
				uint8_t EXTI_index = pGPIOHandle->pinConfig.pinNumber % 4;
				/* Provide clock for System configuration controller (SYSCFG) */
    			SYSCFG_PCLK_ENABLE;
				/* Reset the source input for the EXTIx, also select Port A as the source input for the EXTIx external interrupt by default*/
				SYSCFG->SYSCFG_EXTICR[EXTICR_index] &= ~(0xF << (4 * EXTI_index));
				/* Select the source input for the EXTIx external interrupt*/
				uint8_t portCode = GPIO_TO_EXTI_PORT_CODE(baseAddr);
				SYSCFG->SYSCFG_EXTICR[EXTICR_index] |= (portCode << (4 * EXTI_index));

				/* Setting corresponding GPIO pin as Input mode */
				/* Clear bits fiddle before assign new value */
				pGPIOHandle->pGPIOx->MODER &= ~(3 << (2 * pGPIOHandle->pinConfig.pinNumber));
				pGPIOHandle->pGPIOx->MODER |= (INPUT_MODE << (2 * pGPIOHandle->pinConfig.pinNumber));
				/* Config the pull mode (pull up, pull down...) */
				pupdRegVal = (pGPIOHandle->pinConfig.pullUpDown << (2 * pGPIOHandle->pinConfig.pinNumber));
				pGPIOHandle->pGPIOx->PUPDR |= pupdRegVal;

				/*
				Hardware interrupt selection
				To configure the 23 lines as interrupt sources, use the following procedure:
				• Configure the mask bits of the 23 interrupt lines (EXTI_IMR)
				• Configure the Trigger selection bits of the interrupt lines (EXTI_RTSR and EXTI_FTSR)
				• Configure the enable and mask bits that control the NVIC IRQ channel mapped to the 
				external interrupt controller (EXTI) so that an interrupt coming from one of the 23 lines can be correctly acknowledged.
				*/
				EXTI->EXTI_IMR |= (1 << pGPIOHandle->pinConfig.pinNumber);

				if (INTR_RISING_MODE == pGPIOHandle->pinConfig.pinMode)
				{
					EXTI->EXTI_RTSR |= (1 << pGPIOHandle->pinConfig.pinNumber);
					EXTI->EXTI_FTSR &= ~(1 << pGPIOHandle->pinConfig.pinNumber);
				}
				else if(INTR_FALLING_MODE == pGPIOHandle->pinConfig.pinMode)
				{
					EXTI->EXTI_FTSR |= (1 << pGPIOHandle->pinConfig.pinNumber);
					EXTI->EXTI_RTSR &= ~(1 << pGPIOHandle->pinConfig.pinNumber);
				}
				else if(INTR_BOTH_RF_MODE == pGPIOHandle->pinConfig.pinMode)
				{
					EXTI->EXTI_RTSR |= (1 << pGPIOHandle->pinConfig.pinNumber);
					EXTI->EXTI_FTSR |= (1 << pGPIOHandle->pinConfig.pinNumber);
				}
			}
		}
		/*2. Config the speed*/
		spdRegVal = (pGPIOHandle->pinConfig.pinSpeed << (2 * pGPIOHandle->pinConfig.pinNumber));
		pGPIOHandle->pGPIOx->OSPEEDR |= spdRegVal;
	}
}

void GPIO_DeInit(GPIOx_RegDef_t *pGPIOx)
{
	/*casting to integer value for base address comparsion*/
	uint32_t baseAddr = (uint32_t) pGPIOx;

	if(GPIOA_BASEADDR == baseAddr){GPIOA_RESET_REG;}
	if(GPIOB_BASEADDR == baseAddr){GPIOB_RESET_REG;}
	if(GPIOC_BASEADDR == baseAddr){GPIOC_RESET_REG;}
	if(GPIOD_BASEADDR == baseAddr){GPIOD_RESET_REG;}
	if(GPIOE_BASEADDR == baseAddr){GPIOE_RESET_REG;}
}

uint8_t GPIO_ReadFromPin(GPIOx_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	uint8_t val = 0;

	if ((AVAILABLE_PIN >= pinNumber) && (pinNumber >= 0)){val = (pGPIOx->IDR) & (1 << pinNumber);}
	return val;
}

uint16_t GPIO_ReadFromPort(GPIOx_RegDef_t *pGPIOx)
{
	uint16_t val = 0;

	val = (pGPIOx->IDR);

	return val;
}

void GPIO_WriteToPin(GPIOx_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t inVal)
{
	if ((AVAILABLE_PIN >= pinNumber) && (pinNumber >= 0))
	{
		if(RESET == inVal)
		{
			pGPIOx->BSRR |= (1 << (pinNumber + AVAILABLE_PIN + 1));
		}
		else if(SET == inVal)
		{
			pGPIOx->BSRR |= (1 << pinNumber);
		}
	}
}

void GPIO_WriteToPort(GPIOx_RegDef_t *pGPIOx, uint8_t inVal)
{
	if(RESET == inVal)
	{
		pGPIOx->ODR &= ~0xFF;
	}
	else if(SET == inVal)
	{
		pGPIOx->ODR |= 0xFF;
	}
}

void GPIO_TogglePin(GPIOx_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	if ((AVAILABLE_PIN >= pinNumber) && (pinNumber >= 0)){pGPIOx->ODR ^= (1 << pinNumber);}
}

void GPIO_IRQConfig(IRQn_Type IRQnumber, uint8_t IsEnable)
{
	if(IRQnumber >= 0 && IRQnumber <= Interrupt224_IRQn)
	{	
		if(ENABLE == IsEnable){
			NVIC_EnableIRQ(IRQnumber); 
		}
		else if(DISABLE == IsEnable){
			NVIC_DisableIRQ(IRQnumber);
		}
	}
}

// void GPIO_IRQHandling (IRQn_Type IRQnumber)
// {

// }

void delay(uint32_t delay){
	if(delay != 0){
		for(int i = 0; i <= delay; i++);
	}
}
