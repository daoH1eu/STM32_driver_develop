/*
 * stm32f407xx_gpio.c
 *
 *  Created on: 30th  July, 2023
 *      Author: hieus
 */

#include "stm32f407xx_gpio.h"

void GPIO_ClockControl(GPIOx_RegDef_t *pGPIOx, uint8_t IsEnable)
{

}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint8_t modeRegVal = 0;   	/*mode register value*/
	uint8_t spdRegVal = 0;	   	/*speed register value*/
	uint8_t pupdRegVal = 0;  	/*pull-up/pull-down register*/
	uint8_t outputRegVal = 0; 	/*output type register*/

	/*1. Config the mode*/
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
	}

	/*2. Config the speed*/
	spdRegVal = (pGPIOHandle->pinConfig.pinSpeed << (2 * pGPIOHandle->pinConfig.pinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= spdRegVal;
}

void GPIO_DeInit(GPIOx_RegDef_t *pGPIOx)
{
}

uint8_t GPIO_ReadFromPin(GPIOx_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	return 0;
}

uint16_t GPIO_ReadFromPort(GPIOx_RegDef_t *pGPIOx)
{
	return 0;
}

void GPIO_WriteToPin(GPIOx_RegDef_t *pGPIOx, uint8_t pinNumber)
{
}

void GPIO_WriteToPort(GPIOx_RegDef_t *pGPIOx)
{
}

void GPIO_TogglePin(GPIOx_RegDef_t *pGPIOx, uint8_t pinNumber)
{
}

void GPIO_IRQConfig()
{
}

void GPIO_IRQHandling()
{
}
