/**
 ******************************************************************************
 * @file           : 003_led_button_interrupt.c
 * @author         : daoH1eu
 * @brief          : Button Led Demo
 ******************************************************************************
 */

#include "stm32f407xx_gpio.h"

void setup()
{

    GPIO_Handle_t PA6, PA7, PE3, PE4;
    PA6.pGPIOx = GPIOA;
    PA7.pGPIOx = GPIOA;
    PE3.pGPIOx = GPIOE;
    PE4.pGPIOx = GPIOE;

	/*GPIO config*/
    PA6.pinConfig.pinNumber = GPIO_PIN_6;
    PA6.pinConfig.pinMode = OUTPUT_MODE;
    PA6.pinConfig.pinSpeed = HIGHSPD;
    PA6.pinConfig.ouputType = PUSHPULL;
    PA6.pinConfig.pullUpDown = NOPULL;

    PA7.pinConfig.pinNumber = GPIO_PIN_7;
    PA7.pinConfig.pinMode = OUTPUT_MODE;
    PA7.pinConfig.pinSpeed = HIGHSPD;
    PA7.pinConfig.ouputType = PUSHPULL;
    PA7.pinConfig.pullUpDown = NOPULL;

    PE3.pinConfig.pinNumber = GPIO_PIN_3;
    PE3.pinConfig.pinMode = INTR_FALLING_MODE;
    PE3.pinConfig.pinSpeed = HIGHSPD;
    PE3.pinConfig.pullUpDown = PULLUP;

    PE4.pinConfig.pinNumber = GPIO_PIN_4;
    PE4.pinConfig.pinMode = INTR_FALLING_MODE;
    PE4.pinConfig.pinSpeed = HIGHSPD;
    PE4.pinConfig.pullUpDown = PULLUP;

    GPIO_ClockControl(GPIOA, ENABLE);
    GPIO_ClockControl(GPIOE, ENABLE);

    GPIO_Init(&PA6);
    GPIO_Init(&PA7);
    GPIO_Init(&PE3);
    GPIO_Init(&PE4);

    /*turn off led*/
    GPIO_WriteToPin(GPIOA, GPIO_PIN_6, SET);
    GPIO_WriteToPin(GPIOA, GPIO_PIN_7, SET);

    GPIO_IRQConfig(EXTI4_IRQn, ENABLE);
    GPIO_IRQConfig(EXTI3_IRQn, ENABLE);
}

int main(void)
{
    setup();
    /* Loop forever */
	for(;;);
}

void EXTI3_IRQHandler(void)
{
	/*implement IRQ handler for GPIO_PIN_4*/
    GPIO_TogglePin(GPIOA, GPIO_PIN_6);
    GPIO_TogglePin(GPIOA, GPIO_PIN_7);
	// clear trigger request if it occurred
	EXTI_PENDING_BIT_CLEAR(GPIO_PIN_3);
}

void EXTI4_IRQHandler(void)
{
	/*implement IRQ handler for GPIO_PIN_4*/
    GPIO_TogglePin(GPIOA, GPIO_PIN_6);
    GPIO_TogglePin(GPIOA, GPIO_PIN_7);
	// clear trigger request if it occurred
	EXTI_PENDING_BIT_CLEAR(GPIO_PIN_4);
}