/**
 ******************************************************************************
 * @file           : 001_led_toggle.c
 * @author         : daoH1eu
 * @brief          : Led Toggle Demo
 ******************************************************************************
 */

#include "stm32f407xx.h"

void setup()
{
    /*Setup*/
    GPIO_Handle_t PA6, PA7;
    PA6.pGPIOx = GPIOA;
    PA7.pGPIOx = GPIOA;

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

    GPIO_ClockControl(GPIOA, ENABLE);

    GPIO_Init(&PA6);
    GPIO_Init(&PA7);

    /*turn off led*/
    GPIO_WriteToPin(GPIOA, GPIO_PIN_6, SET);
    GPIO_WriteToPin(GPIOA, GPIO_PIN_7, SET);
}

int main(void)
{
    setup();
    /* Loop forever */
	for(;;){
	    delay(1000);
	    GPIO_TogglePin(GPIOA, GPIO_PIN_6);
	    GPIO_TogglePin(GPIOA, GPIO_PIN_7);
	}
}
