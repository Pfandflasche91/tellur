/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include "stm32f446re.h"
#include <stdint.h>
#include "GPIO.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif


void delayMillis(uint16_t delay);		//delay function
int main(void)
{
	GPIO_Handle_t rectangle;
	rectangle.pGPIOx = GPIOA;
	rectangle.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN0;
	rectangle.GPIO_PinConfig.GPIO_PinMode 			= GPIO_MODE_OUT;
	rectangle.GPIO_PinConfig.GPIO_PinSpeed 			= GPIO_SPEED_FAST;
	rectangle.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OPTYPE_PP;
	rectangle.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_NO_PUPD;

	//Clock activate for GPIOA AHB1ENR
	GPIO_PCLK(GPIOA,ENABLE);
	//GPIO Init
	GPIO_Init(&rectangle);
	while(1)
	{
		//GPIO_Write(GPIOA, GPIO_PIN0, Value);
		GPIO_Write(&rectangle, GPIO_PIN_SET);
		delayMillis(500);
		GPIO_Write(&rectangle, GPIO_PIN_RESET);
		delayMillis(500);
	}
}

void delayMillis(uint16_t delay)
{
	uint16_t i = 0;
	for(;delay>0; --delay)
	{
		for(i = 0; i<1245;++i)
		{
			;
		}
	}
}


