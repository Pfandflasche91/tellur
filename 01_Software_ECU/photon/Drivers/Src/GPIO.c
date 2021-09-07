/*
 * GPIO.c
 *
 *  Created on: 18.08.2021
 *      Author: Maximilian Altrichter
 */

#include "GPIO.h"

/*******************************************************************
 * @function			- GPIO_PCLK
 * @brief				- This function enables or disables peripheral clock for the given GPIO port
 * @param[in]			- base address of the gpio peripheral
 * @param[in]			- Enable or Disable comment
 * @return				- none
 * @Note				- none
 */

void GPIO_PCLK(GPIO_RegDef_t *pGPIOx, uint8_t status)
{
	if(status == ENABLE)
		{
			if(pGPIOx == GPIOA)
			{
				GPIOA_PCLK_EN();
			}else if (pGPIOx == GPIOB)
			{
				GPIOB_PCLK_EN();
			}else if (pGPIOx == GPIOC)
			{
				GPIOC_PCLK_EN();
			}else if (pGPIOx == GPIOD)
			{
				GPIOD_PCLK_EN();
			}else if (pGPIOx == GPIOE)
			{
				GPIOE_PCLK_EN();
			}else if (pGPIOx == GPIOH)
			{
				GPIOH_PCLK_EN();
			}
		}else
		{
			if(pGPIOx == GPIOA)
			{
				GPIOA_PCLK_DI();
			}else if (pGPIOx == GPIOB)
			{
				GPIOB_PCLK_DI();
			}else if (pGPIOx == GPIOC)
			{
				GPIOC_PCLK_DI();
			}else if (pGPIOx == GPIOD)
			{
				GPIOD_PCLK_DI();
			}else if (pGPIOx == GPIOE)
			{
				GPIOE_PCLK_DI();
			}else if (pGPIOx == GPIOH)
			{
				GPIOH_PCLK_DI();
			}

		}
}
