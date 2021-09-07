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

/*******************************************************************
 * @function			- GPIO_init
 * @brief				- This function init the GPIO
 * @param[in]			- Handle strcut of GPIO
 * @return				- none
 * @Note				- none
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t temp=0;

	//1. configure the mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp=0;
	}else
	{

	}
	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp=0;
	//3. configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp=0;
	//4. configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp=0;
	//5. configure the alt functionality
	//TODO:implement alt function
}

/*******************************************************************
 * @function			- GPIO_DeInit
 * @brief				-
 * @param[in]			-
 * @return				-
 * @Note				-
 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}

}

/*******************************************************************
 * @function			- GPIO_Read
 * @brief				-
 * @param[in]			-
 * @return				- 0 or 1
 * @Note				-
 */

uint8_t GPIO_Read(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	return (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
}

/*******************************************************************
 * @function			- GPIO_Write
 * @brief				-
 * @param[in]			-
 * @return				- 0 or 1
 * @Note				-
 */

void GPIO_Write(GPIO_Handle_t *pGPIOHandle, uint8_t Value)
{
	if(Value == 1)
	{
		pGPIOHandle->pGPIOx->ODR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}else
	{
		pGPIOHandle->pGPIOx->ODR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
}
