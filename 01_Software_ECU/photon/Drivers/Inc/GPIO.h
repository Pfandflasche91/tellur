/*
 * GPIO.h
 *
 *  Created on: 17.08.2021
 *      Author: Maximilian Altrichter
 */
#ifndef DRIVERS_INC_GPIO_H_
#define DRIVERS_INC_GPIO_H_

#include "stm32f446re.h"

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t *pGPIOx;				/*!<This holds the base address of the GPIO port to which the pin belongs>*/
	GPIO_PinConfig_t GPIO_PinConfig;	/*!<This holds GPIO pin configuration settings>*/

}GPIO_Handle_t;

/*
 * GPIO_PINNumbers
 */
#define GPIO_PIN0			0
#define GPIO_PIN1			1
#define GPIO_PIN2			2
#define GPIO_PIN3			3
#define GPIO_PIN4			4
#define GPIO_PIN5			5
#define GPIO_PIN6			6
#define GPIO_PIN7			7
#define GPIO_PIN8			8
#define GPIO_PIN9			9
#define GPIO_PIN10			10
#define GPIO_PIN11			11
#define GPIO_PIN12			12
#define GPIO_PIN13			13
#define GPIO_PIN14			14
#define GPIO_PIN15			15

/*
 * GPIO_PinMode
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3

/*
 * GPIO_PinSpeed
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*
 * GPIO_PinOPType
 */
#define GPIO_OPTYPE_PP		0
#define GPIO_OPTYPE_OD		1

/*
 * GPIO_PUPDControl
 */
#define GPIO_NO_PUPD		0
#define GPIO_PU				1
#define GPIO_PD				2

/****************************************************************************************************************
 * 										APIs supported by this driver
 * 					For more information about the APIs check the function definitions
 ****************************************************************************************************************/

void GPIO_PCLK(GPIO_RegDef_t *pGPIOx, uint8_t status);
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
uint8_t GPIO_Read(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
void GPIO_Write(GPIO_Handle_t *pGPIOHandle, uint8_t Value);

#endif /* DRIVERS_INC_GPIO_H_ */
