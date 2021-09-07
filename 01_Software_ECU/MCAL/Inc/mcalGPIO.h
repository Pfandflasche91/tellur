/*
 * gpio.h
 *
 *  Created on: Apr 13, 2020
 *      Author: Ralf Jesse
 *       Email: embedded@ralf-jesse.de
 */

#ifndef MCALGPIO_H_
#define MCALGPIO_H_

#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Enumeration of GPIO pins
 */
typedef enum
{
	PIN0 = 0,
	PIN1,
	PIN2,
	PIN3,
	PIN4,
	PIN5,
	PIN6,
	PIN7,
	PIN8,
	PIN9,
	PIN10,
	PIN11,
	PIN12,
	PIN13,
	PIN14,
	PIN15
} PIN_NUM;

/**
 * @brief Enumeration of alternative pin functions
 */
typedef enum
{
	AF0 = 0,
	AF1,
	AF2,
	AF3,
	AF4,
	AF5,
	AF6,
	AF7,
	AF8,
	AF9,
	AF10,
	AF11,
	AF12,
	AF13,
	AF14,
	AF15
} ALT_FUNC;

/**
 * @brief Enumeration of GPIO pin modes
 */
typedef enum
{
	INPUT = 0,
	OUTPUT,
	ALTFUNC,
	ANALOG
} PIN_MODE;

/**
 * @brief Enumeration of output types
 */
typedef enum
{
	PUSHPULL = 0,
	OPENDRAIN
} OUTPUT_TYPE;

/**
 * @brief Enumeration of pull-up or pull-down configurations
 */
typedef enum
{
	NO_PULLUP_PULLDOWN = 0,
	PULLUP,
	PULLDOWN,
} PUPD_MODE;

/**
 * @brief Enumerations of output speeds
 */
typedef enum
{
	LOW_SPEED = 0,
	MEDIUM_SPEED,
	FAST_SPEED,
	HIGH_SPEED
} GPIO_SPEED;


extern void     gpioInitPort(GPIO_TypeDef *port);
extern void     gpioSelectPinMode(GPIO_TypeDef *port, PIN_NUM pin, PIN_MODE mode);
extern void     gpioSetPin(GPIO_TypeDef *port, PIN_NUM pin);
extern void     gpioResetPin(GPIO_TypeDef *port, PIN_NUM pin);
extern void     gpioTogglePin(GPIO_TypeDef *port, PIN_NUM pin);
extern void     gpioSelectAltFunc(GPIO_TypeDef *port, PIN_NUM pin, ALT_FUNC af);
extern void     gpioSelectPushPullType(GPIO_TypeDef *port, PIN_NUM pin, PUPD_MODE pupd);
extern bool     gpioSetOutputType(GPIO_TypeDef *port, PIN_NUM pin, OUTPUT_TYPE outType);
extern void     gpioSetOutputSpeed(GPIO_TypeDef *port, PIN_NUM pin, GPIO_SPEED speed);
extern uint8_t  gpioGetPinVal(GPIO_TypeDef *port, PIN_NUM pin);
extern uint16_t gpioGetPortVal(GPIO_TypeDef *port);



#endif /* MCALGPIO_H_ */
