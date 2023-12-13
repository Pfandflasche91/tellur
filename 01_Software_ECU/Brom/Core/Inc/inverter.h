/*
 * inverter.h
 *
 *  Created on: Dec 9, 2023
 *      Author: maxim
 */

#ifndef INC_INVERTER_H_
#define INC_INVERTER_H_

#include "stm32f4xx_hal.h"


typedef enum{
	FALSE,
	TRUE
}State;

typedef struct{
	GPIO_TypeDef *port;
	uint16_t pinNUmber;
}GPIOPin;

typedef struct{
	State pha_enable;
	GPIOPin gpioPin_pha_enable;
	char name[7];

}Phase;

typedef struct{
	State EN_5V;
	Phase PhaseA;
	Phase PhaseB;
	Phase PhaseC;
}Inverter;

void transitionEN5V(State state);
void transitionphaMode(State state,Phase phase);
void initInverter(Inverter inverter);

#endif /* INC_INVERTER_H_ */
