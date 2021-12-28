/*
 * timer.h
 *
 *  Created on: 27.12.2021
 *      Author: Maximilian Altrichter
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#include "stm32f446re.h"

typedef struct
{
	uint16_t Counter_period;
	uint16_t Prescaler;
	uint8_t CounterMode;
	uint8_t AutoReloadPreload;
	uint8_t Trigger_Event;
}TIM_Config_t;

typedef struct
{
	TIM_10_11_12_14_RegDef_t *pTIMx;
	TIM_Config_t TIM_Config;
}TIM_Handle_t;

/****************************************************************************************************************
 * 										APIs supported by this driver
 * 					For more information about the APIs check the function definitions
 ****************************************************************************************************************/

void TIM_PCLK(TIM_10_11_12_14_RegDef_t *pTIMx, uint8_t status);
void TIM_Init(TIM_Handle_t *pTIMHandle);

#endif /* INC_TIMER_H_ */
