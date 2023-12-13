/*
 * inverter.c
 *
 *  Created on: Dec 9, 2023
 *      Author: maxim
 */
# include "main.h"
# include "inverter.h"
# include "usbd_cdc_if.h"
# include "globals.h"

void transitionEN5V(State state){
	char result[25];
	char message_start[]="EN5V";
	char message_state[7];

	switch(state){
		case TRUE:
			HAL_GPIO_WritePin(EN_5V_GPIO_Port, EN_5V_Pin, TRUE);
			strcpy(message_state," on\r\n");
			break;
		case FALSE:
			HAL_GPIO_WritePin(EN_5V_GPIO_Port, EN_5V_Pin, FALSE);
			strcpy(message_state," off\r\n");
			break;
		default :
			HAL_GPIO_WritePin(EN_5V_GPIO_Port, EN_5V_Pin, FALSE);
			strcpy(message_state," off\r\n");
			break;
	}

	snprintf(result, sizeof(result),"%s%s",message_start,message_state);
	CDC_Transmit_FS((uint8_t *)result,strlen(result));
}

void transitionphaMode(State state,Phase phase){
	switch(state){
		case TRUE:
			HAL_GPIO_WritePin(phase.gpioPin_pha_enable.port, phase.gpioPin_pha_enable.pinNUmber, TRUE);
			break;
		case FALSE:
			HAL_GPIO_WritePin(phase.gpioPin_pha_enable.port, phase.gpioPin_pha_enable.pinNUmber, FALSE);
			break;
		default :
			HAL_GPIO_WritePin(phase.gpioPin_pha_enable.port, phase.gpioPin_pha_enable.pinNUmber, FALSE);
			break;
		}
}

void initInverter(Inverter inverter){

	transitionEN5V(TRUE);
	HAL_Delay(1000);
	transitionphaMode(TRUE,inverter.PhaseA);
	HAL_Delay(1000);
	transitionphaMode(TRUE,inverter.PhaseB);
	HAL_Delay(1000);
	transitionphaMode(TRUE,inverter.PhaseC);
	HAL_Delay(1000);
	init = false;
}
