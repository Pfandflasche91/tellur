/*
 * control_state_machine.c
 *
 *  Created on: 08.12.2023
 *      Author: maxim
 */

#include "control_state_machine.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include "globals.h"

void transitionToState(ControlStateMachine *statemachine, ControlState state){
	char message_start[]="Switched State to ";
	char message_state[20];
	char message_end[]="\n";
	char result[100];

	switch(state){
		case development_control:
			strcpy(message_state,"development_control");
			statemachine->currentState = development_control;
			break;
		case torque_control:
			strcpy(message_state,"torque_control");
			statemachine->currentState = torque_control;
			break;
		case speed_control:
			strcpy(message_state,"speed_control");
			statemachine->currentState = speed_control;
			break;
		case position_control:
			strcpy(message_state,"position_control");
			statemachine->currentState = position_control;
			break;
		default:
			strcpy(message_state,"off");
			statemachine->currentState = off;
			break;
	}
	message_state[19]='\0';

	snprintf(result, sizeof(result),"%s%s%s",message_start,message_state,message_end);
	CDC_Transmit_FS((uint8_t *)result,strlen(result));
}

void interpretCommandoLine(char message[],uint32_t length){
	char result[100];
	char message_start[]= "I received:";
	char message_end[]="\n";
	char state[5];
	if (message[0] == '-'){
		if (!strncmp("-development",message,11)){
			controlState = development_control;
		}
		strcpy(state,"d");
	}else{
		strcpy(state,"notd");
	}
	snprintf(result, sizeof(result),"%s%s%s",message_start,message,message_end);
	CDC_Transmit_FS((uint8_t *)result,strlen(result));

}
