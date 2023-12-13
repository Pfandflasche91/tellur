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
#include "inverter.h"



void transitionToState(ControlStateMachine *statemachine, ControlState state){
	char message_start[]="Switched State to ";
	char message_state[25];
	char result[50];
	switch(state){
		case development_control:
			strcpy(message_state,"development_control\r\n");
			statemachine->currentState = development_control;
			break;
		case torque_control:
			strcpy(message_state,"torque_control\n");
			statemachine->currentState = torque_control;
			break;
		case speed_control:
			strcpy(message_state,"speed_control\n");
			statemachine->currentState = speed_control;
			break;
		case position_control:
			strcpy(message_state,"position_control\n");
			statemachine->currentState = position_control;
			break;
		case off:
			strcpy(message_state,"off\n");
			statemachine->currentState = off;
			break;
		default:
			strcpy(message_state,"off\n");
			statemachine->currentState = off;
			break;
	}
	snprintf(result, sizeof(result),"%s%s",message_start,message_state);
	CDC_Transmit_FS((uint8_t *)result,strlen(result));
}

void interpretCommandoLine(char message[],uint32_t length){
	char result[100];
	char message_start[]= "invalid command";
	char message_end[]="\n";


	if (message[0] == '-'){
		if (!strncmp("-development",message,11)){
			controlState = development_control;
		}
		if (!strncmp("-off",message,11)){
					controlState = off;
				}

		switch (controlStateMachine.currentState){
			case off:
				break;
			case torque_control:

			case speed_control:

			case position_control:

			case development_control:
				if (!strncmp("-EN5V on",message,8)){
					transitionEN5V(TRUE);
				}
				if (!strncmp("-EN5V off",message,9)){
					transitionEN5V(FALSE);
				}
				if (!strncmp("-init Inverter",message,15)){
					init = true;
				}
				if (!strncmp("-control off",message,11)){
					control_enabled = FALSE;
				}
				if (!strncmp("-control on",message,11)){
					control_enabled = TRUE;
				}
				break;
			default:
				break;
		}
	}else{
		snprintf(result, sizeof(result),"%s%s",message_start,message_end);
		CDC_Transmit_FS((uint8_t *)result,strlen(result));
	}
}
