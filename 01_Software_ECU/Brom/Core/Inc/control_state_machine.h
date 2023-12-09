/*
 * control_state_machine.h
 *
 *  Created on: 08.12.2023
 *      Author: maxim
 */

#ifndef INC_CONTROL_STATE_MACHINE_H_
#define INC_CONTROL_STATE_MACHINE_H_

#include "string.h"
#include "stdio.h"
#include "time.h"
#include "stdlib.h"


typedef enum{
	off,
	torque_control,
	speed_control,
	position_control,
	development_control
}ControlState;

typedef struct{
	ControlState currentState;
}ControlStateMachine;

void transitionToState(ControlStateMachine *statemachine, ControlState state);
void interpretCommandoLine(char message[], uint32_t length);
#endif /* INC_CONTROL_STATE_MACHINE_H_ */
