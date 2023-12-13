/*
 * globals.h
 *
 *  Created on: 08.12.2023
 *      Author: maxim
 */

#ifndef INC_GLOBALS_H_
#define INC_GLOBALS_H_

#include "control_state_machine.h"
#include "inverter.h"
#include "string.h"
#include <stdbool.h>


extern ControlState controlState ;
extern ControlStateMachine controlStateMachine;
extern bool init ;
extern int step;
extern int counter;
extern bool control_enabled;


extern char commandline[250]; // TODO switch handler to interrupt instead of direct in Com
extern Inverter inverter;


#endif /* INC_GLOBALS_H_ */
