/*
 * global.c
 *
 *  Created on: Dec 9, 2023
 *      Author: maxim
 */
#include "globals.h"

ControlStateMachine controlStateMachine;
Inverter inverter;
ControlState controlState ;
bool init = false;
int step = 1;
int counter = 0;
bool control_enabled = FALSE;
