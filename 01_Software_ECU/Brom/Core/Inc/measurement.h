/*
 * measurement.h
 *
 *  Created on: 29.01.2023
 *      Author: maxim
 */

#ifndef INC_MEASUREMENT_H_
#define INC_MEASUREMENT_H_


typedef struct {
	//currents
	float current_a;
	float current_b;
	float current_c;

	//hall
	float hall_a;
	float hall_b;
	float hall_c;

	//encoder
	float encoader_A;
	float encoader_B;

}measurement_data;


#endif /* INC_MEASUREMENT_H_ */
