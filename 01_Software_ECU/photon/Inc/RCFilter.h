/*
 * RCFilter.h
 *
 *  Created on: 18.01.2023
 *      Author: maxim
 */

#ifndef RCFILTER_H_
#define RCFILTER_H_

typedef struct{

	float coeff[2];
	float out[2];

}RCFilter;

void RCFilter_Init(RCFilter *filt, float cutoffFreqHz, float sampleTimeS);
float RCFilter_Update(RCFilter *filt,float inp);

#endif /* RCFILTER_H_ */
