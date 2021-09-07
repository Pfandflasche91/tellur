/**
 * mcalSystem.h
 *
 *  Created on: 16.08.2020
 *      Author: Ralf Jesse
 *       Email: embedded@ralf-jesse.de
 */

#ifndef MCALSYSTEM_H_
#define MCALSYSTEM_H_

#include <stdint.h>
#include <stdbool.h>

#define HSI_VALUE       ((uint32_t) 16000000u)
#define HSE_VALUE       ((uint32_t) 25000000u)


extern uint32_t systemGetSysClock(void);
extern uint32_t systemGetHclkFreq(void);
extern uint32_t systemGetPclk1Freq(void);
extern uint32_t systemGetPclk2Freq(void);


#endif /* MCALSYSTEM_H_ */
