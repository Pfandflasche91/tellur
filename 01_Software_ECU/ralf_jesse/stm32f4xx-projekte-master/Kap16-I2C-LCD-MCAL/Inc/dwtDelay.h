/**
 * dwtDelay.h
 *
 *  Created on: Sep 30, 2020
 *      Author: Ralf Jesse
 *       Email: embedded@ralf-jesse.de
 */

#ifndef DWTDELAY_H_
#define DWTDELAY_H_

/*
 * Simple microseconds delay routine, utilizing ARM's DWT
 * (Data Watchpoint and Trace Unit) and HAL library.
 * Intended to use with gcc compiler, but I hope it can be used
 * with any other C compiler across the Universe (provided that
 * ARM and CMSIS already invented) :)
 * Max K
 *
 *
 * This file is part of DWT_Delay package.
 * DWT_Delay is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * us_delay is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
 * the GNU General Public License for more details.
 * http://www.gnu.org/licenses/.
 */

#ifndef INC_DWT_DELAY_H_
#define INC_DWT_DELAY_H_

#ifdef __cplusplus
extern 'C' {
#endif

//#include "stm32f4xx_hal.h"
#include <stm32f4xx.h>
#include <mcalSystem.h>

#define DWT_RUNNING         (0)

/**
 * RJ: This macro is used to eliminate the division. However,
 *     we will still use a (time consuming) multiplication in
 *     DWT_Delay_us().
 */
/* #define DWT_FREQ         (HAL_RCC_GetHCLKFreq() / 1000000)   */
#define DWT_FREQ            (systemGetHclkFreq()/1000000)


/**
 * @brief Initializes DWT_Cycle_Count for DWT_Delay_us function
 * @return Error DWT counter
 * 1: DWT counter Error
 * 0: DWT counter works
 */
uint32_t DWT_Delay_Init (void);

/**
 * @brief This function provides a delay (in microseconds)
 * @param microseconds: delay in microseconds
 */
__STATIC_INLINE void
DWT_Delay_us (volatile uint32_t microseconds)
{
    uint32_t clk_cycle_start = DWT->CYCCNT;
    /* Go to number of cycles for system */

    /* microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);   */
    microseconds *= DWT_FREQ;

    /* Delay till end */
    while ((DWT->CYCCNT - clk_cycle_start) < microseconds)
    {
        ;
    }
}

#ifdef __cplusplus
}
#endif
#endif /* INC_DWT_DELAY_DWT_DELAY_H_ */


#endif /* DWTDELAY_H_ */
