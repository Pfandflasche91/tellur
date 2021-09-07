/*
 * @brief Funktionssammlung zur Verwendung des SysTick-Timers.
 *
 *  Created on: Apr 13, 2020
 *      Author: Ralf Jesse
 */

#include <mcalGlobalDefines.h>
#include <mcalSysTick.h>
#include <stm32f4xx.h>
#include <system_stm32f4xx.h>

/* Makros */
#define DECREMENT_TIMER( timer )   \
    ( {                            \
        if ( timer > 0 )           \
            --timer;               \
    } )

/**
 * @brief Initialization of the SysTick timer
 *
 * @param[in]  divisor : Sets the tick time of SysTick
 *
 * @param[out] none
 */
void systickInit (uint32_t divisor)
{
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / divisor);
}

/**
 * @brief Sets the tick interval of SysTick
 *
 * @param[in]  *timer  : Pointer to the SysTick timer
 * @param[in]   millis : Tick time of the SysTick timer in milliseconds
 *
 * @param[out]  none
 */
void systickSetMillis (uint32_t *timer, uint32_t millis)
{
	*timer = millis;
}

/**
 * @brief Returns TRUE if the SysTick timer is expired, otherwise FALSE.
 *
 * @param[in]  timer :
 *
 * @param[out] bool
 */
bool isSystickExpired(uint32_t timer)
{
	bool timerState = FALSE;

	if (0 == timer)
	{
		timerState = TRUE;
	}

	return timerState;
}

/**
 * @brief Interrupt service handler (ISR) for the SysTick timer
 *
 * @param[in]  none
 * @param[out] none
 */
void SysTick_Handler(void)
{
	timerTrigger = TRUE;
}

/**
 * @brief Walks through the given list and runs DECREMENT_TIMER for
 *        every single element.
 *
 * @param[in]  *list : Pointer auf ein Array mit Pointern
 * @param[out]  none
 */
void systickUpdateTimers (uint32_t *list, uint8_t arraySize)
{
	uint32_t *timer;

	for (uint8_t i = 0; i < (arraySize); ++i)
	{
		timer = (uint32_t *) list[i];
		DECREMENT_TIMER(*timer);
	}
    timerTrigger = false;
}
