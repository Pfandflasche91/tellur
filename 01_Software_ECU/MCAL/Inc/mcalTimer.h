/**
 * mcalTimer.h
 * @brief Constants and enumerations related to all timer classes.
 *
 *  Created on: 09.07.2020
 *      Author: Ralf Jesse
 *       Email: embedded@ralf-jesse.de
 */

#ifndef MCALTIMER_H_
#define MCALTIMER_H_

#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>

/*********************************************************************/
/**
 * @brief Definitions for TIMx_CR1
 */

typedef enum
{
    BAS_TIMER = 0,    // Basic timer (TIM6/TIM7)
    GP_TIMER,         // General-purpose timer (TIM9 to TIM14)
    AGB_TIMER,        // Advanced GP timer (TIM2 to TIM5)
    AC_TIMER          // Advanced-control timer (TIM1/TIM8)
} TIMER_FAMILY;

/* Clock divider for sampling frquency, Bits 8 + 9 */
typedef enum
{
    CKD_DIV_BY_1 = 0,
    CKD_DIV_BY_2,
    CKD_DIV_BY_4
} CKD_DIV;

/* Auto-Reload preload enable, Bit 7  */
typedef enum
{
    ARPE_BUFFER_OFF  = 0,
    ARPE_BUFFER_ON
} ARPE_BUF;

/* PWM: Edge or center-aligned mode, Bits 6 + 5 */
typedef enum
{
    PWM_EDGE     = 0,
    PWM_CENTER_1,
    PWM_CENTER_2,
    PWM_CENTER_3,
} PWM_MODE;

/* DIR (counting direction), Bit 4 */
typedef enum
{
    CNT_UP = 0,
	CNT_DOWN
} DIR;

/* OPM: Single-shot mode */
typedef enum
{
	ONE_PULSE_MODE_OFF = 0,
	ONE_PULSE_MODE_ON
} OPM_MODE;

/* URS: Defines the source of an update request */
typedef enum
{
    UPDATE_ALL_SOURCES = 0,        // Generates an event on underflow/overflow,
                                   // when setting the UG bit, or update generation
	                               // requested by the slave controller
	UPDATE_OVERFLOW_UNDERFLOW      // Generates an event only on underflow/overflow
} URS;

/* UDIS: Update disable */
typedef enum
{
	UPDATE_ENABLE = 0,
	UPDATE_DISABLE
} UDIS;

/* CEN: Enable/disable timer */
typedef enum
{
	START_TIMER = 0,
	STOP_TIMER
} TIM_START;

/* Capture/Compare operational modes */
typedef enum
{
    CHN_FROZEN      = 0,    // No activity on capture/compare match
    CHN_ACTIVE,             // All channels: Set high on match
    CHN_INACTIVE,           // All channels: Set low on match
    CHN_TOGGLE,             // All channels: Toggle on match
    CHN_FORCE_LOW,          // All channels: Force output to low level
    CHN_FORCE_HIGH,         // All channels: Force output to high level
    CHN_PWM_MODE_1,         // All channels: Set output high while upcounting, else low
    CHN_PWM_MODE_2          // All channels: Set output low while upcounting, else high
} CAPCOMP_MODE;

/* Timer channels */
typedef enum
{
    TIMIO_CH1 = 1,
    TIMIO_CH2,
    TIMIO_CH3,
    TIMIO_CH4
} CHANNEL_NUM;

/* Select between input or output channel */
typedef enum
{
    TIMIO_OUTPUT = 0,
    TIMIO_INPUT
} CHANNEL_TYPE;

typedef enum
{
    UP   = true,
    DOWN = false
} COUNT_DIR;

typedef enum
{
    COMPL_OFF = 0,
    COMPL_ON
} COMPLEMENT_MODE;

typedef enum
{
    CHN_ENABLE         = 1,
    CHN_POLARITY       = 2,
    CHN_COMPL_ENABLE   = 4,
    CHN_COMPL_POLARITY = 8
} CAPCOMP_TYPE;

typedef enum
{
    MOE_OFF = 0,
    MOE_ON
} BDTR_MOE;

typedef enum
{
    AOE_SW_ONLY = 0,
    AOE_SW_AND_EVENT
} BDTR_AOE;

typedef enum
{
    BKP_ACT_HIGH = 0,
    BKP_ACT_LOW
} BDTR_BKP_POLARITY;

typedef enum
{
    BRK_ENABLE = 0,
    BRK_DISABLE
} BDTR_BKE;

/**
 * Function prototypes
 */
extern void     timerBusClkOn(TIM_TypeDef *tim);
extern void     timerBusClkOff(TIM_TypeDef *tim);
extern void     timerSetPrescaler(TIM_TypeDef *tim, uint16_t psc);
extern void     timerSetAutoReloadValue(TIM_TypeDef *tim, uint32_t reload);
extern void     timerEnableInterrupt(TIM_TypeDef *tim);
extern void     timerSetOnePulseMode(TIM_TypeDef *tim, OPM_MODE opm);
extern void     timerStart(TIM_TypeDef *tim);
extern void     timerStop(TIM_TypeDef *tim);
extern void     timerSetPreloadValue(TIM_TypeDef *tim,
                                    CHANNEL_NUM chn,
                                    uint32_t preloadVal);
extern uint32_t timerGetPreloadValue(TIM_TypeDef *tim, CHANNEL_NUM chn);
extern void     timerResetCounter(TIM_TypeDef *tim);

extern bool     timerSetCapCompMode(TIM_TypeDef *tim,
                                    CHANNEL_NUM channel,
                                    CHANNEL_TYPE type,
                                    CAPCOMP_MODE mode);
extern void     timerSelectCapCompOutType(TIM_TypeDef *tim,
                                    CHANNEL_NUM channel,
                                    CAPCOMP_TYPE type);
extern void     timerSetRepetitionCounter(TIM_TypeDef *tim,
                                    CHANNEL_NUM channel,
                                    uint8_t value);

/* Functions used in combination with the BDTR register */
extern void     timerSetBreakAndDeadTime(TIM_TypeDef *tim);
extern void     timerBdtrClearRegister(TIM_TypeDef *tim);
extern void     timerBdtrEnableMainOutput(TIM_TypeDef *tim, BDTR_MOE moeMode);
extern void     timerBdtrAutomaticOutputEnable(TIM_TypeDef *tim, BDTR_AOE aoeMode);
extern void     timerBdtrSetBreakPolarity(TIM_TypeDef *tim, BDTR_BKP_POLARITY bkpMode);
extern void     timerBdtrBreakEnable(TIM_TypeDef *tim, BDTR_BKE bkeMode);


// Not yet implemented timer functions

// Functions which can be used with TIM2 to TIM5 and advanced-control timers
//extern void     timerSetCountDirection(TIM_TypeDef *tim, DIR direction);
//extern void     timerSetSamplingClockDivider(TIM_TypeDef *tim, CKD_DIV divider);
//extern void     timerSetAutoReloadMode(TIM_TypeDef *tim, ARPE_BUF arpeMode);
//extern void     timerSetPWMEdgeMode(TIM_TypeDef *tim, PWM_MODE pwmMode);
//extern void     timerSetUpdtReqSource(TIM_TypeDef *tim, URS urs);
//extern void     timerEnableUpdate(TIM_TypeDef *tim, UDIS udis);




#endif /* MCALTIMER_H_ */
