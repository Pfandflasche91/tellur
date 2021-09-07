/**
 * mcalTimer.c
 * ===========
 *
 *  Created on: 09.07.2020
 *      Author: Ralf Jesse
 *       Email: embedded@ralf-jesse.de
 *
 * @par
 * The STM32F4xx micro controllers support up to three different timer classes:
 * Basic timer, General-purpose timer, and Advanced-control timers. This is a
 * collection of functions which supports the usage of all timers.
 */

#include <mcalTimer.h>

static bool timerVerifyChannels(TIM_TypeDef *tim, CHANNEL_NUM channel);

/**
 * @brief Verification of combination of the timer/channel number.
 *
 * @param[in]  *tim     : Pointer to the timer
 * @param[in]   channel : Number of the channel to be used
 * @parma[out]  bool    : true --> Combination of tim/channel is ok, otherwise false.
 *
 * @par TIM6/TIM7 don't support capture/compare functionality. Other timers provide different
 * numbers of capture/compare channels. This function returns true if the combination of timer
 * and channel number match.
 */
static bool timerVerifyChannels(TIM_TypeDef *tim, CHANNEL_NUM channel)
{
    // No capture/compare channels available for basic timers
    if ((TIM6 == tim) || (TIM7 == tim))
    {
        return false;
    }

    // Timers that provide only one I/O channel
    if (((TIM10 == tim) || (TIM11 == tim) ||
         (TIM13 == tim) || (TIM14 == tim)) && (channel > TIMIO_CH1))
    {
        // Error: Channel number too high. TIM10/11/13/14 provide only two I/O channels
        return false;
    }

    // Timers that provide only two I/O channels
    if (((TIM9 == tim) || (TIM12 == tim)) && (channel > TIMIO_CH2))
    {
        // Error: Channel number too high. TIM9/12 provide only two I/O channels
        return false;
    }

    if ((TIM1 == tim) || (TIM8 == tim))
    {
        return true;
    }

    return true;
}

/**
 * @brief Function to start the bus clock timer tim.
 *
 * @param[in]  *tim : Pointer to the timer
 *
 * @param[out]  none
 */
void timerBusClkOn(TIM_TypeDef *tim)
{
    if (TIM1 == tim)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    }
    else if (TIM2 == tim)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    }
    else if (TIM3 == tim)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    }
    else if (TIM4 == tim)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    }
    else if (TIM5 == tim)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    }
    else if (TIM6 == tim)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    }
    else if (TIM7 == tim)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    }
    else if (TIM8 == tim)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    }
    else if (TIM9 == tim)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
    }
    else if (TIM10 == tim)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
    }
    else if (TIM11 == tim)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
    }
    else if (TIM12 == tim)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;
    }
    else if (TIM13 == tim)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM13EN;
    }
    else if (TIM14 == tim)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    }
    else
    {
        while(1)
        {
            // Timer not supported
        }
    }

    tim->CR1 = 0;  // Reset all bits of TIMx_CR1 to 0
}

/**
 * @brief Function to stop the bus clock of timer tim.
 *
 * @param[in]  *tim : Pointer to the timer
 */
void timerBusClkOff(TIM_TypeDef *tim)
{
    if (TIM1 == tim)
    {
        RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST;
    }
    else if (TIM2 == tim)
    {
        RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST;
    }
    else if (TIM3 == tim)
    {
        RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST;
    }
    else if (TIM4 == tim)
    {
        RCC->APB1RSTR |= RCC_APB1RSTR_TIM4RST;
    }
    else if (TIM5 == tim)
    {
        RCC->APB1RSTR |= RCC_APB1RSTR_TIM5RST;
    }
    else if (TIM6 == tim)
    {
        RCC->APB1RSTR |= RCC_APB1RSTR_TIM6RST;
    }
    else if (TIM7 == tim)
    {
        RCC->APB1RSTR |= RCC_APB1RSTR_TIM7RST;
    }
    else if (TIM8 == tim)
    {
        RCC->APB2RSTR |= RCC_APB2RSTR_TIM8RST;
    }
    else if (TIM9 == tim)
    {
        RCC->APB2RSTR |= RCC_APB2RSTR_TIM9RST;
    }
    else if (TIM10 == tim)
    {
        RCC->APB2RSTR |= RCC_APB2RSTR_TIM10RST;
    }
    else if (TIM11 == tim)
    {
        RCC->APB2RSTR |= RCC_APB2RSTR_TIM11RST;
    }
    else if (TIM12 == tim)
    {
        RCC->APB1RSTR |= RCC_APB1RSTR_TIM12RST;
    }
    else if (TIM13 == tim)
    {
        RCC->APB1RSTR |= RCC_APB1RSTR_TIM13RST;
    }
    else if (TIM14 == tim)
    {
        RCC->APB1RSTR |= RCC_APB1RSTR_TIM14RST;
    }
    else
    {
        while(1)
        {
            // Timer not supported
        }
    }
}

/**
 * @brief Sets the PSC register of the timer.
 *
 * @param[in]  *tim : Pointer to the timer tim
 * @param[in]   psc : New prescaler value
 * @param[out]  none
 */
void timerSetPrescaler(TIM_TypeDef *tim, uint16_t psc)
{
    tim->PSC = psc;
}

/**
 * @brief Sets the Auto-reload registerof the timer.
 *
 * @param[in]  *tim : Pointer to the timer tim
 * @param[in]   psc : New prescaler value
 * @param[out]  none
 *
 * @par Consider that only TIM2 and TIM5 are 32 bit timers.
 *      If the 'reload' value is too high for all the other
 *      timers the reload value is set to 0 and the timer
 *      will not start (ARR MUST be different from 0!).
 */
void timerSetAutoReloadValue(TIM_TypeDef *tim, uint32_t reload)
{
    // 32-bit reload values are only valid for TIM2 and TIM5.
    if ((TIM2 == tim) || (TIM5 == tim))
    {
        tim->ARR = reload;
    }
    else
    {
        // For all other timers: Check that reload does not exceed UIN16_MAX.
        if (reload > UINT16_MAX)
        {
            reload = 0;   // reload value too high --> set to 0 -->
                          // timer will not start!
        }
        tim->ARR = reload;
    }
}

/**
 * @brief Enables the interrupt of timer tim.
 *
 * @param[in]  *tim : Pointer to the timer tim
 * @param[out]  none
 */
void timerEnableInterrupt(TIM_TypeDef *tim)
{
    tim->DIER |= 1;
}

/**
 * @brief Disables the interrupt of timer tim.
 *
 * @param[in]  *tim : Pointer to the timer tim
 * @param[out]  none
 */
void timerDisableInterrupt(TIM_TypeDef *tim)
{
    tim->DIER &= ~1;
}

/**
 * @brief Enables DMA usage of timer tim.
 *
 * @param[in]  *tim : Pointer to the timer tim
 * @param[out]  none
 */
void timerEnableDMA(TIM_TypeDef *tim)
{
    tim->DIER |= 1 << 8;
}

/**
 * @brief Disables DMA usage of timer tim.
 *
 * @param[in]  *tim : Pointer to the timer tim
 * @param[out]  none
 */
void timerDisableDMA(TIM_TypeDef *tim)
{
    tim->DIER &= ~(1 << 8);
}

/**
 * @brief Start timer tim. All timer settings must be finished before starting the timer.
 *
 * @param[in]  *tim : Pointer to the timer tim
 * @param[out]  none
 */
void timerStart(TIM_TypeDef *tim)
{
    tim->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief Stops timer tim.
 *
 * @param[in]  *tim : Pointer to the timer tim
 * @param[out]  none
 */
void timerStop(TIM_TypeDef *tim)
{
    tim->CR1 &= ~1;
}

/**
 * @brief Sets the sampling-clock divider if the timer uses input channels.
 *
 * @param[in]  *tim     : Pointer to the timer
 * @param[in]   divider : Enumerations defined in mcalTimer.h
 *
 * @param[out]  none
 */
void timerSetSamplingClockDivider(TIM_TypeDef *tim, CKD_DIV divider)
{

}

/**
 * @brief Enables/disables the buffer of the Auto-reload/preload mode.
 *
 * @param[in]  *tim      : Pointer to the timer
 * @param[in]   arpeMode : Enumerations defined in mcalTimer.h
 *
 * @param[out]  none
 */
void timerSetAutoReloadMode(TIM_TypeDef *tim, ARPE_BUF arpeMode)
{

}

/**
 * @brief
 *
 * @param[in]  *tim : Pointer to the timer
 */
void timerSetPWMEdgeMode(TIM_TypeDef *tim, PWM_MODE pwmMode)
{

}

/**
 * @brief
 *
 * @param[in]  *tim : Pointer to the timer
 */
void timerSetCountDirection(TIM_TypeDef *tim, DIR direction)
{

}

/**
 * @brief
 *
 * @param[in]  *tim : Pointer to the timer
 */
void timerSetOnePulseMode(TIM_TypeDef *tim, OPM_MODE opm)
{

}

/**
 * @brief
 *
 * @param[in]  *tim : Pointer to the timer
 */
void timerSetUpdtReqSource(TIM_TypeDef *tim, URS urs)
{

}

/**
 * @brief
 *
 * @param[in]  *tim : Pointer to the timer
 */
void timerEnableUpdate(TIM_TypeDef *tim, UDIS udis)
{

}

/**
 * @brief Sets the capture/compare mode of the timer. Checks the number of I/O channels.
 *
 * @param[in]  *tim        : Pointer to the timer
 * @param[in]   channel    : Number of the I/O channel to be configured
 * @param[in]   type       : Selects between input or output type
 * @param[in]   mode       : Operational mode of the I/O channel
 * @param[in]   preloadVal : Sets the preload value
 *
 * @param[out]  bool    : true if the settings are correct otherwise false.
 */
bool timerSetCapCompMode(TIM_TypeDef *tim,
                         CHANNEL_NUM channel,
                         CHANNEL_TYPE type,
                         CAPCOMP_MODE ccMode)
{
    // Check whether timer and channel match
    if (true == timerVerifyChannels(tim, channel))
    {
        // Verify integrity of other parameters
        if ((ccMode < CHN_FROZEN) || (ccMode > CHN_PWM_MODE_2))
        {
            return false;
        }

        if ((type < TIMIO_OUTPUT) || (type > TIMIO_INPUT))
        {
            return false;
        }

        // Configure I/O channel
        switch (channel)
        {
            // Reset previous configuration. The channel will automatically
            // used as output.
            case TIMIO_CH1:
                tim->CCMR1 &= ~(TIM_CCMR1_CC1S_Msk  |     // Use I/O as output
                                TIM_CCMR1_OC1CE_Msk |     // Reset TIM12 OC1E
                                TIM_CCMR1_OC1M_Msk);      // Output compare mode

                tim->CCMR1 |= ccMode << 4;
                tim->CCER  |= TIM_CCER_CC1E;
                break;
            case TIMIO_CH2:
                tim->CCMR1 &= ~(TIM_CCMR1_CC2S_Msk  |     // Use I/O as output
                                TIM_CCMR1_OC2CE_Msk |     // Reset TIM12 OC1E
                                TIM_CCMR1_OC2M_Msk);      // Output compare mode

                tim->CCMR1 |= ccMode << 12;
                tim->CCER  |= TIM_CCER_CC2E;
                break;
            case TIMIO_CH3:
                tim->CCMR2 &= ~(TIM_CCMR2_CC3S_Msk  |     // Use I/O as output
                                TIM_CCMR2_OC3CE_Msk |     // Reset TIM12 OC1E
                                TIM_CCMR2_OC3M_Msk);      // Output compare mode

                tim->CCMR2 |= ccMode << 4;
                tim->CCER  |= TIM_CCER_CC3E;
                break;
            case TIMIO_CH4:
                tim->CCMR2 &= ~(TIM_CCMR2_CC4S_Msk  |     // Use I/O as output
                                TIM_CCMR2_OC4CE_Msk |     // Reset TIM12 OC1E
                                TIM_CCMR2_OC4M_Msk);      // Output compare mode

                tim->CCMR2 |= ccMode << 12;
                tim->CCER  |= TIM_CCER_CC4E;
                break;
        }

        // Configuration is ok
        return true;
    }
    else
    {
        // Configuration failed
        return false;
    }
}

/**
 * @brief Sets the Input-Capture-/Output-Compare-Mode.
 *
 * @param[in]  *tim     : Pointer to the timer
 * @param[in]   channel : I/O channel number
 * @param[in]   mode    : Enable/Disable/Polarity/Complementary mode/Complementary polarity
 */
void timerSelectCapCompOutType(TIM_TypeDef *tim, CHANNEL_NUM channel, CAPCOMP_TYPE type)
{
    if (timerVerifyChannels(tim, channel))
    {
        switch (channel)
        {
            case TIMIO_CH1:
                tim->CCER |= type;
                break;

            case TIMIO_CH2:
                tim->CCER |= (type << 4);
                break;

            case TIMIO_CH3:
                tim->CCER |= (type << 8);
                break;

            case TIMIO_CH4:
                tim->CCER |= (type << 12);
                break;
        }
    }
}

/**
 * @brief Sets the repetition counter of TIM1/TIM8.
 *
 * @param[in]  *tim   : Pointer to the timer
 * @param[in]   value : Number of repetitions before the compare
 *                      registers are updated
 */
void timerSetRepetitionCounter(TIM_TypeDef *tim, CHANNEL_NUM channel, uint8_t value)
{
    if (timerVerifyChannels(tim, channel))
    {
        if ((TIM1 == tim) || (TIM8 == tim))
        {
            tim->RCR = value;
        }
    }
}

/**
 * @brief Sets the CNT register to zero
 *
 * @param[in]  *tim  : Pointer to the timer
 * @param[out] none
 */
void timerResetCounter(TIM_TypeDef *tim)
{
        tim->CNT = 0;
}

/**
 * @brief Sets the CCRx register.
 */
void timerSetPreloadValue(TIM_TypeDef *tim, CHANNEL_NUM channel, uint32_t preloadVal)
{
    // Check whether timer and channel match
    if (true == timerVerifyChannels(tim, channel))
    {
        if (channel == TIMIO_CH1)
        {
            tim->CCR1   = preloadVal;
        }
        else if (channel == TIMIO_CH2)
        {
            tim->CCR2   = preloadVal;
        }
        else if (channel == TIMIO_CH3)
        {
            tim->CCR3   = preloadVal;
        }
        else
        {
            tim->CCR4   = preloadVal;
        }
    }
}

/**
 * @brief Returns the current preload value
 *
 * @param[in]  *tim
 * @param[in]   channel
 * @param[out]  preloadVal : Current value of the CCRn register
 */
uint32_t timerGetPreloadValue(TIM_TypeDef *tim, CHANNEL_NUM channel)
{
    if (true == timerVerifyChannels(tim, channel))
    {
        if (channel == TIMIO_CH1)
        {
            return tim->CCR1;
        }
        else if (channel == TIMIO_CH2)
        {
            return tim->CCR2;
        }
        else if (channel == TIMIO_CH3)
        {
            return tim->CCR3;
        }
        else if (channel == TIMIO_CH4)
        {
            return tim->CCR4;
        }
    }
    return 0;
}

/**
 * @brief Activates/Deactivates the complementary mode of TIM1/TIM8.
 *
 */
bool timerEnableCapCompMode(TIM_TypeDef *tim, CHANNEL_NUM chn, COMPLEMENT_MODE compl)
{
    switch (chn)
    {
        case TIMIO_CH1:
            break;
        case TIMIO_CH2:
            break;
        case TIMIO_CH3:
            break;
        case TIMIO_CH4:
            break;
    }

//    if ((TIM1 == tim) || (TIM8 == tim))
//    {
//        if (TIMIO_CH1 == channel)
//        {
//            if (COMPL_OFF == compl)
//            {
//                tim->CCER |= TIM_CCER_CC1E;
//            }
//            else if (COMPL_ON == compl)
//            {
//                tim->CCER |= TIM_CCER_CC1NP;
//            }
//        }
//        else if (TIMIO_CH2 == chn)
//        {
//
//        }
//        else if (TIMIO_CH3 == chn)
//        {
//
//        }
//        else if (TIMIO_CH4 == chn)
//        {
//
//        }
//
//        return true;
//    }
    return false;
}

/**
 * @brief Clears the BDTR register of TIM1 and TIM8
 *
 * @param[in]  *tim : Pointer to timer tim
 */
void timerBdtrClearRegister(TIM_TypeDef *tim)
{
    if ((TIM1 == tim) || (TIM8 == tim))
    {
        tim->BDTR = 0;
    }
}

/**
 * @brief Modifies the BDTR register and activates/deactivates the main output
 *
 * @param[in]  *tim     : Pointer to timer
 * @param[in]   moeMode : Activates or deactivates the main output
 */
void timerBdtrEnableMainOutput(TIM_TypeDef *tim, BDTR_MOE moeMode)
{
    if ((TIM1 == tim) || (TIM8 == tim))
    {
        if (MOE_OFF == moeMode)
        {
            tim->BDTR &= ~(TIM_BDTR_MOE);
        }
        else
        {
            tim->BDTR |= TIM_BDTR_MOE;
        }
    }
}

/**
 * @brief Activates/deactivates the Break input
 *
 * @param[in]  *tim     : Pointer to timer tim
 * @param[in]   bkpMode : Activation of the Break input
 */
void timerBdtrEnableBreakInput(TIM_TypeDef *tim, BDTR_BKE bkeMode)
{
    if ((TIM1 == tim) || (TIM8 == tim))
    {
        if (BRK_ENABLE == bkeMode)
        {
            tim->BDTR |= TIM_BDTR_BKE;
        }
        else
        {
            tim->BDTR &= ~(TIM_BDTR_BKE);
        }
    }
}
