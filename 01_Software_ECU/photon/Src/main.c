
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>         // Erforderlich wegen size_t
#include <time.h>
#include <stdlib.h>

#include <stm32f4xx.h>
#include <system_stm32f4xx.h>
#include <mcalGPIO.h>
#include <mcalSysTick.h>
#include <mcalTimer/mcalTimer.h>
#include <mcalRCC.h>
#include <mcalFlash.h>

#include "PID.h"
#include "RCFilter.h"

void updateTimer(void);

bool timerTrigger = false;


/* Controller parameters */
#define PID_KP  2.5f
#define PID_KI  20.0f
#define PID_KD  0.01f

#define PID_TAU 5.0f

#define PID_LIM_MIN -10.0f
#define PID_LIM_MAX  10.0f

#define PID_LIM_MIN_INT -5.0f
#define PID_LIM_MAX_INT  5.0f

#define SAMPLE_TIME_S 0.01f

#define SYSCLOCK        (180)                       // Takt soll 180 MHz betragen

/* Simulated dynamical system (first order) */
float TestSystem_Update(float inp);
float measurement = 0.0f;
float measurement_filtered = 0.0f;
float measurement_noise = 0.0f;
/* Simulate response using test system */
float setpoint = 1.0f;

RCFilter rcfilter;
//srand(time(NULL));

void TIM6_DAC_IRQHandler(void);

int main(void)
{
	TIM_TypeDef  *tim  = TIM11;
	TIM_TypeDef  *tim_10  = TIM10;
	TIM_TypeDef  *tim_9  = TIM9;

	TIM_TypeDef  *tim_6  = TIM6;
	uint32_t      preloadVal = 4500UL;
	bool          direction  = 1;

	// Konfiguration von GPIOA/Pin15, Modus: Alternative Funktion AF1
	gpioSelectPort(GPIOA);
	gpioSelectPort(GPIOB);

	gpioSelectPinMode(GPIOB, PIN9, ALTFUNC);
	gpioSelectAltFunc(GPIOB, PIN9, AF3);

	gpioSelectPinMode(GPIOB, PIN8, ALTFUNC);
	gpioSelectAltFunc(GPIOB, PIN8, AF3);

	gpioSelectPinMode(GPIOA, PIN2, ALTFUNC);
	gpioSelectAltFunc(GPIOA, PIN2, AF3);


	// TIM6: Konfiguration
	timerSelectTimer(tim_6);
	timerSetPrescaler(tim_6, 0);
	timerSetAutoReloadValue(tim_6, 9000);
	timerResetCounter(tim_6);

	//timerSetOutputCompareMode(tim_6, TIMIO_CH1, CHN_PWM_MODE_1);
	//timerSetPreloadValue(tim_6, TIMIO_CH1, preloadVal);
	//timerEnableCaptureCompareChannel(tim_6, TIMIO_CH1);
	timerStartTimer(tim_6);

	// TIM9: Konfiguration
	timerSelectTimer(tim_9);
	timerSetPrescaler(tim_9, 0);
	timerSetAutoReloadValue(tim_9, 9000);
	timerResetCounter(tim_9);

	timerSetOutputCompareMode(tim_9, TIMIO_CH1, CHN_PWM_MODE_1);
	timerSetPreloadValue(tim_9, TIMIO_CH1, preloadVal);
	timerEnableCaptureCompareChannel(tim_9, TIMIO_CH1);
	timerStartTimer(tim_9);

	// TIM10: Konfiguration
	timerSelectTimer(tim_10);
	timerSetPrescaler(tim_10, 0);
	timerSetAutoReloadValue(tim_10, 9000);
	timerResetCounter(tim_10);

	timerSetOutputCompareMode(tim_10, TIMIO_CH1, CHN_PWM_MODE_1);
	timerSetPreloadValue(tim_10, TIMIO_CH1, preloadVal);
	timerEnableCaptureCompareChannel(tim_10, TIMIO_CH1);
	timerStartTimer(tim_10);

	// TIM11: Konfiguration
	timerSelectTimer(tim);
	timerSetPrescaler(tim, 0);
	timerSetAutoReloadValue(tim, 9000);
	timerResetCounter(tim);

	timerSetOutputCompareMode(tim, TIMIO_CH1, CHN_PWM_MODE_1);
	timerSetPreloadValue(tim, TIMIO_CH1, preloadVal);
	timerEnableCaptureCompareChannel(tim, TIMIO_CH1);
	timerStartTimer(tim);


	uint32_t pin0Timer = 0UL;
	uint32_t pin1Timer = 0UL;

	uint32_t *timerList[] = { &pin0Timer, &pin1Timer  };
	size_t    arraySize = sizeof(timerList)/sizeof(timerList[0]);

	systickInit(SYSTICK_100US);
	systickSetTicktime(&pin0Timer, 100);
	systickSetTicktime(&pin1Timer, 50);

	gpioSelectPinMode(GPIOA, PIN0, OUTPUT);
	gpioSetOutputType(GPIOA, PIN0, PUSHPULL);
	gpioSelectPushPullMode(GPIOA, PIN0, PULLUP);

	gpioSelectPinMode(GPIOA, PIN1, OUTPUT);
	gpioSetOutputType(GPIOA, PIN1, PUSHPULL);
	gpioSelectPushPullMode(GPIOA, PIN1, PULLUP);

	 /* Initialise PID controller */
	PIDController pid = { PID_KP, PID_KI, PID_KD,
	                       PID_TAU,
	                       PID_LIM_MIN, PID_LIM_MAX,
						   PID_LIM_MIN_INT, PID_LIM_MAX_INT,
	                       SAMPLE_TIME_S };

	 PIDController_Init(&pid);

	 RCFilter_Init(&rcfilter, 5.0, SAMPLE_TIME_S);


	 static uint8_t overrun = 0;
	 static int count = 0;

	 configClock();
	 while (1)
	 {
		 if(timerTrigger == true)
		 {
			 systickUpdateTimerList((uint32_t *) timerList, arraySize);
		 }

		 if(isSystickExpired(pin0Timer))
		 {
			 if (overrun ==1)
			 {
				 count = count+1;
			 }
			 overrun = 1;
			 /* Get measurement from system */
			 measurement = TestSystem_Update(pid.out);

			 /* Compute new control signal */
			 PIDController_Update(&pid, setpoint, measurement);

			 /*noise*/
			 measurement_noise = measurement + 0.01*(rand()% 20);

			 /*RCFilter*/
			 measurement_filtered = RCFilter_Update(&rcfilter, measurement_noise);

			 overrun = 0;

			 timerGetCaptureComparePreloadValue(tim, TIMIO_CH1, &preloadVal);
			 if (1 == direction)
			 {
				 preloadVal = preloadVal * 105/100;
				 timerSetCaptureComparePreloadValue(tim, TIMIO_CH1, preloadVal);

				 if (preloadVal > 10000)
				 {
					 direction = 0;
				 }
			 }
			 else
			 {
				 preloadVal = preloadVal * 95/100;
				 timerSetCaptureComparePreloadValue(tim, TIMIO_CH1, preloadVal);

				 if (preloadVal < 100)
				 {
					 direction = 1;
				 }
			 }
			// gpioTogglePin(GPIOA,PIN0);
			 systickSetTicktime(&pin0Timer, 100);
		 }

		 if(isSystickExpired(pin1Timer))
		 {
		 	gpioTogglePin(GPIOA,PIN1);
		 	systickSetTicktime(&pin1Timer, 50);
		 }

	 }
}

void TIM6_DAC_IRQHandler (void)
{
	gpioTogglePin(GPIOA, PIN0);
}

void configClock(void)
{
    uint32_t pllInputFreq = HSE_VALUE / 1000000;    // Auswahl der Eingangsfrequenz der
                                                    // PLL-Schaltung.
    uint16_t mcuFreq = SYSCLOCK;                    // Taktfrequenz der MCU, z.B. 180 MHz

    // Konfiguration der Waitstates
    flashConfigWaitStates(mcuFreq);

    // Aktivieren der Taktquelle (HSE oder HSI)
    rccEnableHSE();

    // Wir verwenden PLLP als Ausgang der PLL-Schaltung
    rccSelectSysclkSrc(SYSCLKSRC_PLLP);

	#if 0
		// Alternative Funktionen zur Auswahl der SYSCLK-Quelle
		rccSelectSysclkSrc(SYSCLKSRC_HSI);
		rccSelectSysclkSrc(SYSCLKSRC_HSE);
		rccSelectSysclkSrc(SYSCLKSRC_PLLR);
	#endif

		// Auswahl der Taktquelle (HSE oder HSI)
		rccSelectPLLClockSource(PLL_SRC_HSE);

		// Optional: Hier wird SYSCLK an den Ausgang MCO1 weitergeleitet.
		// Hiermit können externe Bauelemente getaktet werden.
		rccAssignClk2MCO(PLL_MAIN, MCO1);

		// Berechnung der Werte von PLLN, PLLM and PLLP
		rccSetSysclkFreq(pllInputFreq, mcuFreq);

		// Auswahl der Prescaler zum Einstellen der AHB- und APB-Frequenzen
		rccSetAHBPrescaler(SYSCLK_DIV_1);
		rccSetAPB1Prescaler(AHB_DIV_4);					// APB1 = 45MHz
		rccSetAPB2Prescaler(AHB_DIV_2);					// APB2 = 90MHz

		// Nicht vergessen: PLL-Schaltkreis aktivieren
		rccEnableMainPLL();
}

float TestSystem_Update(float inp) {

    static float output = 0.0f;
    //static const float alpha = 0.02f;
    int K_PT1 = 1;
    int T_1 = 1;
    //output = (SAMPLE_TIME_S * inp + output) / (1.0f + alpha * SAMPLE_TIME_S);
    output = output +(((K_PT1*inp)-output)*SAMPLE_TIME_S)/(T_1+SAMPLE_TIME_S);
    return output;
}
