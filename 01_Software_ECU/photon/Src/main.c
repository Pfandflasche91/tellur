
#include <stdint.h>
#include <stdbool.h>

#include <stm32f4xx.h>
#include <mcalGPIO.h>
#include "PID.h"

void SysTick_Handler(void);

uint16_t tick = 0;


/* Controller parameters */
#define PID_KP  2.5f
#define PID_KI  25.0f
#define PID_KD  0.5f

#define PID_TAU 0.5f

#define PID_LIM_MIN -10.0f
#define PID_LIM_MAX  10.0f

#define PID_LIM_MIN_INT -5.0f
#define PID_LIM_MAX_INT  5.0f

#define SAMPLE_TIME_S 0.01f


/* Simulated dynamical system (first order) */
float TestSystem_Update(float inp);
float measurement = 0.0f;

int main(void)
{
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 1000); //systick = 1ms

	gpioInitPort(GPIOA);
	gpioSelectPinMode(GPIOA,PIN4,OUTPUT);
	gpioSetOutputType(GPIOA, PIN4,PUSHPULL);
	gpioSelectPushPullType(GPIOA, PIN4,PULLUP);

	 /* Initialise PID controller */
	PIDController pid = { PID_KP, PID_KI, PID_KD,
	                       PID_TAU,
	                       PID_LIM_MIN, PID_LIM_MAX,
						   PID_LIM_MIN_INT, PID_LIM_MAX_INT,
	                       SAMPLE_TIME_S };

	 PIDController_Init(&pid);

	 /* Simulate response using test system */
	 float setpoint = 1.0f;

	 static uint8_t overrun = 0;
	 static count = 0;

	 while (1)
	 {
		 if(tick>9)
		 {
			 if (overrun ==1)
			 {

				 count = count+1;
			 }

			 overrun = 1;
			 if(gpioGetPinVal(GPIOA,PIN4))
			 {
				 gpioResetPin(GPIOA, PIN4);
			 }
			 else
			 {
				 gpioSetPin(GPIOA,PIN4);
			 }
			 /* Get measurement from system */
			 measurement = TestSystem_Update(pid.out);

			 /* Compute new control signal */
			 PIDController_Update(&pid, setpoint, measurement);


			 tick=0;
			 overrun = 0;
		 }

	 }
}

void SysTick_Handler(void)
{
	++tick;
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
