/**
 * @brief I2C-Anwendung mit dem Port-Expander PCF8574A unter Einsatz von I2C1.
 *
 * @par
 * Verwendete Anschlusspins:
 *     PB8 : I2C1 SCL
 *     PB9 : I2C1 SDA
 *
 * Hier wird der PCF8574AN verwendet.
 *
 * @note
 * Die Initialisierung der I2C-Komponenten ist derzeit noch nicht fertig und
 * wird weiterentwickelt. Die Parameter pclk, duty und trise werden ignoriert
 * und intern durch funktionierende Werte ersetzt. Annahme: Die Werte basieren
 * auf pclk = 16 MHz.
 */

/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>

#include <mcalSysTick.h>
#include <mcalGPIO.h>
#include <mcalI2C.h>
#include <mcalSystem.h>


/**
 * I2C-Adresse des PCF8574A.
 * Beachten Sie hier, dass andere Ausführungen des PCF8574 andere
 * Basisadressen haben!
 */
#define PCF8574A_ADDR       (0x70)

bool timerTrigger = false;

int main(void)
{
    uint32_t pclk1Freq = 0UL;

    uint32_t ledTimer = 0UL;
    uint8_t  ledPattern = 0xFF;
    uint8_t  ledPos = 0;

    systickInit(SYSTICK_1MS);
    systickSetMillis(&ledTimer, 500);

    // GPIOB-Bustakt aktivieren wegen der Verwendung von PB8/PB9.
    gpioInitPort(GPIOB);
    gpioSelectPinMode(GPIOB, PIN8, ALTFUNC);
    gpioSelectAltFunc(GPIOB, PIN8, AF4);            // PB8 : I2C1 SCL
    gpioSelectPinMode(GPIOB, PIN9, ALTFUNC);
    gpioSelectAltFunc(GPIOB, PIN9, AF4);            // PB9 : I2C1 SDA
    gpioSetOutputType(GPIOB, PIN8, OPENDRAIN);
    gpioSetOutputType(GPIOB, PIN9, OPENDRAIN);

    /**
     * Die beiden folgenden Zeilen sind nicht erforderlich, wenn
     * die Pull-Up-Widerstände auf der Platine vorhanden sind.
     */
//    gpioSelectPushPullType(GPIOB, PIN8, PULLUP);    // Verwendung des internen Pullup-Widerstandes
//    gpioSelectPushPullType(GPIOB, PIN9, PULLUP);    // Verwendung des internen Pullup-Widerstandes

    // Initialisierung des I2C-Controllers
    pclk1Freq = systemGetPclk1Freq();                 // Get APB1 peripheral clock
    i2cInit(I2C1, pclk1Freq, I2C_DUTY_CYCLE_2, 17, I2C_CLOCK_100);

    /* Hauptprogramm: Endlosschleife */
    while(1)
    {
        if (timerTrigger == TRUE)
        {
            DECREMENT_TIMER(ledTimer);
            timerTrigger = FALSE;
        }

        if (isSystickExpired(ledTimer))
        {
//          i2cReadByte(I2C1, PCF8574A_ADDR, &data);
            ledPattern &= ~(1 << ledPos++);
            i2cSendByte(I2C1, PCF8574A_ADDR, ledPattern);

            if (ledPos > 0x07)
            {
                ledPos = 0;
                ledPattern = 0xFF;
            }
            systickSetMillis(&ledTimer, 500);
        }
    }
}
