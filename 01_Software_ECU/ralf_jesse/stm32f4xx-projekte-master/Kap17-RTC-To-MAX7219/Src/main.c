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

#include <mcalGPIO.h>
#include <mcalSPI.h>
#include <mcalI2C.h>
#include <mcalSystem.h>
#include <ds3231RTC.h>
#include <max7219.h>
#include <mcalSysTick.h>

#include <global.h>

bool timerTrigger = false;

int main(void)
{
    uint32_t pclk1Freq = 0UL;
    uint8_t  seconds   = 0U;
    uint8_t  minutes   = 0U;
    uint8_t  hours     = 0U;
    uint32_t ds3231Timer;
    uint32_t max7219Timer;
    uint32_t *timerList[] = { &ds3231Timer, &max7219Timer };
    uint8_t  arraySize    = sizeof(timerList) / sizeof(uint32_t);


    // Systick-Timer: Initialisierung
    systickInit(SYSTICK_1MS);
    systickSetMillis(&ds3231Timer, 500);
    systickSetMillis(&max7219Timer, 500);

    // SPI: Wir verwenden GPIOA fuer die Kommunikation mit dem MAX7219.
    gpioInitPort(GPIOA);
    gpioSelectPinMode(MAX7219_CS_PORT, MAX7219_CS_PIN, OUTPUT);     // Port and pin are defined in global.h
    gpioSelectPinMode(GPIOA, PIN5, ALTFUNC);    // PA5 = SPI1 Clock
    gpioSelectAltFunc(GPIOA, PIN5, AF5);
    gpioSelectPinMode(GPIOA, PIN7, ALTFUNC);    // PA7 = SPI1 MOSI
    gpioSelectAltFunc(GPIOA, PIN7, AF5);

    // SPI: Basis-Initialisierung
//    spiInitSPI(SPI1, CLK_DIV_16, DATA_FORMAT_8, SSM_ON, SSI_LVL_HIGH, MASTER, SPI_PHASE_EDGE_1, SPI_IDLE_LOW);
    spiInitSPI(SPI1, CLK_DIV_16, DATA_FORMAT_16, SSM_ON, SSI_LVL_HIGH, MASTER, SPI_PHASE_EDGE_1, SPI_IDLE_LOW);

    // MAX7219: Initialisierung
    max7219Init(SPI1);
    max7219SetDecodeMode(SPI1, DECODE_ALL);
    max7219TurnOn(SPI1);
    max7219SetIntensity(SPI1, LVL_7);

    // I2C: Wir verwenden GPIOB.
    gpioInitPort(GPIOB);
    gpioSelectPinMode(GPIOB, PIN8, ALTFUNC);
    gpioSelectAltFunc(GPIOB, PIN8, AF4);            // PB8 : I2C1 SCL
    gpioSelectPinMode(GPIOB, PIN9, ALTFUNC);
    gpioSelectAltFunc(GPIOB, PIN9, AF4);            // PB9 : I2C1 SDA
    gpioSetOutputType(GPIOB, PIN8, OPENDRAIN);
    gpioSetOutputType(GPIOB, PIN9, OPENDRAIN);

    /**
     * Die beiden folgenden Zeilen muessen auskommentiert werden,
     * wenn Sie externe Pull-Up-Widerstände verwenden.
     */
    gpioSelectPushPullType(GPIOB, PIN8, PULLUP);    // Verwendung des internen Pullup-Widerstandes
    gpioSelectPushPullType(GPIOB, PIN9, PULLUP);    // Verwendung des internen Pullup-Widerstandes

    // Initialisierung des I2C-Controllers
    pclk1Freq = systemGetPclk1Freq();               // Get APB1 peripheral clock
    i2cInit(I2C1, pclk1Freq, I2C_DUTY_CYCLE_2, 17, I2C_CLOCK_100);

//    ds3231SetTime(I2C1, DS3231_ADDR, 13, 41, 00);

    while (1)
    {
        if (timerTrigger == true)
        {
            systickUpdateTimers((uint32_t *) timerList, arraySize);
        }

        if (isSystickExpired(ds3231Timer))
        {
            ds3231GetTime(I2C1, DS3231_ADDR, &hours, &minutes, &seconds);
            systickSetMillis(&ds3231Timer, 500);
        }

        if (isSystickExpired(max7219Timer))
        {
            spiSendData(SPI1, MAX7219_CS_PORT, MAX7219_CS_PIN, REG_DIG0, seconds % 10);
            spiSendData(SPI1, MAX7219_CS_PORT, MAX7219_CS_PIN, REG_DIG1, seconds / 10);

            spiSendData(SPI1, MAX7219_CS_PORT, MAX7219_CS_PIN, REG_DIG2, 0x8F);            // Dezimalpunkt

            spiSendData(SPI1, MAX7219_CS_PORT, MAX7219_CS_PIN, REG_DIG3, minutes % 10);
            spiSendData(SPI1, MAX7219_CS_PORT, MAX7219_CS_PIN, REG_DIG4, minutes / 10);

            spiSendData(SPI1, MAX7219_CS_PORT, MAX7219_CS_PIN, REG_DIG5, 0x8F);            // Dezimalpunkt

            spiSendData(SPI1, MAX7219_CS_PORT, MAX7219_CS_PIN, REG_DIG6, hours % 10);
            spiSendData(SPI1, MAX7219_CS_PORT, MAX7219_CS_PIN, REG_DIG7, hours / 10);

            systickSetMillis(&max7219Timer, 500);
         }
    }
}