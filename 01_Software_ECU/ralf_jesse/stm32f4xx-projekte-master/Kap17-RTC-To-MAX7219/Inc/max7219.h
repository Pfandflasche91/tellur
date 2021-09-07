/**
 * max7219.h
 *
 *  Created on: 17.10.2020
 *      Author: Ralf Jesse
 *       Email: embedded@ralf-jesse.de
 */

#ifndef MAX7219_H_
#define MAX7219_H_

#include <stm32f4xx.h>
#include <mcalGPIO.h>

#define MAX7219_CS_PORT    (GPIOA)
#define MAX7219_CS_PIN     (PIN4)

typedef enum
{
    // Use this when using 8-bit transfer
    REG_NOOP        =  0,
    REG_DIG0,
    REG_DIG1,
    REG_DIG2,
    REG_DIG3,
    REG_DIG4,
    REG_DIG5,
    REG_DIG6,
    REG_DIG7,
    REG_DECODE,
    REG_INTENSITY,
    REG_SCAN_LIMIT,
    REG_SHUTDOWN,
    REG_DISPTEST    = 15,
} MAX7219_REG_t;

typedef enum
{
    LVL_MIN         = 0,
    LVL_1,
    LVL_2,
    LVL_3,
    LVL_4,
    LVL_5,
    LVL_6,
    LVL_7,
    LVL_8,
    LVL_9,
    LVL_10,
    LVL_11,
    LVL_12,
    LVL_13,
    LVL_14,
    LVL_MAX
} MAX7219_INTENSITY_t;

typedef enum
{
    TEST_OFF        = 0,
    TEST_ON
} MAX7219_TESTMODE_t;

typedef enum
{
    SHUTDOWN = 0,
    NORMAL_OP
} MAX7219_OPMODE_t;

typedef enum
{
    NO_DECODE           = 0,
    DECODE_DIG0         = 1,
    DECODE_DIG0_TO_DIG3 = 0x0F,
    DECODE_ALL          = 0xFF
} MAX7219_DECODE_t;

extern void max7219Init(SPI_TypeDef *spi);
extern void max7219Test(SPI_TypeDef *spi, MAX7219_TESTMODE_t testMode);
extern void max7219Shutdown(SPI_TypeDef *spi);
extern void max7219TurnOn(SPI_TypeDef *spi);
extern void max7219SetDecodeMode(SPI_TypeDef *spi, MAX7219_DECODE_t mode);
extern void max7219TestDigits(SPI_TypeDef *spi, uint8_t value);
extern void max7219ResetDigits(SPI_TypeDef *spi, uint8_t value);
extern void max7219Clear(SPI_TypeDef *spi);
extern void max7219SetIntensity(SPI_TypeDef *spi, MAX7219_INTENSITY_t lvl);

#endif /* MAX7219_H_ */
