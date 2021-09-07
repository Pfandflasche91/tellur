/**
 * ds3231RTC.c
 *
 *  Created on: Oct 2, 2020
 *      Author: Ralf Jesse
 *       Email: embedded@ralf-jesse.de
 */

#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>

#include <mcalI2C.h>
#include <ds3231RTC.h>


/**
 * @brief Aktiviert bzw. deaktiviert den Blinkmodus der RTC.
 *
 * @param[in]  *i2c       : Pointer auf die I2C-Komponente
 * @param[in]   slaveAddr : I2C-Adresse der RTC
 * @param[in]   reg       : Adresse des RTC-Registers
 * @param[in]   mode      : Verwendet die Enumeration RTC_BLINK_MODE
 *
 * @note
 * Die LED kann nur blinken, wenn der SQW-Ausgang der RTC ueber einen Pullup-Widerstand
 * mit der Versorgungsspannung verbunden ist!
 */
void ds3231SetBlinkMode (I2C_TypeDef *i2c, uint8_t slaveAddr, RTC_BLINK_MODE_t mode)
{
    if (RTC_BLINK_OFF == mode)
    {
        i2cSendByteToSlaveReg(i2c, slaveAddr, DS3231_CTRL, 4);
    }
    else
    {
        i2cSendByteToSlaveReg(i2c, slaveAddr, DS3231_CTRL, 0);
    }
}

/**
 * @brief Einstellung von Stunden/Minuten/Sekunden
 *
 * @param[in]  *i2c       : Pointer auf die I2C-Komponente
 * @param[in]   slaveAddr : I2C-Adresse der RTC
 * @param[in]   hours     : Stunden
 * @param[in]   minutes   : Minuten
 * @param[in]   seconds   : Sekunden
 */
void ds3231SetTime(I2C_TypeDef *i2c, uint8_t slaveAddr, uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    uint8_t time[3] = { 0 };

    time[0] = ds3231DecToBcd(seconds);
    time[1] = ds3231DecToBcd(minutes);
    time[2] = ds3231DecToBcd(hours);

    i2cBurstWrite(i2c, slaveAddr, DS3231_SECONDS, time, 3);
}

/**
 * @brief Abfrage der aktuellen Uhrzeit
 *
 * @param[in]  *i2c       : Pointer auf die I2C-Komponente
 * @param[in]   slaveAddr : I2C-Adresse der RTC
 * @param[in]  *time      : Pointer auf DS3231_TIME_t
 */
void ds3231GetTime(I2C_TypeDef *i2c, uint8_t slaveAddr,
                   uint8_t *hours, uint8_t *minutes, uint8_t *seconds)
{
    uint8_t time[3] = { 0 };

    i2cBurstRead(i2c, slaveAddr, DS3231_SECONDS, time, 3);

    *seconds = ds3231BcdToDec(time[0]);
    *minutes = ds3231BcdToDec(time[1]);
    *hours   = ds3231DecodeHours(time[2]);
}

/**
 * @brief Ermittelt die aktuelle Temperatur
 *
 * @param[in]  *i2c       : Pointer auf die I2C-Komponente
 * @param[in]   slaveAddr : I2C-Adresse der RTC
 * @param[in]  *tempUpper : Pointer auf den Ganzzahlteil der Temperatur
 * @param[in]  *tempLower : Pointer auf den Nachkommateil der Temperatur
 */
void ds3231GetTemperature(I2C_TypeDef *i2c, uint8_t slaveAddr, uint8_t *tempSign, uint8_t *tempUpper, float *tempLower)
{
    uint8_t tl = 0;                         // Temporaerer Wert fuer den Nachkommateil

    i2cReadByteFromSlaveReg(i2c, slaveAddr, DS3231_TEMP_UPPER, tempUpper);
    *tempSign   = *tempUpper & 0x80;        // Temperatur-Vorzeichen = Bit 7 von tempUpper
    *tempUpper &= 0x7F;                     // Temperatur: Ganzzahliger Anteil = Bits 6:0

    i2cReadByteFromSlaveReg(i2c, slaveAddr, DS3231_TEMP_LOWER, &tl);
    tl >>= 6;                               // Temperatur: Nachkommateil (muss noch mit 0.25 multipliziert werden)
    *tempLower = tl * 0.25 * 10;            // Uns interessiert nur der Wert ohne Komma
}

uint8_t ds3231DecToBcd(uint8_t decValue)
{
    uint8_t encoded = ((decValue / 10) << 4) + (decValue % 10);

    return encoded;
}

uint8_t ds3231BcdToDec(uint8_t bcdValue)
{
    uint8_t decoded = bcdValue & 127;

    decoded = (decoded & 0x0F) + 10 * ((decoded & (0x0F << 4)) >> 4);

    return decoded;
}

uint8_t ds3231DecodeHours(uint8_t hours)
{
    if (hours & 0x80)
    {
        hours = (hours & 0x0F) + 12 * ((hours & 0x20) >> 5);
    }
    else
    {
        hours = (hours & 0x0F) + 10 * ((hours & 0x30) >> 4);
    }

    return hours;
}
