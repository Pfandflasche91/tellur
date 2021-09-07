/**
 * ds3231RTC.h
 *
 *  Created on: Oct 2, 2020
 *      Author: Ralf Jesse
 *       Email: embedded@ralf-jesse.de
 */

#ifndef DS3231RTC_H_
#define DS3231RTC_H_

#define DS3231_ADDR     (0xD0)      // Default address is 0x68.

/**
 * Enumeration of the DS3231 registers
 */
typedef enum
{
    DS3231_SECONDS            = 0,  // 00H: Seconds
    DS3231_MINUTES,                 // 01H: Minutes
    DS3231_HOURS,                   // 02H: Sets hours but also AM/PM or 24h mode
    DS3231_DAY_OF_WEEK,             // 03H: Values are from 1 to 7
    DS3231_DAY_OF_MONTH,            // 04H: Automatically corrected for months with < 31 days
    DS3231_MONTH_AND_CENTURY,       // 05H: Century + Months
    DS3231_YEAR,                    // 06H: Current year (00 - 99)
    DS3231_AM1M1,                   // 07H: All AM register define the conditions for alarms (see datasheet)
    DS3231_AM1M2,                   // 08H:
    DS3231_AM1M3,                   // 09H:
    DS3231_AM1M4,                   // 0AH:
    DS3231_AM2M2,                   // 0BH:
    DS3231_AM2M3,                   // 0CH:
    DS3231_AM2M4,                   // 0DH:
    DS3231_CTRL,                    // 0EH: Sets the operation mode of the RTC (see datasheet)
    DS3231_STATUS,                  // 0FH: Status register
    DS3231_XTAL_AGING_OFFS,         // 10H: Offset to correct XTAL aging
    DS3231_TEMP_UPPER,              // 11H: Temperature sensor integer part (upper) (Read only)
    DS3231_TEMP_LOWER               // 12H: Temperature sensor fractional part in steps of 0.25°C
} DS3231_REGISTER_t;

/**
 *
 */
typedef enum
{
    RTC_BLINK_OFF = 0,
    RTC_BLINK_ON
} RTC_BLINK_MODE_t;

extern void     ds3231SetTime(I2C_TypeDef *i2c, uint8_t slaveAddr, uint8_t hours, uint8_t minutes, uint8_t seconds);
extern void     ds3231GetTime(I2C_TypeDef *i2c, uint8_t slaveAddr, uint8_t *hours, uint8_t *minutes, uint8_t *seconds);

extern void     ds3231SetDate(I2C_TypeDef *i2c, uint8_t slaveAddr, uint8_t hours, uint8_t minutes, uint8_t seconds);
extern void     ds3231GetDate(I2C_TypeDef *i2c, uint8_t slaveAddr, uint8_t hours, uint8_t minutes, uint8_t seconds);

extern void     ds3231SetBlinkMode (I2C_TypeDef *i2c, uint8_t slaveAddr, RTC_BLINK_MODE_t mode);
extern void     ds3231GetTemperature(I2C_TypeDef *i2c, uint8_t slaveAddr, uint8_t *tempSign, uint8_t *tempUpper, float *tempLower);

extern uint8_t  ds3231DecToBcd(uint8_t decVal);
extern uint8_t  ds3231BcdToDec(uint8_t bcdVal);
extern uint8_t  ds3231DecodeHours(uint8_t hours);
extern uint8_t  ds3231DecodeYear(uint8_t year);


#endif /* DS3231RTC_H_ */
