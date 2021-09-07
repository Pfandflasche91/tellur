/**
 * mcalI2C.h
 *
 *  Created on: Sep 22, 2020
 *      Author: Ralf Jesse
 *       Email: embedded@ralf-jesse.de
 */

#ifndef MCALI2C_H_
#define MCALI2C_H_

/**
 * @note
 * The following definitions are not yet usable.
 */
//#define I2C_OK                      (0)
//#define I2C_TIMEOUT_FAILURE         (-1)
//#define I2C_SCL_FREQ_NOT_SUPPORTED  (-2)

typedef enum
{
    I2C_OK                      = 0,
    I2C_TIMEOUT_FAILURE         = - 1,
    I2C_SCL_FREQ_NOT_SUPPORTED  = - 2,
    I2C_UNKNOWN_SLAVE_ADDRESS   = -3
} I2C_STATE_t;

/**
 * @brief I2C enumerations
 */
typedef enum
{
    I2C_CLOCK_100 = 0,      // SCL = 100 kHz
    I2C_CLOCK_400           // SCL = 400 kHz
} I2C_CLOCKSPEED_t;

typedef enum
{
    I2C_DUTY_CYCLE_2 = 0,
    IC2_DUTY_CYCLE_16_9
} I2C_DUTY_CYCLE_t;

/**
 * Function prototypes
 */
// General functions
extern int8_t  i2cInit(I2C_TypeDef *i2c, uint32_t pclk, I2C_DUTY_CYCLE_t duty, uint8_t trise, I2C_CLOCKSPEED_t clock);
extern uint8_t i2cFindSlaveAddress(I2C_TypeDef *i2c, uint32_t pclk, I2C_DUTY_CYCLE_t duty, uint8_t trise, I2C_CLOCKSPEED_t clock);

// Sending functions
extern int8_t  i2cSendByte(I2C_TypeDef *i2c, uint8_t saddr, uint8_t data);
extern int8_t  i2cSendByteToSlaveReg(I2C_TypeDef *i2c, uint8_t saddr, uint8_t regAddr, uint8_t data);
extern int8_t  i2cBurstWrite(I2C_TypeDef *i2c, uint8_t saddr, uint8_t regAddr, uint8_t *data, uint8_t numBytes);

// Reading functions
extern int8_t  i2cReadByte(I2C_TypeDef *i2c, uint8_t saddr, uint8_t *data);
extern int8_t  i2cReadByteFromSlaveReg(I2C_TypeDef *i2c, uint8_t saddr, uint8_t regAddr, uint8_t *data);
extern int8_t  i2cBurstRead(I2C_TypeDef *i2c, uint8_t saddr, uint8_t regAddr, uint8_t *data, uint8_t num);



#endif /* MCALI2C_H_ */
