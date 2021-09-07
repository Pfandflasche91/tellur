/**
 * @brief Collextion of functions for I2C components.
 *
 *  Created on: Sep 22, 2020
 *      Author: Ralf Jesse
 *       Email: embedded@ralf-jesse.de
 */

#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>

#include <mcalI2C.h>

/**
 * @brief Some macros which are permanently used to check the status registers
 *
 * @param[in]  i2c : Address of the I2C component
 *
 * @note
 * tout = internal timeout counter
 */
#define I2C_WAIT_BUSY(i2c)                  ( { while (i2c->SR2 & I2C_SR2_BUSY) ; } )
#define I2C_START_COMPLETED(i2c)            ( { while (!(i2c->SR1 & I2C_SR1_SB)) ; } )
#define I2C_STOPP_COMPLETED(i2c)            ( { while (!(i2c->SR1 & I2C_SR1_STOPF)) ; } )
#define I2C_ADDRESS_COMPLETED(i2c)          ( { while (!(i2c->SR1 & I2C_SR1_ADDR)) ; } )
#define I2C_DUMMY_READ_SR1(i2c)             ( { i2c->SR1; } )
#define I2C_DUMMY_READ_SR2(i2c)             ( { i2c->SR2; } )
#define I2C_CHECK_TXBUF_EMPTY(i2c)          ( { while(!(i2c->SR1 & I2C_SR1_TXE)) ; } )
#define I2C_CHECK_RXBUF_NOT_EMPTY(i2c)      ( { while(!(i2c->SR1 & I2C_SR1_RXNE)) ; } )
#define I2C_BYTE_TRANSFER_FINISHED(i2c)     ( { while(!(i2c->SR1 & I2C_SR1_BTF)) ; })
#define I2C_RESET_ACK(i2c)                  ( { i2c->CR1 &= ~I2C_CR1_ACK_Msk; } )
#define I2C_SET_ACK(i2c)                    ( { i2c->CR1 |= I2C_CR1_ACK; } )
#define I2C_SET_POS(i2c)                    ( { i2c->CR1 |= I2C_CR1_POS; } )
#define I2C_RESET_POS(i2c)                  ( { i2c->CR1 &= ~I2C_CR1_POS_Msk; } )

/**
 * @brief Initializes the I2C component of the STM32F4xx
 *
 * @param[in]  *i2c   : Pointer to the I2C component
 * @param[in]   pclk
 * @param[in]   trise : Maximum rise time of the clock
 * @param[in]   clock : I2C clock frequency (100/400 kHz)
 *
 * @note
 * This function is not yet complete. We assume that the peripheral clock is 16 MHz! Although
 * it is required to pass all arguments, pclk/duty/trise are ignored and replaced by working
 * parameters. We assume that the peripheral clock is 16 MHz.
 */
int8_t i2cInit(I2C_TypeDef *i2c, uint32_t pclk, I2C_DUTY_CYCLE_t duty,
        uint8_t trise, I2C_CLOCKSPEED_t clock)
{
    // Activate bus clock
    if (I2C1 == i2c)
    {
        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    }
    else if (I2C2 == i2c)
    {
        RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    }
    else if (I2C3 == i2c)
    {
        RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
    }

    i2c->CR1 &= ~I2C_CR1_PEC_Msk;       // Disable I2C component
    i2c->CR1 = 0x0000;                  // Reset old CR1 settings
    i2c->CR2 = pclk / 1000000;          // Attention: Needs to be tested with other pclk values

    i2c->CCR = 0x00;                    // Reset Clock Control Register
    if (I2C_CLOCK_100 == clock)
    {
        i2c->CCR &= ~I2C_CCR_FS_Msk;    // Select 100 kHz bus clock
        i2c->CCR |= 0x0050;
    }
    else if (I2C_CLOCK_400 == clock)
    {
        i2c->CCR |= I2C_CCR_FS;         // Select 400 kHz bus clock
    }
    else
    {
        return I2C_SCL_FREQ_NOT_SUPPORTED;
    }

    I2C1->TRISE = 0x0011;               // Set max. rise time
    I2C1->CR1 |= I2C_CR1_PE;            // Re-renable I2C component

    return I2C_OK;
}

/**
 * @brief Sends a byte to the I2C slave w/o internal registers immediately.
 *
 * @param[in]  *i2c   : Pointer to the component
 * @param[in]   saddr : Address of the I2C slave
 * @param[in]   data  : Byte that shall be sent
 *
 * @param[out]  value : 0 = Transfer successful / < 0 = Transmission failure
 *
 * @note
 * Failure handling is not yet implemented. This function shall be used
 * in the case when the addresses I2C component provides only one internal
 * register.
 */
int8_t i2cSendByte(I2C_TypeDef *i2c, uint8_t saddr, uint8_t data)
{
    I2C_WAIT_BUSY(i2c);                 // Checks whether the I2C bus is busy

    i2c->CR1 |= I2C_CR1_START;          // Send I2C START signal
    I2C_START_COMPLETED(i2c);           // Wait until START signal has been sent

    i2c->DR = saddr;                    // Send slave address
    I2C_ADDRESS_COMPLETED(i2c);         // Wait for ADDR ACK

    I2C_DUMMY_READ_SR1(i2c);            // Reset SR1
    I2C_DUMMY_READ_SR2(i2c);            // Reset SR2

    I2C_CHECK_TXBUF_EMPTY(i2c);         // Wait until the transmit buffer is empty

    i2c->DR = data;                     // Send data

    I2C_BYTE_TRANSFER_FINISHED(i2c);    // Wait until BTF Flag is set

    i2c->CR1 |= I2C_CR1_STOP;           // Send STOP signal

    return I2C_OK;
}

/**
 * @brief Sends a byte to a functional register of the I2C slave.
 *
 * @param[in]  *i2c     : Pointer to the component
 * @param[in]   saddr   : Address of the I2C slave
 * @param[in]   regAddr : Address of the slave register
 * @param[in]   data    : Byte that shall be sent
 *
 * @param[out]  value : 0 = Transfer successful / < 0 = Transmission failure
 *
 * @note
 * Failure handling is not yet implemented
 */
int8_t i2cSendByteToSlaveReg(I2C_TypeDef *i2c, uint8_t saddr, uint8_t regAddr,
        uint8_t data)
{
    I2C_WAIT_BUSY(i2c);                 // Checks whether the I2C bus is busy

    i2c->CR1 |= I2C_CR1_START;          // Send I2C START signal
    I2C_START_COMPLETED(i2c);           // Wait until START signal has been sent

    i2c->DR = saddr;                    // Send slave address
    I2C_ADDRESS_COMPLETED(i2c);         // Wait for ADDR ACK

    I2C_DUMMY_READ_SR1(i2c);            // Reset SR1
    I2C_DUMMY_READ_SR2(i2c);            // Reset SR2

    I2C_CHECK_TXBUF_EMPTY(i2c);         // Wait until the transmit buffer is empty

    i2c->DR = regAddr;                  // Set the address of the slave register
    I2C_CHECK_TXBUF_EMPTY(i2c);         // Wait until the transmit buffer is empty

    i2c->DR = data;                     // Send data

    I2C_BYTE_TRANSFER_FINISHED(i2c);    // Wait until BTF Flag is set

    i2c->CR1 |= I2C_CR1_STOP;           // Send STOP signal

    return I2C_OK;
}

/**
 * @brief Burst write data to I2C slave.
 *
 * @param[in]  *i2c     : Pointer to the component
 * @param[in]   saddr   : Address of the I2C slave
 * @param[in]   regAddr : Address of the slave register
 * @param[in]   data    : Byte that shall be sent
 *
 * @param[out]  value : 0 = Transfer successful / < 0 = Transmission failure
 *
 * @note
 * Failure handling is not yet implemented
 */
int8_t i2cBurstWrite(I2C_TypeDef *i2c, uint8_t saddr, uint8_t regAddr, uint8_t *data, uint8_t len)
{
    uint8_t numBytes = 0;

    I2C_WAIT_BUSY(i2c);                 // Checks whether the I2C bus is busy

    I2C_RESET_POS(i2c);                 // May only be active in 16-bit mode
    i2c->CR1 |= I2C_CR1_START;          // Send I2C START signal
    I2C_START_COMPLETED(i2c);           // Wait until START signal has been sent

    i2c->DR = saddr;                    // Send slave address
    I2C_ADDRESS_COMPLETED(i2c);         // Wait for ADDR ACK

    I2C_DUMMY_READ_SR1(i2c);            // Reset SR1
    I2C_DUMMY_READ_SR2(i2c);            // Reset SR2

    I2C_CHECK_TXBUF_EMPTY(i2c);         // Data may only be sent when the TX buffer is empty

    i2c->DR = regAddr;                  // This is the address of the first internal register
    while (numBytes < len)
    {
        I2C_CHECK_TXBUF_EMPTY(i2c);
        i2c->DR = *data++;
        numBytes++;
    }

    I2C_BYTE_TRANSFER_FINISHED(i2c);   // Wait until BTF Flag is set

    i2c->CR1 |= I2C_CR1_STOP;          // Send STOP signal

    return I2C_OK;
}

/**
 * @brief Sends a byte to the I2C slave w/o internal registers immediately.
 *
 * @param[in]  *i2c   : Pointer to the component
 * @param[in]   saddr : Address of the I2C slave
 * @param[in]  *data  : Pointer to the variable where the data shall be stored
 *
 * @param[out]  value : 0 = Transfer successful / < 0 = Transmission failure
 *
 * @note
 * Failure handling is not yet implemented.
 */
int8_t i2cReadByte(I2C_TypeDef *i2c, uint8_t saddr, uint8_t *data)
{
    I2C_WAIT_BUSY(i2c);                 // Checks whether the I2C bus is busy

    i2c->CR1 |= I2C_CR1_START;          // Generate I2C START signal
    I2C_START_COMPLETED(i2c);           // Wait until START signal has been sent

    i2c->DR = saddr;                    // Send with bit 0 = '0'
    I2C_ADDRESS_COMPLETED(i2c);         // Wait for ADDR ACK

    I2C_DUMMY_READ_SR1(i2c);            // Reset SR1
    I2C_DUMMY_READ_SR2(i2c);            // Reset SR2

    I2C_CHECK_TXBUF_EMPTY(i2c);         // Wait until transmit buffer is empty

    i2c->CR1 |= I2C_CR1_START;          // Generate I2C RESTART
    I2C_START_COMPLETED(i2c);           // Checks whether the START signal has been sent

    i2c->DR = saddr | 1;                // Resend slave addr with bit 0 = '1'
    I2C_ADDRESS_COMPLETED(i2c);         // Wait for ADDR ACK

    I2C_RESET_ACK(i2c);                 // Disable Acknowledge

    I2C_DUMMY_READ_SR1(i2c);            // Reset SR1
    I2C_DUMMY_READ_SR2(i2c);            // Reset SR2

    i2c->CR1 |= I2C_CR1_STOP;           // Send I2C STOP signal

    I2C_CHECK_RXBUF_NOT_EMPTY(i2c);     // Wait until receive buffer is no longer empty

    *data = i2c->DR;                    // Write data in variable

    return I2C_OK;
}

/**
 * @brief Reads a byte from a functional register of the I2C slave.
 *
 * @param[in]  *i2c     : Pointer to the component
 * @param[in]   saddr   : Address of the I2C slave
 * @param[in]   regAddr : Address of the slave register
 * @param[in]  *data    : Address where the data shall be stored
 *
 * @param[out]  value : 0 = Transfer successful / < 0 = Transmission failure
 *
 * @note
 * Failure handling is not yet implemented. This function shall be used
 * in the case when the addresses I2C component provides only one internal
 * register.
 */
int8_t i2cReadByteFromSlaveReg(I2C_TypeDef *i2c, uint8_t saddr, uint8_t regAddr, uint8_t *data)
{
    I2C_WAIT_BUSY(i2c);                 // Checks whether the I2C bus is busy

    i2c->CR1 |= I2C_CR1_START;          // Send I2C START signal
    I2C_START_COMPLETED(i2c);           // Wait until START signal has been sent

    i2c->DR = saddr;                    // Send with bit 0 = '0'
    I2C_ADDRESS_COMPLETED(i2c);         // Wait for ADDR ACK

    I2C_DUMMY_READ_SR1(i2c);            // Reset SR1
    I2C_DUMMY_READ_SR2(i2c);            // Reset SR2

    i2c->DR = regAddr;                  // Send address of the functional register
    I2C_CHECK_TXBUF_EMPTY(i2c);         // Wait until transmit buffer is empty

    i2c->CR1 |= I2C_CR1_START;          // Generate I2C RESTART
    I2C_START_COMPLETED(i2c);           // Checks whether the START signal has been sent

    i2c->DR = saddr | 1;                // Resend slave addr with bit 0 = '1'
    I2C_ADDRESS_COMPLETED(i2c);         // Wait for ADDR ACK

    I2C_RESET_ACK(i2c);                 // Disable Acknowledge

    I2C_DUMMY_READ_SR1(i2c);            // Reset SR1
    I2C_DUMMY_READ_SR2(i2c);            // Reset SR2

    i2c->CR1 |= I2C_CR1_STOP;           // Send I2C STOP signal

    I2C_CHECK_RXBUF_NOT_EMPTY(i2c);     // Wait until receive buffer is no longer empty
    *data++ = i2c->DR;                  // Write data in variable

    return I2C_OK;
}

/**
 * @brief Burst read from I2C slave.
 *
 * @param[in]  *i2c     : Pointer to the component
 * @param[in]   saddr   : Address of the I2C slave
 * @param[in]   regAddr : Address of the first slave register
 * @param[in]  *data    : Address where the data shall be stored
 *
 * @param[out]  value : 0 = Transfer successful / < 0 = Transmission failure
 *
 * @note
 * Failure handling is not yet implemented
 */
int8_t i2cBurstRead(I2C_TypeDef *i2c, uint8_t saddr, uint8_t regAddr, uint8_t *data, uint8_t num)
{
    I2C_WAIT_BUSY(i2c);
    I2C_RESET_POS(i2c);                     // Must be used only in 16-bit transfer

    i2c->CR1 |= I2C_CR1_START;              // Send I2C START signal
    I2C_START_COMPLETED(i2c);               // Wait until START signal has been sent

    i2c->DR = saddr;                        // Send with bit 0 = '0'
    I2C_ADDRESS_COMPLETED(i2c);             // Wait for ADDR ACK

    I2C_DUMMY_READ_SR1(i2c);                // Reset SR1
    I2C_DUMMY_READ_SR2(i2c);                // Reset SR2

    I2C_CHECK_TXBUF_EMPTY(i2c);             // Wait until transmit buffer is empty
    i2c->DR = regAddr;                      // Send address of the functional register

    I2C_CHECK_TXBUF_EMPTY(i2c);             // Wait until transmit buffer is empty
    i2c->CR1 |= I2C_CR1_START;              // Generate I2C RESTART
    I2C_START_COMPLETED(i2c);               // Checks whether the START signal has been sent

    i2c->DR = saddr | 1;                    // Resend slave addr with bit 0 = '1'
    I2C_ADDRESS_COMPLETED(i2c);             // Wait for ADDR ACK

    I2C_DUMMY_READ_SR1(i2c);                // Reset SR1
    I2C_DUMMY_READ_SR2(i2c);                // Reset SR2

    I2C_SET_ACK(i2c);                       // Enable Acknowledge

    while(num > 0)                          // Start reading multiple values
    {
        if (num == 1U)                      // If there is only one byte left...
        {
            I2C_RESET_ACK(i2c);             // Disable acknowledge
            i2c->CR1 |= I2C_CR1_STOP;       // Generate STOP signal
            I2C_CHECK_RXBUF_NOT_EMPTY(i2c); // Wait until receive buffer is no longer empty
            *data++ = i2c->DR;              // Read data from data register
            break;
        }
        else                                // More than one byte left
        {
            I2C_CHECK_RXBUF_NOT_EMPTY(i2c); // Wait until receive buffer is no longer empty
            (*data++) = i2c->DR;            // Read data from data register
            num--;
        }
    }

    return I2C_OK;
}

#if 0
/* Try to find the I2C address of the display */
uint8_t i2cFindSlaveAddress(I2C_TypeDef *i2c, uint32_t pclk, I2C_DUTY_CYCLE_t duty, uint8_t trise, I2C_CLOCKSPEED_t clock)
{
    uint8_t devAddr = 0;
    int8_t  addressTable[128] = { 0 };
    uint8_t index = 0;

    for (devAddr = 0x40; devAddr < 0x80; devAddr++)
    {
        I2C_WAIT_BUSY(i2c);
        i2c->CR1 |= I2C_CR1_START;
        I2C_START_COMPLETED(i2c);
        i2c->DR   = devAddr;
        if (i2c->SR1 & I2C_SR1_ADDR)
        {
            addressTable[index++] = devAddr;
        }
        i2c->DR |= I2C_CR1_STOP;
    }

    return devAddr;
}
#endif
