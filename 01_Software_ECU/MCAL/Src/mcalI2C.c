/**
 * @defgroup iic  Inter-integrated Circuit (I2C) Functions (mcalIIC.h/.c)
 * @defgroup iic0 Deprecated IIC Functions
 * @ingroup  iic
 * @defgroup iic1 IIC Macros
 * @ingroup  iic
 * @defgroup iic2 IIC Standard Functions
 * @ingroup  iic
 * @defgroup iic3 Advanced IIC Functions
 * @ingroup  iic
 * @defgroup iic4 IIC Enumerations and Definitions
 * @ingroup  iic
 *
 * @file        mcalI2C.c
 * @brief       mcalI2C.c is part of the MCAL library for STM32F4xx.
 * @author      Dipl.-Ing. Ralf Jesse (embedded@ralf-jesse.de)
 * @date        Nov. 12, 2020
 *
 * @version     0.1
 * @copyright   GNU Public License Version 3 (GPLv3)
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License, or any later version.<br>
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.<br>
 * You should have received a copy of the GNU General Public License along with this program. If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>

#include <mcalI2C.h>

/**
 * Some macros which are permanently used to check the status registers
 *
 * @param  i2c : Address of the I2C component
 *
 * @note
 * tout = internal timeout counter
 */
/**
 * @ingroup iic1
 */
#define I2C_WAIT_BUSY(i2c)                  ( { while (i2c->SR2 & I2C_SR2_BUSY) ; } )
/**
 * @ingroup iic1
 */
#define I2C_START_COMPLETED(i2c)            ( { while (!(i2c->SR1 & I2C_SR1_SB)) ; } )
/**
 * @ingroup iic1
 */
#define I2C_STOPP_COMPLETED(i2c)            ( { while (!(i2c->SR1 & I2C_SR1_STOPF)) ; } )
/**
 * @ingroup iic1
 */
#define I2C_ADDRESS_COMPLETED(i2c)          ( { while (!(i2c->SR1 & I2C_SR1_ADDR)) ; } )
/**
 * @ingroup iic1
 */
#define I2C_DUMMY_READ_SR1(i2c)             ( { i2c->SR1; } )
/**
 * @ingroup iic1
 */
#define I2C_DUMMY_READ_SR2(i2c)             ( { i2c->SR2; } )
/**
 * @ingroup iic1
 */
#define I2C_CHECK_TXBUF_EMPTY(i2c)          ( { while(!(i2c->SR1 & I2C_SR1_TXE)) ; } )
/**
 * @ingroup iic1
 */
#define I2C_CHECK_RXBUF_NOT_EMPTY(i2c)      ( { while(!(i2c->SR1 & I2C_SR1_RXNE)) ; } )
/**
 * @ingroup iic1
 */
#define I2C_BYTE_TRANSFER_FINISHED(i2c)     ( { while(!(i2c->SR1 & I2C_SR1_BTF)) ; })
/**
 * @ingroup iic1
 */
#define I2C_RESET_ACK(i2c)                  ( { i2c->CR1 &= ~I2C_CR1_ACK_Msk; } )
/**
 * @ingroup iic1
 */
#define I2C_SET_ACK(i2c)                    ( { i2c->CR1 |= I2C_CR1_ACK; } )
/**
 * @ingroup iic1
 */
#define I2C_SET_POS(i2c)                    ( { i2c->CR1 |= I2C_CR1_POS; } )
/**
 * @ingroup iic1
 */
#define I2C_RESET_POS(i2c)                  ( { i2c->CR1 &= ~I2C_CR1_POS_Msk; } )

/**
 * Function to verify the I2C interface.
 */
static bool i2cVerifyI2C(I2C_TypeDef *i2c)
{
    if ((I2C1 == i2c) || (I2C2 == i2c) || (I2C3 == i2c))
    {
        return true;
    }
    return false;
}

/**
 * Function to verify the I2C duty cycle.
 */
static bool i2cVerifyDutyCycle(I2C_DUTY_CYCLE_t dc)
{
    if ((I2C_DUTY_CYCLE_2 == dc) || (IC2_DUTY_CYCLE_16_9 == dc))
    {
        return true;
    }
    return false;
}

/**
 * Function to verify the I2C clock frequency.
 */
static bool i2cVerifyClkSpd(I2C_CLOCKSPEED_t spd)
{
    if ((I2C_CLOCK_100 == spd) || (I2C_CLOCK_400 == spd))
    {
        return true;
    }
    return false;
}

/**
 * @ingroup iic0
 * Initializes the I2C component of the STM32F4xx
 *
 * @param  *i2c   : Pointer to the I2C component
 * @param   pclk
 * @param   duty
 * @param   trise : Maximum rise time of the clock
 * @param   clock : I2C clock frequency (100/400 kHz)
 *
 * @note
 * This function is not yet complete. We assume that the peripheral clock is 16 MHz! Although
 * it is required to pass all arguments, pclk/duty/trise are ignored and replaced by working
 * parameters. We assume that the peripheral clock is 16 MHz.
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>APB1ENR</td>
 *          <td rowspan="1">I2C3...I2C1</td>
 *          <td rowspan="1">23...21</td>
 *      </tr>
 * </table>
 */
I2C_RETURN_CODE_t i2cInitI2C(I2C_TypeDef *i2c, uint32_t pclk, I2C_DUTY_CYCLE_t duty, uint8_t trise, I2C_CLOCKSPEED_t clock)
{
    if (i2cVerifyDutyCycle(duty) != true)
    {
        return I2C_INVALID_DUTY_CYCLE;
    }
    if (i2cVerifyClkSpd(clock) != true)
    {
        return I2C_INVALID_CLOCK_SPEED;
    }

    i2c->CR1 = 0x0000;                  // Reset old CR1 settings
    i2c->CR1 &= ~I2C_CR1_PE_Msk;        // Disable I2C component
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

    I2C1->TRISE = 0x0011;               // Set max. rise time
    I2C1->CR1 |= I2C_CR1_PE;            // Re-renable I2C component

    return I2C_OK;
}

/**
 * @ingroup iic2
 * Activates the bus clock of the I2C component
 *
 * @param  *i2c   : Pointer to the I2C component
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>APB1ENR</td>
 *          <td rowspan="1">I2C3...I2C1</td>
 *          <td rowspan="1">23...21</td>
 *      </tr>
 * </table>
 */
I2C_RETURN_CODE_t i2cSelectI2C(I2C_TypeDef *i2c)
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

    return I2C_OK;
}

/**
 * @ingroup iic2
 * Deinitializes the desired I2C component (turns bus clock off)
 *
 * @param  *i2c   : Pointer to the I2C component
 *
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>APB1ENR</td>
 *          <td rowspan="1">I2C3...I2C1</td>
 *          <td rowspan="1">23...21</td>
 *      </tr>
 * </table>
 */I2C_RETURN_CODE_t i2cDeselectI2C(I2C_TypeDef *i2c)
{
    if (i2cVerifyI2C(i2c) != true)
    {
        return I2C_INVALID_TYPE;
    }

    // Deactivate bus clock
    if (I2C1 == i2c)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN_Msk;
    }
    else if (I2C2 == i2c)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_I2C2EN_Msk;
    }
    else if (I2C3 == i2c)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_I2C3EN_Msk;
    }
    return I2C_OK;
}

/**
 * @ingroup iic2
 * Sends a byte to the I2C slave w/o internal registers immediately.
 *
 * @param  *i2c   : Pointer to the component
 * @param   saddr : Address of the I2C slave
 * @param   data  : Byte that shall be sent
 *
 * @return I2C_RETURN_CODE_t
 *
 * @note
 * Failure handling is not yet implemented. This function shall be used
 * in the case when the desired I2C component provides only one internal
 * register.
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>SR2</td>
 *          <td rowspan="1">BUSY</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Check whether the component is busy</td>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">START</td>
 *          <td rowspan="1">8</td>
 *          <td rowspan="1">Send START signal</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">START</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Wait until the START signal was generated</td>
 *      </tr>
 *      <tr>
 *          <td>DR</td>
 *          <td rowspan="1">I2C slave address</td>
 *          <td rowspan="1">7...0</td>
 *          <td rowspan="1">Send the slave address</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">ADDR</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Wait until the slave address was acknowledged</td>
 *      </tr>
 *      <tr>
 *          <td>SR1/SR2</td>
 *          <td rowspan="1">Dummy read of SR1/SR2</td>
 *          <td rowspan="1">15...0</td>
 *          <td rowspan="1">Status information are thrown away</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">TxE</td>
 *          <td rowspan="1">7</td>
 *          <td rowspan="1">Wait until the TX buffer is empty again</td>
 *      </tr>
 *      <tr>
 *          <td>DR</td>
 *          <td rowspan="1">Data</td>
 *          <td rowspan="1">7...0</td>
 *          <td rowspan="1">Write data to DR</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">BTF</td>
 *          <td rowspan="1">2</td>
 *          <td rowspan="1">Wait until the BTF (Byte transfer finished) flag is set</td>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">STOPF</td>
 *          <td rowspan="1">4</td>
 *          <td rowspan="1">Send STOP condition</td>
 *      </tr>
 * </table>
 */
I2C_RETURN_CODE_t i2cSendByte(I2C_TypeDef *i2c, uint8_t saddr, uint8_t data)
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
 * @ingroup iic3
 * Sends a byte to a functional register of the I2C slave.
 *
 * @param  *i2c     : Pointer to the component
 * @param   saddr   : Address of the I2C slave
 * @param   regAddr : Address of the slave register
 * @param   data    : Byte that shall be sent
 *
 * @return I2C_RETURN_CODE_t
 *
 * @note
 * Failure handling is not yet implemented
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * The text written in <b>bold</b> letters is required to address a device-specific register.<br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>SR2</td>
 *          <td rowspan="1">BUSY</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Check whether the component is busy</td>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">START</td>
 *          <td rowspan="1">8</td>
 *          <td rowspan="1">Start the data transfer</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">START</td>
 *          <td rowspan="1">0</td>
 *          <td rowspan="1">Wait until the START signal has been generated</td>
 *      </tr>
 *      <tr>
 *          <td>DR</td>
 *          <td rowspan="1">Slave address</td>
 *          <td rowspan="1">7...0</td>
 *          <td rowspan="1">Send the I2C slave address</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">ADDR</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Wait until the address was acknowledged</td>
 *      </tr>
 *      <tr>
 *          <td>SR1/SR2</td>
 *          <td rowspan="1">Dummy read of SR1/SR2</td>
 *          <td rowspan="1">15...0</td>
 *          <td rowspan="1">Status information are thrown away</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">TxE</td>
 *          <td rowspan="1">7</td>
 *          <td rowspan="1">Wait until the TX buffer is empty again</td>
 *      </tr>
 *      <tr>
 *          <td><b>DR</b></td>
 *          <td rowspan="1"><b>Address of the register</b></td>
 *          <td rowspan="1"><b>7...0</b></td>
 *          <td rowspan="1"><b>Send the device-internal address of the register</b></td>
 *      </tr>
 *      <tr>
 *          <td><b>SR1</b></td>
 *          <td rowspan="1"><b>TxE</b></td>
 *          <td rowspan="1"><b>7...0</b></td>
 *          <td rowspan="1"><b>Wait until the TX buffer is empty again</b></td>
 *      </tr>
 *      <tr>
 *          <td>DR</td>
 *          <td rowspan="1">Data</td>
 *          <td rowspan="1">7...0</td>
 *          <td rowspan="1">Write data to DR</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">BTF</td>
 *          <td rowspan="1">2</td>
 *          <td rowspan="1">Wait until the BTF (Byte transfer finished) flag is set</td>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">STOPF</td>
 *          <td rowspan="1">4</td>
 *          <td rowspan="1">Send STOP condition</td>
 *      </tr>
 * </table>
 */
I2C_RETURN_CODE_t i2cSendByteToSlaveReg(I2C_TypeDef *i2c, uint8_t saddr, uint8_t regAddr, uint8_t data)
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
 * @ingroup iic3
 * Burst write data to I2C slave.
 *
 * @param  *i2c     : Pointer to the component
 * @param   saddr   : Address of the I2C slave
 * @param   regAddr : Address of the slave register
 * @param   data    : Byte that shall be sent
 * @param   len     : Number of data elements to be sent
 *
 * @return I2C_RETURN_CODE_t
 *
 * @note
 * Failure handling is not yet implemented
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * The text written in <b>bold</b> letters is repeated according to the length of the register sequence.<br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>SR2</td>
 *          <td rowspan="1">BUSY</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Check whether the component is busy</td>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">POS</td>
 *          <td rowspan="1">11</td>
 *          <td rowspan="1">Reset the PEC bit (Packet error checking)</td>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">START</td>
 *          <td rowspan="1">8</td>
 *          <td rowspan="1">Generate START signal</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">START</td>
 *          <td rowspan="1">0</td>
 *          <td rowspan="1">Wait until the START signal has been generated</td>
 *      </tr>
 *      <tr>
 *          <td>DR</td>
 *          <td rowspan="1">DR</td>
 *          <td rowspan="1">7...0</td>
 *          <td rowspan="1">Write I2C slave address</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">ADDR</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Wait until the address was acknowledged</td>
 *      </tr>
 *      <tr>
 *          <td>SR1/SR2</td>
 *          <td rowspan="1">Dummy read of SR1/SR2</td>
 *          <td rowspan="1">15...0</td>
 *          <td rowspan="1">Status information are thrown away</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">TxE</td>
 *          <td rowspan="1">7</td>
 *          <td rowspan="1">Wait until the TX buffer is empty again</td>
 *      </tr>
 *      <tr>
 *          <td><b>DR</b></td>
 *          <td rowspan="1"><b>Address of the register</b></td>
 *          <td rowspan="1"><b>7...0</b></td>
 *          <td rowspan="1"><b>Send the device-internal start address of the register sequence</b></td>
 *      </tr>
 *      <tr>
 *          <td><b>SR1</b></td>
 *          <td rowspan="1"><b>TxE</b></td>
 *          <td rowspan="1"><b>7...0</b></td>
 *          <td rowspan="1"><b>Wait until the TX buffer is empty again</b></td>
 *      </tr>
 *      <tr>
 *          <td><b>DR</b></td>
 *          <td rowspan="1"><b>Data</b></td>
 *          <td rowspan="1"><b>7...0</b></td>
 *          <td rowspan="1"><b>Write the data sequence according to the length of the sequence to DR</b></td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">BTF</td>
 *          <td rowspan="1">2</td>
 *          <td rowspan="1">Wait until the BTF (Byte transfer finished) flag is set</td>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">STOPF</td>
 *          <td rowspan="1">4</td>
 *          <td rowspan="1">Sends STOP condition</td>
 *      </tr>
 * </table>
 */
I2C_RETURN_CODE_t i2cBurstWrite(I2C_TypeDef *i2c, uint8_t saddr, uint8_t regAddr, uint8_t *data, uint8_t len)
{
    uint8_t numBytes = 0;

    I2C_WAIT_BUSY(i2c);                 // Check whether the I2C bus is busy

    I2C_RESET_POS(i2c);                 // May only be active in 16-bit mode
    i2c->CR1 |= I2C_CR1_START;          // Generate I2C START signal
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
 * @ingroup iic2
 * Reads a byte from the I2C slave w/o internal registers immediately.
 *
 * @param  *i2c   : Pointer to the component
 * @param   saddr : Address of the I2C slave
 * @param  *data  : Pointer to the variable where the data shall be stored
 *
 * @return I2C_RETURN_CODE_t
 *
 * @note
 * Failure handling is not yet implemented.
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>SR2</td>
 *          <td rowspan="1">BUSY</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Check whether the component is busy</td>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">START</td>
 *          <td rowspan="1">8</td>
 *          <td rowspan="1">Generate START signal</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">START</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Wait until the START signal was generated</td>
 *      </tr>
 *      <tr>
 *          <td>DR</td>
 *          <td rowspan="1">I2C slave address</td>
 *          <td rowspan="1">7...0</td>
 *          <td rowspan="1">Send the slave address</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">ADDR</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Wait until the slave address was acknowledged</td>
 *      </tr>
 *      <tr>
 *          <td>SR1/SR2</td>
 *          <td rowspan="1">Dummy read of SR1/SR2</td>
 *          <td rowspan="1">15...0</td>
 *          <td rowspan="1">Status information are thrown away</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">TxE</td>
 *          <td rowspan="1">7</td>
 *          <td rowspan="1">Wait until the TX buffer is empty again</td>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">START</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Generate RESTART signal</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">START</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Wait until the START signal was generated</td>
 *      </tr>
 *      <tr>
 *          <td>DR</td>
 *          <td rowspan="1">Send slave (address | 1) to activate reading from the device</td>
 *          <td rowspan="1">7...0</td>
 *          <td rowspan="1">Don't forget to set bit 0 to '1' to activate reading from the slave</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">ADDR</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Wait until the slave address was acknowledged</td>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">ACK</td>
 *          <td rowspan="1">10</td>
 *          <td rowspan="1">Reset the ACK bit</td>
 *      </tr>
 *      <tr>
 *          <td>SR1/SR2</td>
 *          <td rowspan="1">Dummy read of SR1/SR2</td>
 *          <td rowspan="1">15...0</td>
 *          <td rowspan="1">Status information are thrown away</td>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">STOPF</td>
 *          <td rowspan="1">4</td>
 *          <td rowspan="1">Generate STOP signal</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">RxNE</td>
 *          <td rowspan="1">6</td>
 *          <td rowspan="1">Wait until receive buffer is no longer empty</td>
 *      </tr>
 *      <tr>
 *          <td>DR</td>
 *          <td rowspan="1">Read data</td>
 *          <td rowspan="1">7...0</td>
 *          <td rowspan="1">Provide data to variable which is read by the application</td>
 *      </tr>
 * </table>
 */
I2C_RETURN_CODE_t i2cReadByte(I2C_TypeDef *i2c, uint8_t saddr, uint8_t *data)
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
 * @ingroup iic3
 * Reads a byte from a functional register of the I2C slave.
 *
 * @param  *i2c     : Pointer to the component
 * @param   saddr   : Address of the I2C slave
 * @param   regAddr : Address of the slave register
 * @param  *data    : Address where the data shall be stored
 *
 * @return I2C_RETURN_CODE_t
 *
 * @note
 * Failure handling is not yet implemented. This function shall be used
 * in the case when the addresses I2C component provides only one internal
 * register.
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>SR2</td>
 *          <td rowspan="1">BUSY</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Check whether the component is busy</td>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">START</td>
 *          <td rowspan="1">8</td>
 *          <td rowspan="1">Generate START signal</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">START</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Wait until the START signal was generated</td>
 *      </tr>
 *      <tr>
 *          <td>DR</td>
 *          <td rowspan="1">I2C slave address</td>
 *          <td rowspan="1">7...0</td>
 *          <td rowspan="1">Send the slave address</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">ADDR</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Wait until the slave address was acknowledged</td>
 *      </tr>
 *      <tr>
 *          <td>SR1/SR2</td>
 *          <td rowspan="1">Dummy read of SR1/SR2</td>
 *          <td rowspan="1">15...0</td>
 *          <td rowspan="1">Status information are thrown away</td>
 *      </tr>
 *      <tr>
 *          <td>DR</td>
 *          <td rowspan="1">Set address of the device-internal register</td>
 *          <td rowspan="1">7...0</td>
 *          <td rowspan="1">Send the address</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">TxE</td>
 *          <td rowspan="1">7</td>
 *          <td rowspan="1">Wait until the TX buffer is empty again</td>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">START</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Generate RESTART signal</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">START</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Wait until the RESTART signal was generated</td>
 *      </tr>
 *      <tr>
 *          <td>DR</td>
 *          <td rowspan="1">Send slave (address | 1) to activate reading from the device</td>
 *          <td rowspan="1">7...0</td>
 *          <td rowspan="1">Don't forget to set bit 0 to '1' to activate reading from the slave</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">ADDR</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Wait until the slave address was acknowledged</td>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">ACK</td>
 *          <td rowspan="1">10</td>
 *          <td rowspan="1">Reset the ACK bit</td>
 *      </tr>
 *      <tr>
 *          <td>SR1/SR2</td>
 *          <td rowspan="1">Dummy read of SR1/SR2</td>
 *          <td rowspan="1">15...0</td>
 *          <td rowspan="1">Status information are thrown away</td>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">STOPF</td>
 *          <td rowspan="1">4</td>
 *          <td rowspan="1">Generate STOP signal</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">RxNE</td>
 *          <td rowspan="1">6</td>
 *          <td rowspan="1">Wait until receive buffer is no longer empty</td>
 *      </tr>
 *      <tr>
 *          <td>DR</td>
 *          <td rowspan="1">Read data</td>
 *          <td rowspan="1">7...0</td>
 *          <td rowspan="1">Provide data to a buffer which is read by the application</td>
 *      </tr>
 * </table>
 */
I2C_RETURN_CODE_t i2cReadByteFromSlaveReg(I2C_TypeDef *i2c, uint8_t saddr, uint8_t regAddr, uint8_t *data)
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
 * @ingroup iic3
 * Burst read from I2C slave.
 *
 * @param  *i2c     : Pointer to the component
 * @param   saddr   : Address of the I2C slave
 * @param   regAddr : Address of the first slave register
 * @param  *data    : Address where the data shall be stored
 * @param   num     : Number of data elements to be read
 *
 * @return I2C_RETURN_CODE_t
 *
 * @note
 * Failure handling is not yet implemented
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>SR2</td>
 *          <td rowspan="1">BUSY</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Check whether the component is busy</td>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">POS</td>
 *          <td rowspan="1">11</td>
 *          <td rowspan="1">Packet error checking. Not needed for 8-bit data transfer</td>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">START</td>
 *          <td rowspan="1">8</td>
 *          <td rowspan="1">Generate START signal</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">START</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Wait until the START signal was generated</td>
 *      </tr>
 *      <tr>
 *          <td>DR</td>
 *          <td rowspan="1">I2C slave address</td>
 *          <td rowspan="1">7...0</td>
 *          <td rowspan="1">Send the slave address</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">ADDR</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Wait until the slave address was acknowledged</td>
 *      </tr>
 *      <tr>
 *          <td>SR1/SR2</td>
 *          <td rowspan="1">Dummy read of SR1/SR2</td>
 *          <td rowspan="1">15...0</td>
 *          <td rowspan="1">Status information are thrown away</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">TxE</td>
 *          <td rowspan="1">7</td>
 *          <td rowspan="1">Wait until the TX buffer is empty again</td>
 *      </tr>
 *      <tr>
 *          <td>DR</td>
 *          <td rowspan="1">Set address of the device-internal register</td>
 *          <td rowspan="1">7...0</td>
 *          <td rowspan="1">Send the address</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">TxE</td>
 *          <td rowspan="1">7</td>
 *          <td rowspan="1">Wait until the TX buffer is empty again</td>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">START</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Generate RESTART signal</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">START</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Wait until the RESTART signal was generated</td>
 *      </tr>
 *      <tr>
 *          <td>DR</td>
 *          <td rowspan="1">Send slave (address | 1) to activate reading from the device</td>
 *          <td rowspan="1">7...0</td>
 *          <td rowspan="1">Don't forget to set bit 0 to '1' to activate reading from the slave</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">ADDR</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Wait until the slave address was acknowledged</td>
 *      </tr>
 *      <tr>
 *          <td>SR1/SR2</td>
 *          <td rowspan="1">Dummy read of SR1/SR2</td>
 *          <td rowspan="1">15...0</td>
 *          <td rowspan="1">Status information are thrown away</td>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">ACK</td>
 *          <td rowspan="1">10</td>
 *          <td rowspan="1">Enable Acknowledge</td>
 *      </tr>
 *      <tr>
 *          <td>SR1</td>
 *          <td rowspan="1">RxNE</td>
 *          <td rowspan="1">6</td>
 *          <td rowspan="1">Wait until receive buffer is no longer empty</td>
 *      </tr>
 *      <tr>
 *          <td>DR</td>
 *          <td rowspan="1">Read data</td>
 *          <td rowspan="1">7...0</td>
 *          <td rowspan="1">Repeated read to a buffer as long as there are data. One byte must be left.</td>
 *      </tr>
 *      <tr>
 *          <td><b>CR1</b></td>
 *          <td rowspan="1"><b>ACK</b></td>
 *          <td rowspan="1"><b>10</b></td>
 *          <td rowspan="1"><b>This must be done for the last byte</b></td>
 *      </tr>
 *      <tr>
 *          <td><b>CR1</b></td>
 *          <td rowspan="1"><b>STOPF</b></td>
 *          <td rowspan="1"><b>9</b></td>
 *          <td rowspan="1"><b>This must be done for the last byte</b></td>
 *      </tr>
 *      <tr>
 *          <td><b>SR1</b></td>
 *          <td rowspan="1"><b>RxNE</b></td>
 *          <td rowspan="1"><b>6</b></td>
 *          <td rowspan="1"><b>This must be done for the last byte</b></td>
 *      </tr>
 *      <tr>
 *          <td><b>DR</b></td>
 *          <td rowspan="1"><b>Read final byte into buffer</b></td>
 *          <td rowspan="1"><b>7...0</b></td>
 *          <td rowspan="1"><b>This must be done for the last byte</b></td>
 *      </tr>
 * </table>
 */
I2C_RETURN_CODE_t i2cBurstRead(I2C_TypeDef *i2c, uint8_t saddr, uint8_t regAddr, uint8_t *data, uint8_t num)
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

    while (num > 0)                          // Start reading multiple values
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

/**
 * @ingroup iic2
 * Enables the desired I2C peripheral component.
 *
 * @param  *i2c   : Pointer to the I2C component
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">PE</td>
 *          <td rowspan="1">0</td>
 *      </tr>
 * </table>
 */
I2C_RETURN_CODE_t i2cEnableDevice(I2C_TypeDef *i2c)
{
    i2c->CR1 |= I2C_CR1_PE;
    return I2C_OK;
}

/**
 * @ingroup iic2
 * Disables the desired I2C peripheral component.
 *
 * @param  *i2c   : Pointer to the I2C component
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">PE</td>
 *          <td rowspan="1">0</td>
 *      </tr>
 * </table>
 */
I2C_RETURN_CODE_t i2cDisableDevice(I2C_TypeDef *i2c)
{
    i2c->CR1 &= ~I2C_CR1_PE_Msk;
    return I2C_OK;
}

/**
 * @ingroup iic2
 * Sets the peripheral clock frequency. Don't mix it up with the data transfer speed!
 *
 * @param  *i2c  : Pointer to the I2C component
 * @param   pclk : Clock frequency of the desired I2C component in MHz
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>CR2</td>
 *          <td rowspan="1">FREQ</td>
 *          <td rowspan="1">5...0</td>
 *      </tr>
 * </table>
 */
I2C_RETURN_CODE_t i2cSetPeripheralClockFreq(I2C_TypeDef *i2c, uint8_t pclk)
{
    i2c->CR2 &= ~I2C_CR2_FREQ_Msk;
    i2c->CR2 |= pclk;

    return I2C_OK;
}

/**
 * @ingroup iic2
 * Sets the duty cycle of the desired I2C component.
 *
 * @param  *i2c  : Pointer to the I2C component
 * @param   duty : Duty cycle. Can be IC_DUTY_CYCLE 2 or DUTY_CYCLE_16_9.
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>CCR</td>
 *          <td rowspan="1">DUTY</td>
 *          <td rowspan="1">15</td>
 *      </tr>
 * </table>
 */
I2C_RETURN_CODE_t i2cSetDutyCycle(I2C_TypeDef *i2c, I2C_DUTY_CYCLE_t duty)
{
    i2c->CCR &= ~I2C_CCR_DUTY_Msk;
    i2c->CCR |= duty << I2C_CCR_DUTY_Pos;

    return I2C_OK;
}

/**
 * @ingroup iic2
 * Sets the maximum rise time in Fm/Sm naster mode.
 *
 * @param  *i2c      : Pointer to the I2C component
 * @param   riseTime : Valid values are { 0 ... 63 }.
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>TRISE</td>
 *          <td rowspan="1">TRISE</td>
 *          <td rowspan="1">5 ... 0</td>
 *      </tr>
 * </table>
 */
I2C_RETURN_CODE_t i2cSetRiseTime(I2C_TypeDef *i2c, uint8_t riseTime)
{
    i2c->TRISE &= ~I2C_TRISE_TRISE_Msk;

    if (riseTime >= 0 && riseTime <= 63)
    {
        i2c->TRISE |= riseTime;
    }
    else
    {
        i2c->TRISE |= 0x11;     // This is a reliable value
    }
    return I2C_OK;
}

/**
 * @ingroup iic2
 * Searches I2C peripheral components and returns their I2C address. It returns 0 if the desired address is free.
 *
 * @param  *i2c     : Pointer to the I2C component
 * @param   i2cAddr : The I2C address which shall be tested
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>-</td>
 *          <td rowspan="1">-</td>
 *          <td rowspan="1">-</td>
 *      </tr>
 * </table>
 */
uint8_t i2cFindSlaveAddr(I2C_TypeDef *i2c, uint8_t i2cAddr)
{
    uint32_t simpleDelay;

//    i2c->CR1 |= I2C_CR1_SWRST;
//    for (simpleDelay = 0; simpleDelay < 1000UL; simpleDelay++)
//    {
//        ;
//    }
//    i2c->CR1 &= ~I2C_CR1_SWRST_Msk;



    i2c->CR1 |= I2C_CR1_START;
    while (!(i2c->SR1 & I2C_SR1_SB))
    {
        ;
    }

    i2c->DR = i2cAddr;
    while (!(i2c->SR1) | !(i2c->SR2))
    {
        ;
    }

    while (!(i2c->SR1) | !(i2c->SR2))
    {
        ;
    }

    i2c->CR1 |= I2C_CR1_STOP;
    for (simpleDelay = 0UL; simpleDelay < 10000UL; simpleDelay++)
    {
        ;
    }
    simpleDelay = 0UL;

    if (i2c->SR1 & I2C_SR1_ADDR)
    {
        return i2cAddr;
    }
    else
    {
        return 0;
    }
}

I2C_RETURN_CODE_t i2cResetDevice(I2C_TypeDef *i2c)
{
//    I2C_WAIT_BUSY(i2c);
    i2c->CR1 |= I2C_CR1_SWRST;

    return I2C_OK;
}
