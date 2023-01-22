/**
 * @defgroup spi  Serial Peripheral Interface (SPI) Functions (mcalSPI.h/.c)
 * @defgroup spi0 Deprecated SPI Functions
 * @ingroup  spi
 * @defgroup spi1 SPI Macros
 * @ingroup  spi
 * @defgroup spi2 SPI Standard Functions
 * @ingroup  spi
 * @defgroup spi3 SPI Enumerations and definitions
 * @ingroup  spi
 *
 * @file        mcalSPI.c
 * @brief       mcalSPI.c is part of the MCAL library for STM32F4xx.
 * @author      Dipl.-Ing. Ralf Jesse (embedded@ralf-jesse.de)
 * @date        Nov. 12, 2020
 *
 * @version     0.1
 * @copyright   GNU Public License Version 3 (GPLv3)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <mcalGPIO.h>
#include <mcalSPI.h>

/**
 * Verifies the integrity of the SPI port.
 */
static bool spiVerifySPI(SPI_TypeDef *spi)
{
    if ((SPI1 == spi) || (SPI2 == spi) || (SPI3 == spi) || (SPI4 == spi))
    {
        return true;
    }
    return false;
}

/**
 * Verifies the integrity of the SPI clock divider.
 */
static bool spiVerifyClkDivider(SPI_CLOCK_DIV_t div)
{
    if ((CLK_DIV_2  == div) || (CLK_DIV_4  == div) || (CLK_DIV_8   == div) || (CLK_DIV_16  == div) ||
        (CLK_DIV_32 == div) || (CLK_DIV_64 == div) || (CLK_DIV_128 == div) || (CLK_DIV_256 == div))
    {
        return true;
    }
    return false;
}

/**
 * Verifies the integrity of the SPI data length.
 */
static bool spiVerifyDataLen(SPI_DATALEN_t len)
{
    if ((SPI_DATA_8BIT == len) || (SPI_DATA_16_BIT == len))
    {
        return true;
    }
    return false;
}

/**
 * Verifies the integrity of the SPI Software Slave Management.
 */
static bool spiVerifySSM(SPI_SSM_t ssm)
{
    if ((SSM_ON == ssm) || (SSM_OFF == ssm))
    {
        return true;
    }
    return false;
}

#if 0
// This function is currently not used.
/**
 * Verifies the integrity of the SSI level.
 */
static bool spiVerifySsiLvl(SPI_SSI_LVL_t lvl)
{
    if ((SSI_LVL_HIGH == lvl) || (SSI_LVL_LOW == lvl))
    {
        return true;
    }
    return false;
}
#endif

/**
 * Verifies the integrity of the SPI operational mode.
 */
static bool spiVerifyOpMode(SPI_OPMODE_t om)
{
    if ((MASTER == om) || (SLAVE == om))
    {
        return true;
    }
    return false;
}

/**
 * Verifies the integrity of the SPI trigger phase.
 */
static bool spiVerifyPhase(SPI_PHASE_t ph)
{
    if ((SPI_PHASE_EDGE_1 == ph) || (SPI_PHASE_EDGE_2 == ph))
    {
        return true;
    }
    return false;
}

/**
 * Verifies the integrity of the SPI idle level.
 */
static bool spiVerifyIdlePolarity(SPI_POLARITY_t pol)
{
    if ((SPI_IDLE_LOW == pol) || (SPI_IDLE_HIGH == pol))
    {
        return true;
    }
    return false;
}

/**
 * @ingroup spi2
 * Initialization of a SPI interface
 *
 * @param    *spi      : Pointer to the SPI device
 * @param     div      : Baudrate setting (prescaler for PCLK)
 * @param     len      : Data format 8/16 bit
 * @param     ssm      : Software Slave Management on/off
 * @param     lvl      : If ssm == On --> Use this as NSS level
 * @param     opMode   : Select: Master-/Slave mode
 * @param     phase    : Phase of the trigger
 * @param     polarity : Polarity of the trigger idle state
 * @return   SPI_RETURN_CODE_t
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
 *          <td>APB1ENR, APB2ENR</td>
 *          <td rowspan="1">I2C3/2/1</td>
 *          <td rowspan="1">23...21</td>
 *          <td rowspan="1">Depends on the desired SPI interface</td>
 *      </tr>
 *      <>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">BR</td>
 *          <td rowspan="1">2...0</td>
 *          <td rowspan="1">Selection of the baud rate</td>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">DFF</td>
 *          <td rowspan="1">11</td>
 *          <td rowspan="1">Selects between 8 or 16 bit data format</td>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">SSM</td>
 *          <td rowspan="1">9</td>
 *          <td rowspan="1">Software slave management</td>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">SSI</td>
 *          <td rowspan="1">8</td>
 *          <td rowspan="1">Internal slave select</td>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">MSTR</td>
 *          <td rowspan="1">2</td>
 *          <td rowspan="1">MSTR = 0: Slave / MSTR = 1: Master</td>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">POL</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Polarity selection</td>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">CPHA</td>
 *          <td rowspan="1">0</td>
 *          <td rowspan="1">Phase selection</td>
 *      </tr>
 *      <tr>
 *          <td>CR2</td>
 *          <td rowspan="1">not used</td>
 *          <td rowspan="1">15...0</td>
 *          <td rowspan="1">Currently not completed: Always set to 0</td>
 *      </tr>
 * </table>
 */
SPI_RETURN_CODE_t spiInitSPI(SPI_TypeDef *spi, SPI_CLOCK_DIV_t div, SPI_DATALEN_t len,
                             SPI_SSM_t ssm, SPI_SSI_LVL_t lvl, SPI_OPMODE_t opMode,
                             SPI_PHASE_t phase, SPI_POLARITY_t polarity)
{
    uint16_t cr1 = 0U;

    // Parameter verification
    if (spiVerifyClkDivider(div) != true)
    {
        return SPI_INVALID_CLOCK_DIVIDER;
    }
    if (spiVerifyDataLen(len) != true)
    {
        return SPI_INVALID_DATA_LENGTH;
    }
    if (spiVerifySSM(ssm) != true)
    {
        return SPI_INVALID_SW_SLV_MGMT;
    }
#if 0
    // This is currently not passed as a parameter.
    if (spiVerifySsiLvl(lvl) != true)
    {
        return SPI_INVALID_SSI_LEVEL;
    }
#endif
    if (spiVerifyOpMode(opMode) != true)
    {
        return SPI_INVALID_OP_MODE;
    }
    if (spiVerifyPhase(phase) != true)
    {
        return SPI_INVALID_PHASE;
    }
    if (spiVerifyIdlePolarity(polarity) != true)
    {
        return SPI_INVALID_IDLE_POLARITY;
    }

    /**
     *  All parameter check passed successfully!
     */

    // Setting up the baudrate (PCLK / Pre-Scaler)
    switch (div)
    {
        case CLK_DIV_2:
            cr1 &= ~(SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0);
            break;

        case CLK_DIV_4:
            cr1 |= SPI_CR1_BR_0;
            break;

        case CLK_DIV_8:
            cr1 |= SPI_CR1_BR_1;
            break;

        case CLK_DIV_16:
            cr1 |= (SPI_CR1_BR_1 | SPI_CR1_BR_0);
            break;

        case CLK_DIV_32:
            cr1 |= SPI_CR1_BR_2;
            break;

        case CLK_DIV_64:
            cr1 |= (SPI_CR1_BR_2 | SPI_CR1_BR_0);
            break;

        case CLK_DIV_128:
            cr1 |= (SPI_CR1_BR_2 | SPI_CR1_BR_1);
            break;

        case CLK_DIV_256:
            cr1 |= (SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0);
            break;

        default:
            cr1 |= (SPI_CR1_BR_1 | SPI_CR1_BR_0);
            break;
    }

    // Setting up the data length
    if (SPI_DATA_8BIT == len)
    {
        cr1 &= ~SPI_CR1_DFF_Msk;
    }
    else
    {
        cr1 |= SPI_CR1_DFF;
    }

    // Set SSM and SSI bits
    if (SSM_ON == ssm)
    {
        cr1 |= SPI_CR1_SSM;

        // SSI level works only if SSM is active
        if (SSI_LVL_HIGH == lvl)
        {
            cr1 |= SPI_CR1_SSI;
        }
        else
        {
            cr1 &= ~SPI_CR1_SSI_Msk;
        }
    }
    else
    {
        cr1 &= ~SPI_CR1_SSM_Msk;
    }

    // Select between Master/Slave mode
    if (MASTER == opMode)
    {
        cr1 |= SPI_CR1_MSTR;
    }
    else
    {
        cr1 &= ~SPI_CR1_MSTR_Msk;
    }

    // Set clock phase
    if (SPI_PHASE_EDGE_1 == phase)
    {
        cr1 &= ~SPI_CR1_CPHA_Msk;
    }
    else
    {
        cr1 |= SPI_CR1_CPHA;
    }

    // Set clock polarity
    if (SPI_IDLE_LOW == polarity)
    {
        cr1 &= ~SPI_CR1_CPOL_Msk;
    }
    else
    {
        cr1 |= SPI_CR1_CPOL;
    }

    // Transfer settings to CR1 + CR2
    spi->CR1 = cr1;
    spi->CR2 = 0;                     // Simplified version. Should be modified.

    // Finally, enable SPIn
    spiEnableSPI(spi);

    return SPI_OK;
}

/**
 * @ingroup spi2
 * Turns the bus clock of the desired SPI component on.
 *
 * @param    *spi      : Pointer to the SPI interface
 * @return   SPI_RETURN_CODE_t
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
 *          <td>APB1ENR, APB2ENR</td>
 *          <td rowspan="1">I2C3/2/1</td>
 *          <td rowspan="1">23...21</td>
 *          <td rowspan="1">Depends on the desired SPI interface</td>
 *      </tr>
 * </table>
 */
SPI_RETURN_CODE_t spiSelectSPI(SPI_TypeDef *spi)
{
    // All parameter check passed successfully!

    // Selects the bus clock for SPIn
    if (SPI1 == spi)
    {
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    }
    else if (SPI2 == spi)
    {
        RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    }
    else if (SPI3 == spi)
    {
        RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
    }
    else if (SPI4 == spi)
    {
        RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;
    }

    return SPI_OK;
}

/**
 * @ingroup spi2
 * Deselects the desired SPI component (bus clock is turned off).
 *
 * @param    *spi : Pointer to the SPI interface
 * @return   SPI_RETURN_CODE_t
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
 *          <td>APB1ENR, APB2ENR</td>
 *          <td rowspan="1">I2C3/2/1</td>
 *          <td rowspan="1">23...21</td>
 *          <td rowspan="1">Depends on the desired SPI interface</td>
 *      </tr>
 * </table>
 */SPI_RETURN_CODE_t spiDeselectSPI(SPI_TypeDef *spi)
{
    // Parameter verification
    if (spiVerifySPI(spi) != true)
    {
        return SPI_INVALID_SPI;
    }

    // Selects the bus clock for SPIn
    if (SPI1 == spi)
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN_Msk;
    }
    else if (SPI2 == spi)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN_Msk;
    }
    else if (SPI3 == spi)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_SPI3EN_Msk;
    }
    else if (SPI4 == spi)
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_SPI4EN_Msk;
    }


    return SPI_OK;
}

/**
 * @ingroup spi2
 * Enables the SPI interface
 *
 * @param   *spi : Pointer to the SPI interface
 * @return  SPI_RETURN_CODE_t
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
 *          <td>CR1</td>
 *          <td rowspan="1">SPE</td>
 *          <td rowspan="1">6</td>
 *          <td rowspan="1">Enables the desired SPI interface</td>
 *      </tr>
 * </table>
 */
SPI_RETURN_CODE_t spiEnableSPI(SPI_TypeDef *spi)
{
    // Parameter verification
    if (spiVerifySPI(spi) != true)
    {
        return SPI_INVALID_SPI;
    }

    // All parameter check passed successfully!
    spi->CR1 |= SPI_CR1_SPE;

    return SPI_OK;
}

/**
 * @ingroup spi2
 * Disables the SPI interface
 *
 * @param   *spi : Pointer to SPI interface
 * @return  SPI_RETURN_CODE_t
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
 *          <td>CR1</td>
 *          <td rowspan="1">SPE</td>
 *          <td rowspan="1">6</td>
 *          <td rowspan="1">Disables the desired SPI interface</td>
 *      </tr>
 * </table>
 */
SPI_RETURN_CODE_t spiDisableSPI(SPI_TypeDef *spi)
{
    // Parameter verification
    if (spiVerifySPI(spi) != true)
    {
        return SPI_INVALID_SPI;
    }

    // All parameter check passed successfully!
    spi->CR1 &= ~SPI_CR1_SPE_Msk;

    return SPI_OK;
}

/**
 * @ingroup spi2
 * Writes a byte to the MOSI pin
 *
 * @param   *spi  : Pointer to SPIn
 * @param   *port : Pointer to the CS GPIO port (CS = Chip Select)
 * @param    pin  : Pin number of the GPIO pin of the CS GPIO port
 * @param    data : uint8_t data
 * @return  SPI_RETURN_CODE_t
 *
 * @note
 * Consider that every SPI slave has its' own CS input. Since this
 * function doesn't "know" which GPIO port and which GPIO pin shall
 * be used for this SPI write operation, the user has to provide these
 * information in his project.
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
 *          <td>SR</td>
 *          <td rowspan="1">TxE</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Verify that TXE is '1' before writing data to DR</td>
 *      </tr>
 *      <tr>
 *          <td>SR</td>
 *          <td rowspan="1">BSY</td>
 *          <td rowspan="1">7</td>
 *          <td rowspan="1">Verify that BSY is '0' before writing data to DR</td>
 *      </tr>
 *      <tr>
 *          <td>GPIOx</td>
 *          <td rowspan="1">Pin n</td>
 *          <td rowspan="1">...</td>
 *          <td rowspan="1">Activate Chip Select</td>
 *      </tr>
 *      <tr>
 *          <td>DR</td>
 *          <td rowspan="1">Data (send 8 bit only)</td>
 *          <td rowspan="1">15...0</td>
 *          <td rowspan="1">Write data to DR</td>
 *      </tr>
 * </table>
 */
SPI_RETURN_CODE_t spiWriteByte(SPI_TypeDef *spi, GPIO_TypeDef *port, PIN_NUM_t pin, uint8_t data)
{
    static uint8_t state = SPI_SEND_BYTE_1;

    if (gpioVerifyPin(pin) != true)
    {
        return GPIO_INVALID_PIN;
    }

    // All parameters verified successfully.
    switch (state)
    {
        case SPI_SEND_BYTE_1:
        {
            gpioResetPin(port, pin);              // Set CS input to low level
            while (!(spi->SR & SPI_SR_TXE))         // Wait until the TX register is empty
            {
                ;
            }
            spi->DR = data;                         // Send first byte to data register

            while (spi->SR & SPI_SR_BSY)            // Wait until data has been sent
            {
                // Wait: SPI is still busy
            }
            state = SPI_SEND_BYTE_2;
            break;                                  // Leave switch-case-check
        }

        case SPI_SEND_BYTE_2:
        {
            while (!(spi->SR & SPI_SR_TXE))         // Wait until the TX register is empty
            {
                ;
            }
            spi->DR = data;                         // Send first byte to data register

            while (spi->SR & SPI_SR_BSY)            // Wait until data has been sent
            {
                // Wait: SPI is still busy
            }

            gpioSetPin(port, pin);
            state = SPI_SEND_BYTE_1;
        }
    }

    return SPI_OK;
}

/**
 * @ingroup spi2
 * Writes double-byte to the MOSI pin
 *
 * @param   *spi  : Pointer to SPIn
 * @param   *port : Pointer to the CS GPIO port (CS = Chip Select)
 * @param    pin  : Pin number of the GPIO pin of the CS GPIO port
 * @param    data : uint16_t data
 * @return  SPI_RETURN_CODE_t
 *
 * @note
 * Consider that every SPI slave has its' own CS input. Since this
 * function doesn't "know" which GPIO port and which GPIO pin shall
 * be used for this SPI write operation, the user has to provide these
 * information in his project.
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
 *          <td>SR</td>
 *          <td rowspan="1">TxE</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Verify that TXE is '1' before writing data to DR</td>
 *      </tr>
 *      <tr>
 *          <td>SR</td>
 *          <td rowspan="1">BSY</td>
 *          <td rowspan="1">7</td>
 *          <td rowspan="1">Verify that BSY is '0' before writing data to DR</td>
 *      </tr>
 *      <tr>
 *          <td>GPIOx</td>
 *          <td rowspan="1">Pin n</td>
 *          <td rowspan="1">...</td>
 *          <td rowspan="1">Activate Chip Select</td>
 *      </tr>
 *      <tr>
 *          <td>DR</td>
 *          <td rowspan="1">Data (16 bit)</td>
 *          <td rowspan="1">15...0</td>
 *          <td rowspan="1">Calls spiSendByte() twice</td>
 *      </tr>
 * </table>
 */
SPI_RETURN_CODE_t spiWriteWord(SPI_TypeDef *spi, GPIO_TypeDef *port, PIN_NUM_t pin, uint16_t data)
{
    if (gpioVerifyPin(pin) != true)
    {
        return GPIO_INVALID_PIN;
    }

    // All parameter check passed successfully!

    // Wait until the TX register is empty
    while (!(spi->SR & SPI_SR_TXE))
    {
        ;
    }

    // Reset CS
    gpioResetPin(port, pin);

    // Transfer new data to the transfer register
    spi->DR = data;

    // Waits until data has been sent
    while (spi->SR & SPI_SR_BSY)
    {
        // Wait: SPI is still busy
    }

    // Set CS
    gpioSetPin(port, pin);

    return SPI_OK;
}

/**
 * @ingroup spi2
 * Sends data to the SPI peripheral component in both supported formats (8 and 16 bit).
 * The function decides internally which format must be used. The format depends on the
 * state of the DFF bit in control register SPI_CR1.
 *
 * @param   *spi  : Pointer to SPIn
 * @param   *port : Pointer to the CS GPIO port (CS = Chip Select)
 * @param    pin  : Pin number of the GPIO pin of the CS GPIO port
 * @param    reg  : Number of the peripheral register
 * @param    data : Data which is to be sent to  the device.
 * @return  SPI_RETURN_CODE_t
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
 *          <td>SR</td>
 *          <td rowspan="1">TxE</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Verify that TXE is '1' before writing data to DR</td>
 *      </tr>
 *      <tr>
 *          <td>SR</td>
 *          <td rowspan="1">BSY</td>
 *          <td rowspan="1">7</td>
 *          <td rowspan="1">Verify that BSY is '0' before writing data to DR</td>
 *      </tr>
 *      <tr>
 *          <td>GPIOx</td>
 *          <td rowspan="1">Pin n</td>
 *          <td rowspan="1">...</td>
 *          <td rowspan="1">Activate Chip Select</td>
 *      </tr>
 *      <tr>
 *          <td>DR</td>
 *          <td rowspan="1">Date (8 or 16 bit)</td>
 *          <td rowspan="1">15...0</td>
 *          <td rowspan="1">Write data to DR. Calls either 1x spiWriteWord() or 2x spiWriteByte()</td>
 *      </tr>
 * </table>
 */
SPI_RETURN_CODE_t spiSendData(SPI_TypeDef *spi, GPIO_TypeDef *port, PIN_NUM_t pin, uint8_t reg, uint8_t data)
{
    // Parameter verification
    if (gpioVerifyPin(pin) != true)
    {
        return GPIO_INVALID_PIN;
    }

    // All parameter check passed successfully!

    if (!(spi->CR1 & SPI_CR1_DFF))
    {
        spiWriteByte(spi, port, pin, reg);
        spiWriteByte(spi, port, pin, data);
    }
    else
    {
        spiWriteWord(spi, port, pin, (reg << 8) | data);
    }

    return SPI_OK;
}

//TODO Not yet implemented
/**
 * Not yet implemented
 */
SPI_RETURN_CODE_t spiReadByte(SPI_TypeDef *spi, uint8_t *data)
{
    return SPI_OK;
}

//TODO Not yet implemented
/**
 * Not yet implemented
 */
SPI_RETURN_CODE_t spiReadWord(SPI_TypeDef *spi, uint16_t *data)
{
    return SPI_OK;
}
