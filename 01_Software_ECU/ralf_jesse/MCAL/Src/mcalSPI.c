/**
 * mcalSPI.c
 *
 *  Created on: 17.10.2020
 *      Author: Ralf Jesse
 *       Email: embedded@ralf-jesse.de
 */

#include <mcalGPIO.h>
#include <mcalSPI.h>

/**
 * @brief Initialisierung of SPIn
 *
 * @param[in]   *spi    : Pointer to SPIn
 * @param[in]    div    : Baudrate setting (prescaler for PCLK)
 * @param[in]    len    : Data format 8/16 bit
 * @param[in]    ssm    : Software Slave Management on/off
 * @param[in]    lvl    : If ssm == On --> Use this as NSS level
 * @param[in]    opMode : Select: Master-/Slave mode
 */
void spiInitSPI(SPI_TypeDef *spi, SPI_CLOCK_DIV_t div, SPI_DATALEN_t len,
        SPI_SSM_t ssm, SPI_SSI_LVL_t lvl, SPI_OPMODE_t opMode,
        SPI_PHASE_t phase, SPI_POLARITY_t polarity)
{
    uint16_t cr1 = 0U;

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
}

/**
 * @brief Enables SPIn
 *
 * @param[in]  *spi : Pointer to SPIn
 */
void spiEnableSPI(SPI_TypeDef *spi)
{
    spi->CR1 |= SPI_CR1_SPE;
}

/**
 * @brief Disables SPIn
 *
 * @param[in]  *spi : Pointer to SPIn
 */
void spiDisableSPI(SPI_TypeDef *spi)
{
    spi->CR1 &= ~SPI_CR1_SPE_Msk;
}

/**
 * @brief Writes a byte to the MOSI pin
 *
 * @param[in]  *spi  : Pointer to SPIn
 * @param[in]  *port : Pointer to the CS GPIO port (CS = Chip Select)
 * @param[in]   pin  : Pin number of the GPIO pin of the CS GPIO port
 * @param[in]   data : uint8_t data
 *
 * @note
 * Consider that every SPI slave has its' own CS input. Since this
 * function doesn't "know" which GPIO port and which GPIO pin shall
 * be used for this SPI write operation, the user has to provide these
 * information in his project.
 */
void spiWriteByte(SPI_TypeDef *spi, GPIO_TypeDef *port, PIN_NUM pin, uint8_t data)
{
    static uint8_t state = SPI_SEND_BYTE_1;

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
}

/**
 * @brief Writes double-byte to the MOSI pin
 *
 * @param[in]  *spi  : Pointer to SPIn
 * @param[in]  *port : Pointer to the CS GPIO port (CS = Chip Select)
 * @param[in]   pin  : Pin number of the GPIO pin of the CS GPIO port
 * @param[in]   data : uint16_t data
 *
 * @note
 * Consider that every SPI slave has its' own CS input. Since this
 * function doesn't "know" which GPIO port and which GPIO pin shall
 * be used for this SPI write operation, the user has to provide these
 * information in his project.
 */
void spiWriteWord(SPI_TypeDef *spi, GPIO_TypeDef *port, PIN_NUM pin, uint16_t data)
{
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

}

/**
 * @brief Sends data to the SPI peripheral component in both supported formats (8 and 16 bit).
 *
 * @param[in]  *spi  : Pointer to SPIn
 * @param[in]  *port : Pointer to the CS GPIO port (CS = Chip Select)
 * @param[in]   pin  : Pin number of the GPIO pin of the CS GPIO port
 * @param[in]   reg  : Number of the peripheral register
 * @param[in]   data : Data which is to be sent to  the device.
 */
void spiSendData(SPI_TypeDef *spi, GPIO_TypeDef *port, PIN_NUM pin, uint8_t reg, uint8_t data)
{
    if (!(spi->CR1 & SPI_CR1_DFF))
    {
        spiWriteByte(spi, port, pin, reg);
        spiWriteByte(spi, port, pin, data);
    }
    else
    {
        spiWriteWord(spi, port, pin, (reg << 8) | data);
    }
}


//---- Not yet implemented -----

void spiReadByte(SPI_TypeDef *spi, uint8_t *data)
{

}

void spiReadWord(SPI_TypeDef *spi, uint16_t *data)
{

}
