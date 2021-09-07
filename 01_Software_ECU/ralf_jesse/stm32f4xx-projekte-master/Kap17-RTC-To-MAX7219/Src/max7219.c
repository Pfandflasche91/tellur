/**
 * max7219.c
 *
 *  Created on: 17.10.2020
 *      Author: Ralf Jesse
 *       Email: embedded@ralf-jesse.de
 */

#include <global.h>
#include <max7219.h>
#include <mcalSysTick.h>
#include <mcalGPIO.h>
#include <mcalSPI.h>

void max7219Init(SPI_TypeDef *spi)
{
    // Check the data format (8 or 16 bit) for the selected SPI
    if (!(spi->CR1 & SPI_CR1_DFF))        // Data format is set to 8 Bit
    {
        spiWriteByte(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, REG_DECODE);
        spiWriteByte(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, NO_DECODE);

        spiWriteByte(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, REG_INTENSITY);
        spiWriteByte(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, LVL_3);

        spiWriteByte(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, REG_SCAN_LIMIT);
        spiWriteByte(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, 7);

        spiWriteByte(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, REG_SHUTDOWN);
        spiWriteByte(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, NORMAL_OP);

        spiWriteByte(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, REG_DISPTEST);
        spiWriteByte(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, TEST_OFF);
    }
    else                                // Data format is set to 16 bit
    {
        spiWriteWord(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, ((REG_DECODE     << 8) | NO_DECODE));
        spiWriteWord(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, ((REG_INTENSITY  << 8) | LVL_3));
        spiWriteWord(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, ((REG_SCAN_LIMIT << 8) | 7));
        spiWriteWord(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, ((REG_SHUTDOWN   << 8) | NORMAL_OP));
        spiWriteWord(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, ((REG_DISPTEST   << 8) | TEST_OFF));
    }
}

void max7219TurnOn(SPI_TypeDef *spi)
{
    // Check the data format (8 or 16 bit) for the selected SPI
    if (!(spi->CR1 & !SPI_CR1_DFF))        // Data format is set to 8 Bit
    {
        spiWriteByte(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, REG_SHUTDOWN);
        spiWriteByte(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, NORMAL_OP);
    }
    else                                // Data format is set to 16 bit
    {
        spiWriteWord(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, ((REG_SHUTDOWN << 8) | NORMAL_OP));
    }
}

void max7219Shutdown(SPI_TypeDef *spi)
{
    // Check the data format (8 or 16 bit) for the selected SPI
    if (!(spi->CR1 & !SPI_CR1_DFF))        // Data format is set to 8 Bit
    {
        spiWriteByte(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, REG_SHUTDOWN);
        spiWriteByte(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, 0);
    }
    else                                // Data format is set to 16 bit
    {
        spiWriteWord(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, ((REG_SHUTDOWN << 8) | SHUTDOWN));
    }
}

void max7219SetDecodeMode(SPI_TypeDef *spi, MAX7219_DECODE_t mode)
{
    uint8_t decodeMode = mode;

    // Check the data format (8 or 16 bit) for the selected SPI
    if (!(spi->CR1 & SPI_CR1_DFF))        // Data format is set to 8 Bit
    {
        spiWriteByte(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, REG_DECODE);
        spiWriteByte(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, decodeMode);
    }
    else                                // Data format is set to 16 bit
    {
        spiWriteWord(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, ((REG_DECODE << 8) | decodeMode));
    }
}

void max7219ResetDigits(SPI_TypeDef *spi, uint8_t value)
{
    uint8_t reg = REG_DIG0;

    for (reg = REG_DIG0; reg < REG_DECODE; reg++)
    {
        // Check the data format (8 or 16 bit) for the selected SPI
        if (!(spi->CR1 & SPI_CR1_DFF))        // Data format is set to 8 Bit
        {
            spiWriteByte(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, reg);
            spiWriteByte(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, value);
        }
        else                                // Data format is set to 16 bit
        {
            spiWriteWord(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, ((reg << 8) | value));
        }
    }
}

void max7219Clear(SPI_TypeDef *spi)
{
    uint16_t reg;

    for (reg = REG_DIG0; reg < REG_DIG7; reg++)
    {
        // Check the data format (8 or 16 bit) for the selected SPI
        if (!(spi->CR1 & !SPI_CR1_DFF))        // Data format is set to 8 Bit
        {
            spiWriteByte(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, reg);
            spiWriteByte(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, 0x0F);
        }
        else                                // Data format is set to 16 bit
        {
            spiWriteWord(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, ((reg | 0x0F) & 0x0F));
        }
    }
}

void max7219TestDigits(SPI_TypeDef *spi, uint8_t value)
{
    uint8_t reg = 0;

    for (reg = REG_DIG0; reg < REG_DECODE; reg++)
    {
        // Check the data format (8 or 16 bit) for the selected SPI
        if (!(spi->CR1 & !SPI_CR1_DFF))        // Data format is set to 8 Bit
        {
            spiWriteByte(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, reg);
            spiWriteByte(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, value);
        }
        else                                // Data format is set to 16 bit
        {
            spiWriteWord(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, ((reg << 8) | value));
        }
    }
}

void max7219SetIntensity(SPI_TypeDef *spi, MAX7219_INTENSITY_t lvl)
{
    if (!(spi->CR1 & SPI_CR1_DFF))
    {
        spiWriteByte(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, REG_INTENSITY);
        spiWriteByte(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, lvl);
    }
    else
    {
        spiWriteWord(spi, MAX7219_CS_PORT, MAX7219_CS_PIN, ((REG_INTENSITY << 8) | lvl));
    }
}
