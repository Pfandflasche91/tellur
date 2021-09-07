/**
 * mcalSPI.h
 *
 *  Created on: 17.10.2020
 *      Author: Ralf Jesse
 *       Email: embedded@ralf-jesse.de
 */

#ifndef MCALSPI_H_
#define MCALSPI_H_

#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>

typedef enum
{
  MASTER,
  SLAVE
} SPI_OPMODE_t;

typedef enum
{
    TWO_LINE_UNI,
    ONE_LINE_BIDI
} SPI_BidiMode_t;

typedef enum
{
    DATA_FORMAT_8  = 0,
    DATA_FORMAT_16
} SPI_DATA_FORMAT_t;

typedef enum
{
    CLK_DIV_2       = 0,
    CLK_DIV_4,
    CLK_DIV_8,
    CLK_DIV_16,
    CLK_DIV_32,
    CLK_DIV_64,
    CLK_DIV_128,
    CLK_DIV_256
} SPI_CLOCK_DIV_t;

typedef enum
{
    SPI_DATA_8BIT,
    SPI_DATA_16_BIT
} SPI_DATALEN_t;

typedef enum
{
    SSM_OFF,
    SSM_ON
} SPI_SSM_t;

typedef enum
{
    SSI_LVL_LOW,
    SSI_LVL_HIGH
} SPI_SSI_LVL_t;

typedef enum
{
    SPI_PHASE_EDGE_1,
    SPI_PHASE_EDGE_2
} SPI_PHASE_t;

typedef enum
{
    SPI_IDLE_LOW,
    SPI_IDLE_HIGH
} SPI_POLARITY_t;

typedef enum
{
    SPI_SEND_BYTE_1,
    SPI_SEND_BYTE_2
} SPI_8BIT_STATE_t;

extern void spiInitSPI (SPI_TypeDef *spi, SPI_CLOCK_DIV_t div, SPI_DATALEN_t len,
                        SPI_SSM_t ssm, SPI_SSI_LVL_t lvl, SPI_OPMODE_t opMode,
                        SPI_PHASE_t phase, SPI_POLARITY_t polarity);
extern void spiEnableSPI(SPI_TypeDef *spi);
extern void spiDisableSPI(SPI_TypeDef *spi);
extern void spiWriteByte(SPI_TypeDef *spi, GPIO_TypeDef *port, PIN_NUM pin, uint8_t data);
extern void spiWriteWord(SPI_TypeDef *spi, GPIO_TypeDef *port, PIN_NUM pin, uint16_t data);
extern void spiSendData(SPI_TypeDef *spi, GPIO_TypeDef *port, PIN_NUM pin, uint8_t reg, uint8_t data);

// Noch nicht implementiert
extern void spiReadByte(SPI_TypeDef *spi, uint8_t *data);
extern void spiReadWord(SPI_TypeDef *spi, uint16_t *data);


#endif /* MCALSPI_H_ */
