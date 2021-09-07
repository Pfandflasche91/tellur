/**
 * @brief Implementation of UART/USART control and access of the STM32F446
 *
 * Created on      : 16.08.2020
 *     Author      : Ralf Jesse
 *      Email      : embedded@ralf-jesse.de
 * Copyright       : Ralf Jesse, 2020
 * Other copyrights: Tilen Majerle, 2014
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

#ifndef MCALUSART_H_
#define MCALUSART_H_

#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <mcalSystem.h>
#include <mcalUsart.h>

/**
 * Type definitions and enumerations
 */
#define BUFFERSIZE  (128)

typedef struct
{
    uint8_t inPtr;
    uint8_t outPtr;
    uint8_t buffer[BUFFERSIZE];
} USART_BUFFER_t;

typedef enum
{
    USART_ENABLE = 0,
    USART_DISABLE
} USART_MODE_t;

typedef enum
{
    USART_ON = 0,
    USART_OFF
} USART_BUSCLK;

typedef enum
{
    NO_PARITY = 0,
    EVEN_PARITY,
    ODD_PARITY
} USART_PARITY;

typedef enum
{
    LEN_8BIT = 0,
    LEN_9BIT
} USART_WORDLEN;

typedef enum
{
    HALF_BIT        = 0,
    HALF_STOP       = 0,
    ONE_BIT         = 1,
    ONE_STOP        = 1,
    ONE_DOT_FIVE    = 2,
    TWO_BIT         = 3,
    TWO_STOP        = 3
} USART_STOPBITS;

typedef enum
{
    OVER16 = 0,
    OVER8
} USART_OVER;

typedef enum
{
    RECEIVER_OFF = 0,
    RECEIVER_ON
} USART_RX;

typedef enum
{
    TRANSMITTER_OFF = 0,
    TRANSMITTER_ON
} USART_TX;

typedef enum
{
    OFF = 0,
    ON
} USART_IRQ_MODE;

typedef enum
{
    USART_IRQ_TXEIE = 0,
    USART_IRQ_CTSIE,
    USART_IRQ_TCIE,
    USART_IRQ_RXNEIE,
    USART_IRQ_IDLE,
    USART_IRQ_PEIE,
    USART_IRQ_LBDIE,
    USART_IRQ_EIE
} USART_IRQ_TYPES;

typedef enum
{
    USART_CTS_FLG,
    USART_LBD_FLG,
    USART_TC_FLG,
    USART_RXNE_FLG
} USART_IRQ_FLAG_t;

typedef enum
{
    DMA_TRANSMIT_OFF,
    DMA_TRANSMIT_ON
} USART_DMA_TXMODE_t;

typedef enum
{
    DMA_RECEIVE_OFF,
    DMA_RECEIVE_ON
} USART_DMA_RXMode_t;

typedef enum
{
    // TODO: Konstanten noch nicht definiert
    NOT_YET_DEFINED
} USART_FLOWCTRL_t;

/**
 * Variables
 */
USART_BUFFER_t  *receiveBuffer;
USART_BUFFER_t  *transmitBuffer;

/**
 * Prototypes
 */

/* General UART/USART activities */
extern void usartSelectUsart(USART_TypeDef *usart);
extern void usartDeselectUsart(USART_TypeDef *usart);
extern void usartEnableUsart(USART_TypeDef *usart);
extern void usartDisableUsart(USART_TypeDef *usart);


extern void usartPushDataToBuffer(USART_TypeDef *device, uint16_t data);

/* USART buffer */
extern void usartInitBuffer(void);
extern void usartSendString(USART_TypeDef *usart, char *data);

/* Communication parameters */
extern void usartSetCommParams(USART_TypeDef *usart, uint32_t baudrate,
                               USART_PARITY parity, USART_WORDLEN len,
                               USART_STOPBITS stop);
extern void usartSetBaudrate(USART_TypeDef *usart, uint32_t baudrate);
extern void usartSetWordlength(USART_TypeDef *usart, USART_WORDLEN len);
extern void usartSetParity(USART_TypeDef *usart, USART_PARITY parity);
extern void usartSetNumStopBits(USART_TypeDef *usart, USART_STOPBITS num);
extern void usartSetOversampling(USART_TypeDef *usart, USART_OVER over);
extern void usartEnableReceiver(USART_TypeDef *usart, USART_RX enable);
extern void usartEnableTransmitter(USART_TypeDef *usart, USART_TX enable);
extern void usartSetFlowCtrlMode(USART_TypeDef *usart, USART_FLOWCTRL_t flow);

/* Sending and receiving data */
extern void usartGetByte(USART_TypeDef *usart, uint16_t byte);

/* Interrupts */
extern void usartEnableIrqList(USART_TypeDef *usart, USART_IRQ_TYPES *irqList);
extern void usartEnableIrq(USART_TypeDef *usart, USART_IRQ_TYPES irqType);
extern void usartDisableIrq(USART_TypeDef *usart, USART_IRQ_TYPES irqType);
extern void usartResetIrqFlag(USART_TypeDef *usart, USART_IRQ_FLAG_t irqFlag);

/* DMA */
extern void usartSetDmaTxMode(USART_TypeDef *usart, USART_DMA_TXMODE_t dmaMode);
extern void usartSetDmaRxMode(USART_TypeDef *usart, USART_DMA_RXMode_t dmaMode);

/* Deprecated */
extern void usartStartUsart(USART_TypeDef *usart);

#endif /* MCALUSART_H_ */
