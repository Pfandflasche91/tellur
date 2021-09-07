/**
 * mcalDMA.h
 *
 *  Created on: 26.10.2020
 *      Author: Ralf Jesse
 *       Email: embedded@ralf-jesse.de
 */

#ifndef MCALDMAC_H_
#define MCALDMAC_H_

#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>

#define DMA_MAX_DATA_LEN    (65536)

typedef enum
{
    DMAC_OK                         =  0,
    DMAC_INVALID_DMAC               = -1,
    DMAC_INVALID_STREAM             = -2,
    DMAC_INVALID_CHANNEL            = -3,
    DMAC_INVALID_DIR                = -4,
    DMAC_INVALID_DATA_LEN           = -5,
    DMAC_INVALID_DATA_FORMAT        = -6,
    DMAC_INVALID_SRC_DATA_FORMAT    = -7,
    DMAC_INVALID_DEST_DATA_FORMAT   = -8,
    DMAC_INVALID_IRQ_TYPE           = -9
} DMAC_ERROR_CODE_t;

typedef enum
{
    CHANNEL_0,
    CHANNEL_1,
    CHANNEL_2,
    CHANNEL_3,
    CHANNEL_4,
    CHANNEL_5,
    CHANNEL_6,
    CHANNEL_7
} DMAC_CHANNEL_t;

typedef enum
{
    PER_2_MEM,          // Peripheral --> Memory
    MEM_2_PER,          // Memory     --> Peripheral
    MEM_2_MEM,          // Memory     --> Memory
} DMAC_DIRECTION_t;

typedef enum
{
    BYTE,
    HALFWORD,
    WORD
} DMAC_DATA_FORMAT_t;

typedef enum
{
    FIFO_OFF,
    FIFO_ON
} DMAC_FIFO_MODE_t;

typedef enum
{
    DIRECT_MODE_ERR =  2,
    TX_ERR          =  4,
    TX_HALF         =  8,
    TX_COMPLETE     = 16
} DMAC_IRQ_t;


// Function prototypes
extern DMAC_ERROR_CODE_t dmacInitDMAC(DMA_TypeDef *dmac);
extern DMAC_ERROR_CODE_t dmacEnableStream(DMA_Stream_TypeDef *stream);
extern DMAC_ERROR_CODE_t dmacDisableStream(DMA_Stream_TypeDef *stream);
extern DMAC_ERROR_CODE_t dmacSetStreamAndChannel(DMA_Stream_TypeDef *stream,
                                                 DMAC_CHANNEL_t channel,
                                                 uint32_t src,
                                                 uint32_t dest,
                                                 uint16_t numData,
                                                 DMAC_DATA_FORMAT_t srcFormat,
                                                 DMAC_DATA_FORMAT_t dstFormat,
                                                 DMAC_DIRECTION_t dir,
                                                 DMAC_IRQ_t irqType);
extern DMAC_ERROR_CODE_t dmacClearStreamIrqFlags(DMA_TypeDef *dmac, DMA_Stream_TypeDef *stream);

#endif /* MCALDMAC_H_ */
