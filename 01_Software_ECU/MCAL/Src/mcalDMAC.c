/**
 * mcalDMAC.c
 *
 *  Created on: 26.10.2020
 *      Author: Ralf Jesse
 *       Email: embedded@ralf-jesse.de
 */

#include <mcalDMAC.h>

/**
 * P R I V A T E   F U N C T I O N S
 */

/**
 * @brief Verifies that the DMAC number is correct. With error handling.
 *
 * param[in]   *stream : Must be DMA1_Stream0 ... DMA1_Stream7 / DMA2_Stream0 ... DMA2_Stream7
 * @param[out]  error code
 */
static DMAC_ERROR_CODE_t dmacVerifyDmac(DMA_TypeDef *dmac)
{
    if ((DMA1 != dmac) && (DMA2 != dmac))
    {
        return DMAC_INVALID_DMAC;
    }
    return DMAC_OK;
}

/**
 * @brief Verifies the integrity of the given stream. With error handling.
 *
 * param[in]   *stream : Must be DMA1_Stream0 ... DMA1_Stream7 / DMA2_Stream0 ... DMA2_Stream7
 * @param[out]  error code
 */
static DMAC_ERROR_CODE_t dmacVerifyStream(DMA_Stream_TypeDef *stream)
{
    if ((stream != DMA1_Stream0) && (stream != DMA1_Stream1) &&
        (stream != DMA1_Stream2) && (stream != DMA1_Stream3) &&
        (stream != DMA1_Stream4) && (stream != DMA1_Stream5) &&
        (stream != DMA1_Stream6) && (stream != DMA1_Stream7) &&
        (stream != DMA2_Stream0) && (stream != DMA2_Stream2) &&
        (stream != DMA2_Stream2) && (stream != DMA2_Stream3) &&
        (stream != DMA2_Stream4) && (stream != DMA2_Stream5) &&
        (stream != DMA2_Stream6) && (stream != DMA2_Stream7))
    {
        return DMAC_INVALID_STREAM;
    }
    return DMAC_OK;
}

/**
 * @brief Verifies the integrity of the channel number. With error handling.
 *
 * param[in]   channel : Must be in the range [CHANNEL_0 ... CHANNEL_7]
 * @param[out] error code
 */
static DMAC_ERROR_CODE_t dmacVerifyChannel(DMAC_CHANNEL_t channel)
{
    if ((channel != CHANNEL_0) && (channel != CHANNEL_1) &&
        (channel != CHANNEL_2) && (channel != CHANNEL_3) &&
        (channel != CHANNEL_4) && (channel != CHANNEL_5) &&
        (channel != CHANNEL_6) && (channel != CHANNEL_7))
    {
        return DMAC_INVALID_CHANNEL;
    }
    return DMAC_OK;
}

/**
 * @brief Verifies the integrity of the direction. With error handling.
 *
 * param[in]   dir : Must be in [MEM_2_PER, PER_2_MEM, MEM_2_MEM]
 * @param[out] error code
 */
static DMAC_ERROR_CODE_t dmacVerifyDirection(DMAC_DIRECTION_t dir)
{
    switch (dir)
    {
        case PER_2_MEM:
        case MEM_2_PER:
        case MEM_2_MEM:
            return DMAC_OK;
    }
    return DMAC_INVALID_DIR;
}

/**
 * @brief Verifies the integrity of the data format. With error handling.
 *
 * param[in]   format : Must be in [BYTE, HALFWORD, WORD]
 * @param[out] error code
 */
static DMAC_ERROR_CODE_t dmacVerifyDataFormat(DMAC_DATA_FORMAT_t format)
{
    if ((format == BYTE) || (format == HALFWORD) || (format == WORD))
    {
        return DMAC_OK;
    }
    else
    {
        return DMAC_INVALID_DATA_FORMAT;
    }

}

/**
 * @brief Verifies the integrity of the interrupt type. With error handling.
 *
 * param[in]   irq : Must be in [DIRECT_MODE_ERR, TX_ERR, TX_HALF, TX_COMPLETE]
 * @param[out] error code
 */
static DMAC_ERROR_CODE_t dmacVerifyIrqType(DMAC_IRQ_t irq)
{
    if ((DIRECT_MODE_ERR != irq) && (TX_ERR != irq) && (TX_HALF != irq) && (TX_COMPLETE != irq))
    {
        return DMAC_INVALID_IRQ_TYPE;
    }

    return DMAC_OK;
}

/**
 * @brief Verifies the integrity of the data block size. With error handling.
 *
 * param[in]   numData : 0 < numData < 65536
 * @param[out] error code
 */
static DMAC_ERROR_CODE_t dmacVerifyNumData(uint16_t numData)
{
    if ((numData < 0) || (numData > DMA_MAX_DATA_LEN))
    {
        return DMAC_INVALID_DATA_LEN;
    }

    return DMAC_OK;
}


/**
 * !!! Development not yet finished: Still under construction !!!
 */


/**
 * P U B L I C   F U N C T I O N S
 */


/**
 * @brief Activates the bus clock of the given DMA controller
 *
 * @param[in]   *dmac : Pointer to the DMA controller
 *
 * @param[out]  error code
 */
DMAC_ERROR_CODE_t dmacInitDMAC(DMA_TypeDef *dmac)
{
    if (DMA1 == dmac)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    }
    else if (DMA2 == dmac)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    }
    else
    {
        return DMAC_INVALID_DMAC;
    }

    return DMAC_OK;
}

/**
 * @brief Enables the stream. With error handling.
 *
 * @param[in]  *stream : Pointer to the DMA stream
 * @param[out]  errorCode
 *
 * @note
 * Enable a DMA stream AFTER setting the stream parameters.
 */
DMAC_ERROR_CODE_t dmacEnableStream(DMA_Stream_TypeDef *stream)
{
    if (dmacVerifyStream(stream) != DMAC_OK)
    {
        return DMAC_INVALID_STREAM;
    }
    stream->CR |= DMA_SxCR_EN;
    return DMAC_OK;
}

/**
 * @brief Disables the stream. With error handling.
 *
 * @param[in]  *stream : Pointer to the DMA stream
 * @param[out]  errorCode
 *
 * @note
 * It is required to disable a DMA stream BEFORE writing new settings
 * to the registers.
 */
DMAC_ERROR_CODE_t dmacDisableStream(DMA_Stream_TypeDef *stream)
{
    if (dmacVerifyStream(stream) != DMAC_OK)
    {
        return DMAC_INVALID_STREAM;
    }
    stream->CR &= ~DMA_SxCR_EN_Msk;
    while (stream->CR & DMA_SxCR_EN)
    {
        // Wait until the EN bit of the stream is "0"
    }

    return DMAC_OK;
}

/**
 * @brief Setting up the DMA stream, channel and IRQ type
 *
 * @param[in]   *stream     : Pointer to the stream
 * @param[in]    chn        : Channel
 * @param[in]    src        : Address of the data source
 * @param[in]    dest       : Address of the data destination
 * @param[in]    numData    : Number of the data
 * @param[in]    srcFormat  : Format of the source data size
 * @param[in]    dstFormat  : Format of the destination data size
 * @param[in]    dir        : Direction of the data transfer
 * @param[in]    irqType    : Interrupt types
 *
 * @param[out]   errorCode
 *
 * @note
 * Before calling this function the given stream must be disabled. After
 * a successful return don't forget to
 *    + enable the DMA stream
 *    + add the stream interrupt to NVIC
 */
DMAC_ERROR_CODE_t dmacSetStreamAndChannel(DMA_Stream_TypeDef *stream,
                                          DMAC_CHANNEL_t chn,
                                          uint32_t src,
                                          uint32_t dest,
                                          uint16_t numData,
                                          DMAC_DATA_FORMAT_t srcFormat,
                                          DMAC_DATA_FORMAT_t dstFormat,
                                          DMAC_DIRECTION_t dir,
                                          DMAC_IRQ_t irqType)
{
    // Verify the validity of the given parameters
    if (dmacVerifyStream(stream) != DMAC_OK)
    {
        return DMAC_INVALID_STREAM;
    }
    if (dmacVerifyChannel(chn) != DMAC_OK)
    {
        return DMAC_INVALID_CHANNEL;
    }
    if (dmacVerifyDirection(dir) != DMAC_OK)
    {
        return DMAC_INVALID_DIR;
    }
    if (dmacVerifyNumData(numData) != DMAC_OK)
    {
        return DMAC_INVALID_DATA_LEN;
    }
    if (dmacVerifyDataFormat(srcFormat) != DMAC_OK)
    {
        return DMAC_INVALID_SRC_DATA_FORMAT;
    }
    if (dmacVerifyDataFormat(dstFormat) != DMAC_OK)
    {
        return DMAC_INVALID_DEST_DATA_FORMAT;
    }
    if (dmacVerifyIrqType(irqType) != DMAC_OK)
    {
        return DMAC_INVALID_IRQ_TYPE;
    }

    // All parameter verfication passed with success. We can now continue
    // with the setup of the DMA stream.
    stream->CR    = chn << DMA_SxCR_CHSEL_Pos;          // Set the channel
    stream->CR   |= irqType;                            // OR-ed interrupt types
    stream->CR   &= ~DMA_SxCR_MSIZE_Msk;                // Reset MSIZE
    stream->CR   |= srcFormat << DMA_SxCR_MSIZE_Pos;    // Set MSIZE to srcFormat
    stream->CR   &= ~DMA_SxCR_PSIZE_Msk;                // Reset PSIZE
    stream->CR   |= dstFormat << DMA_SxCR_PSIZE_Pos;    // Set PSIZE to dstFormat
    stream->CR   &= ~DMA_SxCR_DIR_Msk;                  // Reset DIR
    stream->CR   |= dir << DMA_SxCR_DIR_Pos;            // Set DIR to dir
    stream->CR   |= DMA_SxCR_MINC;                      // Always use auto-increment

    stream->M0AR  = src;                                // Set address of the data source
    stream->PAR   = dest;                               // Set address of the destination
    stream->NDTR  = numData;                            // Set size of data block

    return DMAC_OK;
}

/**
 * @brief Clears all interrupt flags of the given DMA/Stream combination
 */
DMAC_ERROR_CODE_t dmacClearStreamIrqFlags(DMA_TypeDef *dmac, DMA_Stream_TypeDef *stream)
{
    if (dmacVerifyDmac(dmac) != DMAC_OK)
    {
        return DMAC_INVALID_DMAC;
    }
    if (dmacVerifyStream(stream) != DMAC_OK)
    {
        return DMAC_INVALID_STREAM;
    }

    // Reset all interrupt flags for DMA_Stream0
    if ((DMA1_Stream0 == stream) || (DMA2_Stream0 == stream))
    {
        dmac->LIFCR |= (DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0);
    }

    // Reset all interrupt flags for DMAx_Stream1
    if ((DMA1_Stream1 == stream) || (DMA2_Stream1 == stream))
    {
        dmac->LIFCR |= (DMA_LIFCR_CFEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTCIF1);
    }

    // Reset all interrupt flags for DMA_Stream2
    if ((DMA1_Stream2 == stream) || (DMA2_Stream2 == stream))
    {
        dmac->LIFCR |= (DMA_LIFCR_CFEIF2 | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CTEIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTCIF2);
    }

    // Reset all interrupt flags for DMA_Stream3
    if ((DMA1_Stream3 == stream) || (DMA2_Stream3 == stream))
    {
        dmac->LIFCR |= (DMA_LIFCR_CFEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTCIF3);
    }

    // Reset all interrupt flags for DMA_Stream4
    if ((DMA1_Stream4 == stream) || (DMA2_Stream4 == stream))
    {
        dmac->HIFCR |= (DMA_HIFCR_CFEIF4 | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CTEIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTCIF4);
    }

    // Reset all interrupt flags for DMA_Stream5
    if ((DMA1_Stream5 == stream) || (DMA2_Stream5 == stream))
    {
        dmac->HIFCR |= (DMA_HIFCR_CFEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5);
    }

    // Reset all interrupt flags for DMA_Stream6
    if ((DMA1_Stream6 == stream) || (DMA2_Stream6 == stream))
    {
        dmac->HIFCR |= (DMA_HIFCR_CFEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTCIF6);
    }

    // Reset all interrupt flags for DMA_Stream7
    if ((DMA1_Stream7 == stream) || (DMA2_Stream7 == stream))
    {
        dmac->HIFCR |= (DMA_HIFCR_CFEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTCIF7);
    }

    return DMAC_OK;
}
