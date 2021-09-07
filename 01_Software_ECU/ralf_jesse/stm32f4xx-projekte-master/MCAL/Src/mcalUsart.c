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

#include <mcalUsart.h>

#define USART_TX_EMPTY(usart)  ((usart)->SR & USART_SR_TXE)
#define USART_WAIT(usart)      do { while(!USART_TX_EMPTY(usart)); } while(0)

void usartEnableUsart(USART_TypeDef *usart)
{
    if (USART1 == usart)
    {
        USART1->CR1 |= USART_CR1_UE;
    }
    else if (USART2 == usart)
    {
        USART2->CR1 |= USART_CR1_UE;
    }
    else if (USART3 == usart)
    {
        USART3->CR1 |= USART_CR1_UE;
    }
    else if (UART4 == usart)
    {
        UART4->CR1 |= USART_CR1_UE;
    }
    else if (UART5 == usart)
    {
        UART5->CR1 |= USART_CR1_UE;
    }
    else if (USART6 == usart)
    {
        USART6->CR1 |= USART_CR1_UE;
    }
}

/**
 * @brief This is an exact copy of usartEnableUsart() (see function above).
 *
 * @nore
 * Deprecated. This function should not longer be used.
 */
void usartStartUsart(USART_TypeDef *usart)
{
    if (USART1 == usart)
    {
        USART1->CR1 |= USART_CR1_UE;
    }
    else if (USART2 == usart)
    {
        USART2->CR1 |= USART_CR1_UE;
    }
    else if (USART3 == usart)
    {
        USART3->CR1 |= USART_CR1_UE;
    }
    else if (UART4 == usart)
    {
        UART4->CR1 |= USART_CR1_UE;
    }
    else if (UART5 == usart)
    {
        UART5->CR1 |= USART_CR1_UE;
    }
    else if (USART6 == usart)
    {
        USART6->CR1 |= USART_CR1_UE;
    }
}

void usartDisableUsart(USART_TypeDef *usart)
{
    if (USART1 == usart)
    {
        USART1->CR1 &= ~USART_CR1_UE_Msk;
    }
    else if (USART2 == usart)
    {
        USART2->CR1 &= ~USART_CR1_UE_Msk;
    }
    else if (USART3 == usart)
    {
        USART3->CR1 &= ~USART_CR1_UE_Msk;
    }
    else if (UART4 == usart)
    {
        UART4->CR1 &= ~USART_CR1_UE_Msk;
    }
    else if (UART5 == usart)
    {
        UART5->CR1 &= ~USART_CR1_UE_Msk;
    }
    else if (USART6 == usart)
    {
        USART6->CR1 &= ~USART_CR1_UE_Msk;
    }
}

/**
 * @brief Turns the bus clock of *usart on.
 *
 * @param[in]  *usart : Pointer to the USART/UART type
 */
void usartSelectUsart(USART_TypeDef *usart)
{
    if (USART1 == usart)
    {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    }

    if (USART2 == usart)
    {
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    }

    if (USART3 == usart)
    {
        RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    }

    if (UART4 == usart)
    {
        RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
    }

    if (UART5 == usart)
    {
        RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
    }

    if (USART6 == usart)
    {
        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
    }
}

/**
 * @brief Turns the bus clock of *usart off.
 *
 * @param[in]  *usart : Pointer to the USART/UART type
 */
void usartDeselectUsart(USART_TypeDef *usart)
{
    if (USART1 == usart)
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
    }

    if (USART2 == usart)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
    }

    if (USART3 == usart)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN;
    }

    if (UART4 == usart)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_UART4EN;
    }

    if (UART5 == usart)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_UART5EN;
    }

    if (USART6 == usart)
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_USART6EN;
    }
}

// TODO: USART Flow control noch nicht implementiert
void usartSetFlowCtrlMode(USART_TypeDef *usart, USART_FLOWCTRL_t flow)
{

}

/**
 * @brief Enables the requested USART IRQ type.
 *
 * @param[in]  *usart   : Pointer to the UART/USART
 * @param[in]   irqType : The IRQ that shall be enabled
 */
void usartEnableIrq(USART_TypeDef *usart, USART_IRQ_TYPES irqType)
{
    switch (irqType)
    {
        /* Transmit buffer empty */
        case USART_IRQ_TXEIE:
            usart->CR1 |= USART_CR1_TXEIE;
            break;

        /* Clear-to-send */
        case USART_IRQ_CTSIE:
            usart->CR3 |= USART_CR3_CTSIE;
            break;

        /* Transmission complete */
        case USART_IRQ_TCIE:
            usart->CR1 |= USART_CR1_TCIE;
            break;

        /* Error: Detects framing/overrun/noise errors */
        case USART_IRQ_EIE:
            usart->CR3 |= USART_CR3_EIE;
            break;

        /* Receive buffer not empty */
        case USART_IRQ_RXNEIE:
            usart->CR1 |= USART_CR1_RXNEIE;
            break;

        /* UART/USART idle (no activity) */
        case USART_IRQ_IDLE:
            usart->CR1 |= USART_CR1_IDLEIE;
            break;

        /* Parity error */
        case USART_IRQ_PEIE:
            usart->CR1 |= USART_CR1_PEIE;
            break;

        /* Line break */
        case USART_IRQ_LBDIE:
            break;
    }
}

/**
 * @brief Disables the requested USART IRQ type
 *
 * @param[in]  *usart   : Pointer to the UART/USART
 * @param[in]   irqType : Interrupt which shall be disabled
 */
void usartDisableIrq(USART_TypeDef *usart, USART_IRQ_TYPES irqType)
{
    switch (irqType)
    {
        /* Transmit buffer empty */
        case USART_IRQ_TXEIE:
            usart->CR1 &= ~USART_CR1_TXEIE_Msk;
            break;

        /* Clear-to-send */
        case USART_IRQ_CTSIE:
            usart->CR3 &= ~USART_CR3_CTSIE_Msk;
            break;

        /* Transmission complete */
        case USART_IRQ_TCIE:
            usart->CR1 &= ~USART_CR1_TCIE_Msk;
            break;

        /* Error: Detects framing/overrun/noise errors */
        case USART_IRQ_EIE:
            usart->CR3 &= ~USART_CR3_EIE_Msk;
            break;

        /* Receive buffer not empty */
        case USART_IRQ_RXNEIE:
            usart->CR1 &= ~USART_CR1_RXNEIE_Msk;
            break;

        /* UART/USART idle (no activity) */
        case USART_IRQ_IDLE:
            usart->CR1 &= ~USART_CR1_IDLEIE_Msk;
            break;

        /* Parity error */
        case USART_IRQ_PEIE:
            usart->CR1 &= ~USART_CR1_PEIE_Msk;
            break;

        /* Line break */
        case USART_IRQ_LBDIE:
            usart->CR2 &= ~USART_CR2_LBDIE_Msk;
            break;
    }
}

/**
 * @brief Sets the baud rate register BRR according to SYSCLK
 *
 * @param[in]  *usart    : Pointer to USARTn
 * @param[in]   baudrate : baudrate in bps
 */
void usartSetBaudrate(USART_TypeDef *usart, uint32_t baudrate)
{
    uint32_t systemClock = 0u;
    uint8_t  over = 0;
    uint32_t baudRateMantissa = 0.0f;
    uint32_t baudRateFraction = 0.0f;

    SystemCoreClockUpdate();
    systemClock = systemGetSysClock();

    over = usart->CR1 & USART_CR1_OVER8;

    baudRateMantissa = (systemClock / (8 * (2 - over) * baudrate));    // Korrekt = 104 bei 9600 bps @ 16 MHz
    baudRateFraction = (systemClock - (baudRateMantissa * 8 * (2 - over) * baudrate)) / baudrate;

    usart->BRR = baudRateMantissa << 4 | baudRateFraction;
}

/**
 * @brief Turns the bus clock of usart on or off.
 *
 * @param[in]  *usart : Pointer to the USART
 * @param[in]   mode  : Either USART_ON or USART_OFF
 */
void usartBusClk(USART_TypeDef *usart, USART_BUSCLK mode)
{
}

/**
 * @brief Set USART communication parameters
 *
 * @param[in]  *usart    : Pointer to the USART
 * @param[in]   baudrate : Baudrate in bps
 * @param[in]   parity   : Parity (None, even, odd)
 * @param[in]   len      : Wordlength (8 or 9 bit)
 * @param[in]   stop     : Number of stop bits
 */
void usartSetCommParams(USART_TypeDef *usart,
                        uint32_t baudrate,
                        USART_PARITY parity,
                        USART_WORDLEN len,
                        USART_STOPBITS num)
{
    usartSelectUsart(usart);
    usartSetBaudrate(usart, baudrate);              // Set baudrate
    usartEnableUsart(usart);
    usartSetWordlength(usart, len);
    usartSetNumStopBits(usart, num);
    usartSetParity(usart, parity);
    usartEnableReceiver(usart, RECEIVER_ON);        // Always activate receiver ...
    usartEnableTransmitter(usart, TRANSMITTER_ON);  // ... and transmitter
}

/**
 * @brief Sets the word length of the data word. Sets the M bit of the USART control register 1.
 *
 * @param[in]  *usart   : Pointer to the USART
 * @param[in]   wordLen : Length of the data word
 */
void usartSetWordlength(USART_TypeDef *usart, USART_WORDLEN len)
{
    if (LEN_8BIT == len)
    {
        usart->CR1 &= ~USART_CR1_M;
    }
    else
    {
        usart->CR1 |= USART_CR1_M;
    }
}

/**
 * @brief Sets the parity of the serial communication.
 *
 * @param[in]  *usart  : Pointer to the USART
 * @param[in]   parity : Parity setting
 */
void usartSetParity(USART_TypeDef *usart, USART_PARITY parity)
{
    if (NO_PARITY == parity)
    {
        usart->CR1 &= ~USART_CR1_PCE;       // No parity
    }
    else
    {
        if (EVEN_PARITY == parity)
        {
            usart->CR1 |= USART_CR1_PCE;
            usart->CR1 &= ~USART_CR1_PS;    // Even parity
        }
        else
        {
            usart->CR1 |= USART_CR1_PCE;
            usart->CR1 |= USART_CR1_PS;     // Odd parity
        }
    }
}

/**
 * @brief Sets the number of stop bits.
 *
 * @param[in]  *usart  : Pointer to the USART
 * @param[in]   parity : Parity setting
 */
void usartSetNumStopBits(USART_TypeDef *usart, USART_STOPBITS num)
{
    usart->CR2 &= ~USART_CR2_STOP_Msk;          // Default: 1 stop bit

    switch (num)
    {
        case HALF_BIT:
            usart->CR2 |= USART_CR2_STOP_0;     // 0.5 stop bits
            break;

        case ONE_BIT:
            usart->CR2 &= ~USART_CR2_STOP_Msk;  // 1 stop bit
            break;

        case ONE_DOT_FIVE:
            usart->CR2 |= USART_CR2_STOP_1;     // 1.5 stop bits
            break;

        case TWO_BIT:
            usart->CR2 |= USART_CR2_STOP;       // 2 stop bits

        default:
            usart->CR2 &= ~USART_CR2_STOP_Msk;  // 1 stop bit
            break;
    }
}

/**
 * @brief Selects between 8x/16x over sampling. 16x is default.
 *
 * @param[in]  *usart : Pointer to the USART
 * @param[in]   over  : Over-sampling mode
 */
void usartSetOversampling(USART_TypeDef *usart, USART_OVER over)
{
    usart->CR1 &= ~USART_CR1_OVER8_Msk;         // Default is 16x

    if (OVER8 == over)
    {
        usart->CR1 |= USART_CR1_OVER8;
    }
}

/**
 * @brief Enables/disables the USART receiver
 *
 * @param[in]  *usart  : Pointer to the USART
 * @param[in]   enable : Enables/disables the receiver
 */
void usartEnableReceiver(USART_TypeDef *usart, USART_RX enable)
{
    if (RECEIVER_OFF == enable)
    {
        usart->CR1 &= ~USART_CR1_RE_Msk;
    }
    else
    {
        usart->CR1 |= USART_CR1_RE;
    }
}

/**
 * @brief Enables/disables the USART transmitter
 *
 * @param[in]  *usart  : Pointer to the USART
 * @param[in]   enable : Enables/disables the transmitter
 */
void usartEnableTransmitter(USART_TypeDef *usart, USART_TX enable)
{
    if (TRANSMITTER_OFF == enable)
    {
        usart->CR1 &= ~USART_CR1_TE_Msk;
    }
    else
    {
        usart->CR1 |= USART_CR1_TE;
    }
}

/**
 * @brief This function enables all IRQ types which are provided by the *irqList.
 *
 * @param[in]  *usart   : Pointer to the UART/USART
 * @param[in]  *irqList : Pointer to the list which provides IRQ types
 */
void usartEnableIrqList(USART_TypeDef *usart, USART_IRQ_TYPES *irqList)
{
    uint8_t         i = 0;
    uint8_t         numIrqs = 0;
    USART_IRQ_TYPES irqType;

    numIrqs = sizeof(irqList)/sizeof(uint8_t);
    for (i = 0; i < numIrqs; i++)
    {
        irqType = irqList[i];
        usartEnableIrq(usart, irqType);
    }
}

#if 0
/**
 * @brief Pushes incoming data to the device buffer.
 */
void usartPushDataToBuffer(USART_TypeDef *device, uint16_t data)
{
//    if (device->currentPosInBuffer < device->bufferSize)
//    {
//        // Check overflow of the buffer
//        if (device->head == device->bufferSize)
//        {
//            device->head = 0;   // Restart from the beginning
//        }
//    }

    // Add data to the buffer
//    device->buffer[device->head] = data;
//    device->head++;
//    device->currentPosInBuffer++;
}


/**
 * @brief Reads one byte from the buffer
 */
uint16_t usartPullDataFromBuffer(USART_t *device)
{
    uint16_t byte = 0;
    if ((device->currentPosInBuffer > 0) || (device->head != device->tail))
    {
        if (device->tail == device->head)
        {
            device->tail = 0;
        }

        byte = device->buffer[device->tail];
        device->tail++;

        if (device->currentPosInBuffer)
        {
            device->currentPosInBuffer--;
        }
    }
    return byte;
}

void usartSendBuf(USART_TypeDef *usart, Ringbuffer_t *rb, uint8_t *buffer)
{
    while (*buffer)
    {
        USART_WAIT(usart);
        usart->DR = buffer[rb->readPtr];
        rb->readPtr++;
        USART_WAIT(usart);
    }
}
#endif

/**
 * @brief Interrupt handler for USART2
 */
//void USART2_IRQHandler(void)
//{
//    USART_BUFFER_t *buf;
//    uint8_t         data = 0;
//
//    /** Receive data via USART2 **/
//    if (USART2->SR & USART_SR_RXNE)
//    {
//        USART_WAIT(USART2);
//        data = USART2->DR;                  // Reading automatically resets the RXNE flag
//        USART_WAIT(USART2);
//
//        buf = receiveBuffer;                // Assign address of the receive buffer
//
//        if (((buf->inPtr - buf->outPtr) & ~(BUFFERSIZE - 1)) == 0)
//        {
//            buf->buffer[buf->inPtr & (BUFFERSIZE - 1)] = data;
//            buf->inPtr++;
//        }
//    }
//
//    /** Transmit data via USART2 **/
//    if (USART2->SR & USART_SR_TXE)
//    {
//        USART2->SR &= ~USART_SR_TXE_Msk;
//        buf = transmitBuffer;               // Assign the address of the transmit buffer
//
//        /** If buf->inPtr != buf->outPtr --> Buffer has data **/
//        if (buf->inPtr != buf->outPtr)
//        {
//            USART2->DR = (buf->buffer[buf->outPtr & (BUFFERSIZE - 1)] & 0x01FF);
//            buf->outPtr++;
//        }
//        else    /** Transmit buffer is empty **/
//        {
//            USART2->CR1 &= ~USART_CR1_TXEIE_Msk;
//        }
//    }
//}

void usartSendString(USART_TypeDef *usart, char *data)
{
    while (*data)
    {
        USART_WAIT(usart);
        usart->DR = *data++ & 0x01FF;
        USART_WAIT(usart);
    }
}

void usartInitBuffer(void)
{
    transmitBuffer->inPtr  = 0;
    transmitBuffer->outPtr = 0;

    receiveBuffer->inPtr   = 0;
    receiveBuffer->outPtr  = 0;
}

/**
 * @brief Resets the given IRQ type flag
 */
void usartResetIrqFlag(USART_TypeDef *usart, USART_IRQ_FLAG_t irqFlag)
{
    switch (irqFlag)
    {
        case USART_CTS_FLG:
            usart->SR &= ~USART_SR_TC_Msk;
            break;
        case USART_LBD_FLG:
            usart->SR &= ~USART_SR_LBD_Msk;
            break;
        case USART_TC_FLG:
            usart->SR &= ~USART_SR_TC_Msk;
            break;
        case USART_RXNE_FLG:
            usart->SR &= ~USART_SR_RXNE_Msk;
            break;

        default:
            break;
    }
}

/**
 * @brief Sets the DMA mode for the transmitter. No error handling.
 */
void usartSetDmaTxMode(USART_TypeDef *usart, USART_DMA_TXMODE_t dmaMode)
{
    if (DMA_TRANSMIT_OFF == dmaMode)
    {
        usart->CR3 &= ~USART_CR3_DMAT_Msk;
    }
    else
    {
        usart->CR3 |= USART_CR3_DMAT;
    }
}

/**
 * @brief Sets the DMA mode for the receiver. No error handling.
 */
void usartSetDmaRxMode(USART_TypeDef *usart, USART_DMA_RXMode_t dmaMode)
{
    if (DMA_RECEIVE_OFF == dmaMode)
    {
        usart->CR3 &= ~USART_CR3_DMAR_Msk;
    }
    else
    {
        usart->CR3 |= USART_CR3_DMAR;
    }
}
