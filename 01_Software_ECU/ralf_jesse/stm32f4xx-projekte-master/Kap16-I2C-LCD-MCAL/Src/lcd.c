/*
 * lcd.c
 *
 *  Created on: Sep 18, 2020
 *      Author: Ralf Jesse
 */
#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>

#include "mcalI2C.h"
#include "dwtDelay.h"
#include "lcd.h"

/**
 * @brief Initialisierung des HD44780-Display-Controllers
 *
 * @param[in]  *i2c : Pointer auf die I2C-Komponente
 *
 * @note
 * Der HD44780 bzw kompatible Typen reagieren sehr empfindlich auf
 * die zeitliche Abfolge der Kommandos. Es ist daher moeglich, dass
 * das Display nicht zuverlaessig funktioniert. Meine Tests ergaben,
 * dass Probleme haeufig dann auftreten, wenn die Funktion lcdSendChar()
 * verwendet wird. Alternativ kann dann die Funktion lcdSendString()
 * eingesetzt werden: In diesem Fall ist die Datenlaenge dann auf den
 * Wert 1 zu setzen.
 */
void lcdInit(I2C_TypeDef *i2c)
{
    /* 4-bit initialization */
    DWT_Delay_us(45000);                        // Warte mind. 40 us nach dem Reset
    lcdSendCmd(i2c, LCD_ADDR, LCD_8_BIT_MODE);  // Sende 3x LCD_8_BIT_MODE
    DWT_Delay_us(5000);
    lcdSendCmd(i2c, LCD_ADDR, LCD_8_BIT_MODE);
    DWT_Delay_us(1000);
    lcdSendCmd(i2c, LCD_ADDR, LCD_8_BIT_MODE);
    DWT_Delay_us(10000);
    lcdSendCmd(i2c, LCD_ADDR, LCD_4_BIT_MODE);  // Umschaltung auf LCD_4_BIT_Mode
//    DWT_Delay_us(10000);

    /**
     * Aktivieren Sie das von Ihnen verwendete Display in lcd.h!
     */
#if (MY_LCD == 420)
    lcdSendCmd(i2c, LCD_ADDR, LCD_NUM_LINES_4 | LCD_FONT_5X7);
    DWT_Delay_us(1000);

    // Display-Modus (DM)
    lcdSendCmd(i2c, LCD_ADDR, LCD_DM);
    DWT_Delay_us(1000);
    lcdSendCmd(i2c, LCD_ADDR, LCD_CLEAR);       // LCD-Anzeige leeren
    DWT_Delay_us(2000);

    // Cursor-Einstellungen
//    lcdSendCmd(i2c, LCD_ADDR, LCD_CURSOR_HOME | LCD_CURSOR_INCREASE | LCD_CURSOR_ON);
    lcdSendCmd(i2c, LCD_ADDR, 0x06);
    DWT_Delay_us(1000);

    // Cursor und Display einschalten
    lcdSendCmd(i2c, LCD_ADDR, LCD_CURSOR_ON | LCD_DISPLAY_ON);
    DWT_Delay_us(1000000);

 #elif (MY_LCD == 220)
    lcdSendCmd(i2c, LCD_ADDR, LCD_NUM_LINES_2 | LCD_FONT_5X7);
    DWT_Delay_us(1000);

    // Display-Modus (DM)
    lcdSendCmd(i2c, LCD_ADDR, LCD_DM);
    DWT_Delay_us(1000);
    lcdSendCmd(i2c, LCD_ADDR, LCD_CLEAR);       // LCD-Anzeige leeren
    DWT_Delay_us(2000);

    // Cursor-Einstellungen
    DWT_Delay_us(1000);

    // Cursor und Display einschalten
    lcdSendCmd(i2c, LCD_ADDR, LCD_CURSOR_ON | LCD_DISPLAY_ON);

#elif (MY_LCD == 216)
    lcdSendCmd(i2c, LCD_ADDR, LCD_NUM_LINES_2 | LCD_FONT_5X7);
    DWT_Delay_us(1000);

    // Display-Modus (DM)
    lcdSendCmd(i2c, LCD_ADDR, LCD_DM);
    DWT_Delay_us(1000);
    lcdSendCmd(i2c, LCD_ADDR, LCD_CLEAR);       // LCD-Anzeige leeren
    DWT_Delay_us(2000);

    // Cursor-Einstellungen
    DWT_Delay_us(1000);

    // Cursor und Display einschalten
    lcdSendCmd(i2c, LCD_ADDR, LCD_CURSOR_ON | LCD_DISPLAY_ON);
#endif  // Display: Einstellung der Betriebsart
}

/**
 * @brief Sendet Kommandos an den LCD-Controller
 */
void lcdSendCmd(I2C_TypeDef *i2c, uint8_t slaveAddr, uint8_t cmd)
{
    uint8_t  upperNibble = cmd & 0xF0;
    uint8_t  lowerNibble = (cmd << 4) & 0xF0;
    uint8_t  toSend[4]   = { 0 };

    // EN und RS einstellen
    toSend[0] = upperNibble | (LCD_EN    & (~LCD_RS));   // EN = 1, RS = 0
    toSend[1] = upperNibble | ((~LCD_EN) & (~LCD_RS));   // EN = 0, RS = 0
    toSend[2] = lowerNibble | (LCD_EN    & (~LCD_RS));   // EN = 1, RS = 0
    toSend[3] = lowerNibble | ((~LCD_EN) & (~LCD_RS));   // EN = 0, RS = 0

    i2cSendByte(i2c, slaveAddr, toSend[0]);
    i2cSendByte(i2c, slaveAddr, toSend[1]);
    i2cSendByte(i2c, slaveAddr, toSend[2]);
    i2cSendByte(i2c, slaveAddr, toSend[3]);

    if (cmd < 2)
    {
        DWT_Delay_us(2000);
    }
    else
    {
        DWT_Delay_us(1000);
    }
}

/**
 * @brief Sendet Textdaten an den LCD-Controller
 */
void lcdSendData(I2C_TypeDef *i2c, uint8_t slaveAddr, uint8_t data)
{
    uint8_t  upperNibble = data & 0xF0;
    uint8_t  lowerNibble = (data << 4) & 0xF0;
    uint8_t  toSend[4]   = { 0 };

    toSend[0] = upperNibble | (LCD_EN    | LCD_RS);      // EN = 1, RS = 1
    toSend[1] = upperNibble | ((~LCD_EN) | LCD_RS);      // EN = 0, RS = 1
    toSend[2] = lowerNibble | (LCD_EN    | LCD_RS);      // EN = 1, RS = 1
    toSend[3] = lowerNibble | ((~LCD_EN) | LCD_RS);      // EN = 0, RS = 1

    i2cSendByte(i2c, slaveAddr, toSend[0]);
    i2cSendByte(i2c, slaveAddr, toSend[1]);
    i2cSendByte(i2c, slaveAddr, toSend[2]);
    i2cSendByte(i2c, slaveAddr, toSend[3]);

    DWT_Delay_us(1000);
}

/**
 * @brief Sendet ein einzelnes Zeichen zum Display.
 *
 * @param[in]  *i2c       : Pointer auf die I2C-Komponente
 * @param[in]   slaveAddr : I2C-Adresse des LCD
 * @param[in]   line      : Zeile
 * @param[in]   col       : Spalte
 * @param[in]   text      : Zeichen
 *
 * @note
 *
 */
void lcdSendChar(I2C_TypeDef *i2c, uint8_t slaveAddr, uint8_t line, uint8_t col, char data)
{
    uint8_t numData = 0;
    uint8_t lineNum = line;

    /* Setup  */
    switch (line)
    {
        case 1:
            lineNum = LCD_LINE_1;
            break;

        case 2:
            lineNum = LCD_LINE_2;
            break;

        case 3:
            if (LCD_NUM_LINES == 4)
            {
                lineNum = LCD_LINE_3;
            }
            break;

        case 4:
            if (LCD_NUM_LINES == 4)
            {
                lineNum = LCD_LINE_4;
            }
            break;

        default:
            lineNum = LCD_LINE_1;
            break;
    }

    if (col < 1)
    {
        col = 1;
    }

    lcdSendCmd(i2c, slaveAddr, lineNum + col - 1);

    // Sieht seltsam aus. Das direkte Senden der Daten funktioniert aber
    // bei mir nicht.
    for (numData = 0; numData < 1; numData++)
    {
        lcdSendData(i2c, slaveAddr, data);
        DWT_Delay_us(1000);
    }
}

/**
 * @brief Sendet einen String zum Display.
 *
 * @param[in]  *i2c       : Pointer auf die I2C-Komponente
 * @param[in]   slaveAddr : I2C-Adresse des LCD
 * @param[in]   line      : Zeile
 * @param[in]   col       : Spalte
 * @param[in]  *text      : Pointer auf die Zeichenkette
 * @param[in]   dataLen   : Anzahl der Zeichen
 */
void lcdSendString(I2C_TypeDef *i2c, uint8_t slaveAddr, uint8_t line, uint8_t col, char *text, uint8_t dataLen)
{
    uint8_t lineNum = line;
    uint8_t numData = 0;

    /* Setup  */
    switch (line)
    {
        case 1:
            lineNum = LCD_LINE_1;
            break;

        case 2:
            lineNum = LCD_LINE_2;
            break;

        case 3:
            if (LCD_NUM_LINES == 4)
            {
                lineNum = LCD_LINE_3;
            }
            break;

        case 4:
            if (LCD_NUM_LINES == 4)
            {
                lineNum = LCD_LINE_4;
            }
            break;

        default:
            lineNum = LCD_LINE_1;
            break;
    }

    if (col < 1)
    {
        col = 1;
    }

    lcdSendCmd(i2c, slaveAddr, lineNum + col - 1);
    for (numData = 0; numData < dataLen; numData++)
    {
        lcdSendData(i2c, slaveAddr, *text++);
        DWT_Delay_us(1000);
    }
}
