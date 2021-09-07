/**
 * lcd.h
 *
 *  Created on: Sep 30, 2020
 *      Author: Ralf Jesse
 *       Email: embedded@ralf-jesse.de
 */

#ifndef LCD_H_
#define LCD_H_

/* Eigenschaften des Displays */
#define LCD_ADDR                (0x4E)      // I2C-Adresse des Displays
#define MY_LCD                  (420)       // 4 Zeilen mit je 20 Zeichen
//#define MY_LCD                  (220)       // 2 Zeilen mit je 20 Zeichen
//#define MY_LCD                  (216)       // 2 Zeilen mit je 16 Zeichen


/**
 * Kommandos und andere Definitionen fuer den LCD-Controller
 */
/* Steuerung des EN- bzw. RS-Pins */
#define LCD_RS                  (0x01)              /* Register Select pin   = P0 of the PCF8574 */
#define LCD_RW                  (0x02)              /* Read/Write pin        = P1 of the PCF8574 */
#define LCD_EN                  (0x04)              /* LCD Enable pin        = P2 of the PCF8574 */
#define LCD_BACKLIGHT           (0x08)              /* Backlight control pin = P3 of the PCF8574 */

/* LCD: Function set (FS) */
#define LCD_FS                  (0x20)
#define LCD_8_BIT_MODE          (LCD_FS | 0x10)     /* Set 8-bit mode      */
#define LCD_4_BIT_MODE          (LCD_FS | 0x20)     /* Set 4-bit mode      */
#define LCD_NUM_LINES_4         (LCD_FS | 0x09)     /* Number of lines = 4 */
#define LCD_NUM_LINES_2         (LCD_FS | 0x08)     /* Number of lines = 2 */
#define LCD_NUM_LINES_1         (LCD_FS | 0x00)     /* Number of lines = 1 */
#define LCD_FONT_5X7            (LCD_FS | 0x04)     /* Font size = 5x7     */
#define LCD_FONT_5X10           (LCD_FS | 0x00)     /* Font size = 5x10    */

/* LCD: Entry mode (EM) */
#define LCD_EM                  (0x04)
#define LCD_CURSOR_INCREASE     (LCD_EM | 0x02)     /* Shift cursor to the right */
#define LCD_CURSOR_DECREASE     (LCD_EM | 0x00)     /* Shift cursor to the left  */
#define LCD_DISP_SHIFT_ON       (LCD_EM | 0x01)     /* Shifts the display        */
#define LCD_DISP_SHIFT_OFF      (LCD_EM | 0x00)     /* Display not shifted       */

/* LCD: Display mode (DM) */
#define LCD_DM                  (0x08)
#define LCD_DISPLAY_ON          (LCD_DM | 0x04)
#define LCD_DISPLAY_OFF         (LCD_DM | 0x00)
#define LCD_CURSOR_ON           (LCD_DM | 0x02)

/* Simple LCD commands */
#define LCD_CLEAR               (0x01)
#define LCD_CURSOR_HOME         (0x02)
#define LCD_INIT                (0x30)

/* Display line numbers */
#define LCD_NUM_LINES           (0x04)
#define LCD_LINE_1              (0x80)
#define LCD_LINE_2              (0xC0)
#define LCD_LINE_3              (0x94)
#define LCD_LINE_4              (0xD4)

typedef enum
{
    LCD_CMD = 0,
    LCD_DATA
} LCD_OP_MODE;


/* Function prototypes */
extern void lcdInit (I2C_TypeDef *i2c);
extern void lcdSendCmd(I2C_TypeDef *i2c, uint8_t slaveAddr, uint8_t cmd);
extern void lcdSendData(I2C_TypeDef *i2c, uint8_t slaveAddr, uint8_t data);

extern void lcdSendChar (I2C_TypeDef *i2c,
                         uint8_t slaveAddr,
                         uint8_t line,        // Zeile
                         uint8_t col,         // Spalte
                         char    text         // Zeichen
                        );

extern void lcdSendString (I2C_TypeDef *i2c,
                           uint8_t slaveAddr,
                           uint8_t line,        // Zeile
                           uint8_t col,         // Spalte
                           char *text,          // Pointer auf den Text
                           uint8_t dataLen);    // Laenge des Textes

#endif /* LCD_H_ */
