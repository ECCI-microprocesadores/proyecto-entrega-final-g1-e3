#ifndef LCD_H
#define LCD_H

#include <xc.h>
#define _XTAL_FREQ 64000000  // Frecuencia del oscilador interno

// Definición de pines del LCD
#define RS LATDbits.LATD5
#define EN LATDbits.LATD6
#define D4 LATCbits.LATC4
#define D5 LATCbits.LATC5
#define D6 LATCbits.LATC6
#define D7 LATCbits.LATC7

// Prototipos de funciones
void LCD_Init(void);
void LCD_Command(unsigned char cmd);
void LCD_Char(unsigned char data);
void LCD_String(const char *str);
void LCD_SetCursor(unsigned char row, unsigned char col);
void LCD_Clear(void);
void LCD_CREATECHAR(unsigned char location, unsigned char *charMap);
void LCD_Enable(void);

#endif  // LCD_H

