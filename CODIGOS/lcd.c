#include "lcd.h"

void LCD_Enable(void) {
    EN = 1;
    __delay_us(5);
    EN = 0;
    __delay_us(100);
}

void LCD_Command(unsigned char cmd) {
    RS = 0;
    D4 = (cmd >> 4) & 1;
    D5 = (cmd >> 5) & 1;
    D6 = (cmd >> 6) & 1;
    D7 = (cmd >> 7) & 1;
    LCD_Enable();

    D4 = (cmd >> 0) & 1;
    D5 = (cmd >> 1) & 1;
    D6 = (cmd >> 2) & 1;
    D7 = (cmd >> 3) & 1;
    LCD_Enable();

    if (cmd == 0x01 || cmd == 0x02)
        __delay_ms(2);
}

void LCD_Char(unsigned char data) {
    RS = 1;
    D4 = (data >> 4) & 1;
    D5 = (data >> 5) & 1;
    D6 = (data >> 6) & 1;
    D7 = (data >> 7) & 1;
    LCD_Enable();

    D4 = (data >> 0) & 1;
    D5 = (data >> 1) & 1;
    D6 = (data >> 2) & 1;
    D7 = (data >> 3) & 1;
    LCD_Enable();
}

void LCD_Clear(void) {
    LCD_Command(0x01);
    __delay_ms(2);
}

void LCD_Init(void) {
    TRISC = 0x00;
    TRISD = 0x00;
    __delay_ms(20);

    RS = 0;
    D4 = 0; D5 = 0; D6 = 1; D7 = 0;  // Comando 0x30 >> 4
    LCD_Enable();
    __delay_ms(5);

    LCD_Enable();
    __delay_us(150);

    LCD_Enable();

    D4 = 0; D5 = 1; D6 = 0; D7 = 0;  // Comando 0x20 >> 4 (modo 4 bits)
    LCD_Enable();

    LCD_Command(0x28);  // LCD 4 bits, 2 líneas, 5x8
    LCD_Command(0x0C);  // Display ON, cursor OFF
    LCD_Command(0x06);  // Incremento automático del cursor
    LCD_Clear();
}

void LCD_String(const char *str) {
    while (*str) {
        LCD_Char(*str++);
    }
}

void LCD_SetCursor(unsigned char row, unsigned char col) {
    if (row == 1)
        LCD_Command(0x80 + col);
    else
        LCD_Command(0xC0 + col);
}

void LCD_CREATECHAR(unsigned char location, unsigned char *charMap) {
    location &= 0x07;
    LCD_Command(0x40 + (location * 8));
    for (int i = 0; i < 8; i++) {
        LCD_Char(charMap[i]);
    }
}
