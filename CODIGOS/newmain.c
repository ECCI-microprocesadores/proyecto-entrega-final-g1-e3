#pragma config FOSC = INTIO67  
#pragma config PLLCFG = ON
#pragma config WDTEN = OFF
#pragma config BOREN = SBORDIS

#include <xc.h>
#include "control.h"
#include <string.h>
#include "lcd.h"

#define _XTAL_FREQ 64000000UL

// Pines de sensores
#define LS PORTBbits.RB0  // Sensor izquierdo
#define RS PORTBbits.RB1  // Sensor derecho

unsigned char duty = 37;
signed int read_time = 100;


void main(void) {
    // Configurar oscilador interno a 64 MHz
    OSCCON = 0b01110000;
    OSCTUNEbits.PLLEN = 1;
    LCD_Init();
    __delay_ms(2000);
    LCD_Clear();

    

  

    // Configurar pines de motores como salida
    TRISD &= ~(0x0F);  // RD0-RD3
    LATD &= ~(0x0F);

    // Configurar sensores como entrada digital
    TRISBbits.TRISB0 = 1;  // RB0
    TRISBbits.TRISB1 = 1;  // RB1
    ANSELBbits.ANSB0 = 0;
    ANSELBbits.ANSB1 = 0;

    setupPWM();
    
 char estadoAnterior[10] = "";

  
    while (1) {


        // Leer sensores múltiples veces y tomar mayoría (filtro sencillo)
        unsigned char i;
        unsigned char countL = 0;
        unsigned char countR = 0;

        for (i = 0; i < 5; i++) {
            if (LS) countL++;
            if (RS) countR++;
            __delay_us(read_time); // Pequeño retardo entre lecturas
        }

        unsigned char leftValue = (countL >= 3) ? 1 : 0;
        unsigned char rightValue = (countR >= 3) ? 1 : 0;

        // Decisión basada en sensores con filtro
        if (leftValue == 1 && rightValue == 0) {
            turnLeft(duty);
            if (strcmp(estadoAnterior, "Izquierda") != 0) {
                LCD_Clear();
                LCD_SetCursor(1, 0);
                LCD_String("Izquierda");
                strcpy(estadoAnterior, "Izquierda");
            }
        } else if (leftValue == 0 && rightValue == 1) {
            turnRight(duty);
            if (strcmp(estadoAnterior, "Derecha") != 0) {
                LCD_Clear();
                LCD_SetCursor(1, 0);
                LCD_String("Derecha");
                strcpy(estadoAnterior, "Derecha");
            }
        } else if (leftValue == 0 && rightValue == 0) {
            forward(duty);
            if (strcmp(estadoAnterior, "Recto") != 0) {
                LCD_Clear();
                LCD_SetCursor(1, 0);
                LCD_String("Recto");
                
            }
        } else {
            stopMotors();
            if (strcmp(estadoAnterior, "No linea") != 0) {
                LCD_Clear();
                LCD_SetCursor(1, 0);
                LCD_String("No linea");
                strcpy(estadoAnterior, "No linea");
            }
        }

        __delay_ms(1); // Pequeño delay para evitar ruido o rebotes
    }

}
