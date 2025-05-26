#include "control.h"

void setupPWM(void) {
    // Configurar RC2 como salida PWM para CCP1
    TRISCbits.TRISC2 = 0;
    CCP1CON = 0b00001100;
    CCPR1L = 0;

    // Configurar RC1 como salida PWM para CCP2
    TRISCbits.TRISC1 = 0;
    CCP2CON = 0b00001100;
    CCPR2L = 0;

    // Timer2 para PWM
    PR2 = 60;
    T2CON = 0b00000111; // Prescaler 1:16, Timer2 ON
}

void setDuty1(unsigned char val) {
    CCPR1L = (val == 0) ? 1 : val;
}

void setDuty2(unsigned char val) {
    CCPR2L = (val == 0) ? 1 : val;
}

void forward(unsigned char val) {
    IN1 = 1; IN2 = 0;
    IN3 = 0; IN4 = 1;
    setDuty1(val);
    setDuty2(val);
}

void reverseMotor1(unsigned char val) {
    IN1 = 0; IN2 = 1;  // Invertir Motor 1
    IN3 = 0; IN4 = 1;  // Motor 2 hacia adelante
    setDuty1(val);
    setDuty2(val);
}

void turnRight(unsigned char val) {
    IN1 = 1; IN2 = 0;
    IN3 = 0; IN4 = 0;
    setDuty1(val);
    setDuty2(0);
}

void turnLeft(unsigned char val) {
    IN1 = 0; IN2 = 0;
    IN3 = 0; IN4 = 1;
    setDuty1(0);
    setDuty2(val);
}

void stopMotors(void) {
    IN1 = 0; IN2 = 0;
    IN3 = 0; IN4 = 0;
    setDuty1(0);
    setDuty2(0);
}
