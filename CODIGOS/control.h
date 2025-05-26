#ifndef CONTROL_H
#define CONTROL_H

#include <xc.h>

// Pines de control de motores (L298N)
#define IN1 LATDbits.LATD0
#define IN2 LATDbits.LATD1
#define IN3 LATDbits.LATD2
#define IN4 LATDbits.LATD3

void setupPWM(void);
void setDuty1(unsigned char val);
void setDuty2(unsigned char val);
void forward(unsigned char val);
void turnRight(unsigned char val);
void turnLeft(unsigned char val);
void stopMotors(void);

#endif
