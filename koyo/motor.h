#ifndef MOTOR_H_INCLUDED
#define MOTOR_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define LDIR 10
#define LPWM 3
#define RPWM 6
#define RDIR 5

void motor_init();
void motor_control(int L,int R);
void motor_control2(int L,int R);
#endif

