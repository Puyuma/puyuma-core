/**
 * Access RTDM device for software PWM from user space.
 * Copyright (c) 2017 Shao-Hua Wang.
 */

#define LOW 0
#define HIGH 1
#define INPUT 0
#define OUTPUT 1

void pinMode(int filp, int pin, int status);
void digitalWrite(int filp, int pin, int status);
void softPwmCreate(int filp, int pin, int min, int max);
void softPwmWrite(int filp, int pin, int duty_cycle);
