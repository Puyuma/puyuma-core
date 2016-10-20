#ifndef MOTOR_H
#define MOTORR_H

#define MOTOR_PWM_MAX 1000
#define MOTOR_PWM_MIN 0

#include <sys/types.h>

using namespace std;

void motor_init();
void test_motor();
void set_motor_pwm(int8_t left_pwm, int8_t right_pwm);

#endif
