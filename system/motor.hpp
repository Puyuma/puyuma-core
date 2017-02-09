#ifndef MOTOR_H
#define MOTORR_H

#define MOTOR_PWM_MAX 100
#define MOTOR_PWM_MIN 0

#define MOTOR_BIAS_RATIO 0.01

#include <sys/types.h>

using namespace std;

void motor_init();
void test_motor();
void set_motor_pwm(int8_t left_pwm, int8_t right_pwm);
void halt_motor();
bool read_motor_calibration(string _yaml_path);
void forward_motor();

#endif
