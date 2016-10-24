#include <stdlib.h>
#include <wiringPi.h>
#include <softPwm.h>

#include "motor.hpp"

#define L298N_IN1 0
#define L298N_IN2 1
#define L298N_IN3 22
#define L298N_IN4 26

void motor_init()
{
	setenv("WIRINGPI_GPIOMEM", "1", 1);

	wiringPiSetup() ;

	pinMode(L298N_IN4, OUTPUT); //GPIO12 -> L298N IN4
	pinMode(L298N_IN3, OUTPUT); //GPIO6  -> L298N IN3
	pinMode(L298N_IN2, OUTPUT);  //GPIO18 -> L298N IN2
	pinMode(L298N_IN1, OUTPUT);  //GPIO17 -> L298N IN1

	softPwmCreate(L298N_IN2, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
	softPwmCreate(L298N_IN4, MOTOR_PWM_MIN, MOTOR_PWM_MAX);

	//digitalWrite(L298N_IN4, LOW);
	digitalWrite(L298N_IN3, LOW);
	//digitalWrite(L298N_IN2, LOW);
	digitalWrite(L298N_IN1, LOW);
}

void test_motor()
{
	digitalWrite(L298N_IN1, LOW);
	//digitalWrite(L298N_IN2, HIGH);
	softPwmWrite(L298N_IN2, 100);

	digitalWrite(L298N_IN3, LOW);
	//digitalWrite(L298N_IN4, HIGH);
	softPwmWrite(L298N_IN4, 100);

}

void set_motor_pwm(int8_t left_pwm, int8_t right_pwm)
{
	softPwmWrite(L298N_IN4, right_pwm);
	softPwmWrite(L298N_IN2, left_pwm);
}

void halt_motor()
{
	set_motor_pwm(0, 0);
}
