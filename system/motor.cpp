#include <stdlib.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <yaml-cpp/yaml.h>

#include "motor.hpp"
#include "controller.hpp"

#define L298N_IN1 0
#define L298N_IN2 1
#define L298N_IN3 4
#define L298N_IN4 26

int motor_pwm_bias;

bool read_motor_calibration(string _yaml_path)
{
	try {
		YAML::Node yaml = YAML::LoadFile(_yaml_path + "motor.yaml");
		ROS_INFO("%s", _yaml_path.c_str());

		motor_pwm_bias = (int)(yaml["motor_bias"].as<float>() * MOTOR_PWM_MAX);
		ROS_INFO("Motor bias: %d", motor_pwm_bias);
	} catch(...) {
		motor_pwm_bias = 0;

		return false;
	}

	return true;
}

void motor_init()
{
	setenv("WIRINGPI_GPIOMEM", "1", 1);

	wiringPiSetup() ;

	pinMode(L298N_IN4, OUTPUT);
	pinMode(L298N_IN3, OUTPUT);
	pinMode(L298N_IN2, OUTPUT);
	pinMode(L298N_IN1, OUTPUT);

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
	softPwmWrite(L298N_IN4, right_pwm + motor_pwm_bias);
	softPwmWrite(L298N_IN2, left_pwm - motor_pwm_bias);
}

void halt_motor()
{
	set_motor_pwm(0, 0);

	pid_halt();
}
