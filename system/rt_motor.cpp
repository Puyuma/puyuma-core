#include <stdlib.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <yaml-cpp/yaml.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>

#include "KsoftPWM/pwm_user.h"
#include "KsoftPWM/xenopwm.h"

#include "motor.hpp"
#include "controller.hpp"

#define L298N_IN1 0
#define L298N_IN2 1
#define L298N_IN3 4
#define L298N_IN4 26

int motor_pwm_bias;
int filp;

bool read_motor_calibration(string _yaml_path)
{
	_yaml_path = _yaml_path + "motor.yaml";
	try {
		YAML::Node yaml = YAML::LoadFile(_yaml_path);

		motor_pwm_bias = (int)(yaml["motor_bias"].as<float>() * MOTOR_PWM_MAX);
		ROS_INFO("Motor bias: %d", motor_pwm_bias);
	} catch(...) {
		motor_pwm_bias = 0;
		ROS_INFO("Failed to load %s", _yaml_path.c_str());
		return false;
	}

	return true;
}

void motor_init()
{
	filp = open("/dev/rtdm/xenoPWM", 0);
	if (filp < 0) {
		printf("failed to open device xenoPWM : %s\n", strerror(errno));
	}

	pinMode(filp, L298N_IN4, OUTPUT);
	pinMode(filp, L298N_IN3, OUTPUT);
	pinMode(filp, L298N_IN2, OUTPUT);
	pinMode(filp, L298N_IN1, OUTPUT);

	softPwmCreate(filp, L298N_IN2, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
	softPwmCreate(filp, L298N_IN4, MOTOR_PWM_MIN, MOTOR_PWM_MAX);

	digitalWrite(filp, L298N_IN3, LOW);
	digitalWrite(filp, L298N_IN1, LOW);
}

void test_motor()
{
	digitalWrite(filp, L298N_IN1, LOW);
	softPwmWrite(filp, L298N_IN2, 100);

	digitalWrite(filp, L298N_IN3, LOW);
	softPwmWrite(filp, L298N_IN4, 100);

}

void set_motor_pwm(int8_t left_pwm, int8_t right_pwm)
{
	softPwmWrite(filp, L298N_IN4, right_pwm + motor_pwm_bias);
	softPwmWrite(filp, L298N_IN2, left_pwm - motor_pwm_bias);
}

void halt_motor()
{
	set_motor_pwm(0, 0);

	pid_halt();
}

void forward_motor(int right, int left)
{
	set_motor_pwm((int8_t) right, (int8_t) left);
}
