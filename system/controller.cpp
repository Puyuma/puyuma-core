#include <string>

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "controller.hpp"
#include "motor.hpp"

using namespace std;

float kp_d, ki_d, kd_d;
float kp_phi, ki_phi, kd_phi;

bool load_pid_param(string _yaml_path)
{
	try {
		_yaml_path = _yaml_path + "pid.yaml";

		ROS_INFO("%s", _yaml_path.c_str());

		YAML::Node yaml = YAML::LoadFile(_yaml_path);

		kp_d = yaml["pid_d"]["kp"].as<float>();
		ki_d = yaml["pid_d"]["ki"].as<float>();
		kd_d = yaml["pid_d"]["kd"].as<float>();
		kp_phi = yaml["pid_phi"]["kp"].as<float>();
		ki_phi = yaml["pid_phi"]["ki"].as<float>();
		kd_phi = yaml["pid_phi"]["kd"].as<float>();

		ROS_INFO("PID d controller:");
		ROS_INFO("[Kp:%f, Ki:%f, Kd:%f]\n", kp_d, ki_d, kd_phi);

		ROS_INFO("PID phi controller:");
		ROS_INFO("[Kp:%f, Ki:%f, Kd:%f]\n", kp_phi, ki_phi, kd_phi);
	} catch(...) {
		//TODO: Load default

		kp_d = 0;
		ki_d = 0;
		kd_d = 0;
		kp_phi = 0;
		ki_phi = 0;
		kd_phi = 0;

		return false;
	}

	return true;
}

static void bound(int min, int max, int x)
{
	if(x < min) {
		x = min;
	} else if(x > max) {
		x = max;
	}
}

static float pid_d_control(int d)
{
	return kp_d * d;
}

static float pid_phi_control(int phi)
{
	return kp_phi * phi;
}

void self_driving_controller(int d, int phi)
{
	int pwm_left, pwm_right;

	int pwm = pid_d_control(d) + pid_phi_control(phi);

	pwm_left = THROTTLE_BASE + pwm;
	pwm_right = THROTTLE_BASE - pwm;

	bound(MOTOR_PWM_MIN, MOTOR_PWM_MAX, pwm_left);
	bound(MOTOR_PWM_MIN, MOTOR_PWM_MAX, pwm_right);

	set_motor_pwm(pwm_left, pwm_right);
}
