#include <string>

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "controller.hpp"
#include "lane_detector.hpp"
#include "motor.hpp"

using namespace std;

pid_control_t pid_d;
pid_control_t pid_phi;

bool load_pid_param(string _yaml_path)
{
	try {
		_yaml_path = _yaml_path + "pid.yaml";

		ROS_INFO("%s", _yaml_path.c_str());

		YAML::Node yaml = YAML::LoadFile(_yaml_path);

		pid_d.kp = yaml["pid_d"]["kp"].as<float>();
		pid_d.ki = yaml["pid_d"]["ki"].as<float>();
		pid_d.kd = yaml["pid_d"]["kd"].as<float>();
		pid_phi.kp = yaml["pid_phi"]["kp"].as<float>();
		pid_phi.ki = yaml["pid_phi"]["ki"].as<float>();
		pid_phi.kd = yaml["pid_phi"]["kd"].as<float>();

		ROS_INFO("PID d controller:");
		ROS_INFO("[Kp:%f, Ki:%f, Kd:%f]\n", pid_d.kp, pid_d.ki, pid_d.kd);

		ROS_INFO("PID phi controller:");
		ROS_INFO("[Kp:%f, Ki:%f, Kd:%f]\n", pid_phi.kp, pid_phi.ki, pid_phi.kd);
	} catch(...) {
		//TODO: Load default

		pid_d.kp = 0;
		pid_d.ki = 0;
		pid_d.kd = 0;
		pid_phi.kp = 0;
		pid_phi.ki = 0;
		pid_phi.kd = 0;

		return false;
	}

	return true;
}

void bound(int min, int max, int& x)
{
	if(x < min) {
		x = min;
	} else if(x > max) {
		x = max;
	}
}

void bound(float min, float max, float& x)
{
	if(x < min) {
		x = min;
	} else if(x > max) {
		x = max;
	}
}

static float pid_d_control(int d_current, float d_setpoint, pid_control_t& pid)
{
	float p_term, d_term;
	float current_error = d_current - d_setpoint;

	ros::Time current_time = ros::Time::now();

	//Calculate the P term
	p_term = pid.kp * current_error;

	if(pid.been_init == false) {
		pid.been_init = true;
		d_term = 0; //No D term
	} else if(pid.been_halted == true) {
		pid.been_halted = false;
		d_term = 0; //No D term
	} else {
		//Calculate the time interval
		float delta_t = current_time.toSec() - pid.previous_time.toSec();

		//Calculate the D term
		d_term = pid.kd * (current_error - pid.previous_error) / delta_t;
	}

	pid.previous_error = current_error;
	pid.previous_time = current_time;

	return p_term + d_term;
}

static float pid_phi_control(float phi_current, float phi_setpoint, pid_control_t& pid)
{
	float p_term, d_term;
	float current_error = phi_current - phi_setpoint;

	ros::Time current_time = ros::Time::now();

	//Calculate the P term
	p_term = pid.kp * current_error;

	if(pid.been_init == false) {
		pid.been_init = true;
		d_term = 0; //No D term
	} else if(pid.been_halted == true) {
		pid.been_halted = false;
		d_term = 0; //No D term
	} else {
		//Calculate the time interval
		float delta_t = current_time.toSec() - pid.previous_time.toSec();

		//Calculate the D term
		d_term = pid.kd * (current_error - pid.previous_error) / delta_t;
	}

	pid.previous_error = current_error;
	pid.previous_time = current_time;

	return p_term + d_term;

}

void pid_halt()
{
	pid_d.been_halted = true;
	pid_phi.been_halted = true;
}

void self_driving_controller(float d, float phi)
{
	int pwm_left, pwm_right;

	float phi_setpoint = pid_d_control(d, 0, pid_d);

	bound(PHI_MIN, PHI_MAX, phi_setpoint);

	int pwm = (int)pid_phi_control(phi, phi_setpoint, pid_phi);

	pwm_left = THROTTLE_BASE + pwm;
	pwm_right = THROTTLE_BASE - pwm;

	bound(MOTOR_PWM_MIN, MOTOR_PWM_MAX, pwm_left);
	bound(MOTOR_PWM_MIN, MOTOR_PWM_MAX, pwm_right);

	set_motor_pwm(pwm_left, pwm_right);
}
