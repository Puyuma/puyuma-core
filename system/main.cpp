#include <sstream>
#include <string>
#include <iostream>

#include <highgui.h>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <raspicam/raspicam_cv.h>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>

#include <xenobot/threshold_setting.h>
#include <xenobot/wheel_command.h>
#include "lane_detector.hpp"
#include "motor.hpp"
#include "controller.hpp"
#include "camera.hpp"

using namespace cv;

LaneDetector* lane_detector;

//Joystick
bool joystick_triggered;
ros::Time joystick_trigger_time;

void handle_joystick()
{
	if(joystick_triggered == true) {
		if((ros::Time::now().toSec() - joystick_trigger_time.toSec()) > 0.2) {
			joystick_triggered = false;
			set_motor_pwm(0, 0);
		}
	}
}

void threshold_setting_callback(const xenobot::threshold_setting& threshold_setting_msg)
{
	lane_detector->set_hsv(
		threshold_setting_msg.outer_h_max,
		threshold_setting_msg.outer_h_min,
		threshold_setting_msg.outer_s_max,
		threshold_setting_msg.outer_s_min,
		threshold_setting_msg.outer_v_max,
		threshold_setting_msg.outer_v_min,
		threshold_setting_msg.inner_h_max,
		threshold_setting_msg.inner_h_min,
		threshold_setting_msg.inner_s_max,
		threshold_setting_msg.inner_s_min,
		threshold_setting_msg.inner_v_max,
		threshold_setting_msg.inner_v_min
	);
}

void wheel_command_callback(const xenobot::wheel_command& wheel_msg)
{
	set_motor_pwm(wheel_msg.left_pwm, wheel_msg.right_pwm);

	joystick_triggered = true;
	ros::Time joystick_trigger_time = ros::Time::now();
}

int main(int argc, char* argv[])
{
	/* ROS initialization */
	ros::init(argc, argv, "xenobot");
        ros::Time::init();
	ros::NodeHandle nh;
        ros::Rate loop_rate(30);

	ros::NodeHandle node;

	ros::Publisher raw_image_publisher = 
		node.advertise<sensor_msgs::Image>("xenobot/raw_image", 1000);
	ros::Publisher distort_image_publisher = 
		node.advertise<sensor_msgs::Image>("xenobot/distort_image", 1000);

	ros::Subscriber threshold_setting_subscriber =
		node.subscribe("/xenobot/calibration/threshold_setting", 1000, threshold_setting_callback);
	ros::Subscriber wheel_command_subscriber =
		node.subscribe("/xenobot/wheel_command", 1, wheel_command_callback);

	/* Read ROS parameters */
	string machine_name;
	if(nh.getParam("machine_name", machine_name) == false) {
		ROS_INFO("Abort: no machine name assigned!");
		ROS_INFO("you may try:");
		ROS_INFO("roslaunch xenobot activate_controller veh:=machine_name");
		return 0;
	}

	string yaml_path;
	if(nh.getParam("config_path", yaml_path) == false) {
		ROS_INFO("Abort: no configuration path assigned!");
		ROS_INFO("Instead of doing rosrun command, you should try roslaunch command");
		return 0;
	}

	bool calibrate_mode;
	bool received_param = nh.getParam("calibrate", calibrate_mode);

	if(calibrate_mode == true || received_param == true) {
		ROS_INFO("Calibration mode is enabled");
		lane_detector = new LaneDetector(yaml_path + machine_name + "/", true);
	} else {
		lane_detector = new LaneDetector(yaml_path + machine_name + "/", false);
	}

	string test = yaml_path + machine_name + "/";
	ROS_INFO("%s", test.c_str());	

	//Load extrinsic calibration data and color thresholding setting
	if(lane_detector->load_yaml_setting() == false) {
		return 0;
	}

	//Load intrinsic calibration data
	cv::Mat camera_matrix, distort_coffecient;

	if(load_intrinsic_calibration(yaml_path + machine_name + "/",
		camera_matrix, distort_coffecient) == false) {
		return 0;
	}

	//Load PID parameters
	if(load_pid_param(yaml_path + machine_name + "/") == false) {
		ROS_INFO("PID parameter is not exist, load the default setting!");
	}

	/* Setup Raspicam */
	raspicam::RaspiCam_Cv camera;

	if(!camera_setup(camera)) {
		ROS_INFO("Abort: failed to open pi camera!");
		return 0;
	}

        cv::Mat frame;

	/* Motor initialization */
	motor_init();

	while(1) {
		camera.grab();
		camera.retrieve(frame);

		cv::Mat distort_image;

		cv::undistort(frame, distort_image, camera_matrix, distort_coffecient);

		sensor_msgs::ImagePtr raw_img_msg =
			cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

		sensor_msgs::ImagePtr distort_img_msg =
			cv_bridge::CvImage(std_msgs::Header(), "bgr8", distort_image).toImageMsg();

		raw_image_publisher.publish(raw_img_msg);
		distort_image_publisher.publish(distort_img_msg);

		/* Lane estimation */
		float d = 0, phi = 0;
		vector<Vec4f> outer_lines;
		vector<Vec4f> inner_lines;
		Vec4f predicted_lane;
		lane_detector->lane_detect(distort_image, outer_lines, inner_lines, predicted_lane);
		bool get_pose = lane_detector->pose_estimate(predicted_lane, d, phi);
		lane_detector->publish_images();

		/* PID controller */
		if(get_pose == true) {
			self_driving_controller(d, phi);
		} else {
			halt_motor();	
		}

		//handle_joystick();
		//ros::spinOnce();
	}

	return 0;
}
