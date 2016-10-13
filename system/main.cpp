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
#include "controller.hpp"

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

	/* Setup Raspicam */
	raspicam::RaspiCam_Cv camera;
	camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
	camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	camera.set(CV_CAP_PROP_BRIGHTNESS, 50);
	camera.set(CV_CAP_PROP_CONTRAST, 50);
	camera.set(CV_CAP_PROP_SATURATION, 50);
	camera.set(CV_CAP_PROP_GAIN, 50);
	camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
	camera.set(CV_CAP_PROP_EXPOSURE, -1);
	camera.set(CV_CAP_PROP_WHITE_BALANCE_RED_V, 1);
	camera.set(CV_CAP_PROP_WHITE_BALANCE_BLUE_U, 1);

	if(!camera.open()) {
		ROS_INFO("failed to open pi camera!\n");
		return 0;
	}

        cv::Mat frame;

	lane_detector = new LaneDetector();

	motor_init();

	while(1) {
		camera.grab();
		camera.retrieve(frame);

		cv::Mat camera_matrix = (cv::Mat1d(3, 3) << 279.087996, 0.000000, 329.895256,
							    0.000000, 278.852468, 189.532203,
							    0.000000, 0.000000, 1.000000);

		cv::Mat distort_coffecient = (cv::Mat1d(1, 5) <<
						-0.275519, 0.051598, 0.003164, -0.000453, 0.000000);

		cv::Mat H = (cv::Mat1d(3, 3) << 4.015384e-05, -0.0002008101, -0.1583213,
						0.0008264009, 2.63818e-05, -0.2518232,
						5.908231e-05, -0.007253319, 1);

		cv::Mat distort_image;
		cv::undistort(frame, distort_image, camera_matrix, distort_coffecient);

		sensor_msgs::ImagePtr raw_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
		sensor_msgs::ImagePtr distort_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", distort_image).toImageMsg();

		raw_image_publisher.publish(raw_img_msg);
		distort_image_publisher.publish(distort_img_msg);

		lane_detector->lane_detect(distort_image);
		lane_detector->publish_images();

		//cv::imshow("pi camera", distort_image);

		handle_joystick();
		ros::spinOnce();
	}

	return 0;
}
