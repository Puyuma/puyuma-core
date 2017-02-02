#include <sstream>
#include <string>
#include <iostream>
#include <thread>

#include <highgui.h>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <raspicam/raspicam_cv.h>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include "queue.hpp"

#include <xenobot/threshold_setting.h>
#include <xenobot/wheel_command.h>
#include "lane_detector.hpp"
#include "motor.hpp"
#include "controller.hpp"
#include "camera.hpp"

using namespace cv;

LaneDetector* lane_detector;
cv::Mat camera_matrix, distort_coffecient;


//Joystick
bool joystick_triggered;
ros::Time joystick_trigger_time;

//JOYSTICK_MODE, SELF_DRIVING_MODE, STOP_MODE
int mode = SELF_DRIVING_MODE; //JOYSTICK_MODE

ros::Publisher raw_image_publisher;
ros::Publisher distort_image_publisher;
ros::Subscriber threshold_setting_subscriber;
ros::Subscriber wheel_command_subscriber;

/* Camera */
raspicam::RaspiCam_Cv camera;
Queue<cv::Mat> raw_image_queue;

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
	if(mode == JOYSTICK_MODE) {
		set_motor_pwm(wheel_msg.left_pwm, wheel_msg.right_pwm);

		joystick_triggered = true;
		ros::Time joystick_trigger_time = ros::Time::now();
	}
}

void load_yaml_parameter()
{
	ros::NodeHandle nh;

	/* Read ROS parameters */
	string machine_name;
	if(nh.getParam("machine_name", machine_name) == false) {
		ROS_INFO("Abort: no machine name assigned!");
		ROS_INFO("you may try:");
		ROS_INFO("roslaunch xenobot activate_controller veh:=machine_name");
		return;
	}

	string yaml_path;
	if(nh.getParam("config_path", yaml_path) == false) {
		ROS_INFO("Abort: no configuration path assigned!");
		ROS_INFO("Instead of doing rosrun command, you should try roslaunch command");
		return;
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
		return;
	}

	//Load intrinsic calibration data
	if(load_intrinsic_calibration(yaml_path + machine_name + "/",
		camera_matrix, distort_coffecient) == false) {
		return;
	}

	//Load motor calibration data
	if(read_motor_calibration(yaml_path + machine_name + "/") == false) {
		ROS_INFO("Can't find motor calibration data, load default.");
	}

	//Load PID parameters
	if(load_pid_param(yaml_path + machine_name + "/") == false) {
		ROS_INFO("PID parameter is not exist, load the default setting!");
	}

}

bool save_yaml_parameter(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &res)
{
	bool result = lane_detector->save_thresholding_yaml();
	if(result == true)
	{
		res.success = true;
		res.message = "Yaml_parameter saved";
		ROS_INFO("Yaml_parameter saved");
		return true;
	}
	else{
		res.success = false;
		res.message = "Fail to save yaml_parameter";
		ROS_INFO("Fail to save yaml_parameter");
		return false;
	}
}

void camera_thread_handler()
{
	cv::Mat frame;

	while(1) {
		camera.grab();
		camera.retrieve(frame);

		raw_image_queue.push(frame);

		std::this_thread::yield();
	}
}

void self_driving_thread_handler()
{
	cv::Mat frame;

	while(1) {
		raw_image_queue.pop(frame);

		cv::Mat distort_image;

		cv::undistort(frame, distort_image, camera_matrix, distort_coffecient);

#if 0
		sensor_msgs::ImagePtr raw_img_msg =
			cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

		sensor_msgs::ImagePtr distort_img_msg =
			cv_bridge::CvImage(std_msgs::Header(), "bgr8", distort_image).toImageMsg();

		raw_image_publisher.publish(raw_img_msg);
		distort_image_publisher.publish(distort_img_msg);
#endif

		/* Lane estimation */
		float d = 0, phi = 0;
		bool get_pose = lane_detector->lane_estimate(distort_image, d, phi);

		/* Joystick mode */
		if(mode == JOYSTICK_MODE) {
			handle_joystick();
			continue;
		}

#if 0 //Tune motor bias
		set_motor_pwm(70, 70);	
		continue;
#endif

		/* PID controller */
		if(get_pose == true) {
			self_driving_controller(d, phi);
		} else {
			halt_motor();
		}

		std::this_thread::yield();
	}

}

void ros_spin_thread_handler()
{
	while(1) {
		ros::spinOnce();

		std::this_thread::yield();
	}
}

int main(int argc, char* argv[])
{
	/* ROS initialization */
	ros::init(argc, argv, "xenobot");
        ros::Time::init();
        ros::Rate loop_rate(30);

	ros::NodeHandle node;

	raw_image_publisher = 
		node.advertise<sensor_msgs::Image>("xenobot/raw_image", 10);
	distort_image_publisher = 
		node.advertise<sensor_msgs::Image>("xenobot/distort_image", 10);
	threshold_setting_subscriber =
		node.subscribe("/xenobot/calibration/threshold_setting", 10, threshold_setting_callback);
	wheel_command_subscriber =
		node.subscribe("/xenobot/wheel_command", 1, wheel_command_callback);

	ros::ServiceServer save_yaml_srv = node.advertiseService("save_yaml_parameter", save_yaml_parameter);


	load_yaml_parameter();

	/* Motor initialization */
	motor_init();

	/* Camera initialization */
	if(!camera_setup(camera)) {
		ROS_INFO("Abort: failed to open pi camera!");
		return 0;
	}

	/* Threads */
	thread self_driving_thread(self_driving_thread_handler);
	thread camera_thread(camera_thread_handler);
	thread ros_spin_thread(ros_spin_thread_handler);

	pause();

	return 0;
}

__attribute__((destructor))void end()
{
	halt_motor();
}
