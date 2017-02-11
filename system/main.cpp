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
#include "apriltags_detector.hpp"

using namespace cv;

LaneDetector* lane_detector;
cv::Mat camera_matrix, distort_coffecient;


//Joystick
bool joystick_triggered;
ros::Time joystick_trigger_time;

//JOYSTICK_MODE, SELF_DRIVING_MODE, STOP_MODE
int mode = SELF_DRIVING_MODE; //JOYSTICK_MODE
bool calibrate_mode = false;

ros::Publisher raw_image_publisher;
ros::Publisher distort_image_publisher;
ros::Subscriber threshold_setting_subscriber;
ros::Subscriber wheel_command_subscriber;
ros::ServiceServer save_yaml_srv;

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
		threshold_setting_msg.color,
		threshold_setting_msg.h_min,
		threshold_setting_msg.h_max,
		threshold_setting_msg.s_min,
		threshold_setting_msg.s_max,
		threshold_setting_msg.v_min,
		threshold_setting_msg.v_max
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

bool save_yaml_parameter(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &res)
{
	bool result = lane_detector->save_thresholding_yaml();
	if(result == true)
	{
		res.success = true;
		res.message = "Yaml_parameter saved";
		return true;
	}
	else{
		res.success = false;
		res.message = "Fail to save yaml_parameter";
		return false;
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


	if(calibrate_mode == true) {
		ROS_INFO("Calibration mode is enabled");
		lane_detector = new LaneDetector(yaml_path + machine_name + "/", true);
	} else {
		lane_detector = new LaneDetector(yaml_path + machine_name + "/", false);
	}

	string test = yaml_path + machine_name + "/";

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

		if(mode == STOP_MODE){
			halt_motor();
			std::this_thread::yield();
			continue;
		}

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
			forward_motor();
			//halt_motor();
		}

		std::this_thread::yield();
	}

}

void apriltags_detector_handler()
{
    ApriltagsDetector detector;
    int apriltags_id;

    cv::Mat image;
    cv::Mat image_gray;

    while(1)
    {

        raw_image_queue.front(image);
        apriltags_id = detector.processImage(image, image_gray);

        switch(apriltags_id)
        {
            case 0:
                mode = STOP_MODE;
                break;

            default:
                mode = SELF_DRIVING_MODE;
                break;
        }

        cout << "Motor mode: " << mode << "\n";



        std::this_thread::yield();
    }
    cv::waitKey(1);
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

	ros::NodeHandle node("xenobot");
	ros::NodeHandle nh;
	if(!nh.getParam("/calibrate", calibrate_mode)) 
		ROS_INFO("Fail to get calibration mode,use \"true\" instead of \"1\".");

	if(calibrate_mode){
		raw_image_publisher = 
			node.advertise<sensor_msgs::Image>("raw_image", 10);
		distort_image_publisher = 
			node.advertise<sensor_msgs::Image>("distort_image", 10);
		threshold_setting_subscriber =
			node.subscribe("calibration/threshold_setting", 10, threshold_setting_callback);
		save_yaml_srv = 
			node.advertiseService("/xenobot/save_yaml_parameter", save_yaml_parameter);
	}

	wheel_command_subscriber =
		node.subscribe("wheel_command", 1, wheel_command_callback);


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
	thread apriltags_detector_thread(apriltags_detector_handler);

	pause();

	return 0;
}

__attribute__((destructor))void end()
{
	halt_motor();
}
