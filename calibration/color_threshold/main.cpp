#include <sstream>
#include <string>
#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include <ros/console.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <hsv_threshold.h>
#include "setting.h"

//ros msg
#include <xenobot/threshold_setting.h>
#include <std_srvs/Trigger.h>
#include <xenobot/SendHsv.h>

using namespace cv;

int save = 0;
ros::Publisher threshold_setting_pub;
ros::ServiceClient save_file_srv;

cv::Mat marked_image;

void button_cb(int ,void*);
void on_trackbar(int, void *);

//class for Color Calibration
class ColorCalibration {

public:
ros::Subscriber threshold_image_sub;
HsvThreshold hsv;
cv::Mat threshold_image;
char color;
std::string title;


void create_trackbars()
{
	//create window for trackbars
	std::string window_name = title+"_threshold_setting";
	namedWindow(window_name, WINDOW_AUTOSIZE);

	//create trackbars and insert them into window
	createTrackbar("H_MIN",window_name,&(hsv.h_min),180,on_trackbar,this);
	createTrackbar("H_MAX",window_name,&(hsv.h_max),180,on_trackbar,this);
	createTrackbar("S_MIN",window_name,&(hsv.s_min),255,on_trackbar,this);
	createTrackbar("S_MAX",window_name,&(hsv.s_max),255,on_trackbar,this);
	createTrackbar("V_MIN",window_name,&(hsv.v_min),255,on_trackbar,this);
	createTrackbar("V_MAX",window_name,&(hsv.v_max),255,on_trackbar,this);
	createTrackbar("V_MAX",window_name,&(hsv.v_max),255,on_trackbar,this);

	//save button
	createTrackbar("Save file",window_name,&save, 1, button_cb);
}

void threshold_image_cb(const sensor_msgs::Image& new_image_msg)
{
	cv_bridge::CvImageConstPtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(new_image_msg, "8UC1");

	threshold_image = cv_ptr->image;

	cv::imshow(title +" threshold image", threshold_image);
}

void set_color(std::string color_title,char color)
{
	this->title = color_title;
	this->color = color;
}

}; //class ColorCalibration


void button_cb(int state,void* _void)
{
	std_srvs::Trigger srv;
	if(save_file_srv.call(srv))
		ROS_INFO("%s", srv.response.message.c_str());
	else
		ROS_ERROR("Failed to call service");

	destroyAllWindows();
	ROS_INFO("destroy_window");
	ros::shutdown();
}

void on_trackbar(int, void *data)
{
    xenobot::threshold_setting setting_msg;

	ColorCalibration* cc = (ColorCalibration*)data;

    setting_msg.color = (short)cc->color;
    setting_msg.h_min = cc->hsv.h_min;
    setting_msg.h_max = cc->hsv.h_max;
    setting_msg.s_min = cc->hsv.s_min;
    setting_msg.s_max = cc->hsv.s_max;
    setting_msg.v_min = cc->hsv.v_min;
    setting_msg.v_max = cc->hsv.v_max;

    threshold_setting_pub.publish(setting_msg);
}

void marked_image_callback(const sensor_msgs::Image& new_image_msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(new_image_msg, "8UC3");

    marked_image = cv_ptr->image;

    cv::imshow("Lane marked image", marked_image);
}

int main(int argc, char* argv[])
{
	/* ROS initialization */
	ros::init(argc, argv, "xenobot_calibration_panel");
	ros::Time::init();

	ros::NodeHandle nh("xenobot");
	ros::NodeHandle private_nh("~");

	ros::ServiceClient send_hsv_srv = nh.serviceClient<xenobot::SendHsv>("send_hsv_threshold");
	ros::Subscriber marked_image_sub =
			nh.subscribe("marked_image", 5, marked_image_callback);

	save_file_srv = nh.serviceClient<std_srvs::Trigger>("save_yaml_parameter");
	threshold_setting_pub = nh.advertise<xenobot::threshold_setting>("calibration/threshold_setting", 5);

	int color_spec;
	if(private_nh.getParam("color", color_spec) == false) {
		ROS_INFO("Abort: specify the color to calibration!");
		ROS_INFO(" _color:= [1 | 2 | 3]");
		ROS_INFO(" 1:yellow , 2:white , 3:red");
		return -1;
	}

	char color;
	std::string title;

	switch(color_spec) {
		case 1:
			color = 'y';
			title = "inner";
			break;
		case 2:
			color = 'w';
			title = "outer";
			break;
		case 3:
			color = 'r';
			title = "red";
			break;
		default: ROS_ERROR("Color specified not found!");
	}
	ColorCalibration color_calibration;
	color_calibration.set_color(title,color);
	color_calibration.threshold_image_sub = nh.subscribe(title+"_threshold_image", 5,&ColorCalibration::threshold_image_cb,&color_calibration);

	//Call rosservice to get hsv current value
	xenobot::SendHsv srv;

	srv.request.color = color_calibration.color;

	if(send_hsv_srv.call(srv)) {
		ROS_INFO("Get %s hsv threshold",color_calibration.title.c_str());
		color_calibration.hsv.set_hsv(
			srv.response.h_min,srv.response.h_max,
			srv.response.s_min,srv.response.h_min,
			srv.response.v_min,srv.response.v_max);
	}
	else
		ROS_ERROR("Failed to call service");

	color_calibration.create_trackbars();

	while(ros::ok()) {
		waitKey(1);
		ros::spinOnce();
	}

	return 0;
}
