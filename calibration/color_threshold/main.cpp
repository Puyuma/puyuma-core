#include <sstream>
#include <string>
#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include <ros/console.h>
#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <xenobot/threshold_setting.h>

#include "setting.h"

using namespace cv;


int save = 0;
std::string yaml_path;
std::string machine_name;
ros::Publisher threshold_setting_pub;

cv::Mat outer_threshold_image;
cv::Mat inner_threshold_image;
cv::Mat marked_image;
YAML::Node yaml_node;

int i_h_min_threshold = FINE_CALIBRATED_H_MIN;
int i_h_max_threshold = FINE_CALIBRATED_H_MAX;
int i_s_min_threshold = FINE_CALIBRATED_S_MIN;
int i_s_max_threshold = FINE_CALIBRATED_S_MAX;
int i_v_min_threshold = FINE_CALIBRATED_V_MIN;
int i_v_max_threshold = FINE_CALIBRATED_V_MAX;


int o_h_min_threshold = FINE_CALIBRATED_H_MIN;
int o_h_max_threshold = FINE_CALIBRATED_H_MAX;
int o_s_min_threshold = FINE_CALIBRATED_S_MIN;
int o_s_max_threshold = FINE_CALIBRATED_S_MAX;
int o_v_min_threshold = FINE_CALIBRATED_V_MIN;
int o_v_max_threshold = FINE_CALIBRATED_V_MAX;

void button_cb(int ,void*);


void on_trackbar(int, void *)
{
	xenobot::threshold_setting setting_msg;

	setting_msg.outer_h_min = o_h_min_threshold;
	setting_msg.outer_h_max = o_h_max_threshold;
	setting_msg.outer_s_min = o_s_min_threshold;
	setting_msg.outer_s_max = o_s_max_threshold;
	setting_msg.outer_v_min = o_v_min_threshold;
	setting_msg.outer_v_max = o_v_max_threshold;

	setting_msg.inner_h_min = i_h_min_threshold;
	setting_msg.inner_h_max = i_h_max_threshold;
	setting_msg.inner_s_min = i_s_min_threshold;
	setting_msg.inner_s_max = i_s_max_threshold;
	setting_msg.inner_v_min = i_v_min_threshold;
	setting_msg.inner_v_max = i_v_max_threshold;

	threshold_setting_pub.publish(setting_msg);
}

void create_trackbars_w()
{
	//create window for trackbars
	namedWindow("Outer_Threshold setting", 0);

	//create trackbars and insert them into window
	createTrackbar("H_MIN", "Outer_Threshold setting", &o_h_min_threshold, 256, on_trackbar);
	createTrackbar("H_MAX", "Outer_Threshold setting", &o_h_max_threshold, 256, on_trackbar);
	createTrackbar("S_MIN", "Outer_Threshold setting", &o_s_min_threshold, 256, on_trackbar);
	createTrackbar("S_MAX", "Outer_Threshold setting", &o_s_max_threshold, 256, on_trackbar);
	createTrackbar("V_MIN", "Outer_Threshold setting", &o_v_min_threshold, 256, on_trackbar);
	createTrackbar("V_MAX", "Outer_Threshold setting", &o_v_max_threshold, 256, on_trackbar);
	createTrackbar("V_MAX", "Outer_Threshold setting", &o_v_max_threshold, 256, on_trackbar);
	createTrackbar("Save file", "Outer_Threshold setting", &save, 1, button_cb);
}

void create_trackbars_y()
{
	namedWindow("Inner_Threshold setting",0);
	 createButton("button6",button_cb,NULL,CV_PUSH_BUTTON,1);	

	createTrackbar("H_MIN", "Inner_Threshold setting", &i_h_min_threshold, 256, on_trackbar);
	createTrackbar("H_MAX", "Inner_Threshold setting", &i_h_max_threshold, 256, on_trackbar);
	createTrackbar("S_MIN", "Inner_Threshold setting", &i_s_min_threshold, 256, on_trackbar);
	createTrackbar("S_MAX", "Inner_Threshold setting", &i_s_max_threshold, 256, on_trackbar);
	createTrackbar("V_MIN", "Inner_Threshold setting", &i_v_min_threshold, 256, on_trackbar);
	createTrackbar("V_MAX", "Inner_Threshold setting", &i_v_max_threshold, 256, on_trackbar);

	createTrackbar("Save file", "Inner_Threshold setting", &save, 1, button_cb);
}

void marked_image_callback(const sensor_msgs::Image& new_image_msg)
{
	cv_bridge::CvImageConstPtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(new_image_msg, "8UC3");

	marked_image = cv_ptr->image;

	cv::imshow("Lane marked image", marked_image);

	//ROS_INFO("Received marked image");
}

void outer_threshold_image_callback(const sensor_msgs::Image& new_image_msg)
{
	cv_bridge::CvImageConstPtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(new_image_msg, "8UC1");

	outer_threshold_image = cv_ptr->image;

	cv::imshow("Outer threshold image", outer_threshold_image);

	//ROS_INFO("Received outer threshold image");
}


void inner_threshold_image_callback(const sensor_msgs::Image& new_image_msg)
{
	cv_bridge::CvImageConstPtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(new_image_msg, "8UC1");

	inner_threshold_image = cv_ptr->image;

	cv::imshow("Inner threshold image", inner_threshold_image);

	//ROS_INFO("Received inner threshold image");
}

bool load_yaml(std::string yaml_path)
{
	try
	{
		yaml_path = yaml_path +  "hsv_thresholding.yaml";

		std::cout << "Load yaml file: " << yaml_path << "\n";

		yaml_node = YAML::LoadFile(yaml_path);

		YAML::Node inner_node = yaml_node["inner"];
		i_h_min_threshold = inner_node["h_min"].as<int>();
		i_h_max_threshold = inner_node["h_max"].as<int>();
		i_s_min_threshold = inner_node["s_min"].as<int>();
		i_s_max_threshold = inner_node["s_max"].as<int>();
		i_v_min_threshold = inner_node["v_min"].as<int>();
		i_v_max_threshold = inner_node["v_max"].as<int>();
		
		YAML::Node outer_node = yaml_node["outer"];
		o_h_min_threshold = outer_node["h_min"].as<int>();
		o_h_max_threshold = outer_node["h_max"].as<int>();
		o_s_min_threshold = outer_node["s_min"].as<int>();
		o_s_max_threshold = outer_node["s_max"].as<int>();
		o_v_min_threshold = outer_node["v_min"].as<int>();
		o_v_max_threshold = outer_node["v_max"].as<int>();
	}
	catch(...)
	{
		ROS_INFO("Load file fail");
		exit(0);
	}	
}

void save_to_file(std::string save_file_name)
{
	std::ofstream fout(save_file_name.c_str()); 

	YAML::Node inner_node = yaml_node["inner"];

	inner_node["h_min"] = i_h_min_threshold;
	inner_node["h_max"] = i_h_max_threshold;
	inner_node["s_min"] = i_s_min_threshold;
	inner_node["s_max"] = i_s_max_threshold;
	inner_node["v_min"] = i_v_min_threshold;
	inner_node["v_max"] = i_v_max_threshold;
        
	YAML::Node outer_node = yaml_node["outer"];

	outer_node["h_min"] = o_h_min_threshold;
	outer_node["h_max"] = o_h_max_threshold;
	outer_node["s_min"] = o_s_min_threshold;
	outer_node["s_max"] = o_s_max_threshold;
	outer_node["v_min"] = o_v_min_threshold;
	outer_node["v_max"] = o_v_max_threshold;

	fout << yaml_node; // dump it back into the file
}

void button_cb(int state,void* _void)
{
	try
	{
		std::string save_file_name = yaml_path +  "hsv_thresholding.yaml";//.tmp";
		save_to_file(save_file_name);
		ROS_INFO("Save success");
	}
	catch(...)
	{
		ROS_WARN("Save file fail");
	}

	destroyAllWindows();
	ROS_INFO("destroy_window");
	ros::shutdown();	
	return;
}

int main(int argc, char* argv[])
{
	/* ROS initialization */
	ros::init(argc, argv, "xenobot_calibration_panel");
        ros::Time::init();
        ros::Rate loop_rate(1);

	ros::NodeHandle nh("xenobot");
	ros::NodeHandle private_nh("~");

	if(nh.getParam("/config_path", yaml_path) == false) {
		ROS_INFO("Abort: no configuration path assigned!");
		ROS_INFO("Instead of doing rosrun command, you should try roslaunch command");
		return 0;
	}

	if(nh.getParam("/machine_name", machine_name) == false) {
		ROS_INFO("Abort: no machine name assigned!");
		return 0;
	}

	//yaml_path = "/home/ponsheng/xenobot/catkin_ws/src/xenobot/config/";
	yaml_path = yaml_path + machine_name + "/";
	std::cout << "Yaml_path: " << yaml_path << "\n";
	load_yaml(yaml_path);
	
	std::string color_spec;
	
	if(private_nh.getParam("color", color_spec) == false) {
		ROS_INFO("Abort: specify the color to calibration!");
		ROS_INFO(" _color:= [yellow | white | both]");
		
		return 0;
	}

	threshold_setting_pub =
		nh.advertise<xenobot::threshold_setting>("calibration/threshold_setting", 5);

	ros::Subscriber marked_image_sub =
                nh.subscribe("marked_image", 5, marked_image_callback);

	ros::Subscriber inner_threshold_image_sub;
	ros::Subscriber outer_threshold_image_sub;

	if(color_spec == "yellow" || color_spec == "both")
	{
		inner_threshold_image_sub =
                nh.subscribe("inner_threshold_image", 5, inner_threshold_image_callback);
		create_trackbars_y();

	}
	
	if(color_spec == "white" || color_spec == "both")
	{
		outer_threshold_image_sub =
                nh.subscribe("outer_threshold_image", 5, outer_threshold_image_callback);
		create_trackbars_w();
	}

	//waitKey(0);
	while(ros::ok()) {
		waitKey(1);

		ros::spinOnce();
	}	

	return 0;
}
