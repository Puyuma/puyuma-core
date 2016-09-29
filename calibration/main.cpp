#include <sstream>
#include <string>
#include <iostream>

#include "ros/ros.h"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <xenobot/threshold_setting.h>

#include "setting.h"

using namespace cv;

ros::Publisher threshold_setting_pub;

cv::Mat outer_threshold_image;
cv::Mat inner_threshold_image;
cv::Mat marked_image;

int h_min_threshold = FINE_CALIBRATED_H_MIN;
int h_max_threshold = FINE_CALIBRATED_H_MAX;
int s_min_threshold = FINE_CALIBRATED_S_MIN;
int s_max_threshold = FINE_CALIBRATED_S_MAX;
int v_min_threshold = FINE_CALIBRATED_V_MIN;
int v_max_threshold = FINE_CALIBRATED_V_MAX;

void on_trackbar(int, void *)
{
	xenobot::threshold_setting setting_msg;

	setting_msg.outer_h_min = h_min_threshold;
	setting_msg.outer_h_max = h_max_threshold;
	setting_msg.outer_s_min = s_min_threshold;
	setting_msg.outer_s_max = s_max_threshold;
	setting_msg.outer_v_min = v_min_threshold;
	setting_msg.outer_v_max = v_max_threshold;

	setting_msg.inner_h_min = h_min_threshold;
	setting_msg.inner_h_max = h_max_threshold;
	setting_msg.inner_s_min = s_min_threshold;
	setting_msg.inner_s_max = s_max_threshold;
	setting_msg.inner_v_min = v_min_threshold;
	setting_msg.inner_v_max = v_max_threshold;

	threshold_setting_pub.publish(setting_msg);
}

void create_trackbars()
{
	//create window for trackbars
	namedWindow("Threshold setting", 0);
	
	//create trackbars and insert them into window
	createTrackbar("H_MIN", "Threshold setting", &h_min_threshold, 256, on_trackbar);
	createTrackbar("H_MAX", "Threshold setting", &h_max_threshold, 256, on_trackbar);
	createTrackbar("S_MIN", "Threshold setting", &s_min_threshold, 256, on_trackbar);
	createTrackbar("S_MAX", "Threshold setting", &s_max_threshold, 256, on_trackbar);
	createTrackbar("V_MIN", "Threshold setting", &v_min_threshold, 256, on_trackbar);
	createTrackbar("V_MAX", "Threshold setting", &v_max_threshold, 256, on_trackbar);

	createTrackbar("H_MIN", "Threshold setting", &h_min_threshold, 256, on_trackbar);
	createTrackbar("H_MAX", "Threshold setting", &h_max_threshold, 256, on_trackbar);
	createTrackbar("S_MIN", "Threshold setting", &s_min_threshold, 256, on_trackbar);
	createTrackbar("S_MAX", "Threshold setting", &s_max_threshold, 256, on_trackbar);
	createTrackbar("V_MIN", "Threshold setting", &v_min_threshold, 256, on_trackbar);
	createTrackbar("V_MAX", "Threshold setting", &v_max_threshold, 256, on_trackbar);
}

void marked_image_callback(const sensor_msgs::Image& new_image_msg)
{
	cv_bridge::CvImageConstPtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(new_image_msg, "8UC3");

	marked_image = cv_ptr->image;

	cv::imshow("Threshold setting", marked_image);

	ROS_INFO("Received marked image");
}

void outer_threshold_image_callback(const sensor_msgs::Image& new_image_msg)
{
	cv_bridge::CvImageConstPtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(new_image_msg, "8UC1");

	outer_threshold_image = cv_ptr->image;

	cv::imshow("Threshold setting", outer_threshold_image);

	ROS_INFO("Received outer threshold image");
}

void inner_threshold_image_callback(const sensor_msgs::Image& new_image_msg)
{
	cv_bridge::CvImageConstPtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(new_image_msg, "8UC1");

	inner_threshold_image = cv_ptr->image;

	cv::imshow("Threshold setting", inner_threshold_image);

	ROS_INFO("Received inner threshold image");
}

int main(int argc, char* argv[])
{
	/* ROS initialization */
	ros::init(argc, argv, "calibration_panel");
        ros::Time::init();
        ros::Rate loop_rate(1);

	ros::NodeHandle node;

	threshold_setting_pub =
		node.advertise<xenobot::threshold_setting>("/calibration/threshold_setting", 1000);

	ros::Subscriber marked_image_sub =
                node.subscribe("/xenobot/marked_image", 1000, marked_image_callback);

	ros::Subscriber outer_threshold_image_sub =
                node.subscribe("/xenobot/outer_threshold_image", 1000, outer_threshold_image_callback);

	ros::Subscriber inner_threshold_image_sub =
                node.subscribe("/xenobot/inner_threshold_image", 1000, inner_threshold_image_callback);

	create_trackbars();

	while(1) {
		waitKey(1);
	}

	return 0;
}
