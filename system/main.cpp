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
#include "lane_detector.hpp"

using namespace cv;

LaneDetector* lane_detector;

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

	while(1) {
		camera.grab();
		camera.retrieve(frame);

		cv::Mat camera_matrix = (cv::Mat1d(3, 3) << 279.087996, 0.000000, 329.895256, 0.000000, 278.852468, 189.532203, 0.000000, 0.000000, 1.000000);
		cv::Mat distort_coffecient = (cv::Mat1d(1, 5) << -0.275519, 0.051598, 0.003164, -0.000453, 0.000000);

		cv::Mat distort_image;
		cv::undistort(frame, distort_image, camera_matrix, distort_coffecient);

		sensor_msgs::ImagePtr raw_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
		sensor_msgs::ImagePtr distort_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", distort_image).toImageMsg();

		raw_image_publisher.publish(raw_img_msg);
		distort_image_publisher.publish(distort_img_msg);

		lane_detector->lane_detect(distort_image);
		lane_detector->publish_images();

		//cv::imshow("pi camera", distort_image);

		ros::spinOnce();
	}

	return 0;
}
