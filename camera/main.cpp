#include <sstream>
#include <string>
#include <iostream>

#include <highgui.h>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <raspicam/raspicam_cv.h>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>

#include "lane_detector.hpp"

using namespace cv;

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

	/* Setup Raspicam */
	raspicam::RaspiCam_Cv camera;
	camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
	camera.set(CV_CAP_PROP_FRAME_WIDTH, 320);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
	camera.set(CV_CAP_PROP_EXPOSURE, 100);

	if(!camera.open()) {
		ROS_INFO("failed to open pi camera!\n");
		return 0;
	}

        cv::Mat frame;

	LaneDetector lane_detector;
	lane_detector.tune_hsv_thresholding();

	while(1) {
		camera.grab();
		camera.retrieve(frame);

		cv::Mat camera_matrix = (cv::Mat1d(3, 3) << 137.738809, 0.000000, 168.051971, 0.000000, 137.673367, 102.337151, 0.000000, 0.000000, 1.000000);
		cv::Mat distort_coffecient = (cv::Mat1d(1, 5) << -0.255694, 0.039229, 0.001029, -0.002089, 0.000000);

		cv::Mat distort_image;
		cv::undistort(frame, distort_image, camera_matrix, distort_coffecient);

		sensor_msgs::ImagePtr raw_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
		sensor_msgs::ImagePtr distort_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", distort_image).toImageMsg();

		raw_image_publisher.publish(raw_img_msg);
		distort_image_publisher.publish(distort_img_msg);

		lane_detector.lane_detect(distort_image);
		lane_detector.publish_images();

		//cv::imshow("pi camera", distort_image);

		ros::spinOnce();
	}

	return 0;
}
