#include <sstream>
#include <string>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <raspicam/raspicam_cv.h>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>

using namespace cv;

int main(int argc, char* argv[])
{
	/* ROS initialization */
	ros::init(argc, argv, "xenobot_camera_test");
        ros::Time::init();
	ros::NodeHandle nh;
        ros::Rate loop_rate(30);

	ros::NodeHandle node;

	ros::Publisher raw_image_publisher = 
		node.advertise<sensor_msgs::Image>("xenobot/raw_image", 1000);

	/* Setup Raspicam */
	raspicam::RaspiCam_Cv camera;
	camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
	camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	//camera.set(CV_CAP_PROP_BRIGHTNESS, 50);
	//camera.set(CV_CAP_PROP_CONTRAST, 50);
	//camera.set(CV_CAP_PROP_SATURATION, 50);
	//camera.set(CV_CAP_PROP_GAIN, 50);
	//camera.set(CV_CAP_PROP_EXPOSURE, 10);
	//camera.set(CV_CAP_PROP_WHITE_BALANCE_RED_V, 1);
	//camera.set(CV_CAP_PROP_WHITE_BALANCE_BLUE_U, 1);

	if(!camera.open()) {
		ROS_INFO("failed to open pi camera!\n");
		return 0;
	}

        cv::Mat raw_image;

	while(1) {
		camera.grab();
		camera.retrieve(raw_image);

		sensor_msgs::ImagePtr raw_img_msg =
			cv_bridge::CvImage(std_msgs::Header(), "bgr8", raw_image).toImageMsg();
		raw_image_publisher.publish(raw_img_msg);

		//cv::imshow("Raw image", raw_image);
		//waitKey(1);
	}

	return 0;
}
