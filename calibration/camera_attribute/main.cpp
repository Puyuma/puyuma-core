#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <raspicam/raspicam_cv.h>
#include <cv_bridge/cv_bridge.h>
#include <camera.hpp>

using namespace cv;

raspicam::RaspiCam_Cv camera;


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "camera_attribute");
	ros::NodeHandle node;
	ros::Rate loop_rate(10);
	//ros::ServiceServer srv = node.advertiseService("", save_yaml_parameter);;

	cv::Mat frame, bgr_frame;

	if(!camera_setup(camera)) {
		ROS_INFO("Abort: failed to open pi camera!");
		return 0;
	}

	ros::Publisher raw_image_publisher = node.advertise<sensor_msgs::Image>("raw_image", 1);

	while(ros::ok()) {
		camera.grab();
		camera.retrieve(frame);
		sensor_msgs::ImagePtr img_msg =
			cv_bridge::CvImage(std_msgs::Header(), "rgb8", frame).toImageMsg();
		raw_image_publisher.publish(img_msg);
		loop_rate.sleep();
	}
	return 0;
}
