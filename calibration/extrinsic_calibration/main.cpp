#include <sstream>
#include <string>
#include <iostream>

#include <highgui.h>
#include <cv.h>
#include <raspicam/raspicam_cv.h>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>

using namespace cv;

bool estimate_homography(cv::Mat& rectified_image)
{
	std::vector<cv::Point2f> corners;

	int board_w = 7, board_h = 5;
	cv::Size board_size(board_w, board_h);

	bool found = findChessboardCorners(rectified_image, board_size, corners);

	if(found) {
		cornerSubPix(rectified_image, corners, cv::Size(11, 11), cv::Size(-1, -1),
			cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
	}
}

int main(int argc, char* argv[])
{
	/* ROS initialization */
	ros::init(argc, argv, "xenobot");
        ros::Time::init();
	ros::NodeHandle nh;
        ros::Rate loop_rate(30);

	ros::Publisher homography_image_publisher = 
		node.advertise<sensor_msgs::Image>("xenobot/homography_image", 1000);

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

        cv::Mat raw_image;

	while(1) {
		camera.grab();
		camera.retrieve(raw_image);

		cv::Mat camera_matrix = (cv::Mat1d(3, 3) << 279.087996, 0.000000, 329.895256, 0.000000, 278.852468, 189.532203, 0.000000, 0.000000, 1.000000);
		cv::Mat distort_coffecient = (cv::Mat1d(1, 5) << -0.275519, 0.051598, 0.003164, -0.000453, 0.000000);

		cv::Mat distort_image;
		cv::undistort(raw_image, distort_image, camera_matrix, distort_coffecient);

		sensor_msgs::ImagePtr homography_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", distort_image).toImageMsg();

		homography_image_publisher.publish(homography_img_msg);

		//cv::imshow("pi camera", distort_image);

		ros::spinOnce();
	}

	return 0;
}
