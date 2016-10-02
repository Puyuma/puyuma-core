#include <sstream>
#include <string>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <raspicam/raspicam_cv.h>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>

using namespace cv;

void mark_checkboard_corners(cv::Mat& rectified_image, std::vector<cv::Point2f>& corners)
{
	cv::Mat marked_image;
	rectified_image.copyTo(marked_image);

	for(size_t i = 0; i < corners.size(); i++) {
		cv::Point2f point = corners[i];

		cv::circle(marked_image, Point(point.x, point.y), 1, Scalar(0, 0, 255), 1, CV_AA, 0);
	}

	cv::imshow("Extrinsic calibration", marked_image);

	cvWaitKey(1);	
}

bool estimate_homography(cv::Mat& rectified_image, cv::Mat& H)
{
	std::vector<cv::Point2f> corners;

	int board_w = 7, board_h = 5;
	cv::Size board_size(board_w, board_h);

	float square_size = 0.031f;

	bool found = findChessboardCorners(rectified_image, board_size, corners);

	/* Find checkboard and find the corner */
	if(found) {
		/* Get better resolution of corner's location */
		cornerSubPix(rectified_image, corners, cv::Size(11, 11), cv::Size(-1, -1),
			cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
	} else {
		ROS_INFO("Can't find the checkboard!");
	}

	mark_checkboard_corners(rectified_image, corners);

	/* Generate source points and destination points that transform by Homography matrix */
	std::vector<cv::Point2f> grond_plane_points; //Calculate by the known parameter (E.g: checkboard square size, row size, column size...)
	std::vector<cv::Point2f> image_plane_points; //Observe from the image
	grond_plane_points.resize(board_w * board_h);
	image_plane_points.resize(board_w * board_h);

	for(int row = 0; row < board_h; row++) {
		for(int column = 0; column < board_w; column++) {
			grond_plane_points[row * board_w + column] = cv::Point2f((float)row * square_size, (float)column * square_size);
		}
	}

	H = cv::findHomography(image_plane_points, grond_plane_points, CV_RANSAC);
}

int main(int argc, char* argv[])
{
	/* ROS initialization */
	ros::init(argc, argv, "xenobot_extrinsic_calibration");
        ros::Time::init();
	ros::NodeHandle nh;
        ros::Rate loop_rate(30);

	ros::NodeHandle node;

	ros::Publisher homography_image_publisher = 
		node.advertise<sensor_msgs::Image>("xenobot/homography_image", 1000);

	/* Setup Raspicam */
	raspicam::RaspiCam_Cv camera;
	camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
	camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	camera.set(CV_CAP_PROP_BRIGHTNESS, -1);
	camera.set(CV_CAP_PROP_CONTRAST, 50);
	camera.set(CV_CAP_PROP_SATURATION, 50);
	camera.set(CV_CAP_PROP_GAIN, 50);
	camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
	camera.set(CV_CAP_PROP_EXPOSURE, 50);
	camera.set(CV_CAP_PROP_WHITE_BALANCE_RED_V, 1);
	camera.set(CV_CAP_PROP_WHITE_BALANCE_BLUE_U, 1);

	if(!camera.open()) {
		ROS_INFO("failed to open pi camera!\n");
		return 0;
	}

        cv::Mat raw_image;
	cv::Mat H; //Homography matrix

	while(1) {
		camera.grab();
		camera.retrieve(raw_image);

		cv::Mat camera_matrix = (cv::Mat1d(3, 3) << 279.087996, 0.000000, 329.895256, 0.000000, 278.852468, 189.532203, 0.000000, 0.000000, 1.000000);
		cv::Mat distort_coffecient = (cv::Mat1d(1, 5) << -0.275519, 0.051598, 0.003164, -0.000453, 0.000000);

		cv::Mat distort_image;
		cv::undistort(raw_image, distort_image, camera_matrix, distort_coffecient);

		estimate_homography(distort_image, H);

		sensor_msgs::ImagePtr homography_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", distort_image).toImageMsg();

		//homography_image_publisher.publish(homography_img_msg);

		//cv::imshow("pi camera", distort_image);

		ros::spinOnce();
	}

	return 0;
}
