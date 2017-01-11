#include <sstream>
#include <string>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <raspicam/raspicam_cv.h>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>

#include "setting.hpp"

using namespace cv;

void save_homography_matrix(cv::Mat& H)
{
	cv::Mat _H;
	H.copyTo(_H);

	_H.convertTo(_H, CV_64F);

	for(int i = 0; i < 3; i++) {
		ROS_INFO("[%.5f %.5f %.5f]",
			_H.at<double>(i, 0),
			_H.at<double>(i, 1),
			_H.at<double>(i, 2)
		);
	}
}

void mark_checkboard_corners(cv::Mat& rectified_image, std::vector<cv::Point2f>& corners)
{
	cv::Mat marked_image;
	rectified_image.copyTo(marked_image);

	for(size_t i = 0; i < corners.size(); i++) {
		cv::Point2f point = corners[i];

		char index[50] = {'\0'};

		sprintf(index, "%d", i);

		cv::circle(marked_image, Point(point.x, point.y), 1, Scalar(0, 0, 255), 2, CV_AA, 0);
		putText(marked_image, index, Point(point.x, point.y + 10), FONT_HERSHEY_DUPLEX, 1, Scalar(0, 255, 0));
	}

	cv::imshow("Extrinsic calibration", marked_image);

	cvWaitKey(1);	
}

bool estimate_homography(cv::Mat& rectified_image, cv::Mat& H)
{
	std::vector<cv::Point2f> corners;

	int board_w = 7, board_h = 5;
	cv::Size board_size(board_w, board_h);

	bool found = findChessboardCorners(rectified_image, board_size, corners, 
		CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_FILTER_QUADS
	);

	/* Check the board if found or not */
	if(found == true) {
		/* Get better resolution of corner's location */
		cornerSubPix(rectified_image, corners, cv::Size(11, 11), cv::Size(-1, -1),
			cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

		ROS_INFO("Find the checkboard!");
	} else {
		ROS_INFO("Can't find the checkboard, please adjust the light or reduce the noise!");
		return false;
	}

	mark_checkboard_corners(rectified_image, corners);

	/* Check if order is wrong, if so, arrange it */
	cv::Point2f corner_low_right = corners[0];
	cv::Point2f corner_up_right = corners[(board_h - 1) * board_w];
	cv::Point2f corner_up_left = corners[board_h * board_w - 1];

	bool h_flipped = false, v_flipped = false;

	if(corner_up_left.x > corner_up_right.x) h_flipped = true;
	if(corner_low_right.y < corner_up_right.y) v_flipped = true;

	/* Generate source points and destination points that transform by Homography matrix */
	std::vector<cv::Point2f> ground_plane_points; //Calculate by the known parameter (E.g: checkboard square size, row size, column size...)
	std::vector<cv::Point2f> image_plane_points; //Observe from the image
	ground_plane_points.resize(board_w * board_h);
	image_plane_points.resize(board_w * board_h);

	float x_offset = 0.f;
	float y_offset = -0.f;
	cv::Point2f offset= cv::Point2f(x_offset, y_offset);

	for(int row = 0; row < board_h; row++) {
		for(int column = 0; column < board_w; column++) {
			ground_plane_points[board_w * board_h - (row * board_w + column) - 1] = cv::Point2f(float(column) * SQUARE_WIDTH + OFFSET_X, float(row) * SQUARE_HEIGHT + OFFSET_Y);

			image_plane_points[row * board_w + column] =
				corners[
					(v_flipped ? board_h - 1 - row : row) * board_w +
					(h_flipped ? board_w - 1 - column : column)
				];
		}
	}

	mark_checkboard_corners(rectified_image, ground_plane_points);

	H = cv::findHomography(image_plane_points, ground_plane_points, CV_RANSAC);

#if 1
	cv::Mat test;
	warpPerspective(rectified_image, test, H, rectified_image.size());

	imshow("ground projection", test);

	waitKey(0);

	cv::destroyWindow("ground projection");
#endif

	cv::destroyWindow("Extrinsic calibration");

	ROS_INFO("Sucessfully calculated the Homography Matrix:");

	save_homography_matrix(H);

	return true;
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
	camera.set(CV_CAP_PROP_FORMAT, CV_8UC1);
	camera.set(CV_CAP_PROP_FRAME_WIDTH, 320);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
	camera.set(CV_CAP_PROP_BRIGHTNESS, 50);
	camera.set(CV_CAP_PROP_CONTRAST, 50);
	camera.set(CV_CAP_PROP_SATURATION, 100);
	camera.set(CV_CAP_PROP_GAIN, 1);
	camera.set(CV_CAP_PROP_EXPOSURE, 10);
	//camera.set(CV_CAP_PROP_WHITE_BALANCE_RED_V, 1);
	//camera.set(CV_CAP_PROP_WHITE_BALANCE_BLUE_U, 1);

	if(!camera.open()) {
		ROS_INFO("failed to open pi camera!\n");
		return 0;
	}

        cv::Mat raw_image, grey_image, ground_projected_image;
	cv::Mat H; //Homography matrix

	bool get_H = false;

	while(1) {
		camera.grab();
		camera.retrieve(raw_image);

		cv::Mat camera_matrix = (cv::Mat1d(3, 3) << 136.106985, 0.000000, 166.663269,
							0.000000, 136.627212, 105.393529,
							0.000000, 0.000000, 1.000000);
		cv::Mat distort_coffecient = (cv::Mat1d(1, 5) <<
			-0.246384, 0.037375, 0.000300, -0.001282, 0.000000);

		cv::Mat distort_image;
		cv::undistort(raw_image, distort_image, camera_matrix, distort_coffecient);

		//Image sharpening
		Mat temp_image;
		cv::GaussianBlur(distort_image, temp_image, Size(0, 0) , 10);
		cv::addWeighted(distort_image, 1.8, temp_image, -0.8, 0, distort_image) ;

		if(get_H == false) {
			if(estimate_homography(distort_image, H) == true) {
				get_H = true;
			}
		} else {
			warpPerspective(distort_image, ground_projected_image, H, distort_image.size());

			cv::imshow("Homography image", ground_projected_image);
		}

		cv::imshow("Raw image", distort_image);

		waitKey(1);	


		//sensor_msgs::ImagePtr homography_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", distort_image).toImageMsg();

		//homography_image_publisher.publish(homography_img_msg);

		//cv::imshow("pi camera", distort_image);

		//ros::spinOnce();
	}

	return 0;
}
