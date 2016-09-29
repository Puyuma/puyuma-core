#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include "lane_detector.hpp"

using namespace cv;

bool tune_outter_road = true;
bool tune_inner_road = true;

void on_trackbar(int, void *)
{
}

LaneDetector::LaneDetector() :
	outer_threshold_h_min(0), outer_threshold_h_max(256),
	outer_threshold_s_min(0), outer_threshold_s_max(256),
	outer_threshold_v_min(0), outer_threshold_v_max(256),
	inner_threshold_h_min(0), inner_threshold_h_max(256),
	inner_threshold_s_min(0), inner_threshold_s_max(256),
	inner_threshold_v_min(0), inner_threshold_v_max(256)
{
	//Hardcode calibration
	outer_threshold_h_min = 50, outer_threshold_h_max = 256;
	outer_threshold_s_min = 0, outer_threshold_s_max = 76;
	outer_threshold_v_min = 75, outer_threshold_v_max = 236;

	//Hardcode calibration
	inner_threshold_h_min = 30, inner_threshold_h_max = 48;
	inner_threshold_s_min = 31, inner_threshold_s_max = 211;
	inner_threshold_v_min = 121, inner_threshold_v_max = 256;

        outer_threshold_img_publisher = node.advertise<sensor_msgs::Image>("xenobot/outer_threshold_image", 1000);
	outter_hough_img_publisher = node.advertise<sensor_msgs::Image>("xenobot/outer_hough_image", 1000);
        inner_threshold_img_publisher = node.advertise<sensor_msgs::Image>("xenobot/inner_threshold_image", 1000);
	inner_hough_img_publisher = node.advertise<sensor_msgs::Image>("xenobot/inner_hough_image", 1000);
        marked_image_publisher = node.advertise<sensor_msgs::Image>("xenobot/marked_image", 1000);
}

void LaneDetector::publish_images()
{
	sensor_msgs::ImagePtr img_msg;

	img_msg = cv_bridge::CvImage(std_msgs::Header(), "8UC1", outer_threshold_image).toImageMsg();
	outer_threshold_img_publisher.publish(img_msg);

	img_msg = cv_bridge::CvImage(std_msgs::Header(), "8UC1", outer_hough_image).toImageMsg();
	outter_hough_img_publisher.publish(img_msg);

	img_msg = cv_bridge::CvImage(std_msgs::Header(), "8UC1", inner_threshold_image).toImageMsg();
	inner_threshold_img_publisher.publish(img_msg);

	img_msg = cv_bridge::CvImage(std_msgs::Header(), "8UC1", inner_hough_image).toImageMsg();
	inner_hough_img_publisher.publish(img_msg);

	img_msg = cv_bridge::CvImage(std_msgs::Header(), "8UC3", lane_mark_image).toImageMsg();
	marked_image_publisher.publish(img_msg);
}

void LaneDetector::tune_hsv_thresholding()
{
	//create window for trackbars
	namedWindow("tune hsv", 0);
	
	if(tune_outter_road == true) {

	//create trackbars and insert them into window
		createTrackbar("outer H_MIN", "tune hsv", &outer_threshold_h_min, 256, on_trackbar);
		createTrackbar("outer H_MAX", "tune hsv", &outer_threshold_h_max, 256, on_trackbar);
		createTrackbar("outer S_MIN", "tune hsv", &outer_threshold_s_min, 256, on_trackbar);
		createTrackbar("outer S_MAX", "tune hsv", &outer_threshold_s_max, 256, on_trackbar);
		createTrackbar("outer V_MIN", "tune hsv", &outer_threshold_v_min, 256, on_trackbar);
		createTrackbar("outer V_MAX", "tune hsv", &outer_threshold_v_max, 256, on_trackbar);
	}

	if(tune_inner_road == true) {
		createTrackbar("inner H_MIN", "tune hsv", &inner_threshold_h_min, 256, on_trackbar);
		createTrackbar("inner H_MAX", "tune hsv", &inner_threshold_h_max, 256, on_trackbar);
		createTrackbar("inner S_MIN", "tune hsv", &inner_threshold_s_min, 256, on_trackbar);
		createTrackbar("inner S_MAX", "tune hsv", &inner_threshold_s_max, 256, on_trackbar);
		createTrackbar("inner V_MIN", "tune hsv", &inner_threshold_v_min, 256, on_trackbar);
		createTrackbar("inner V_MAX", "tune hsv", &inner_threshold_v_max, 256, on_trackbar);
	}
}

void LaneDetector::mark_lane(cv::Mat& lane_mark_image, vector<Vec4i>& lines, Scalar line_color, Scalar dot_color, Scalar text_color)
{
	for(size_t i = 0; i < lines.size(); i++) {  
		Vec4i line = lines[i];
		cv::line(lane_mark_image, Point(line[0], line[1]), Point(line[2], line[3]), line_color, 3, CV_AA);
	}  
}

#define ANGLE_DIFF 1.0
#define rad_to_deg(theta) (theta * 57.29557795)

void calculate_best_fit_slope(vector<Vec4i>& lines, double& angle_max, double& angle_min, double& angle_best, vector<double>& angle_list)
{
	if(lines.size() == 0) {
		return;
	}

	Vec4i line = lines[0];

	double x = (double)line[2] - line[0];
	double y = (double)line[3] - line[1];
	double angle = rad_to_deg(asin(y / x));

	angle_max = angle_min = angle;

	angle_list.clear();
	angle_list.push_back(angle);

	//Create slope list & fine max / min slope
	for(size_t i = 1; i < lines.size(); i++) {
		line = lines[i];

		//Calculate the ratio of x and y
		x = (double)line[2] - line[0];
		y = (double)line[3] - line[1];

		angle = rad_to_deg(asin(y / x));

		angle_list.push_back(angle);

		if(angle > angle_max) {
			angle_max = angle;
		} else if(angle < angle_min) {
			angle_min = angle;
		}
	}

	int max_fit_count = 0;
	double test_angle = angle_min;

	//RANSAC
	do {
		int fit_count = 0;
		for(size_t i = 0; i < lines.size(); i++) {
			if(abs(test_angle - angle_list[i]) < ANGLE_DIFF) {
				fit_count++;
			}
		}

		//Find new best fit slope
		if(fit_count > max_fit_count) {
			max_fit_count = fit_count;
			angle_best = test_angle;
		}

		test_angle += ANGLE_DIFF;
	} while(test_angle <= angle_max);
}

void LaneDetector::lane_detect(cv::Mat& raw_image)
{
	cv::cvtColor(raw_image, outer_hsv_image, COLOR_BGR2HSV);
	cv::cvtColor(raw_image, inner_hsv_image, COLOR_BGR2HSV);

	cv::inRange(
		outer_hsv_image,
		Scalar(outer_threshold_h_min, outer_threshold_s_min, outer_threshold_v_min),
		Scalar(outer_threshold_h_max, outer_threshold_s_max, outer_threshold_v_max),
		outer_threshold_image
	);

	cv::inRange(
		inner_hsv_image,
		Scalar(inner_threshold_h_min, inner_threshold_s_min, inner_threshold_v_min),
		Scalar(inner_threshold_h_max, inner_threshold_s_max, inner_threshold_v_max),
		inner_threshold_image
	);

	cv::Canny(outer_threshold_image, outer_canny_image, 100, 200, 3);
	cv::Canny(inner_threshold_image, inner_canny_image, 100, 200, 3);

	vector<Vec4i> outer_lines, inner_lines;
	cv::HoughLinesP(outer_canny_image, outer_lines, 1, CV_PI / 180, 80, 50, 10);	
	cv::HoughLinesP(inner_canny_image, inner_lines, 1, CV_PI / 180, 80, 50, 10);	

	raw_image.copyTo(lane_mark_image);
	mark_lane(lane_mark_image, outer_lines, Scalar(0, 0, 255), Scalar(255, 0, 0),  Scalar(0, 255, 0));
	mark_lane(lane_mark_image, inner_lines, Scalar(0, 80, 255), Scalar(255, 0, 0),  Scalar(0, 255, 0));

	double outer_angle_max, outer_angle_min, outer_angle_best = 0;
	double inner_angle_max, inner_angle_min, inner_angle_best = 0;
	vector<double> outer_angle_list, inner_angle_list;

	calculate_best_fit_slope(outer_lines, outer_angle_max, outer_angle_min, outer_angle_best, outer_angle_list);
	calculate_best_fit_slope(inner_lines, inner_angle_max, inner_angle_min, inner_angle_best, inner_angle_list);

	ROS_INFO("[OUTER LINE]RANSAC best angle: %lf", outer_angle_best);
	ROS_INFO("[INNER LINE]RANSAC best angle: %lf", inner_angle_best);

	double drive_angle = (outer_angle_best + inner_angle_best) / 2;

	//Calibration
	if(tune_outter_road || tune_inner_road) {
		cvWaitKey(1);
	}

#if 0
	cv::imshow("outter canny image", outer_canny_image);
	cv::imshow("outter hough tranform image", outer_hough_image);
	cv::imshow("inner canny image", inner_canny_image);
	cv::imshow("inner hough tranform image", inner_hough_image);

	cv::imshow("original rgb image", raw_image);
	cv::imshow("outter threshold image", outer_threshold_image);	
	cv::imshow("inner threshold image", inner_threshold_image);	

	cv::imshow("marked image", lane_mark_image);
#endif
}
