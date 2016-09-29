#ifndef LANE_DETECTOR_H
#define LANE_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

class LaneDetector {
	private:
	int outer_threshold_h_min, outer_threshold_h_max;
	int outer_threshold_s_min, outer_threshold_s_max;
	int outer_threshold_v_min, outer_threshold_v_max;

	int inner_threshold_h_min, inner_threshold_h_max;
	int inner_threshold_s_min, inner_threshold_s_max;
	int inner_threshold_v_min, inner_threshold_v_max;

	cv::Mat outer_hsv_image, outer_canny_image, outer_threshold_image, outer_hough_image;
	cv::Mat inner_hsv_image, inner_canny_image, inner_threshold_image, inner_hough_image;
	cv::Mat lane_mark_image;

	ros::NodeHandle node;
        ros::Publisher outer_threshold_img_publisher, outter_hough_img_publisher;
        ros::Publisher inner_threshold_img_publisher, inner_hough_img_publisher;
	ros::Publisher marked_image_publisher;

	void mark_lane(cv::Mat& lane_mark_image, vector<Vec4i>& lines, Scalar line_color, Scalar dot_color, Scalar text_color);

	public:
	LaneDetector();

	void tune_hsv_thresholding();
	void lane_detect(cv::Mat& image);
	void publish_images();
};

#endif
