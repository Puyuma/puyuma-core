#ifndef LANE_DETECTOR_H
#define LANE_DETECTOR_H

#include <opencv2/opencv.hpp>

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

	void mark_lane(cv::Mat& lane_mark_image, vector<Vec4i>& lines, Scalar line_color, Scalar dot_color, Scalar text_color);

	public:
	LaneDetector() :
		outer_threshold_h_min(0), outer_threshold_h_max(256),
		outer_threshold_s_min(0), outer_threshold_s_max(256),
		outer_threshold_v_min(0), outer_threshold_v_max(256),
		inner_threshold_h_min(0), inner_threshold_h_max(256),
		inner_threshold_s_min(0), inner_threshold_s_max(256),
		inner_threshold_v_min(0), inner_threshold_v_max(256)
	{
		//Hardcode calibration
		outer_threshold_h_min = 0, outer_threshold_h_max = 75;
		outer_threshold_s_min = 135, outer_threshold_s_max = 175;
		outer_threshold_v_min = 69, outer_threshold_v_max = 226;

		//Hardcode calibration
		inner_threshold_h_min = 0, inner_threshold_h_max = 75;
		inner_threshold_s_min = 135, inner_threshold_s_max = 175;
		inner_threshold_v_min = 69, inner_threshold_v_max = 226;
	}

	void tune_hsv_thresholding();
	void lane_detect(cv::Mat& image);

};

#endif
