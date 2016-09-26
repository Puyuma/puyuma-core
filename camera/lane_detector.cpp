#include <opencv2/opencv.hpp>

#include "lane_detector.hpp"

using namespace cv;

bool tune_outter_road = true;
bool tune_inner_road;

void on_trackbar(int, void *)
{
}

void LaneDetector::tune_hsv_thresholding()
{
	//create window for trackbars
	namedWindow("tune hsv", 0);
	
	if(tune_outter_road == true) {

	//create trackbars and insert them into window
		createTrackbar("H_MIN", "tune hsv", &outer_threshold_h_min, 256, on_trackbar);
		createTrackbar("H_MAX", "tune hsv", &outer_threshold_h_max, 256, on_trackbar);
		createTrackbar("S_MIN", "tune hsv", &outer_threshold_s_min, 256, on_trackbar);
		createTrackbar("S_MAX", "tune hsv", &outer_threshold_s_max, 256, on_trackbar);
		createTrackbar("V_MIN", "tune hsv", &outer_threshold_v_min, 256, on_trackbar);
		createTrackbar("V_MAX", "tune hsv", &outer_threshold_v_max, 256, on_trackbar);
	}

	if(tune_inner_road == true) {
		createTrackbar("H_MIN", "tune hsv", &inner_threshold_h_min, 256, on_trackbar);
		createTrackbar("H_MAX", "tune hsv", &inner_threshold_h_max, 256, on_trackbar);
		createTrackbar("S_MIN", "tune hsv", &inner_threshold_s_min, 256, on_trackbar);
		createTrackbar("S_MAX", "tune hsv", &inner_threshold_s_max, 256, on_trackbar);
		createTrackbar("V_MIN", "tune hsv", &inner_threshold_v_min, 256, on_trackbar);
		createTrackbar("V_MAX", "tune hsv", &inner_threshold_v_max, 256, on_trackbar);
	}
}

void LaneDetector::mark_lane(cv::Mat& lane_mark_image, vector<Vec4i>& lines, Scalar line_color, Scalar dot_color, Scalar text_color)
{
	for(size_t i = 0; i < lines.size(); i++) {  
		Vec4i line = lines[i];
		cv::line(lane_mark_image, Point(line[0], line[1]), Point(line[2], line[3]), line_color, 1, CV_AA);
	}  
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

	vector<Vec4i> outter_lines, inner_lines;
	cv::HoughLinesP(outer_canny_image, outter_lines, 1, CV_PI / 180, 80, 50, 10);	
	cv::HoughLinesP(outer_canny_image, inner_lines, 1, CV_PI / 180, 80, 50, 10);	

	mark_lane(lane_mark_image, outter_lines, Scalar(0, 0, 255), Scalar(255, 0, 0),  Scalar(0, 255, 0));
	//mark_lane(lane_mark_image, inner_lines, Scalar(0, 0, 255), Scalar(255, 0, 0),  Scalar(0, 255, 0));

#if 0
	cv::imshow("outter canny image", outer_canny_image);
	cv::imshow("outter hough tranform image", outer_hough_image);
	cv::imshow("inner canny image", inner_canny_image);
	cv::imshow("inner hough tranform image", inner_hough_image);
#endif

	cv::imshow("original rgb image", raw_image);
	cv::imshow("outter threshold image", outer_threshold_image);	
	cv::imshow("inner threshold image", inner_threshold_image);	

	cv::imshow("marked image", lane_mark_image);
}
