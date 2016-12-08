#include <fstream>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>

#include "lane_detector.hpp"
#include "xeno_math.hpp"

#define DRAW_DEBUG_INFO 1

#define rad_to_deg(phi) (phi * 57.2957795)

using namespace cv;

enum SEGMENT_COLOR {WHITE, YELLOW, RED};

void on_trackbar(int, void *)
{
}

LaneDetector::LaneDetector(string _yaml_path, bool calibrate_mode) :
	outer_threshold_h_min(0), outer_threshold_h_max(256),
	outer_threshold_s_min(0), outer_threshold_s_max(256),
	outer_threshold_v_min(0), outer_threshold_v_max(256),
	inner_threshold_h_min(0), inner_threshold_h_max(256),
	inner_threshold_s_min(0), inner_threshold_s_max(256),
	inner_threshold_v_min(0), inner_threshold_v_max(256)
{
	this->calibrate_mode = calibrate_mode;

	yaml_path = _yaml_path;

	if(calibrate_mode == true) {
        	outer_threshold_img_publisher =
			node.advertise<sensor_msgs::Image>("xenobot/outer_threshold_image", 1000);

		outter_hough_img_publisher =
			node.advertise<sensor_msgs::Image>("xenobot/outer_hough_image", 1000);

       		canny_img_publisher =
			node.advertise<sensor_msgs::Image>("xenobot/canny_image", 1000);

       		inner_threshold_img_publisher =
			node.advertise<sensor_msgs::Image>("xenobot/inner_threshold_image", 1000);

		inner_hough_img_publisher =
			node.advertise<sensor_msgs::Image>("xenobot/inner_hough_image", 1000);

	        marked_image_publisher =
			node.advertise<sensor_msgs::Image>("xenobot/marked_image", 1000);
	}
}

void LaneDetector::publish_images()
{
	sensor_msgs::ImagePtr img_msg;

	img_msg = cv_bridge::CvImage(std_msgs::Header(), "8UC3", lane_mark_image).toImageMsg();
	marked_image_publisher.publish(img_msg);

	if(calibrate_mode == false) {
		return;
	}

	img_msg = cv_bridge::CvImage(std_msgs::Header(), "8UC1", canny_image).toImageMsg();
	canny_img_publisher.publish(img_msg);

	img_msg = cv_bridge::CvImage(std_msgs::Header(), "8UC1", outer_threshold_image).toImageMsg();
	outer_threshold_img_publisher.publish(img_msg);

	img_msg = cv_bridge::CvImage(std_msgs::Header(), "8UC1", outer_hough_image).toImageMsg();
	outter_hough_img_publisher.publish(img_msg);

	img_msg = cv_bridge::CvImage(std_msgs::Header(), "8UC1", inner_threshold_image).toImageMsg();
	inner_threshold_img_publisher.publish(img_msg);

	img_msg = cv_bridge::CvImage(std_msgs::Header(), "8UC1", inner_hough_image).toImageMsg();
	inner_hough_img_publisher.publish(img_msg);
}

void LaneDetector::set_hsv(
	double outer_h_max, double outer_h_min, double outer_s_max,
	double outer_s_min, double outer_v_max, double outer_v_min,
	double inner_h_max, double inner_h_min, double inner_s_max,
	double inner_s_min, double inner_v_max, double inner_v_min)
{
	outer_threshold_h_min = outer_h_min;
	outer_threshold_h_max = outer_h_max;
	outer_threshold_s_min = outer_s_min;
	outer_threshold_s_max = outer_s_max;
	outer_threshold_v_min = outer_v_min;
	outer_threshold_v_max = outer_v_max;

	inner_threshold_h_min = inner_h_min;
	inner_threshold_h_max = inner_h_max;
	inner_threshold_s_min = inner_s_min;
	inner_threshold_s_max = inner_s_max;
	inner_threshold_v_min = inner_v_min;
	inner_threshold_v_max = inner_v_max;
}

bool LaneDetector::load_yaml_setting()
{
	if(read_threshold_setting(yaml_path + "hsv_thresholding.yaml") == false) {
		ROS_INFO("Abort: Can't find color thresholding setting!");
		return false;
	}

	if(read_extrinsic_calibration(yaml_path + "extrinsic_calibration.yaml") == false) {
		ROS_INFO("Abort: Can't find extrinsic calibration data!");
		return false;
	}
}

bool LaneDetector::read_extrinsic_calibration(string _yaml_path)
{
	try {
		ROS_INFO("%s", _yaml_path.c_str());

		YAML::Node yaml = YAML::LoadFile(_yaml_path);

		double homography_array[9];

		for(int i = 0; i < 9; i++) { 
			homography_array[i] = yaml["homography_matrix"]["data"][i].as<double>();
		}

		H = cv::Mat(3, 3, CV_64F, homography_array);
	} catch(...) {
		return false;
	}

	return true;
}

bool LaneDetector::read_threshold_setting(string _yaml_path)
{
	int outer_h_max, outer_s_max, outer_v_max;
	int outer_h_min, outer_s_min, outer_v_min;
	int inner_h_max, inner_s_max, inner_v_max;
	int inner_h_min, inner_s_min, inner_v_min;

	try {
		ROS_INFO("%s", _yaml_path.c_str());

		YAML::Node yaml = YAML::LoadFile(_yaml_path);

		outer_h_min = yaml["outer_h_min"].as<int>();
		outer_h_max = yaml["outer_h_max"].as<int>();
		outer_s_min = yaml["outer_s_min"].as<int>();
		outer_s_max = yaml["outer_s_max"].as<int>();
		outer_v_min = yaml["outer_v_min"].as<int>();
		outer_v_max = yaml["outer_v_max"].as<int>();

		inner_h_min = yaml["inner_h_min"].as<int>();
		inner_h_max = yaml["inner_h_max"].as<int>();
		inner_s_min = yaml["inner_s_min"].as<int>();
		inner_s_max = yaml["inner_s_max"].as<int>();
		inner_v_min = yaml["inner_v_min"].as<int>();
		inner_v_max = yaml["inner_v_max"].as<int>();
	} catch(...) {
		return false;
	}

	set_hsv(
		outer_h_max, outer_h_min,
		outer_s_max, outer_s_min,
		outer_v_max, outer_v_min,
		inner_h_max, inner_h_min,
		inner_s_max, inner_s_min,
		inner_v_max, inner_v_min
	);

	return true;
}

void LaneDetector::append_yaml_data(YAML::Emitter& yaml_handler, string key, int value)
{
	yaml_handler << YAML::Key << key;
	yaml_handler << YAML::Value << value;
}

void LaneDetector::save_thresholding_yaml()
{
	YAML::Emitter out;

	out << YAML::BeginMap;
	append_yaml_data(out, "outer_h_max", outer_threshold_h_max);
	append_yaml_data(out, "outer_h_min", outer_threshold_h_min);
	append_yaml_data(out, "outer_s_max", outer_threshold_s_max);
	append_yaml_data(out, "outer_s_min", outer_threshold_s_min);
	append_yaml_data(out, "outer_v_max", outer_threshold_v_max);
	append_yaml_data(out, "outer_v_min", outer_threshold_v_min);

	append_yaml_data(out, "inner_h_max", inner_threshold_h_max);
	append_yaml_data(out, "inner_h_min", inner_threshold_h_min);
	append_yaml_data(out, "inner_s_max", inner_threshold_s_max);
	append_yaml_data(out, "inner_s_min", inner_threshold_s_min);
	append_yaml_data(out, "inner_v_max", inner_threshold_v_max);
	append_yaml_data(out, "inner_v_min", inner_threshold_v_min);

	/* Save yaml into file */
	string file_path = yaml_path + "hsv_thresholding.yaml";

	fstream fp;
	fp.open(file_path.c_str(), ios::out);

	if(fp != 0) {
		fp << out.c_str() << endl; //Write yaml
	} else {
		ROS_INFO("Failed to save the thresholding setting!");
	}
}

void LaneDetector::calculate_best_fittedline(vector<Vec4f>& lines, Vec4f& best_fitted_line)
{
	/* OpenCV returns a normalized vector (vx, vy),
	   and (x0, y0), a point on the fitted line*/

	//best_fitted_line[0] -> vx
	//best_fitted_line[1] -> vy
	//best_fitted_line[2] -> x0
	//best_fitted_line[3] -> x1

	//The equation of this line is:
        //(x,y) = (x0,y0) + t * (vx, vy)

	std::vector<Point2f> points;
	for(size_t i = 0; i < lines.size(); i++) {
		points.push_back(Point2f(lines[i][0], lines[i][1]));
		points.push_back(Point2f(lines[i][2], lines[i][3]));
	}

	cv::fitLine(Mat(points), best_fitted_line ,CV_DIST_L2, 0, 0.01, 0.01);
}

void LaneDetector::mark_lane(cv::Mat& lane_mark_image, vector<Vec4f>& lines, Scalar line_color, Scalar dot_color, Scalar text_color)
{
	char text[50] = {'\0'};

	for(size_t i = 0; i < lines.size(); i++) {
		Vec4f line = lines[i];
		
		int mid_x = (line[0] + line[2]) / 2;
		int mid_y = (line[1] + line[3]) / 2;

		cv::line(lane_mark_image, Point(line[0], line[1]),
			Point(line[2], line[3]), line_color, 3, CV_AA);

		cv::circle(lane_mark_image, Point(line[0], line[1]), 3, dot_color, 2, CV_AA, 0);
		cv::circle(lane_mark_image, Point(line[2], line[3]), 3, dot_color, 2, CV_AA, 0);

		cv::putText(lane_mark_image, "1", Point(line[0], line[1] + 10),
			FONT_HERSHEY_COMPLEX_SMALL, 1, text_color);
		cv::putText(lane_mark_image, "2", Point(line[2], line[3] + 10),
			FONT_HERSHEY_COMPLEX_SMALL, 1, text_color);
	}  
}

void LaneDetector::homography_transform(cv::Mat& raw_image, cv::Mat& homograhy_image)
{
	cv::Mat H = (cv::Mat1d(3, 3) << -2.69663, -2.79935, 1201.62048,
		0.00661, -6.97268, 1599.55896,
		0.00007, -0.00868, 1.00000);

	warpPerspective(raw_image, homograhy_image, H, raw_image.size());
}

cv::Mat test_homography_transform(cv::Mat& rectified_image)
{
        cv::Mat H = (cv::Mat1d(3, 3) << -2.69663, -2.79935, 1201.62048,
                                         0.00661, -6.97268, 1599.55896,
                                         0.00007, -0.00868, 1.00000);

        cv::Mat homograhy_image;
        warpPerspective(rectified_image, homograhy_image, H, rectified_image.size());

        return homograhy_image;
}

void LaneDetector::image_to_gnd(float& pixel_x, float& pixel_y, float& gnd_x, float& gnd_y)
{
	gnd_x = pixel_x * (BOARD_WIDTH + 2) * BOARD_BOX_SIZE / IMAGE_WIDTH;
	gnd_y = pixel_y * (BOARD_HEIGHT + 2) * BOARD_BOX_SIZE / IMAGE_HEIGHT; 
}

void LaneDetector::gnd_to_image(float& pixel_x, float& pixel_y, float& gnd_x, float& gnd_y)
{
	pixel_x = gnd_x / (BOARD_WIDTH + 2) * BOARD_BOX_SIZE / IMAGE_WIDTH;
	pixel_y = gnd_y / (BOARD_HEIGHT + 2) * BOARD_BOX_SIZE / IMAGE_HEIGHT;
}

//Calculate the mid line by using white segments and yellow segments
void LaneDetector::shift_segment(vector<Vec4f>& lines, float shift_length)
{
	for(size_t i = 0; i < lines.size(); i++) {
		Point2f p1, p2;

		/* Swap if p1 is higher */
		if(p1.y > p2.y) {
			Point2f tmp;
			tmp = p1;
			p1 = p2;
			p2 = tmp;
		}

		//Construct the unit segment vector
		Point2f t_hat = p2 - p1;
		normalize(t_hat);

		//Construct the normal vector with shift length
		Point2f n_hat(shift_length * -t_hat.y, shift_length * t_hat.x);

		//Shift the segment
		p1 += n_hat;
		p2 += n_hat;
	}
}

void LaneDetector::lane_detect(cv::Mat& raw_image,
	vector<Vec4f>& outer_lines, vector<Vec4f>& inner_lines)
{
#if 0
	cv::Mat homography_image;
	homography_transform(raw_image, homography_image);

	homography_image.copyTo(raw_image);
#endif
	raw_image = test_homography_transform(raw_image);

	/* RGB to HSV */
	cv::cvtColor(raw_image, outer_hsv_image, COLOR_BGR2HSV);
	cv::cvtColor(raw_image, inner_hsv_image, COLOR_BGR2HSV);

	/* Binarization (Color thresholding) */
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

	cv::Mat outer_gray_image, inner_gray_image;
	cv::cvtColor(raw_image, outer_gray_image, CV_BGR2GRAY);
	cv::cvtColor(raw_image, inner_gray_image, CV_BGR2GRAY);

	/* Canny */
	cv::Mat preprocess_canny_image;
	cv::Canny(outer_gray_image, preprocess_canny_image, 0, 100, 3);

	/* Deliation */
	Mat deliation_element = cv::getStructuringElement(MORPH_RECT, Size(3, 3));
	cv::dilate(preprocess_canny_image, canny_image, deliation_element);

	/* Bitwise and (cany image and color binarization image) */
	cv::Mat outer_bitwise_and_image, inner_bitwise_and_image;

	cv::bitwise_and(outer_threshold_image, canny_image, outer_bitwise_and_image);
	cv::bitwise_and(inner_threshold_image, canny_image, inner_bitwise_and_image);

	/* Hough transform */
	cv::HoughLinesP(outer_bitwise_and_image, outer_lines, 1, CV_PI / 180, 80, 50, 5);	
	cv::HoughLinesP(inner_bitwise_and_image, inner_lines, 1, CV_PI / 180, 80, 50, 5);	

	raw_image.copyTo(lane_mark_image);
	mark_lane(lane_mark_image, outer_lines, Scalar(0, 0, 255), Scalar(255, 0, 0),  Scalar(0, 255, 0));
	mark_lane(lane_mark_image, inner_lines, Scalar(255, 0, 0), Scalar(0, 0, 255),  Scalar(0, 255, 0));

#ifdef NEW_MULTILINE_ALGORITHM
	shift_segment(outer_lines, +(w + lw / 2));
	shift_segment(inner_lines, -(w + ly / 2));

	/* Merge 2 segments */
	Vec4f mid_lines;
	mid_lines.reserve(outer_lines.size() + inner_lines.size());
	mid_lines.insert(mid_lines.end(), outer_lines.begin(), outer_lines.end());

	mid_lines.insert(mid_lines.end(), inner_lines.begin(), inner_lines.end());
#endif

#if 1
	Vec4f best_fitted_line;
	calculate_best_fittedline(inner_lines, best_fitted_line);

	/* Plot */
	float x, y;
	float t = 100;

	x = best_fitted_line[2] + t * best_fitted_line[0];
	y = best_fitted_line[3] + t * best_fitted_line[1];

	cv::line(
		lane_mark_image,
		Point(best_fitted_line[2], best_fitted_line[3]), Point(x, y),
		Scalar(0, 0, 255), 3, CV_AA
	);
#endif
}

bool LaneDetector::pose_estimate(vector<Vec4f>& lines, float& d, float& phi)
{
	if(lines.size() == 0) {
		putText(lane_mark_image, "Error: Cannot detect any segment", Point(15, 15),
                FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 0, 255));

		return false;
	} else {
		putText(lane_mark_image, "Self-driving mode on", Point(15, 15),
                FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255, 0));
	}

	Vec4f best_fitted_line;
        calculate_best_fittedline(lines, best_fitted_line);

	float t = 100;
	Point2f _p1, _p2;
	_p1.x = best_fitted_line[2] + t * best_fitted_line[0];
	_p1.y = best_fitted_line[3] + t * best_fitted_line[1];
	_p2.x = best_fitted_line[2] - t * best_fitted_line[0];
	_p2.y = best_fitted_line[3] - t * best_fitted_line[1];

	/* Set new origin */
	_p1.x -= SEMI_IMAGE_WIDTH;
	_p2.x -= SEMI_IMAGE_WIDTH;

	Point2f p1, p2;
	image_to_gnd(_p1.x, _p1.y, p1.x, p1.y);
	image_to_gnd(_p2.x, _p2.y, p2.x, p2.y);

	/* Swap if pi is higher */
	if(p1.y > p2.y) {
		Point2f tmp;
		tmp = p1;
		p1 = p2;
		p2 = tmp;
	}

	/* Estimate d */
	Point2f t_hat = p2 - p1;
	normalize(t_hat);

	/* Estimate phi */
	phi = rad_to_deg(atan2f(t_hat.y, t_hat.x)) - 90.0f;

	Point2f n_hat(-t_hat.y, t_hat.x); //normal vector

	float d1 = inner_product(n_hat, p1);
	float d2 = inner_product(n_hat, p2);

	d = (d1 + d2) / 2; //lateral displacement

#if DRAW_DEBUG_INFO
	cv::line(
		lane_mark_image,
		p1, p2,
		Scalar(0, 0, 255), 3, CV_AA
	);

	char debug_text[30];
	sprintf(debug_text, "d=%.1fcm,phi=%.1fdegree", d, phi);
	
	putText(lane_mark_image, debug_text, Point(15, 40),
		FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255, 0));
#endif	

	return true;
}
