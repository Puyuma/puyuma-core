#include <fstream>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>

#include <xenobot/segment.h>
#include <xenobot/segmentArray.h>

#include "lane_detector.hpp"
#include "xeno_math.hpp"

#include "controller.hpp"

#define DRAW_DEBUG_INFO 1

#define rad_to_deg(phi) (phi * 57.2957795)

using namespace cv;

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

		histogram_publisher =
			node.advertise<xenobot::segmentArray>("/xenobot/segment_data", 1000);;
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

void LaneDetector::line_fitting(vector<Vec4f>& lines, Vec4f& best_fitted_line)
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

#if 0
		cv::line(lane_mark_image, Point(line[0], line[1]),
			Point(line[2], line[3]), line_color, 3, CV_AA);
#endif
		cv::circle(lane_mark_image, Point(line[0], line[1]), 3, dot_color, 2, CV_AA, 0);
		cv::circle(lane_mark_image, Point(line[2], line[3]), 3, dot_color, 2, CV_AA, 0);
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

/* Convert image size from pixel to centimeter */
void LaneDetector::image_to_gnd(float& pixel_x, float& pixel_y, float& gnd_x, float& gnd_y)
{
	gnd_x = pixel_x * (((BOARD_WIDTH + 2) * BOARD_BOX_SIZE) / IMAGE_WIDTH);
	gnd_y = pixel_y * (((BOARD_HEIGHT + 2) * BOARD_BOX_SIZE) / IMAGE_HEIGHT); 
}

/* Convert image size from centimeter to pixel */
void LaneDetector::gnd_to_image(float& pixel_x, float& pixel_y, float& gnd_x, float& gnd_y)
{
	pixel_x = gnd_x * (IMAGE_WIDTH / ((BOARD_WIDTH + 2) * BOARD_BOX_SIZE));
	pixel_y = gnd_y * (IMAGE_HEIGHT / ((BOARD_HEIGHT + 2) * BOARD_BOX_SIZE));
}

//Calculate the mid line by using white segments and yellow segments
void LaneDetector::shift_segment(vector<Vec4f>& lines, float shift_length)
{
	for(size_t i = 0; i < lines.size(); i++) {
		Point2f p1(lines[i][0], lines[i][1]);
		Point2f p2(lines[i][2], lines[i][3]);

		/* Swap if p1 is higher */
		if(p1.y < p2.y) {
			Point2f tmp;
			tmp = p1;
			p1 = p2;
			p2 = tmp;
		}

		//Construct the unit segment vector
		Point2f t_hat = p2 - p1;
		normalize(t_hat);

		//Construct the normal vector with shift length
		Point2f n_hat_gnd(shift_length * -t_hat.y, shift_length * t_hat.x);
		Point2f n_hat_img;
		gnd_to_image(n_hat_img.x, n_hat_img.y, n_hat_gnd.x, n_hat_gnd.y); 		

		//Shift the segment
		//P1(x,y)
		lines[i][0] += n_hat_img.x;
		lines[i][1] += n_hat_img.y;
		//P2(x,y)
                lines[i][2] += n_hat_img.x;
		lines[i][3] += n_hat_img.y;
	}
}

/* 
 * Check the hough transformed line is at the right or left edge of the lane
 * Return false if the result is undetermined
 */
bool LaneDetector::edge_recognize(cv::Mat& threshold_image, Vec4f& lane_segment, int& result)
{
        Point2f p1, p2;
        p1.x = lane_segment[0];
        p1.y = lane_segment[1];
        p2.x = lane_segment[2];
        p2.y = lane_segment[3];

	/* Swap if p1 is higher */
	if(p1.y < p2.y) {
		Point2f tmp;
		tmp = p1;
		p1 = p2;
		p2 = tmp;
	}

	//Vector normalization
	Point2f t_hat = p2 - p1;
	normalize(t_hat);

	//Midpoint between p1 and p2
	Point2f midpoint = (p2 + p1) / 2;

	//Find the normal vector
	Point2f n_hat(-t_hat.y, t_hat.x);

	int left_cnt = 0, right_cnt = 0; //Left, right accumulator
	int x, y;

	for(int i = 0 ; i < 20; i++) {
		x = ceil(midpoint.x + n_hat.x * i);
		y = ceil(midpoint.y + n_hat.y * i);

		/* Do saturation if out of boundary */
		bound(0, IMAGE_WIDTH, x);
		bound(0, IMAGE_HEIGHT, y);

		if(threshold_image.at<uint8_t>(Point(x, y)) >= 255) {
			left_cnt++;
		}

		/* Do saturation if out of boundary */
		bound(0, IMAGE_WIDTH, x);
		bound(0, IMAGE_HEIGHT, y);

		x = ceil(midpoint.x - n_hat.x * i);
		y = ceil(midpoint.y - n_hat.y * i);

		if(threshold_image.at<uint8_t>(Point(x, y)) >= 255) {
			right_cnt++;
		}
	}

	ROS_INFO("L:%d R:%d", left_cnt, right_cnt);

	if(left_cnt > 14 && right_cnt > 14) {return false;}
	if(left_cnt < 14 && right_cnt < 14) {return false;}

	if(left_cnt > 14) {result = LEFT_EDGE;}
	if(right_cnt > 14) {result = RIGHT_EDGE;}

#if 1
	if(result == LEFT_EDGE) {
		putText(lane_mark_image, "l", midpoint, FONT_HERSHEY_COMPLEX_SMALL,
			1, Scalar(0, 0, 255));
	} else {
			putText(lane_mark_image, "r", midpoint, FONT_HERSHEY_COMPLEX_SMALL,
			1, Scalar(0, 0, 255));
	}
#endif

	return true;
}

void LaneDetector::lane_detect(cv::Mat& raw_image,
	vector<Vec4f>& outer_lines, vector<Vec4f>& inner_lines, Vec4f& predicted_lane)
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

#ifndef HISTOGRAM_FILTER

	//Single vote
	float d_i = 0, phi_i = 0;
	
	//Save every phi and d
	vector<float> phi_list;
	vector<float> d_list;

	/* 2D Histogram, size = row * column */
	float vote_box[HISTOGRAM_R_SIZE][HISTOGRAM_C_SIZE] = {0.0f};

	xenobot::segment segment;
	xenobot::segmentArray segments_msg;

	/* Generate the vote */
	for(size_t i = 0; i < outer_lines.size(); i++) {
		int left_or_right;
		bool ret = 
			edge_recognize(outer_threshold_image, outer_lines[i], left_or_right);

		if(ret == true) {
			//ROS_INFO("line is %s", left_or_right == LEFT_EDGE ? "left" : "right");
		} else {
			continue; //Skip
		}

		generate_vote(outer_lines[i], d_i, phi_i, left_or_right, WHITE);

		phi_list.push_back(phi_i);
		d_list.push_back(d_i);

		//Vote to ...
		int _i = (int)round((phi_i - PHI_MIN) / DELTA_PHI);
		int _j = (int)round((d_i - D_MIN) / DELTA_D);

		//Drop the vote if it is out of the boundary
		if(_i >= HISTOGRAM_R_SIZE || _j >= HISTOGRAM_C_SIZE) {
			continue;	
		}

		vote_box[_i][_j] += 1.0f; //Assume that every vote is equally important

		//ROS message
		segment.d = d_i;
		segment.phi = phi_i;
		segment.color = 0; //YELLOW
		segments_msg.segments.push_back(segment);
		//ROS_INFO("d:%f phi:%f", d_i, phi_i);
	}

	for(size_t i = 0; i < inner_lines.size(); i++) {
		int left_or_right;
		bool ret = 
			edge_recognize(inner_threshold_image, inner_lines[i], left_or_right);

		if(ret == true) {
			//ROS_INFO("line is %s", left_or_right == LEFT_EDGE ? "left" : "right");
		} else {
			continue; //Skip
		}

		generate_vote(inner_lines[i], d_i, phi_i, left_or_right, YELLOW);

		phi_list.push_back(phi_i);
		d_list.push_back(d_i);

		//Vote to ...
		int _i = (int)round((phi_i - PHI_MIN) / DELTA_PHI);
		int _j = (int)round((d_i - D_MIN) / DELTA_D);

		//Drop the vote if it is out of the boundary
		if(_i >= HISTOGRAM_R_SIZE || _j >= HISTOGRAM_C_SIZE) {
			continue;	
		}

		vote_box[_i][_j] += 1.0; //Assume that every vote is equally important

		//ROS message
		segment.d = d_i;
		segment.phi = phi_i;
		segment.color = 0; //YELLOW
		segments_msg.segments.push_back(segment);
		//ROS_INFO("d:%f phi:%f", d_i, phi_i);
	}

#endif

#if 0

	/* Now find who has be voted the most */
	int highest_vote_i = 0, highest_vote_j = 0;

	for(int i = 0; i < HISTOGRAM_R_SIZE; i++) {
		for(int j = 0; j < HISTOGRAM_C_SIZE; j++) {
			if(vote_box[i][j] > vote_box[highest_vote_i][highest_vote_j]) {
				//Change the highest vote index
				highest_vote_i = i;
				highest_vote_j = j;
			}
		}
	}

	/* Convert i, j to most possible phi and d range */
	float predicted_phi = DELTA_PHI * highest_vote_i + PHI_MIN;
	float predicted_d = DELTA_D * highest_vote_j + D_MIN;

	/* phi and d should in the range of predicted value +- delta/2 */
	float phi_up_bound = predicted_phi - DELTA_PHI / 2;
	float phi_low_bound = predicted_phi - DELTA_PHI / 2;
	float d_up_bound = predicted_d - DELTA_D / 2;
	float d_low_bound = predicted_d - DELTA_D / 2;

	float phi_mean = 0.0, d_mean = 0.0;
	int phi_sample_cnt, d_sample_cnt;

	/* Calculate the mean of the vote (TODO:Weighted average?) */
	for(size_t i = 0; i < phi_list.size(); i++) {
		phi_i = phi_list.at(i);

		if(phi_i >= phi_low_bound && phi_i <= phi_up_bound) {
			phi_mean += phi_i;
			phi_sample_cnt++;
		}
	}

	phi_mean /= (float)phi_sample_cnt;

	for(size_t i = 0; i < d_list.size(); i++) {
		d_i = d_list.at(i);

		if(d_i >= d_low_bound && d_i <= d_up_bound) {
			phi_mean += d_i;
			d_sample_cnt++;
		}
	}

	d_mean /= (float)d_sample_cnt;

	ROS_INFO("Histogram filter phi:%f | d:%f", phi_mean, d_mean);
#endif

#ifndef THIS_IS_OLD_FILTER
	shift_segment(outer_lines, -(W + L_W) / 2.0);
	shift_segment(inner_lines, +(W + L_Y) / 2.0);

	/* Merge 2 segment lists */
	vector<Vec4f> mid_lines;
	mid_lines.reserve(outer_lines.size() + inner_lines.size());
	mid_lines.insert(mid_lines.end(), outer_lines.begin(), outer_lines.end());
	mid_lines.insert(mid_lines.end(), inner_lines.begin(), inner_lines.end());

	/* Line fiting */
	line_fitting(mid_lines, predicted_lane);

	generate_vote(predicted_lane, d_i, phi_i, 0, 0);

	//ROS message
	segment.d = d_i;
	segment.phi = phi_i;
	segment.color = 1; //YELLOW
	segments_msg.segments.push_back(segment);
	//ROS_INFO("d:%f phi:%f", d_i, phi_i);

	histogram_publisher.publish(segments_msg);
#endif

#if 1
	if(mid_lines.size() == 0) {
		putText(lane_mark_image, "Error: Cannot detect any segment", Point(15, 15),
                FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 0, 255));

		//return false;
	} else {
		putText(lane_mark_image, "Self-driving mode on", Point(15, 15),
                FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255, 0));
	}

	/* Plot */
	float x,y, t = 100;

	x = predicted_lane[2] + t * predicted_lane[0];
	y = predicted_lane[3] + t * predicted_lane[1];

	cv::line(
		lane_mark_image,
		Point(predicted_lane[2], predicted_lane[3]), Point(x, y),
		Scalar(0, 0, 255), 3, CV_AA
	);
#endif
}

bool LaneDetector::pose_estimate(Vec4f& lane_segment, float& d, float& phi)
{
	int t = 100;
	Point2f _p1, _p2;
	_p1.x = lane_segment[2] + t * lane_segment[0];
	_p1.y = lane_segment[3] + t * lane_segment[1];
	_p2.x = lane_segment[2] - t * lane_segment[0];
	_p2.y = lane_segment[3] - t * lane_segment[1];

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

//Input a segment then generate a vote (d and phi)
bool LaneDetector::generate_vote(Vec4f& lane_segment, float& d,
	float& phi, int left_or_right, int color)
{
	Point2f _p1, _p2;
	_p1.x = lane_segment[0];
	_p1.y = lane_segment[1];
	_p2.x = lane_segment[2];
	_p2.y = lane_segment[3];

	/* Set new origin */
	_p1.x -= SEMI_IMAGE_WIDTH;
	_p2.x -= SEMI_IMAGE_WIDTH;

	Point2f p1, p2;
	image_to_gnd(_p1.x, _p1.y, p1.x, p1.y);
	image_to_gnd(_p2.x, _p2.y, p2.x, p2.y);

	/* Swap if pi is higher */
	if(p1.y < p2.y) {
		Point2f tmp;
		tmp = p1;
		p1 = p2;
		p2 = tmp;
	}

	/* Estimate d */
	Point2f t_hat = p2 - p1;
	normalize(t_hat);

	/* Estimate phi */
	phi = rad_to_deg(atan2f(t_hat.y, t_hat.x)) + 90.0f;

	Point2f n_hat(-t_hat.y, t_hat.x); //normal vector

	float d1 = inner_product(n_hat, p1);
	float d2 = inner_product(n_hat, p2);

	d = (d1 + d2) / 2; //lateral displacement

	d -= W / 2;

	if(color == WHITE) {
		if(left_or_right == RIGHT_EDGE) {
			d += L_W;
		}
	} else if(color == YELLOW) {
		if(left_or_right == LEFT_EDGE) {
			d += L_Y;
		}
	}

	return true;
}
