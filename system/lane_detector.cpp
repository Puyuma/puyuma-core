#include <fstream>
#include <thread>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>

#include <std_msgs/Float32.h>
#include <xenobot/segment.h>
#include <xenobot/segmentArray.h>

#include "lane_detector.hpp"
#include "xeno_math.hpp"

#include "controller.hpp"

#define DRAW_DEBUG_INFO 1

#define rad_to_deg(phi) (phi * 57.2957795)
#define PI 3.141592

using namespace std;
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

	roi_offset_x = 0;
	roi_offset_y = IMAGE_HEIGHT / 2; 

	if(calibrate_mode == true) {
        	outer_threshold_img_publisher =
			node.advertise<sensor_msgs::Image>("xenobot/outer_threshold_image", 10);

       		canny_img_publisher =
			node.advertise<sensor_msgs::Image>("xenobot/canny_image", 10);

       		inner_threshold_img_publisher =
			node.advertise<sensor_msgs::Image>("xenobot/inner_threshold_image", 10);

	        marked_image_publisher =
			node.advertise<sensor_msgs::Image>("xenobot/marked_image", 10);

		bird_view_img_publisher = 
			node.advertise<sensor_msgs::Image>("xenobot/bird_view_image", 10);

		histogram_publisher =
			node.advertise<xenobot::segmentArray>("/xenobot/segment_data", 10);

		pose_d_publisher =
			node.advertise<std_msgs::Float32>("/xenobot/pose/d", 10);

		pose_phi_publisher =
			node.advertise<std_msgs::Float32>("/xenobot/pose/phi", 10);
	}
}

void LaneDetector::publish_images(
	cv::Mat& lane_mark_image, cv::Mat& canny_image,
	cv::Mat& outer_threshold_image, cv::Mat& inner_threshold_image,
	cv::Mat& bird_view_image)
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

	img_msg = cv_bridge::CvImage(std_msgs::Header(), "8UC1", inner_threshold_image).toImageMsg();
	inner_threshold_img_publisher.publish(img_msg);

	img_msg = cv_bridge::CvImage(std_msgs::Header(), "8UC3", bird_view_image).toImageMsg();
	bird_view_img_publisher.publish(img_msg);
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

		H = new cv::Mat(3, 3, CV_64F);

		ROS_INFO("Extrinsic matrix:");

			for(int i = 0; i < 3; i++) {
				for(int j = 0; j < 3; j++) {
					H->at<double>(i, j) =
				yaml["homography_matrix"]["data"][i * 3 + j].as<double>();
			}

			ROS_INFO("[%.5f %.5f %.5f]",
				H->at<double>(i, 0),
				H->at<double>(i, 1),
				H->at<double>(i, 2)
			);
		}
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

	if(!fp) {
		fp << out.c_str() << endl; //Write yaml
	} else {
		ROS_INFO("Failed to save the thresholding setting!");
	}
}

void LaneDetector::mark_lane(cv::Mat& lane_mark_image, vector<segment_t>& lines,
	Scalar line_color, Scalar dot_color, Scalar text_color)
{
	for(size_t i = 0; i < lines.size(); i++) {
		cv::line(lane_mark_image,
			Point(lines.at(i).untransformed.x1 + roi_offset_x,
				lines.at(i).untransformed.y1 + roi_offset_y),
			Point(lines.at(i).untransformed.x2 + roi_offset_x,
				lines.at(i).untransformed.y2 + roi_offset_y),
			line_color, 3, CV_AA
		);

		cv::circle(lane_mark_image,
			Point(lines.at(i).untransformed.x1 + roi_offset_x,
				lines.at(i).untransformed.y1 + roi_offset_y),
			3, dot_color, 2, CV_AA, 0
		);
		cv::circle(lane_mark_image,
			Point(lines.at(i).untransformed.x2 + roi_offset_x,
				lines.at(i).untransformed.y2 + roi_offset_y),
			3, dot_color, 2, CV_AA, 0
		);
	}  
}

void LaneDetector::draw_bird_view_image(cv::Mat& original_image, cv::Mat& bird_view_image)
{
	warpPerspective(original_image, bird_view_image, *H, original_image.size());
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

/* 
 * Check the hough transformed line is at the right or left edge of the lane
 * Return false if the result is undetermined
 */
bool LaneDetector::single_edge_recognize(cv::Mat& threshold_image, segment_t& lane_segment)
{
	/*
	 * XXX:This function is not completed!
	 *     The detectoion should be done on the homography plane!
	 */

        Point2f p1, p2;
        p1.x = lane_segment.untransformed.x1;
        p1.y = lane_segment.untransformed.y1;
        p2.x = lane_segment.untransformed.x2;
        p2.y = lane_segment.untransformed.y2;

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

	for(int i = 0 ; i < SIDE_DETECT_PIXEL_CNT; i++) {
		x = ceil(midpoint.x + n_hat.x * i);
		y = ceil(midpoint.y + n_hat.y * i);

		/* Do saturation if out of boundary */
		bound(0, IMAGE_WIDTH, x);
		bound(0, IMAGE_HEIGHT / 2, y);

		if(threshold_image.at<uint8_t>(Point(x, y)) >= 255) {
			left_cnt++;
		}

		x = ceil(midpoint.x - n_hat.x * i);
		y = ceil(midpoint.y - n_hat.y * i);

		/* Do saturation if out of boundary */
		bound(0, IMAGE_WIDTH, x);
		bound(0, IMAGE_HEIGHT / 2, y);

		if(threshold_image.at<uint8_t>(Point(x, y)) >= 255) {
			right_cnt++;
		}
	}

	//ROS_INFO("L:%d R:%d", left_cnt, right_cnt);

	if(left_cnt > SIDE_DETECT_THREDHOLD && right_cnt > SIDE_DETECT_THREDHOLD) {
		lane_segment.side = UNKNOWN_SIDE;
		return false;
	}
	if(left_cnt < SIDE_DETECT_THREDHOLD && right_cnt < SIDE_DETECT_THREDHOLD) {
		lane_segment.side = UNKNOWN_SIDE;
		return false;
	}

	if(left_cnt > SIDE_DETECT_THREDHOLD) {lane_segment.side = LEFT_EDGE;}
	if(right_cnt > SIDE_DETECT_THREDHOLD) {lane_segment.side = RIGHT_EDGE;}

	return true;
}

void LaneDetector::segments_side_recognize(vector<Vec4f>& cv_segments,
	vector<segment_t>& xeno_segments, cv::Mat& threshold_image)
{
	for(size_t i = 0; i < cv_segments.size(); i++) {
		segment_t new_xeno_segment;

		new_xeno_segment.untransformed.x1 = cv_segments[i][0];
		new_xeno_segment.untransformed.y1 = cv_segments[i][1];
		new_xeno_segment.untransformed.x2 = cv_segments[i][2];
		new_xeno_segment.untransformed.y2 = cv_segments[i][3];

		single_edge_recognize(threshold_image, new_xeno_segment);

		xeno_segments.push_back(new_xeno_segment);
	}
}

void LaneDetector::draw_segment_side(cv::Mat& lane_mark_image, vector<segment_t>& lane_segments)
{
	Point2f midpoint;

	for(size_t i = 0; i < lane_segments.size(); i++) {
		midpoint.x = (lane_segments.at(i).untransformed.x1 +
			lane_segments.at(i).untransformed.x2) / 2 + roi_offset_x;
		midpoint.y = (lane_segments.at(i).untransformed.y1 +
			lane_segments[i].untransformed.y2) / 2 + roi_offset_y;

		if(lane_segments.at(i).side == LEFT_EDGE) {
			putText(lane_mark_image, "l", midpoint, FONT_HERSHEY_COMPLEX_SMALL,
				1, Scalar(0, 0.7, 255));
		} else if(lane_segments.at(i).side == RIGHT_EDGE) {
			putText(lane_mark_image, "r", midpoint, FONT_HERSHEY_COMPLEX_SMALL,
				1, Scalar(0, 0.7, 255));
		}
	}	
}

void LaneDetector::find_region_of_interest(cv::Mat& original_image, cv::Mat& roi_image)
{
	cv::Rect region(0, IMAGE_HEIGHT / 2, IMAGE_WIDTH, IMAGE_HEIGHT / 2);

	roi_image = original_image(region);
}

void LaneDetector::draw_region_of_interest(cv::Mat lane_mark_image)
{
	Point2f p1(0, roi_offset_y);
	Point2f p2(IMAGE_WIDTH, roi_offset_y);
	Point2f p3(IMAGE_WIDTH, IMAGE_HEIGHT);
	Point2f p4(0, IMAGE_HEIGHT);

	cv::line(lane_mark_image, p1, p2, Scalar(255, 128, 0), 2, CV_AA);
	cv::line(lane_mark_image, p2, p3, Scalar(255, 128, 0), 2, CV_AA);
	cv::line(lane_mark_image, p3, p4, Scalar(255, 128, 0), 2, CV_AA);
	cv::line(lane_mark_image, p4, p1, Scalar(255, 128, 0), 2, CV_AA);

	putText(lane_mark_image, "Region of interest",
		Point(roi_offset_x + 10, IMAGE_HEIGHT - 10), 
		FONT_HERSHEY_COMPLEX_SMALL, 0.7, Scalar(255, 128, 0)
	);
}

void LaneDetector::segment_homography_transform(vector<segment_t>& lines)
{
	for(size_t i = 0; i < lines.size(); i++) {
		vector<Point2f> segment;
		vector<Point2f> segment_transformed;

		Point2f point;
		point.x = lines.at(i).untransformed.x1 + roi_offset_x;
		point.y = lines.at(i).untransformed.y1 + roi_offset_y;
		segment.push_back(point);
		point.x = lines.at(i).untransformed.x2 + roi_offset_x;
		point.y = lines.at(i).untransformed.y2 + roi_offset_y;
		segment.push_back(point);

		perspectiveTransform(segment, segment_transformed, *H);

		lines.at(i).x1 = segment_transformed.at(0).x;
		lines.at(i).y1 = segment_transformed.at(0).y;
		lines.at(i).x2 = segment_transformed.at(1).x;
		lines.at(i).y2 = segment_transformed.at(1).y;

		//ROS_INFO("p1(%f,%f) p2(%f,%f)", lines[i][0], lines[i][1], lines[i][2], lines[i][3]);
	}
}

void LaneDetector::send_sucess_visualize_image_thread(
	/* Images */
	cv::Mat distorted_image, cv::Mat canny_image,
	cv::Mat outer_threshold_image, cv::Mat inner_threshold_image,
	/* Segments */
	vector<segment_t> outer_lines, vector<segment_t> inner_lines,
	/* Pose */
	float d, float phi, xenobot::segmentArray segments_msg)
{
	cv::Mat lane_mark_image = distorted_image;

#if 1
	/* Bird view image */
	cv::Mat bird_view_image;
	draw_bird_view_image(distorted_image, bird_view_image);
#endif

	/* Lane mark image */
	mark_lane(lane_mark_image, outer_lines, Scalar(0, 0, 255), Scalar(255, 0, 0),  Scalar(0, 255, 0));
	mark_lane(lane_mark_image, inner_lines, Scalar(255, 0, 0), Scalar(0, 0, 255),  Scalar(0, 255, 0));
	draw_segment_side(lane_mark_image, outer_lines);
	draw_segment_side(lane_mark_image, inner_lines);
	draw_region_of_interest(lane_mark_image);

	/* Draw car status */
	putText(lane_mark_image, "Self-driving mode on", Point(10, 15),
		FONT_HERSHEY_COMPLEX_SMALL, 0.7, Scalar(0, 255, 0));

	char status_text[60];
	sprintf(status_text, "d=%.1fcm,phi=%.1fdegree", d, phi);
	putText(lane_mark_image, status_text, Point(10, 40),
		FONT_HERSHEY_COMPLEX_SMALL, 0.7, Scalar(0, 255, 0));

	publish_images(
		lane_mark_image, canny_image,
		outer_threshold_image, inner_threshold_image,
		bird_view_image
	);

	histogram_publisher.publish(segments_msg);

	std_msgs::Float32 pose_msg;
	pose_msg.data = d;
	pose_d_publisher.publish(pose_msg);
	pose_msg.data = phi;
	pose_phi_publisher.publish(pose_msg);

	ROS_INFO("phi:%f | d:%f", phi, d);
}

void LaneDetector::send_failed_visualize_image_thread(
	/* Images */
	cv::Mat distorted_image, cv::Mat canny_image,
	cv::Mat outer_threshold_image, cv::Mat inner_threshold_image)
{
	cv::Mat lane_mark_image = distorted_image;

#if 1
	/* Bird view image */
	cv::Mat bird_view_image;
	draw_bird_view_image(distorted_image, bird_view_image);
#endif

	draw_region_of_interest(lane_mark_image);

	/* Draw car status */
	putText(lane_mark_image, "Failed to estimate the lane", Point(10, 15),
		FONT_HERSHEY_COMPLEX_SMALL, 0.7, Scalar(0, 0, 255));

	publish_images(
		lane_mark_image, canny_image,
		outer_threshold_image, inner_threshold_image,
		bird_view_image
	);
}


void LaneDetector::send_visualize_image(
	/* Images */
	cv::Mat& distorted_image, cv::Mat& canny_image,
	cv::Mat& outer_threshold_image, cv::Mat& inner_threshold_image,
	/* Segments */
	vector<segment_t>& outer_lines, vector<segment_t>& inner_lines,
	/* Pose */
	float& d, float& phi, xenobot::segmentArray& segments_msg)
{
	thread send_image_thread(
		&LaneDetector::send_sucess_visualize_image_thread,
		this,
		distorted_image,
		canny_image,
		outer_threshold_image,
		inner_threshold_image,
		outer_lines, inner_lines,
		d, phi, segments_msg
	);

	send_image_thread.detach();
}

void LaneDetector::send_visualize_image(
	/* Images */
	cv::Mat& distorted_image, cv::Mat& canny_image,
	cv::Mat& outer_threshold_image, cv::Mat& inner_threshold_image)
{
	thread send_image_thread(
		&LaneDetector::send_failed_visualize_image_thread,
		this,
		distorted_image,
		canny_image,
		outer_threshold_image,
		inner_threshold_image
	);

	send_image_thread.detach();
}

bool LaneDetector::lane_estimate(cv::Mat& raw_image, float& final_d, float& final_phi)
{
	/* Find the region of interest for lane */
	cv::Mat roi_image;
	find_region_of_interest(raw_image, roi_image);

	/* RGB to HSV */
	cv::cvtColor(roi_image, outer_hsv_image, COLOR_BGR2HSV);
	cv::cvtColor(roi_image, inner_hsv_image, COLOR_BGR2HSV);

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
	cv::cvtColor(roi_image, outer_gray_image, CV_BGR2GRAY);
	cv::cvtColor(roi_image, inner_gray_image, CV_BGR2GRAY);

	/* Canny */
	cv::Mat preprocess_canny_image;
	cv::Canny(outer_gray_image, preprocess_canny_image, CANNY_THRESHOLD_1,
		CANNY_THRESHOLD_2, 3);

	/* Deliation */
	Mat deliation_element = cv::getStructuringElement(MORPH_RECT, Size(3, 3));
	cv::dilate(preprocess_canny_image, canny_image, deliation_element);

	/* Bitwise and (cany image and color binarization image) */
	cv::Mat outer_bitwise_and_image, inner_bitwise_and_image;

	cv::bitwise_and(outer_threshold_image, canny_image, outer_bitwise_and_image);
	cv::bitwise_and(inner_threshold_image, canny_image, inner_bitwise_and_image);

	/* Hough transform */
	vector<Vec4f> outer_cv_lines, inner_cv_lines;
	cv::HoughLinesP(outer_bitwise_and_image, outer_cv_lines, 1, CV_PI / 180, HOUGH_THRESHOLD, 50, 5);	
	cv::HoughLinesP(inner_bitwise_and_image, inner_cv_lines, 1, CV_PI / 180, HOUGH_THRESHOLD, 50, 5);	

	/* Convert to xeno segment and do the side detection */
	vector<segment_t> outer_xeno_lines, inner_xeno_lines;
	segments_side_recognize(outer_cv_lines, outer_xeno_lines, outer_threshold_image);
	segments_side_recognize(inner_cv_lines, inner_xeno_lines, inner_threshold_image);

	if(outer_xeno_lines.size() == 0 && inner_xeno_lines.size() == 0) {
		send_visualize_image(
			raw_image,
			canny_image,
			outer_threshold_image,
			inner_threshold_image
		);

		ROS_INFO("Failed to estimate the lane [No segment found]");

		return false;
	}

	/*Perspective transformation */
	segment_homography_transform(outer_xeno_lines);
	segment_homography_transform(inner_xeno_lines);

	/* Histogram filtering */

	//Vote count
	int vote_count = 0;
	
	/* 2D Histogram, size = row * column */
	float vote_box[HISTOGRAM_R_SIZE][HISTOGRAM_C_SIZE] = {0.0f};

	xenobot::segment segment;
	xenobot::segmentArray segments_msg;

	/* Generate the vote */
	for(size_t i = 0; i < outer_xeno_lines.size(); i++) {
		float d_i, phi_i;

		if(generate_vote(outer_xeno_lines.at(i), d_i, phi_i, WHITE) == false) {
			outer_xeno_lines.erase(outer_xeno_lines.begin() + i);
			continue;
		}

		outer_xeno_lines.at(i).d = d_i;
		outer_xeno_lines.at(i).phi = phi_i;

		vote_count++;

		//Vote to ...
		int _i = (int)round((phi_i - PHI_MIN) / DELTA_PHI);
		int _j = (int)round((d_i - D_MIN) / DELTA_D);

		//Drop the vote if it is out of the boundary
		if(_i >= HISTOGRAM_R_SIZE || _j >= HISTOGRAM_C_SIZE) {
			outer_xeno_lines.erase(outer_xeno_lines.begin() + i) ;
			continue;	
		}

		vote_box[_i][_j] += 1.0f; //Assume that every vote is equally important

#if 1
		//ROS message
		segment.d = d_i;
		segment.phi = phi_i;
		segment.color = 0; //WHITE
		segments_msg.segments.push_back(segment);
		//ROS_INFO("d:%f phi:%f", d_i, phi_i);
#endif
	}

	for(size_t i = 0; i < inner_xeno_lines.size(); i++) {
		float d_i, phi_i;

		if(generate_vote(inner_xeno_lines.at(i), d_i, phi_i, YELLOW) == false) {
			inner_xeno_lines.erase(inner_xeno_lines.begin() + i);
			continue;
		}

		inner_xeno_lines.at(i).d = d_i;
		inner_xeno_lines.at(i).phi = phi_i;

		vote_count++;

		//Vote to ...
		int _i = (int)round((phi_i - PHI_MIN) / DELTA_PHI);
		int _j = (int)round((d_i - D_MIN) / DELTA_D);

		//Drop the vote if it is out of the boundary
		if(_i >= HISTOGRAM_R_SIZE || _j >= HISTOGRAM_C_SIZE) {
			inner_xeno_lines.erase(inner_xeno_lines.begin() + i);
			continue;	
		}

		vote_box[_i][_j] += 1.0; //Assume that every vote is equally important

#if 1
		//ROS message
		segment.d = d_i;
		segment.phi = phi_i;
		segment.color = 0; //WHITE
		segments_msg.segments.push_back(segment);
		//ROS_INFO("d:%f phi:%f", d_i, phi_i);
#endif
	}

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

	if(vote_box[highest_vote_i][highest_vote_j] < HISTOGRAM_FILTER_THRESHOLD) {
		send_visualize_image(
			raw_image,
			canny_image,
			outer_threshold_image,
			inner_threshold_image
		);

		ROS_INFO("Failed to estimate the lane [Less than threshold value]");

		return false;
	}

	/* Convert i, j to most possible phi and d range */
	float predicted_phi = DELTA_PHI * highest_vote_i + PHI_MIN;
	float predicted_d = DELTA_D * highest_vote_j + D_MIN;

	//ROS_INFO("Predicted phi:%f, d:%f", predicted_phi, predicted_d);

	/* phi and d should in the range of predicted value +- delta/2 */
	float phi_up_bound = predicted_phi + DELTA_PHI / 2;
	float phi_low_bound = predicted_phi - DELTA_PHI / 2;
	float d_up_bound = predicted_d + DELTA_D / 2;
	float d_low_bound = predicted_d - DELTA_D / 2;

	float phi_mean = 0.0, d_mean = 0.0;
	int phi_sample_cnt = 0, d_sample_cnt = 0;

	/* Calculate the mean of the vote (TODO:Weighted average?) */
	for(size_t i = 0; i < inner_xeno_lines.size(); i++) {
		float phi_i, d_i;
		phi_i = inner_xeno_lines.at(i).phi;
		d_i = inner_xeno_lines.at(i).d;

		if(phi_i >= phi_low_bound && phi_i <= phi_up_bound) {
			phi_mean += phi_i;
			phi_sample_cnt++;
		}

		if(d_i >= d_low_bound && d_i <= d_up_bound) {
			d_mean += d_i;
			d_sample_cnt++;
		}
	}

	for(size_t i = 0; i < outer_xeno_lines.size(); i++) {
		float phi_i, d_i;
		phi_i = outer_xeno_lines.at(i).phi;
		d_i = outer_xeno_lines.at(i).d;

		if(phi_i >= phi_low_bound && phi_i <= phi_up_bound) {
			phi_mean += phi_i;
			phi_sample_cnt++;
		}

		if(d_i >= d_low_bound && d_i <= d_up_bound) {
			d_mean += d_i;
			d_sample_cnt++;
		}
	}

	if(phi_sample_cnt == 0 || d_sample_cnt == 0) {
		send_visualize_image(
			raw_image,
			canny_image,
			outer_threshold_image,
			inner_threshold_image
		);

		ROS_INFO("Failed to estimate the lane [Sample count equals zero]");

		return false;
	}

	phi_mean /= (float)phi_sample_cnt;
	d_mean /= (float)d_sample_cnt;

	final_d = d_mean;
	final_phi = phi_mean;

#if 1
	//ROS message
	segment.d = d_mean;
	segment.phi = phi_mean;
	segment.color = 2; //RED
	segments_msg.segments.push_back(segment);
#endif

	send_visualize_image(
		raw_image,
		canny_image,
		outer_threshold_image,
		inner_threshold_image,
		outer_xeno_lines, inner_xeno_lines,
		final_d, final_phi, segments_msg
	);

	return true;
}


//Input a segment then generate a vote (d and phi)
bool LaneDetector::generate_vote(segment_t& lane_segment, float& d,
	float& phi, int color)
{
	if(lane_segment.side == UNKNOWN_SIDE) {
		return false;
	}

	Point2f _p1, _p2;
	_p1.x = lane_segment.x1;
	_p1.y = lane_segment.y1;
	_p2.x = lane_segment.x2;
	_p2.y = lane_segment.y2;

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
	phi = atan2f(t_hat.y, t_hat.x) + PI / 2;

	Point2f n_hat(-t_hat.y, t_hat.x); //normal vector

	Point2f offset_vector = n_hat;

	float steady_bias = 4; //cm

	if(color == WHITE) {
		if(lane_segment.side == RIGHT_EDGE) {
			offset_vector *= -(W / 2) - L_W;
		} else {
			offset_vector *= -(W / 2);
		}
	} else if(color == YELLOW) {
		if(lane_segment.side == LEFT_EDGE) {
			offset_vector *= +(W / 2) + L_Y + steady_bias;
		} else {
			offset_vector *= +(W / 2) + steady_bias;
		}
	}

	p1 += offset_vector;
	p2 += offset_vector;

	p1.x -= CAMERA_TO_CENTER * sin(phi);
	p1.y -= CAMERA_TO_CENTER * cos(phi);
	p2.x -= CAMERA_TO_CENTER * sin(phi);
	p2.y -= CAMERA_TO_CENTER * cos(phi);

	float d1 = inner_product(n_hat, p1);
	float d2 = inner_product(n_hat, p2);

	d = (d1 + d2) / 2; //lateral displacement

	//TODO:referive the geometry formulas!
	d *= -1;

	phi = rad_to_deg(phi);

	return true;
}
