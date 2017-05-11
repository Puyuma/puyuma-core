#ifndef LANE_DETECTOR_H
#define LANE_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <cv_bridge/cv_bridge.h>

#include <hsv_threshold.h>

//ros generate msg,srv file
#include <xenobot/segment.h>
#include <xenobot/segmentArray.h>
#include <xenobot/SendHsv.h>
#include "controller.hpp"

using namespace std;

#define IMAGE_WIDTH 320.0 //pixel
#define IMAGE_HEIGHT 240.0

#define SEMI_IMAGE_WIDTH ((int)IMAGE_WIDTH / 2)
#define SEMI_IMAGE_HEIGHT ((int)IMAGE_HEIGHT/ 2)

/* Checkboard parameters */
#define BOARD_BOX_SIZE 3.1 //cm
#define BOARD_WIDTH 6.0
#define BOARD_HEIGHT 4.0

/* Lane parameters */
#define L_W 5.0 //cm
#define L_Y 2.5
#define L_R 2.4
#define W 16.5

/* Car parameter */
#define CAMERA_TO_CENTER 10.0 //cm

/* Historgram filter parameters */
#define DELTA_PHI 2.0 //degree
#define DELTA_D 2.0 //cm
#define PHI_MIN (-90.0)
#define PHI_MAX (+90.0)
#define D_MIN (-25.0)
#define D_MAX (+25.0)
#define HISTOGRAM_R_SIZE (int)((PHI_MAX - PHI_MIN) / DELTA_PHI) //phi
#define HISTOGRAM_C_SIZE (int)((D_MAX - D_MIN) / DELTA_D) //d
#define HISTOGRAM_FILTER_THRESHOLD (vote_count / 3)

/* Lane detector parameter */
#define CANNY_THRESHOLD_1 50
#define CANNY_THRESHOLD_2 200
#define HOUGH_THRESHOLD 50
#define SIDE_DETECT_PIXEL_CNT 20
#define SIDE_DETECT_THREDHOLD 14

using namespace std;
using namespace cv;

enum SEGMENT_COLOR {WHITE, YELLOW, RED, RESULT, UNKNOWN_COLOR};
enum SEGMENT_EDGE {LEFT_EDGE, RIGHT_EDGE, UNKNOWN_SIDE};

enum IntersectionMode {
	GO_STRAIGHT_MODE,
	TURN_LEFT_MODE,
	TURN_RIGHT_MODE,
	STOP_LINE_MODE
};

typedef struct {
	int side; //Left or right?

	struct {
		float x1;
		float y1;

		float x2;
		float y2;	
	} untransformed;

	float x1;
	float y1;

	float x2;
	float y2;

	float d;
	float phi;
} segment_t;

class TurnRange {
	public:
	TurnRange();
	void set_range_self(int, int);
	void set_range_intersection(Direction, int, int, int);

	float predicted_phi;
	float predicted_d;
	float phi_up_bound;
	float phi_low_bound;
	float d_up_bound;
	float d_low_bound;
};

class LaneDetector {
	private:
	string yaml_path;

	vector<string> color_title = {"outer", "inner", "red"};
	const char color_char[3] = {'w', 'y', 'r'};

	HsvThreshold inner_threshold;
	HsvThreshold outer_threshold;
	HsvThreshold red_threshold;

	cv::Mat raw_image;
	vector<segment_t> outer_xeno_lines, inner_xeno_lines, red_xeno_lines;
	double d, phi;
	xenobot::segmentArray segments_msg;

	cv::Mat* H; //Homography matrix

	float roi_offset_x;
	float roi_offset_y;
	int success_estimate;

	bool calibrate_mode;
	enum ControllerMode mode;
	enum IntersectionMode intersection_mode;
	enum Direction direction;

	ros::Publisher raw_img_publisher;
	ros::Publisher outer_threshold_img_publisher, outter_hough_img_publisher;
	ros::Publisher inner_threshold_img_publisher, inner_hough_img_publisher;
	ros::Publisher red_threshold_img_publisher;
	ros::Publisher canny_img_publisher;
	ros::Publisher marked_image_publisher;
	ros::Publisher bird_view_img_publisher;
	ros::Publisher histogram_publisher;
	ros::Publisher pose_d_publisher;
	ros::Publisher pose_phi_publisher;

	bool read_threshold_setting(string yaml_path);
	bool read_extrinsic_calibration(string yaml_path);
	Point3f point_transform_image_to_ground(int pixel_x, int pixel_y);
	void append_yaml_data(YAML::Emitter& yaml_handler, string key, int value);
	void image_to_gnd(float& pixel_x, float& pixel_y, float& gnd_x, float& gnd_y);
	void gnd_to_image(float& pixel_x, float& pixel_y, float& gnd_x, float& gnd_y);
	bool single_edge_recognize(cv::Mat& threshold_image, segment_t& lane_segment);
	void find_region_of_interest(cv::Mat& original_image, cv::Mat& roi_image);
	void segment_homography_transform(vector<segment_t>& lines);
	void segments_side_recognize(vector<Vec4f>& cv_segments,
		vector<segment_t>& xeno_segments, cv::Mat& threshold_image);

	/* Visualization */
	void mark_lane(cv::Mat& lane_mark_image, vector<segment_t>& lines,
		Scalar line_color, Scalar dot_color, Scalar text_color);
	void draw_segment_side(cv::Mat& lane_mark_image, vector<segment_t>& xeno_segments);
	void draw_bird_view_image(cv::Mat& original_image, cv::Mat& bird_view_image);
	void draw_region_of_interest(cv::Mat lane_mark_image);
	void check_cross_intersection(int, int);
	void send_lanemark_image_thread(int case_type);
	void send_visualize_image_thread(
		cv::Mat distorted_image, cv::Mat canny_image,
		cv::Mat outer_threshold_image, cv::Mat inner_threshold_image,cv::Mat red_threshold_image);
	void send_lanemark_image(int case_type);
	void send_visualize_image(cv::Mat& distorted_image, cv::Mat& canny_image, cv::Mat& outer_threshold_image,
		cv::Mat& inner_threshold_image, cv::Mat& red_threshold_image);
	void publish_images(
		cv::Mat& distorted_image, cv::Mat& canny_image, cv::Mat& outer_threshold_image,
		cv::Mat& inner_threshold_image, cv::Mat& red_threshold_image, cv::Mat& bird_view_image);


	/* lane estimation */
	bool image_preprocess(cv::Mat& raw_image);
	bool generate_vote(segment_t& lane_segment, float& d, float& phi, int color);
	bool find_highest_vote(int& highest_vote_i, int& highest_vote_j, xenobot::segmentArray& segments_msg);
	bool histogram_filter(float& filtered_phi, float& filtered_d);

	public:
	LaneDetector(string yaml_path, bool calibrate_mode);
	bool lane_estimate(cv::Mat& raw_image, float& final_d, float& final_phi);
	void set_mode(enum ControllerMode mode);
	void set_direction(enum Direction direction);
	/* HSV parameter setting */
	bool set_hsv(char color,int h_min,int h_max,int s_min,int s_max,int v_min,int v_max);
	bool save_thresholding_yaml();
	bool load_yaml_setting();
	void send_hsv(char color,xenobot::SendHsv::Response &res);
	HsvThreshold* get_threshold(char color);	

	int forwarding;
};

#endif
