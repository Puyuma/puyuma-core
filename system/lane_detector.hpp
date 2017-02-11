#ifndef LANE_DETECTOR_H
#define LANE_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <cv_bridge/cv_bridge.h>

#include <xenobot/segment.h>
#include <xenobot/segmentArray.h>

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

enum SEGMENT_COLOR {WHITE, YELLOW, RED, UNKNOWN_COLOR};
enum {LEFT_EDGE, RIGHT_EDGE, UNKNOWN_SIDE};

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

class HsvThreshold {
	public:
	int h_min;
	int h_max;
	int s_min;
	int s_max;
	int v_min;
	int v_max;

	HsvThreshold(int h_min,int h_max, int s_min,int s_max, int v_min, int v_max):
	h_min(h_min),h_max(h_max),s_min(s_min),s_max(s_max),v_min(v_min),v_max(v_max){}

	void set_hsv(int h_min,int h_max, int s_min,int s_max, int v_min, int v_max)
	{
		this->h_min=h_min;
		this->h_max=h_max;
		this->s_min=s_min;
		this->s_max=s_max;
		this->v_min=v_min;
		this->v_max=v_max;
	}
};

class LaneDetector {
	private:
	string yaml_path;

	HsvThreshold inner_threshold;
	HsvThreshold outer_threshold;

	cv::Mat outer_hsv_image, outer_threshold_image;
	cv::Mat inner_hsv_image, inner_threshold_image;
	cv::Mat lane_mark_image, bird_view_image;
	cv::Mat canny_image;

	cv::Mat* H; //Homography matrix

	float roi_offset_x;
	float roi_offset_y;

	bool calibrate_mode;

	ros::NodeHandle node;
	ros::Publisher outer_threshold_img_publisher, outter_hough_img_publisher;
	ros::Publisher inner_threshold_img_publisher, inner_hough_img_publisher;
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
	bool generate_vote(segment_t& lane_segment, float& d, float& phi, int color);

	/* Visualization */
	void mark_lane(cv::Mat& lane_mark_image, vector<segment_t>& lines,
		Scalar line_color, Scalar dot_color, Scalar text_color);
	void publish_images(
		cv::Mat& lane_mark_image, cv::Mat& canny_image,
		cv::Mat& outer_threshold_image, cv::Mat& inner_threshold_image,
		cv::Mat& bird_view_image);
	void draw_segment_side(cv::Mat& lane_mark_image, vector<segment_t>& xeno_segments);
	void draw_bird_view_image(cv::Mat& original_image, cv::Mat& bird_view_image);
	void draw_region_of_interest(cv::Mat lane_mark_image);

	void send_sucess_visualize_image_thread(
		cv::Mat distorted_image, cv::Mat canny_image,
		cv::Mat outer_threshold_image, cv::Mat inner_threshold_image,
		vector<segment_t> outer_lines, vector<segment_t> inner_lines,
		float d, float phi, xenobot::segmentArray segments_msg);

	void send_failed_visualize_image_thread(
		cv::Mat distorted_image, cv::Mat canny_image,
		cv::Mat outer_threshold_image, cv::Mat inner_threshold_image);

	void send_visualize_image(
		cv::Mat& distorted_image, cv::Mat& canny_image,
		cv::Mat& outer_threshold_image, cv::Mat& inner_threshold_image,
		vector<segment_t>& outer_lines, vector<segment_t>& inner_lines,
		float& d, float& phi, xenobot::segmentArray& segments_msg);

	void send_visualize_image(cv::Mat& distorted_image, cv::Mat& canny_image,
		cv::Mat& outer_threshold_image, cv::Mat& inner_threshold_image);

	public:
	LaneDetector(string yaml_path, bool calibrate_mode);

	bool lane_estimate(cv::Mat& raw_image, float& final_d, float& final_phi);
	bool set_hsv(char color,int h_min,int h_max,int s_min,int s_max,int v_min,int v_max);
	bool save_thresholding_yaml();
	bool load_yaml_setting();
};

#endif
