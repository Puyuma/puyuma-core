#ifndef LANE_DETECTOR_H
#define LANE_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <cv_bridge/cv_bridge.h>

#define IMAGE_WIDTH 640.0 //pixel
#define IMAGE_HEIGHT 480.0

#define SEMI_IMAGE_WIDTH 320.0
#define SEMI_IMAGE_HEIGHT 240.0

/* Checkboard parameters */
#define BOARD_BOX_SIZE 3.1 //cm
#define BOARD_WIDTH 6.0
#define BOARD_HEIGHT 4.0

/* Lane parameters */
#define L_W 5.0 //cm
#define L_Y 2.5
#define W 16.5

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
#define HOUGH_THRESHOLD 100
#define SIDE_DETECT_PIXEL_CNT 20
#define SIDE_DETECT_THREDHOLD 14

using namespace std;
using namespace cv;

enum SEGMENT_COLOR {WHITE, YELLOW, RED, UNKNOWN_COLOR};
enum {LEFT_EDGE, RIGHT_EDGE, UNKNOWN_SIDE};

typedef struct {
	int side; //Left or right?

	float x1;
	float y1;

	float x2;
	float y2;
} segment_t;

class LaneDetector {
	private:
	string yaml_path;

	int outer_threshold_h_min, outer_threshold_h_max;
	int outer_threshold_s_min, outer_threshold_s_max;
	int outer_threshold_v_min, outer_threshold_v_max;

	int inner_threshold_h_min, inner_threshold_h_max;
	int inner_threshold_s_min, inner_threshold_s_max;
	int inner_threshold_v_min, inner_threshold_v_max;

	cv::Mat outer_hsv_image, outer_threshold_image, outer_hough_image;
	cv::Mat inner_hsv_image, inner_threshold_image, inner_hough_image;
	cv::Mat lane_mark_image, bird_view_image;
	cv::Mat canny_image;

	cv::Mat H; //Homography matrix

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

	bool read_threshold_setting(string yaml_path);
	bool read_extrinsic_calibration(string yaml_path);
	void mark_lane(cv::Mat& lane_mark_image, vector<Vec4f>& lines, Scalar line_color, Scalar dot_color, Scalar text_color);
	Point3f point_transform_image_to_ground(int pixel_x, int pixel_y);
	void append_yaml_data(YAML::Emitter& yaml_handler, string key, int value);
	void line_fitting(vector<Vec4f>& lines, Vec4f& best_fitted_line);
	void image_to_gnd(float& pixel_x, float& pixel_y, float& gnd_x, float& gnd_y);
	void gnd_to_image(float& pixel_x, float& pixel_y, float& gnd_x, float& gnd_y);
	bool single_edge_recognize(cv::Mat& threshold_image, segment_t& lane_segment);
	void draw_segment_side(cv::Mat& lane_mark_image, vector<segment_t>& xeno_segments);
	void draw_bird_view_image(cv::Mat& bird_view_image,
		vector<segment_t>& outer_segments, Scalar outer_segment_color,
		vector<segment_t>& inner_segments, Scalar inner_segment_color);
	void find_region_of_interest(cv::Mat& original_image, cv::Mat& roi_image);
	void draw_region_of_interest(cv::Mat lane_mark_image);
	void segment_homography_transform(vector<segment_t>& lines);
	void segments_side_recognize(vector<Vec4f>& cv_segments,
		vector<segment_t>& xeno_segments, cv::Mat& threshold_image);
	bool generate_vote(segment_t& lane_segment, float& d, float& phi, int color);

	public:
	LaneDetector(string yaml_path, bool calibrate_mode);

	bool lane_estimate(cv::Mat& raw_image, float& final_d, float& final_phi);
	void publish_images();
	void set_hsv(
	       double outer_h_max, double outer_h_min, double outer_s_max,
	       double outer_s_min, double outer_v_max, double outer_v_min,
	       double inner_h_max, double inner_h_min, double inner_s_max,
	       double inner_s_min, double inner_v_max, double inner_v_min
	);

	void save_thresholding_yaml();
	bool load_yaml_setting();
};

#endif
