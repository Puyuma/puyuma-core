#include <fstream>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include "lane_detector.hpp"
#include <yaml-cpp/yaml.h>

using namespace cv;

void on_trackbar(int, void *)
{
}

void LaneDetector::ros_node_setup()
{
        outer_threshold_img_publisher = node.advertise<sensor_msgs::Image>("xenobot/outer_threshold_image", 1000);
	outter_hough_img_publisher = node.advertise<sensor_msgs::Image>("xenobot/outer_hough_image", 1000);
        inner_threshold_img_publisher = node.advertise<sensor_msgs::Image>("xenobot/inner_threshold_image", 1000);
	inner_hough_img_publisher = node.advertise<sensor_msgs::Image>("xenobot/inner_hough_image", 1000);
        marked_image_publisher = node.advertise<sensor_msgs::Image>("xenobot/marked_image", 1000);
}

LaneDetector::LaneDetector(string yaml_path) :
	outer_threshold_h_min(0), outer_threshold_h_max(256),
	outer_threshold_s_min(0), outer_threshold_s_max(256),
	outer_threshold_v_min(0), outer_threshold_v_max(256),
	inner_threshold_h_min(0), inner_threshold_h_max(256),
	inner_threshold_s_min(0), inner_threshold_s_max(256),
	inner_threshold_v_min(0), inner_threshold_v_max(256)
{
	ros_node_setup();
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

bool LaneDetector::read_extrinsic_calibration(string yaml_path)
{
	try {
		YAML::Node yaml = YAML::LoadFile(yaml_path + "extrinsic_calibration.yaml");

		double homography_array[9];

		for(int i = 0; i < 9; i++) { 
			homography_array[i] = yaml["camera_matrix"]["data"][i].as<double>();
		}

		H = cv::Mat(3, 3, CV_64F, homography_array);
	} catch(...) {
		return false;
	}
}

bool LaneDetector::read_threshold_setting(string yaml_path)
{
	int outer_h_max, outer_s_max, outer_v_max;
	int outer_h_min, outer_s_min, outer_v_min;
	int inner_h_max, inner_s_max, inner_v_max;
	int inner_h_min, inner_s_min, inner_v_min;

	try {
		YAML::Node yaml = YAML::LoadFile(yaml_path + "intrinsic_calibration.yaml");

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

void LaneDetector::mark_lane(cv::Mat& lane_mark_image, vector<Vec4i>& lines, Scalar line_color, Scalar dot_color, Scalar text_color)
{
	char text[20] = {'\0'};

	for(size_t i = 0; i < lines.size(); i++) {
		Vec4i line = lines[i];
		
		int mid_x = (line[0] + line[2]) / 2;
		int mid_y = (line[1] + line[3]) / 2;

#if 0
		/* Ground coordinate estimation */
		Point3f ground_point = point_transform_image_to_ground(mid_x, mid_y);
		sprintf(text, "s%d(%f,%f)", i, ground_point.x, ground_point.y);
		putText(lane_mark_image, text, Point(mid_x, mid_y), FONT_HERSHEY_DUPLEX, 1, text_color);
#endif

		cv::line(lane_mark_image, Point(line[0], line[1]), Point(line[2], line[3]), line_color, 3, CV_AA);
	}  
}

/* Transform point from image plane (pixel) to ground point */
Point3f LaneDetector::point_transform_image_to_ground(int pixel_x, int pixel_y)
{
	cv::Point3f point_image(static_cast<float>(pixel_x), static_cast<float>(pixel_y), 1.0f);

	cv::Mat point_ground_mat = H * cv::Mat(point_image);
	cv::Point3f point_ground(point_ground_mat);

	ROS_INFO("%f %f %f", point_ground.x, point_ground.y, point_ground.z);

	point_ground.x /= point_ground.z;
	point_ground.y /= point_ground.z;
	point_ground.z = 0.0f;

	return point_ground;
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

void LaneDetector::lane_detect(cv::Mat& raw_image)
{
#if 1
	raw_image = test_homography_transform(raw_image);
#endif

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
	mark_lane(lane_mark_image, inner_lines, Scalar(255, 0, 0), Scalar(255, 0, 0),  Scalar(0, 255, 0));

	double outer_angle_max, outer_angle_min, outer_angle_best = 0;
	double inner_angle_max, inner_angle_min, inner_angle_best = 0;
	vector<double> outer_angle_list, inner_angle_list;

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
