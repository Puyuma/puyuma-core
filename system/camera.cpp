#include <sstream>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <highgui.h>
#include <cv.h>
#include <raspicam/raspicam_cv.h>
#include <yaml-cpp/yaml.h>

using namespace std;

bool load_intrinsic_calibration(string yaml_path,
	cv::Mat& camera_matrix, cv::Mat& distort_coefficients)
{
	try {
		YAML::Node yaml = YAML::LoadFile(yaml_path + "intrinsic_calibration.yaml");

		double intrinsic_array[9];
		double distort_array[5];

		for(int i = 0; i < 9; i++) {
			intrinsic_array[i] = yaml["camera_matrix"]["data"][i].as<double>();
		}

		camera_matrix = (cv::Mat1d(3, 3) << 
			intrinsic_array[0], intrinsic_array[1], intrinsic_array[2],
			intrinsic_array[3], intrinsic_array[4], intrinsic_array[5],
			intrinsic_array[6], intrinsic_array[7], intrinsic_array[8]
		);

		for(int i = 0; i < 5; i++) {
			distort_array[i] = yaml["distortion_coefficients"]["data"][i].as<double>();
		}

		distort_coefficients = (cv::Mat1d(1, 5) << 
			distort_array[0], distort_array[1],
			distort_array[2], distort_array[3], distort_array[4]
		);

		ROS_INFO("Camera matrix:\n[%f %f %f]\t\n[%f %f %f]\t\n[%f %f %f]",
			intrinsic_array[0], intrinsic_array[1], intrinsic_array[2],
			intrinsic_array[3], intrinsic_array[4], intrinsic_array[5],
			intrinsic_array[6], intrinsic_array[7], intrinsic_array[8]
		);

		ROS_INFO("Distortion coefficients:\n[%f %f %f %f %f]\n",
			distort_array[0], distort_array[1],
			distort_array[2], distort_array[3], distort_array[4]);
	} catch(...) {
		return false;
	}

	return true;
}

bool camera_setup(raspicam::RaspiCam_Cv& camera)
{
	camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
	camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	camera.set(CV_CAP_PROP_BRIGHTNESS, 50);
	camera.set(CV_CAP_PROP_CONTRAST, 50);
	camera.set(CV_CAP_PROP_SATURATION, 50);
	camera.set(CV_CAP_PROP_GAIN, 50);
	camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
	camera.set(CV_CAP_PROP_EXPOSURE, -1);
	camera.set(CV_CAP_PROP_WHITE_BALANCE_RED_V, 1);
	camera.set(CV_CAP_PROP_WHITE_BALANCE_BLUE_U, 1);

	return camera.open();
}
