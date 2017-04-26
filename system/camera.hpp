#ifndef CAMERA_H
#define CAMERA_H

#include <raspicam/raspicam_cv.h>

bool camera_setup(raspicam::RaspiCam_Cv& camera);
bool load_intrinsic_calibration(std::string yaml_path,
	cv::Mat& camera_matrix, cv::Mat& distort_coefficient);

#endif 
