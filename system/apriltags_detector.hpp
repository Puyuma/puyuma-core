#include <iostream>
#include <cstring>
#include <vector>

#include "opencv2/opencv.hpp"

// April tags detector and various families that can be selected by command line option
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag36h11.h"

class ApriltagsDetector
{

	AprilTags::TagDetector* m_tagDetector;
	AprilTags::TagCodes m_tagCodes;
	bool m_draw; // draw image and April tag detections?

public:

	// default constructor
	ApriltagsDetector();

	// changing the tag family
	void setTagCodes(string);
	void print_detection(AprilTags::TagDetection&) const ;
	int processImage(cv::Mat&, cv::Mat&);
}; // Demo

