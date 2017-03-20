#include "apriltags_detector.hpp"
#define WINDOWNAME "Apriltags Image"

ApriltagsDetector::ApriltagsDetector() :
	m_draw(false),
//	m_tagDetector(NULL),
	m_tagCodes(AprilTags::tagCodes36h11)
{    
	m_tagDetector = new AprilTags::TagDetector(m_tagCodes);
    if (m_draw) 
	{
		cv::namedWindow(WINDOWNAME, 1);
    }

}

// changing the tag family
void ApriltagsDetector::setTagCodes(string s)
{
	if (s=="16h5")
	{
		m_tagCodes = AprilTags::tagCodes16h5;
	}
	else if (s=="25h7")
	{
		m_tagCodes = AprilTags::tagCodes25h7;
	}
	else if (s=="25h9")
	{
		m_tagCodes = AprilTags::tagCodes25h9;
	}
	else if (s=="36h9")
	{
		m_tagCodes = AprilTags::tagCodes36h9;
	}
	else if (s=="36h11")
	{
		m_tagCodes = AprilTags::tagCodes36h11;
	}
	else
	{
		cout << "Invalid tag family specified" << endl;
		exit(1);
	}
}

void ApriltagsDetector::print_detection(AprilTags::TagDetection& detection) const
{
	cout << "  Id: " << detection.id
	     << " (Hamming: " << detection.hammingDistance << ")";
}

int ApriltagsDetector::processImage(cv::Mat& image, cv::Mat& image_gray)
{
	cv::cvtColor(image, image_gray, CV_BGR2GRAY);
	vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);
	// print out each detection
	cout << detections.size() << " tags detected:" << endl;
	for (int i=0; i<detections.size(); i++)
	{
		print_detection(detections[i]);
	}
	// show the current image including any detections
	if (m_draw)
	{
		for (int i=0; i<detections.size(); i++)
		{
			// also highlight in the image
			detections[i].draw(image);
		}
		imshow(WINDOWNAME, image); // OpenCV call
	}
	
	if(detections.size() > 0)
		return detections[0].id;
	else
		return -1;
}

