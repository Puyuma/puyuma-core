#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <raspicam/raspicam_cv.h>
#include <cv_bridge/cv_bridge.h>
#include <camera.hpp>

#include <xenobot/SetAtt.h>

using namespace cv;

raspicam::RaspiCam_Cv camera;

//OpenCV only for cameras attribute
int atts[] = {
	CV_CAP_PROP_BRIGHTNESS, //Brightness of the image
	CV_CAP_PROP_CONTRAST, //Contrast of the image
	CV_CAP_PROP_SATURATION, //Saturation of the image
	CV_CAP_PROP_HUE, //Hue of the image (only for cameras).
	CV_CAP_PROP_GAIN, //Gain of the image (only for cameras).
	CV_CAP_PROP_EXPOSURE, //Exposure (only for cameras).
	CV_CAP_PROP_CONVERT_RGB //Boolean flags indicating whether images should be converted to RGB.
};
std::string names[] = {
	"CV_CAP_PROP_BRIGHTNESS",
    "CV_CAP_PROP_CONTRAST",
    "CV_CAP_PROP_SATURATION",
    "CV_CAP_PROP_HUE",
    "CV_CAP_PROP_GAIN",
    "CV_CAP_PROP_EXPOSURE",
    "CV_CAP_PROP_CONVERT_RGB"
};
bool set_att(xenobot::SetAtt::Request &req,xenobot::SetAtt::Response &res)
{
	ROS_INFO("set att");
	std::string result= "\n";

	if(req.name==0) {

		int arr_size = (sizeof(atts)/sizeof(int));
		// print all  information
		for(int i=0; i < arr_size; i++) {
			result += std::to_string(i+1);
			result += " " + names[i];
			result += ": ";
			result += std::to_string((int)camera.get(atts[i]));
			result += "\n";

		}
	} else if(req.name <= sizeof(atts)) {
		int i = req.name-1;
		if(camera.set(atts[i],req.value)) {
			result += "Set " + names[i] + " " + std::to_string(req.value);
		} else {
			result += "Failed";
		}
		ROS_INFO("%s",result.c_str());
	}

	res.msg = result;

	return true;
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "camera_attribute");
	ros::NodeHandle node;
	ros::Rate loop_rate(5);
	ros::ServiceServer srv = node.advertiseService("set_att", set_att);;
	cv::Mat frame, bgr_frame;

	if(!camera_setup(camera)) {
		ROS_INFO("Abort: failed to open pi camera!");
		return 0;
	}

	ros::Publisher raw_image_publisher = node.advertise<sensor_msgs::Image>("raw_image", 1);

	while(ros::ok()) {
		camera.grab();
		camera.retrieve(frame);
		sensor_msgs::ImagePtr img_msg =
			cv_bridge::CvImage(std_msgs::Header(), "rgb8", frame).toImageMsg();
		raw_image_publisher.publish(img_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
