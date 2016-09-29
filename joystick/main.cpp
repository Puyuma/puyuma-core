#include <iostream>
#include <stdio.h>

#include <ros/ros.h>

#define KEY_UP    72
#define KEY_DOWN  80
#define KEY_LEFT  75
#define KEY_RIGHT 77

using namespace std;

int main(int argc, char* argv[])
{
	/* ROS initialization */
	ros::init(argc, argv, "xenobot");
	ros::Time::init();
	//ros::Rate loop_rate(1000);

	char key;

	ROS_INFO("Joystick start:");

	while(1) {
		//key = getch();

		switch(key) {
		case KEY_UP:
			ROS_INFO("KEY_UP");
			break;
		case KEY_DOWN:
			ROS_INFO("KEY_DOWN");
			break;
		case KEY_LEFT:
			ROS_INFO("KEY_LEFT");
			break;
		case KEY_RIGHT:
			ROS_INFO("KEY_RIGHT");
			break;
		}

		ros::spinOnce();
	}

	return 0;
}
