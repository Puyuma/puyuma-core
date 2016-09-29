#include <iostream>
#include <ros/ros.h>

using namespace std;

int main(int argc, char* argv[])
{
	/* ROS initialization */
	ros::init(argc, argv, "xenobot");
	ros::Time::init();
	ros::Rate loop_rate(1000);

	initscr();
	cbreak;

	char c;

	ROS_INFO("Joystick start:");

	while(1) {
		c = getch();

		switch(c) {
		case 'w':
		case 'W':
			ROS_INFO("KEY_UP");
			break;
		case 's':
		case 'S':
			ROS_INFO("KEY_DOWN");
			break;
		case 'a':
		case 'A':
			ROS_INFO("KEY_LEFT");
			break;
		case 'd':
		case 'D':
			ROS_INFO("KEY_RIGHT");
			break;
		default:
			ROS_INFO("%c", c);
		}
	}

	endwin();

	return 0;
}
