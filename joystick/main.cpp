#include <iostream>
#include <ros/ros.h>
#include <curses.h>

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

	while(1) {
		if(getch() != 27) {
			cout << "\e[1;1H\e[2J";
			ROS_INFO("Press arrow key to drive the car");
			continue;
		}

		cout << "\e[1;1H\e[2J";

		getch();

		switch(c = getch()) {
		case 'A':
			ROS_INFO("KEY_UP\r");
			break;
		case 'B':
			ROS_INFO("KEY_DOWN\r");
			break;
		case 'C':
			ROS_INFO("KEY_LEFT\r");
			break;
		case 'D':
			ROS_INFO("KEY_RIGHT\r");
			break;
		}
	}

	endwin();

	return 0;
}
