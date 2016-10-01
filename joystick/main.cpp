#include <iostream>
#include <ros/ros.h>
#include <curses.h>

#include <xenobot/wheel_command.h>

using namespace std;

#define _KEY_UP    'A'
#define _KEY_DOWN  'B'
#define _KEY_LEFT  'C'
#define _KEY_RIGHT 'D'

ros::Publisher wheel_command_publisher;

char read_arrow_key()
{
	if(getch() == 27) {
		getch();

		return getch();
	}

	return 0;
}

void publish_wheel_command(char key)
{
	xenobot::wheel_command msg;

	switch(key) {
	case _KEY_UP:
		msg.left_pwm = 100;
		msg.right_pwm = 100;
		ROS_INFO("KEY_UP\r");
		break;
	case _KEY_DOWN:
		ROS_INFO("KEY_DOWN (Not support reverse drive)\r");
		return;
	case _KEY_LEFT:
		msg.left_pwm = 50;
		msg.right_pwm = 100;
		ROS_INFO("KEY_LEFT\r");
		break;
	case _KEY_RIGHT:
		msg.left_pwm = 100;
		msg.right_pwm = 50;
		ROS_INFO("KEY_RIGHT\r");
		break;
	}

	wheel_command_publisher.publish(msg);
}

int main(int argc, char* argv[])
{
	/* ROS initialization */
	ros::init(argc, argv, "xenobot_joystick");
	ros::Time::init();
	ros::Rate loop_rate(1000);

	ros::NodeHandle node;

	wheel_command_publisher =
		node.advertise<xenobot::wheel_command>("xenobot/wheel_command", 1000);

	initscr();
	cbreak;

	char key;

	while(1) {
		key = read_arrow_key();

		cout << "\e[1;1H\e[2J";

		publish_wheel_command(key);
	}

	endwin();

	return 0;
}
