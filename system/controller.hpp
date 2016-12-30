#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <string>

#define THROTTLE_BASE 35 //25% of the throttle

using namespace std;

enum {
	JOYSTICK_MODE,
	SELF_DRIVING_MODE,
	STOP_MODE
} ControllerMode;

bool load_pid_param(string _yaml_path);
void self_driving_controller(int d, int phi);

void bound(int min, int max, int x);

#endif
