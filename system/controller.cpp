#include "gnublin.h"

void test_motor()
{
	gnublin_gpio motor;

	motor.pinMode(12, OUTPUT);
	motor.pinMode(6, OUTPUT);
	motor.pinMode(18, OUTPUT);
	motor.pinMode(17, OUTPUT);

	motor.digitalWrite(12, HIGH);
	motor.digitalWrite(6, LOW);
	motor.digitalWrite(18, HIGH);
	motor.digitalWrite(17, LOW);
}
