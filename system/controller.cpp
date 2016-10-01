#include <wiringPi.h>

void test_motor()
{
	wiringPiSetup() ;

	pinMode(12, OUTPUT);
	pinMode(6, OUTPUT);
	pinMode(18, OUTPUT);
	pinMode(17, OUTPUT);

	digitalWrite(12, HIGH);
	digitalWrite(6, LOW);
	digitalWrite(18, HIGH);
	digitalWrite(17, LOW);
}
