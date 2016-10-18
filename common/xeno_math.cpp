#include <opencv2/opencv.hpp>

using namespace cv;

float inner_product(Point2f p1, Point2f p2)
{
	return p1.x * p2.x + p1.y * p2.y;
}

float magnatitude(Point2f p)
{
	return sqrt(p.x * p.x + p.y * p.y);
}

void normalize(Point2f& p)
{
	float mag = magnatitude(p);
	p.x /= mag;
	p.y /= mag;
}
