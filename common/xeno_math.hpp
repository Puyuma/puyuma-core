#ifndef XENO_MATH_H
#define XENO_MATH_H

#include <opencv2/opencv.hpp>

using namespace cv;

float inner_product(Point2f p1, Point2f p2);
float magnatitude(Point2f p);
void normalize(Point2f& p);

#endif
