#pragma once
#include <math.h>
#include <iostream>
#include <opencv2\opencv.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\stitching\stitcher.hpp>
#include <assert.h>
#include <string>

#define PI 3.14159265358979323846


//#define OPENCV_3
#ifndef OPENCV_3
	#define OPENCV_2
#endif

using namespace cv;

const double M_PI = PI;
const double ERR = 1e-7;

inline double round(double a) {return cvRound(a);}
inline double square(double a) {return pow(a,2);}


