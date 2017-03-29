#pragma once
#include <math.h>
#include <iostream>
#include <opencv2\opencv.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <assert.h>
#include <string.h>
#include "MyLog.h"

#define PI 3.14159265358979323846
#define RESOURCE_PATH ".\\Resources\\"
#define OUTPUT_PATH ".\\Outputs\\"
#define TEMP_PATH ".\\Temp\\"
#define LOG_PATH ".\\Logs\\"


#define OPENCV_3
#ifndef OPENCV_3
	#define OPENCV_2
#else
	//may define sth
	#define OPENCV3_CONTRIB
#endif

#define RUN_MAIN
#ifndef RUN_MAIN
	#define RUN_TEST
#endif
//#define SHOW_IMAGE
//#define TRY_CATCH

using namespace cv;

const double M_PI = PI;
const double ERR = 1e-7;

inline double round(const double a) {return cvRound(a);}
inline double square(const double a) {return pow(a,2);}
#define GET_STR(msg,s)	\
	{\
		std::stringstream ss;\
		ss << msg;\
		s=ss.str();\
	}

template<typename T> 
std::string vec2str(const std::vector<T> & v) {
	std::stringstream ss;
	ss << "{";
	for (auto i:v) {ss << i << ", ";}
	ss << "}";
	return ss.str();
}


inline bool _cmp_p2f(const cv::Point2f &a, const cv::Point2f &b) {
	return a.x == b.x ? a.y < b.y : a.x < b.x;
}
