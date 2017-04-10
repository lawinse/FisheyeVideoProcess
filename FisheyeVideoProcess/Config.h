#pragma once
#include <math.h>
#include <iostream>
#include <ctime>
#include <cstdlib>
#include <opencv2\opencv.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <assert.h>
#include <string.h>
#include "MyLog.h"
using namespace cv;
#define PI CV_PI
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

const double M_PI = PI;
const double ERR = 1e-7;

inline double round(const double a) {return cvRound(a);}
inline double square(const double a) {return pow(a,2);}
/* Convert stream content to std::string*/
#define GET_STR(msg,s)	\
	{\
		std::stringstream ss;\
		ss << msg;\
		s=ss.str();\
	}
/* Convert std::vector<T> content to std::string */
template<typename T> 
std::string vec2str(const std::vector<T> & v) {
	std::stringstream ss;
	ss << "{";
	for (auto i:v) {ss << i << ", ";}
	ss << "}";
	return ss.str();
}

/* Combine multiple hashcode */
template<class T>
inline void hash_combine(int &seed, const T &v) {
	std::hash<T> hasher;
	seed ^= hasher(v) + 0x9e3779b9  +(seed << 6) + (seed >> 2);
}

/* Default comparator of cv::Point2f */
inline bool _cmp_p2f(const cv::Point2f &a, const cv::Point2f &b) {
	return a.x == b.x ? a.y < b.y : a.x < b.x;
}

/* Convert timestamp to local time */
inline std::string timetodate(time_t const timer) {
	struct tm *l=localtime(&timer);
	char buf[128];
	sprintf(buf,"%02d:%02d:%02d",l->tm_hour,l->tm_min,l->tm_sec);
	std::string s(buf);
	return s;
}