#pragma once

#include "Config.h"
enum CorrectingType {
	/* Copy from the very first version */
	BASIC_FORWARD,
	BASIC_REVERSED,

	/* LL group should act the same as above */
	LONG_LAT_MAPPING_FORWARD,
	LONG_LAT_MAPPING_REVERSED,

	PERSPECTIVE_LONG_LAT_MAPPING_CAM_LENS_MOD_FORWARD,
	PERSPECTIVE_LONG_LAT_MAPPING_CAM_LENS_MOD_REVERSED,
};

enum DistanceMappingType {
	LONG_LAT,
	PERSPECTIVE,
};


struct CorrectingParams {
	CorrectingType ctype;
	Point2i centerOfCircle;
	int radiusOfCircle;
	DistanceMappingType dmType;
	/*
		const double theta_left = 0;
		const double phi_up = 0;
		const double camFieldAngle = PI;  // TOSOLVE: remains to be tuned
	*/

	CorrectingParams(
		CorrectingType		_ctype = BASIC_REVERSED,
		Point2i				_center = Point2i(0,0),
		int					_radius = 0,
		DistanceMappingType	_dmType = PERSPECTIVE){
			ctype = _ctype;
			centerOfCircle = _center;
			radiusOfCircle = _radius;
			dmType = _dmType;
	}
};


class CorrectingUtil {
private:
	const double camFieldAngle = PI;

	void basicCorrecting(Mat &src, Mat &dst, CorrectingType ctype);
	void LLMCorrecting(Mat &src, Mat &dst, Point2i center, int radius, CorrectingType ctype);
	void PLLMCLMCorrentingForward(Mat &src, Mat &dst, Point2i center, int radius);
	void PLLMCLMCorrentingReversed(
		Mat &src, Mat &dst, Point2i center, int radius, DistanceMappingType dmtype);	// w = PI/2
	
	// helper function
	double getPhiFromV(double v);
	void rotateEarth(double &x, double &y, double &z);
public:
	CorrectingUtil(){};
	~CorrectingUtil(){};
	void doCorrect(Mat &srcImage, Mat &dstImage, CorrectingParams cParams = CorrectingParams());
	
};
