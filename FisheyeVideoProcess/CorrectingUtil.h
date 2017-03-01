#pragma once

#include "Config.h"
#include <hash_map>
enum CorrectingType {
	/* Copy from the very first version */
	BASIC_FORWARD,
	BASIC_REVERSED,

	/* LL group should act the same as above */
	LONG_LAT_MAPPING_FORWARD,
	LONG_LAT_MAPPING_REVERSED,

	PERSPECTIVE_LONG_LAT_MAPPING_CAM_LENS_MOD_FORWARD,
	PERSPECTIVE_LONG_LAT_MAPPING_CAM_LENS_MOD_REVERSED,

	OPENCV,
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
	bool use_reMap;
	/*
		const double theta_left = 0;
		const double phi_up = 0;
		const double camFieldAngle = PI;  // TOSOLVE: remains to be tuned
	*/

	bool operator == (const CorrectingParams &obj) const {
		return ctype == obj.ctype && centerOfCircle == obj.centerOfCircle
			&& radiusOfCircle == obj.radiusOfCircle
			&& dmType == obj.dmType
			&& use_reMap == obj.use_reMap;
	}


	CorrectingParams(
		CorrectingType		_ctype = BASIC_REVERSED,
		Point2i				_center = Point2i(0,0),
		int					_radius = 0,
		DistanceMappingType	_dmType = LONG_LAT,
		bool 				_use_ReMap = true){
			ctype = _ctype;
			centerOfCircle = _center;
			radiusOfCircle = _radius;
			dmType = _dmType;
			use_reMap = _use_ReMap;
	}
};

struct ReMapping{
	// For memorization   map[dst] = src;
	bool bMapped;
	stdext::hash_map<long long, std::pair<int,int>> map;

	ReMapping(){clear();}
	void clear() {map.clear(); bMapped = false;}
	bool isMapped() {return bMapped && !(map.size() == 0);}
	std::pair<int,int> get(std::pair<int,int> dstPos) {
		return map[dstPos.first*(long long)INT_MAX + dstPos.second];
	}
	void set(std::pair<int,int>srcPos, std::pair<int,int>dstPos) {
		bMapped = true;
		map[dstPos.first*(long long)INT_MAX + dstPos.second] = srcPos;
	}

	bool reMap(Mat &srcImage, Mat &dstImage) {
		if (!isMapped()) return false;
		for (stdext::hash_map<long long, std::pair<int,int>>::iterator it=map.begin(); it!=map.end(); ++it) {
			int i,j,i_dst,j_dst;
			i_dst = (it->first)/INT_MAX, j_dst = (it->first)%INT_MAX;
			i = (it->second).first, j = (it->second).second;
			dstImage.at<Vec3b>(i_dst,j_dst)[0] = srcImage.at<Vec3b>(i,j)[0];
			dstImage.at<Vec3b>(i_dst,j_dst)[1] = srcImage.at<Vec3b>(i,j)[1];
			dstImage.at<Vec3b>(i_dst,j_dst)[2] = srcImage.at<Vec3b>(i,j)[2];
		}
		return true;
	}
};

class CorrectingUtil {
private:
	#define camFieldAngle PI

	ReMapping pixelReMapping;
	CorrectingParams _cParams;
	void basicCorrecting(Mat &src, Mat &dst, CorrectingType ctype);
	void LLMCorrecting(Mat &src, Mat &dst, Point2i center, int radius, CorrectingType ctype);
	void PLLMCLMCorrentingForward(Mat &src, Mat &dst, Point2i center, int radius);
	void PLLMCLMCorrentingReversed(
		Mat &src, Mat &dst, Point2i center, int radius, DistanceMappingType dmtype);	// w = PI/2
	
	// helper function
	double getPhiFromV(double v);
	void rotateEarth(double &x, double &y, double &z);
public:
	CorrectingUtil(){pixelReMapping = ReMapping();};
	~CorrectingUtil(){};
	void doCorrect(Mat &srcImage, Mat &dstImage, CorrectingParams cParams = CorrectingParams());
};
