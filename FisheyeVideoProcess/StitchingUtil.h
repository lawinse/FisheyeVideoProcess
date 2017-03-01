#pragma once
#include "Config.h"
#ifdef OPENCV_3
	#include<opencv2\stitching.hpp>
#else
	#include <opencv2\stitching\stitcher.hpp>
#endif

enum StitchingType {
	OPENCV_DEFAULT,
	OPENCV_TUNED,

	FACEBOOK,  /* In fact only use its keypoint mathc alogorithm, reference: surrond360 */

	SELF_SURF,
	SELF_SIFT,
};

enum StitchingPolicy {
	// Copy from the very first version
	DIRECT,
	// Experimental
	STITCH_DOUBLE_SIDE,
};

class StitchingUtil {
private:
	void opencvStitching(std::vector<Mat> &srcs, Mat &dstImage, StitchingType sType);
	void facebookKeyPointMatching(Mat &left, Mat &right, std::vector<std::pair<Point2f, Point2f>> &matchedPair);
	// TODO: Facebook Stitching method
	void facebookStitching();
	void selfKeyPointMatching(Mat &left, Mat &right, std::vector<std::pair<Point2f, Point2f>> &matchedPair, StitchingType sType);
	void selfStitchingSAfterMatching(
		Mat &left, Mat &right, Mat &leftOri, Mat &rightOri, std::vector<std::pair<Point2f, Point2f>> &matchedPair, Mat &dstImage);

	// helper functiom
	/* facebook matching, need opencv3.0+ supported actually */
	void matchWithBRISK(const Mat&, const Mat&, std::vector<std::pair<Point2f, Point2f>>& );
	void matchWithORB(const Mat&, const Mat&, std::vector<std::pair<Point2f, Point2f>>& );
#ifdef OPENCV_3
	void matchWithAKAZE(const Mat&, const Mat&, std::vector<std::pair<Point2f, Point2f>>& );
#endif

	cv::Stitcher opencvStitcherBuild(StitchingType sType);

	void unzipMatchedPair(std::vector<std::pair<Point2f, Point2f>> &, std::vector<Point2f> &, std::vector<Point2f> &);
	void getGrayScale(std::vector<Mat> &, std::vector<Mat> &);
	static bool _cmp_p2f(const Point2f &a, const Point2f &b);
	static bool _cmp_pp2f(const std::pair<Point2f, Point2f> &a, const std::pair<Point2f, Point2f> &b);

	void _stitch(std::vector<Mat> &srcs, Mat &dstImage, StitchingType sType);

public:
	StitchingUtil(){};
	~StitchingUtil(){};
	
	void doStitch(std::vector<Mat> &srcs, Mat &dstImage, StitchingPolicy sp = STITCH_DOUBLE_SIDE, StitchingType sType = OPENCV_DEFAULT);
};

