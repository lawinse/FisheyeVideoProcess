#pragma once
#include "Config.h"

enum StitchingType {
	OPENCV_DEFAULT,
	OPENCV_TUNED,

	FACEBOOK,  /* In fact only use its keypoint mathc alogorithm, reference: surrond360 */

	SELF_SURF,
	SELF_SIFT,
};

enum StitchingPolicy {
	// Copy from the very first version
	STITCH_ONE_SIDE,
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

	Stitcher opencvStitcherBuild(StitchingType sType);

	void unzipMatchedPair(std::vector<std::pair<Point2f, Point2f>> &, std::vector<Point2f> &, std::vector<Point2f> &);
	void getGrayScale(std::vector<Mat> &, std::vector<Mat> &);
#ifdef OPENCV_3
	void matchWithAKAZE(const Mat&, const Mat&, std::vector<std::pair<Point2f, Point2f>>& );
#endif
	void _stitch(std::vector<Mat> &srcs, Mat &dstImage, StitchingType sType);

public:
	StitchingUtil(){};
	~StitchingUtil(){};
	
	void doStitch(std::vector<Mat> &srcs, Mat &dstImage, StitchingType sType, StitchingPolicy sp);
};

