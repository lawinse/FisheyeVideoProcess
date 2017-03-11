#pragma once

#include "Config.h"
#ifdef OPENCV_3
	#include<opencv2\stitching.hpp>
	#include <opencv2\stitching\detail\matchers.hpp>
	#include <opencv2\xfeatures2d\nonfree.hpp>
#else
	#include <opencv2\stitching\stitcher.hpp>
	#include <opencv2\nonfree\nonfree.hpp>
	#include <opencv2\nonfree\features2d.hpp>
#endif

enum StitchingType {
	// OPENCV built-in somehow just can't work even in simplest case.
	// Deprecated
	OPENCV_DEFAULT,
	OPENCV_TUNED,

	OPENCV_SELF_DEV,

	FACEBOOK,  /* In fact only use its keypoint mathc alogorithm, reference: surrond360 */

	SELF_SURF,
	SELF_SIFT,
};

enum StitchingPolicy {
	// Copy from the very first version
	DIRECT,
	
	STITCH_ONE_SIDE,
	// Experimental
	STITCH_DOUBLE_SIDE,
};

struct OpenCVStitchParam {
		double workMegapix;
		double seamMegapix;
		double composeMegapix;
		float conf_thresh;
		cv::detail::WaveCorrectKind wave_correct;
		int expos_comp_type;
		float match_conf;
		int blend_type;
		float blend_strength;

		OpenCVStitchParam() {
			workMegapix = 0.8;
			seamMegapix = 0.1;
			composeMegapix = 0.8;
			conf_thresh = 0.7;
			wave_correct = cv::detail::WAVE_CORRECT_HORIZ;
			expos_comp_type = cv::detail::ExposureCompensator::GAIN_BLOCKS;
			match_conf = 0.3f;
			blend_type = cv::detail::Blender::MULTI_BAND;
			blend_strength = 5;
		}
};

class StitchingUtil {
private:
	#define kFlannMaxDistScale 3
	#define kFlannMaxDistThreshold 0.04
	#define kFlannNumTrees 4 // by default
	#define defaultMaskRatio std::make_pair(0.25,0.8) 	/* widthParam = 0.25, heightParam = 0.8 by default*/
	#define OVERLAP_RATIO_DOUBLESIDE 0.25
	


	void opencvStitching(const std::vector<Mat> &srcs, Mat &dstImage, StitchingType sType);
	std::vector<UMat> convertMatToUMat(std::vector<Mat> &input);
	void facebookKeyPointMatching(Mat &left, Mat &right, std::vector<std::pair<Point2f, Point2f>> &matchedPair);
	// TODO: Facebook Stitching method
	void facebookStitching();
	void selfKeyPointMatching(Mat &left, Mat &right, std::vector<std::pair<Point2f, Point2f>> &matchedPair, StitchingType sType);
	void selfStitchingSAfterMatching(
		const Mat &left, const Mat &right, const Mat &leftOri, const Mat &rightOri, std::vector<std::pair<Point2f, Point2f>> &matchedPair, Mat &dstImage);

	// helper function
	/* facebook matching, need opencv3.0+ supported actually */
	void matchWithBRISK(const Mat&, const Mat&, std::vector<std::pair<Point2f, Point2f>>& );
	void matchWithORB(const Mat&, const Mat&, std::vector<std::pair<Point2f, Point2f>>& );
#ifdef OPENCV_3
	void matchWithAKAZE(const Mat&, const Mat&, std::vector<std::pair<Point2f, Point2f>>& );
#endif
	
	static bool almostBlack(const Vec3b &);
	void showMatchingPair(
		const Mat &left, const std::vector<KeyPoint> &kptL,
		const Mat &right, const std::vector<KeyPoint> &kptR, const std::vector<DMatch> &goodMatches);

	cv::Stitcher opencvStitcherBuild(StitchingType sType);

	void unzipMatchedPair(std::vector<std::pair<Point2f, Point2f>> &, std::vector<Point2f> &, std::vector<Point2f> &);
	void getGrayScaleAndFiltered(const std::vector<Mat> &, std::vector<Mat> &);
	static bool _cmp_p2f(const Point2f &a, const Point2f &b);
	static bool _cmp_pp2f(const std::pair<Point2f, Point2f> &a, const std::pair<Point2f, Point2f> &b);

	void _stitch(const std::vector<Mat> &srcs, Mat &dstImage, StitchingType sType, std::pair<double, double> &ratio=defaultMaskRatio);
	void _stitchDoubleSide(std::vector<Mat> &srcs, Mat &dstImage, StitchingType sType);

public:
	OpenCVStitchParam osParam;
	StitchingUtil(){osParam = OpenCVStitchParam();}
	~StitchingUtil(){};

	static Mat getMask(const Mat &srcImage, bool isLeft, std::pair<double, double> &ratio=defaultMaskRatio);

	static std::vector<cv::Rect> getMaskROI(const Mat &srcImage, bool isLeft, std::pair<double, double> &ratio=defaultMaskRatio);
	void opencvSelfStitching(
		const std::vector<Mat> &srcs, Mat &dstImage, std::pair<double, double> &maskRatio=defaultMaskRatio);
	void opencvSelfStitching(
		const std::vector<Mat> &srcs, Mat &dstImage, const Size resizeSz, std::pair<double, double> &maskRatio=defaultMaskRatio);
	static void removeBlackPixelByBound(Mat &, Mat &);
	void doStitch(std::vector<Mat> &srcs, Mat &dstImage, StitchingPolicy sp = STITCH_ONE_SIDE, StitchingType sType = OPENCV_DEFAULT);
};

