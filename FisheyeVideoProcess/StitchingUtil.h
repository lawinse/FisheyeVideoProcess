#pragma once

#include "Config.h"
#include <unordered_map>

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
	
	OPENCV_DEFAULT,// Deprecated
	OPENCV_TUNED,// Deprecated

	OPENCV_SELF_DEV,

	FACEBOOK, // Deprecated. In fact only use its keypoint mathc alogorithm, reference: surrond360 

	SELF_SURF,// Deprecated
	SELF_SIFT,// Deprecated
};

enum StitchingPolicy {
	// Copy from the very first version
	DIRECT, // Deprecated
	
	STITCH_ONE_SIDE, // Deprecated
	
	STITCH_DOUBLE_SIDE,
	
	// Experimental, not stable
	STITCH_DOUBLE_SIDE_ONCE_TIME,
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

class StitchingInfo;
typedef std::vector<StitchingInfo> StitchingInfoGroup;

class StitchingInfo {
public:
	int imgCnt;
	double nonBlackRatio;
	float warpedImageScale;
	Size resizeSz;
	
	std::pair<double, double> maskRatio;
	std::vector<Range> ranges;	// only cares width-wise
	std::vector<cv::detail::CameraParams> cameras;

	StitchingInfo(){clear();}
	friend std::ostream& operator <<(std::ostream&, const StitchingInfo&);
	void clear();
	bool isNull();
	void setRanges(const std::vector<Point> &corners, const std::vector<Size> &sizes);
	void setRanges(const Range &fullImgeRange);
	bool isSuccess() const;
	double evaluate() const;

	static bool isSuccess(const StitchingInfoGroup &);
	static double evaluate(const StitchingInfoGroup &);
};

class LocalStitchingInfoGroup {
	#define LSIG_WINDOW_SIZE 5
	#define LSIG_BEST_NUM 1
	int wSize;
	int startIdx, endIdx;
	std::unordered_map<int, std::pair<StitchingInfoGroup,double>> groups;
	std::unordered_map<int, std::vector<Mat>> stitchingWaitingBuff;
	std::vector<std::pair<int, Mat>> stitchedBuff;



public:
	LocalStitchingInfoGroup(int _wSize = LSIG_WINDOW_SIZE):wSize(_wSize),startIdx(0),endIdx(0){
	}
	int getEndIdx() const {return endIdx;}
	bool cover(int l, int r) {return startIdx <= l && endIdx >= r;}
	bool empty() const {return endIdx-startIdx == 0;}
	int push_back(StitchingInfoGroup g);
	void addToWaitingBuff(int fidx, std::vector<Mat>&);
	bool getFromWaitingBuff(int fidx, std::vector<Mat>& v) {
		auto ret = stitchingWaitingBuff.find(fidx);
		if (ret != stitchingWaitingBuff.end()) {
			v = (*ret).second; return true;
		} else {
			LOG_ERR("Cannot find " << fidx << " frame src data.")
			return false;
		}
		
	}
	void addToStitchedBuff(int fidx, Mat& m) {stitchedBuff.push_back(std::make_pair(fidx,m.clone()));}
	std::vector<std::pair<int, Mat>>* getStitchedBuff() {return &stitchedBuff;}
	void clearStitchedBuff() {stitchedBuff.clear();}
	StitchingInfoGroup getAver(int head, int tail);

};



class StitchingUtil {
private:
	#define kFlannMaxDistScale 3
	#define kFlannMaxDistThreshold 0.04
	#define kFlannNumTrees 4 // by default
	#define defaultMaskRatio std::make_pair(0.25,0.8) 	/* widthParam = 0.25, heightParam = 0.8 by default*/
	#define OVERLAP_RATIO_DOUBLESIDE 0.25
	#define OVERLAP_RATIO_DOUBLESIDE_4 0.15
	#define BLACK_TOLERANCE 3
	#define NONBLACK_REMAIN_FLOOR 0.75
	


	StitchingInfo opencvStitching(const std::vector<Mat> &srcs, Mat &dstImage, StitchingType sType);
	std::vector<UMat> convertMatToUMat(std::vector<Mat> &input);
	void facebookKeyPointMatching(Mat &left, Mat &right, std::vector<std::pair<Point2f, Point2f>> &matchedPair);
	// TODO: Facebook Stitching method
	void facebookStitching();
	void selfKeyPointMatching(Mat &left, Mat &right, std::vector<std::pair<Point2f, Point2f>> &matchedPair, StitchingType sType);
	void selfStitchingSAfterMatching(
		const Mat &left, const Mat &right, const Mat &leftOri, const Mat &rightOri,
		std::vector<std::pair<Point2f, Point2f>> &matchedPair, Mat &dstImage);

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

	StitchingInfo _stitch(
		const std::vector<Mat> &srcs, Mat &dstImage, StitchingType sType,StitchingInfo &sInfoNotNull, std::pair<double, double> &ratio=defaultMaskRatio );
	StitchingInfoGroup _stitchDoubleSide(std::vector<Mat> &srcs, Mat &dstImage, StitchingInfoGroup &, const StitchingPolicy sp, const StitchingType sType);

	static void removeBlackPixelByDoubleScan(Mat &, Mat &, StitchingInfo &);
	static bool removeBlackPixelByContourBound(Mat &, Mat &, StitchingInfo &);
	static bool checkInterior(const Mat& mask, const Rect& interiorBB, bool &top, bool &bottom, bool &left, bool &right);
public:
	OpenCVStitchParam osParam;
	StitchingUtil(){osParam = OpenCVStitchParam();}
	~StitchingUtil(){};

	static Mat getMask(const Mat &srcImage, bool isLeft, std::pair<double, double> &ratio=defaultMaskRatio);
	static std::vector<cv::Rect> getMaskROI(const Mat &srcImage, bool isLeft, std::pair<double, double> &ratio=defaultMaskRatio);

	static std::vector<cv::Rect> getMaskROI(const Mat &srcImage, int index, int total, std::pair<double, double> &ratio=defaultMaskRatio);

	StitchingInfo opencvSelfStitching(
		const std::vector<Mat> &srcs, Mat &dstImage,StitchingInfo &sInfo, std::pair<double, double> &maskRatio=defaultMaskRatio);
	StitchingInfo opencvSelfStitching(
		const std::vector<Mat> &srcs, Mat &dstImage, const Size resizeSz, StitchingInfo &sInfo, std::pair<double, double> &maskRatio=defaultMaskRatio);
	static void removeBlackPixel(Mat &src, Mat &dst, StitchingInfo &sInfo) {
		if (!removeBlackPixelByContourBound(src,dst,sInfo)) removeBlackPixelByDoubleScan(src,dst, sInfo);
	}
	StitchingInfoGroup doStitch(
		std::vector<Mat> &srcs, Mat &dstImage,StitchingInfoGroup &, StitchingPolicy sp = STITCH_ONE_SIDE, StitchingType sType = OPENCV_DEFAULT);
};

