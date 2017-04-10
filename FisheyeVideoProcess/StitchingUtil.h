#pragma once

#include "Config.h"
#include <unordered_map>
#include <unordered_set>
#include ".\Supplements\RewarpableWarper.h"
#include ".\OtherUtils\IntervalBestValueMaintainer.h"

#ifdef OPENCV_3
	#include <opencv2\stitching.hpp>
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
	STITCH_DOUBLE_SIDE_NOT_DIRECTION_CORRECTION,
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
		bool isRealStitching;

		OpenCVStitchParam() {
			workMegapix = 0.8;
			seamMegapix = 0.1;
			composeMegapix = 0.8;
			conf_thresh = 0.7;
			wave_correct = cv::detail::WAVE_CORRECT_HORIZ;
			expos_comp_type = cv::detail::ExposureCompensator::GAIN_BLOCKS;
			match_conf = 0.3f;
			blend_type = cv::detail::Blender::MULTI_BAND;
			isRealStitching = true;
			blend_strength = 5;
		}
};

class StitchingUtil;
class StitchingInfo;
typedef std::vector<StitchingInfo> StitchingInfoGroup;

/* Stitching Information, storage of the detail of each stitching op */
class StitchingInfo {
public:
	int imgCnt;
	double nonBlackRatio;
	//float warpedImageScale;
	Size resizeSz;
	int srcType;
	

	std::vector<cv::detail::ImageFeatures> features;
	std::pair<double, double> maskRatio;

	/* Indicates width-wise ranges of each src in stitched result*/
	std::vector<Range> ranges;

	std::vector<cv::detail::CameraParams> cameras;
	Mat projData;
	std::vector<supp::ResultRoi> resultRois;
	std::vector<supp::PlaneLinearTransformHelper> pltHelpers;

	StitchingInfo(){clear();}
	StitchingInfo(const StitchingInfo &sinfo);
	StitchingInfo& operator = (const StitchingInfo &sinfo);

	friend std::ostream& operator <<(std::ostream&, const StitchingInfo&);
	void clear();
	bool isNull() const;
	void setRanges(const std::vector<Point> &corners, const std::vector<Size> &sizes);
	void setRanges(const Range &fullImgeRange);
	bool isSuccess() const;
	double evaluate() const;
	float getWarpScale() const;
	bool setToCamerasInternalParam(std::vector<cv::detail::CameraParams> &cameras);
	void setFromCamerasInternalParam(std::vector<cv::detail::CameraParams> &cameras);
	float getLastScale() const {return projData.at<float>(projData.rows-1,0);}
	float getAverFocal() const {return getWarpScale();}

	static bool isSuccess(const StitchingInfoGroup &);
	/* Score <class StitchingInfoGroup> */
	static double evaluate(const StitchingInfoGroup &);
	/* Calculate an average <class StitchingInfoGroup> */
	static void getAverageSIG(const std::vector<StitchingInfoGroup*> &pSIGs, StitchingInfoGroup &ret);
};

/* A window-size of <class StitchingInfoGroup> */
class LocalStitchingInfoGroup {
	#define LSIG_WINDOW_SIZE 30		// Set to INT_MAX meaning Global
	#define LSIG_MOVEING_WINDOWS 1	// Indicate whehter the window is moved or fixed
	#define LSIG_BEST_CAND_NUM 1
	#define LSIG_SELECT_NUM LSIG_BEST_CAND_NUM
	#define LSIG_MAX_STITCHED_BUFF_SIZE 3
	#define LSIG_MAX_WAITING_BUFF_SIZE 10
	int wSize;
	IntervalBestValueMaintainer<StitchingInfoGroup,double> groups;
	StitchingInfoGroup preSuccessSIG;
	std::unordered_map<int, std::vector<Mat>> stitchingWaitingBuff;
	std::unordered_map<int, int> stitchingWaitingBuffPersistedSize;
	std::vector<std::pair<int, Mat>> stitchedBuff;
	
	// For resultRois
	std::vector<std::vector<supp::ResultRoi>> resultRoisBase;
	std::unordered_set<int> resultRoisUsedFrameBase;
	std::vector<std::vector<supp::ResultRoi>> resultRoiCur;
	std::unordered_set<int> resultRoisUsedFrameCur;
	std::vector<std::vector<supp::PlaneLinearTransformHelper>> pltHelperGroup;

public:
	LocalStitchingInfoGroup(int _wSize = LSIG_WINDOW_SIZE):wSize(_wSize){
		groups = IntervalBestValueMaintainer<StitchingInfoGroup,double>(
			int(LSIG_BEST_CAND_NUM),int(LSIG_WINDOW_SIZE),&StitchingInfo::evaluate);
	}
	/* Indicates whether <class LocalStitchingInfoGroup> covers the given range*/
	bool cover(int l, int r) {return groups.isCandCovered(l,r);}
	bool empty() const {return groups.getCandNum() == 0;}
	std::pair<int,int> getCovered() const {return groups.getRange();}
	/* Add a new <class StitchingInfoGroup> */
	void push_back(int fidx, StitchingInfoGroup& g);

	/* WaitingBuff stores frames waited to be stitched */
	void addToWaitingBuff(int fidx, std::vector<Mat>&);
	bool getFromWaitingBuff(int fidx, std::vector<Mat>& v);
	void removeFromWaitingBuff(int fidx);

	/* StitchedBuff stores frames stitched */
	bool isStitchedBuffFull() const {return stitchedBuff.size() >= LSIG_MAX_STITCHED_BUFF_SIZE;}
	void addToStitchedBuff(int fidx, Mat& m);
	std::vector<std::pair<int, Mat>>* getStitchedBuff() {return &stitchedBuff;}
	void clearStitchedBuff() {stitchedBuff.clear();}

	/* Obtain averaged <class StitchingInfoGroup> from <class LocalStitchingInfoGroup> */
	StitchingInfoGroup getAver(int head, int tail, std::vector<int>&, StitchingUtil &);
	/* Set <class PlaneLinearTransformHelper> for <class LocalStitchingInfoGroup> */
	bool adjustPltForLSIG(StitchingInfoGroup &, const std::vector<int>&, StitchingUtil &);

};



class StitchingUtil {
private:
	#define kFlannMaxDistScale 3
	#define kFlannMaxDistThreshold 0.04
	#define kFlannNumTrees 4 // by default
	#define OVERLAP_RATIO_DOUBLESIDE 0.20
	#define defaultMaskRatio std::make_pair(OVERLAP_RATIO_DOUBLESIDE,0.8) 	/* widthParam = 0.20, heightParam = 0.8 by default*/
	#define OVERLAP_RATIO_DOUBLESIDE_4 0.15
	#define BLACK_TOLERANCE 3
	#define NONBLACK_REMAIN_FLOOR 0.70

	/* Unify the resized size of each step */
	#define FIX_RESIZE_0 Size(1440,1440)
	#define FIX_RESIZE_1 Size(2020,1440)
	#define FIX_RESIZE_2 Size(1750,1440)
	
	/* Original opencv Stitcher. Deprecated*/
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
	/* Indicates whether a BGR pixel is considered black */
	static bool almostBlack(const Vec3b &);
	void showMatchingPair(
		const Mat &left, const std::vector<KeyPoint> &kptL,
		const Mat &right, const std::vector<KeyPoint> &kptR, const std::vector<DMatch> &goodMatches);

	cv::Stitcher opencvStitcherBuild(StitchingType sType);	//Deprecated

	void unzipMatchedPair(std::vector<std::pair<Point2f, Point2f>> &, std::vector<Point2f> &, std::vector<Point2f> &);
	void getGrayScaleAndFiltered(const std::vector<Mat> &, std::vector<Mat> &);

	/* Stitching unit op */
	StitchingInfo _stitch(
		const std::vector<Mat> &srcs, Mat &dstImage, StitchingType sType,StitchingInfo &sInfoNotNull, const Size resizeSz = Size(), std::pair<double, double> &ratio=defaultMaskRatio );
	/* Stitching multiple time trying to reduce seam */
	StitchingInfoGroup _stitchDoubleSide(std::vector<Mat> &srcs, Mat &dstImage, StitchingInfoGroup &, const StitchingPolicy sp, const StitchingType sType);

	/* Different ways to find max interior rectangle to remove black pixels surrounded */
	static bool removeBlackPixelByDoubleScan(Mat &, Mat &, StitchingInfo &);
	static bool removeBlackPixelByContourBound(Mat &, Mat &, StitchingInfo &);
	static bool checkInterior(const Mat& mask, const Rect& interiorBB, bool &top, bool &bottom, bool &left, bool &right);
public:
	OpenCVStitchParam osParam;
	StitchingType stitchingType;
	StitchingPolicy stitchingPolicy;

	StitchingUtil(){osParam = OpenCVStitchParam();}
	~StitchingUtil(){};

	/* Get ROI Mask */
	static Mat getMask(const Mat &srcImage, bool isLeft, std::pair<double, double> &ratio=defaultMaskRatio);
	static std::vector<cv::Rect> getMaskROI(const Mat &srcImage, bool isLeft, std::pair<double, double> &ratio=defaultMaskRatio);
	static std::vector<cv::Rect> getMaskROI(const Mat &srcImage, int index, int total, std::pair<double, double> &ratio=defaultMaskRatio);

	/* Stitching pipeline based on opencv Stitcher */
	StitchingInfo opencvSelfStitching(
		const std::vector<Mat> &srcs, Mat &dstImage,StitchingInfo &sInfo, std::pair<double, double> &maskRatio=defaultMaskRatio);
	StitchingInfo opencvSelfStitching(
		const std::vector<Mat> &srcs, Mat &dstImage, const Size resizeSz, StitchingInfo &sInfo, std::pair<double, double> &maskRatio=defaultMaskRatio);
	
	static void removeBlackPixel(Mat &src, Mat &dst, StitchingInfo &sInfo);
	
	/* Stitching interface */
	StitchingInfoGroup doStitch(
		std::vector<Mat> &srcs, Mat &dstImage,StitchingInfoGroup &, StitchingPolicy sp = STITCH_ONE_SIDE, StitchingType sType = OPENCV_DEFAULT);
};

