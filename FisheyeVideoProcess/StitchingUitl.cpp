#include "StitchingUtil.h"
#include <opencv2\features2d\features2d.hpp>

using namespace cv::detail;

void StitchingUtil::matchWithBRISK(
	const Mat &left, const Mat &right, std::vector<std::pair<Point2f, Point2f>> &matchedPair) {
	static const int kFlannMaxDistScale = 3;
	static const double kFlannMaxDistThreshold = 0.04;

	std::vector<KeyPoint> kptsL, kptsR;
	Mat descL, descR;
	std::vector<DMatch> goodMatches;

#ifdef OPENCV_3
	Ptr<BRISK> brisk = BRISK::create();
	brisk->detectAndCompute(imageL, noArray(), kptsL, descL);
	brisk->detectAndCompute(imageR, noArray(), kptsR, descR);
#else
	//TOSOLVE: 2.4.9-style incovation, not sure it works
	const int Thresh = 60, Octave = 4;
	const float PatternScales = 1.0f;
	cv::BRISK briskDetector(Thresh, Octave, PatternScales);
	briskDetector.create("Feature2D.BRISK");
	briskDetector.detect(left, kptsL);
	briskDetector.compute(left, kptsL, descL);
	briskDetector.detect(right, kptsR);
	briskDetector.compute(right, kptsR, descR);
#endif

	descL.convertTo(descL, CV_32F);
	descR.convertTo(descR, CV_32F);

	static const int kFlannNumTrees = 4;
	FlannBasedMatcher matcher(new flann::KDTreeIndexParams(kFlannNumTrees));
	vector<DMatch> flannMatches;
	matcher.match(descL, descR, flannMatches);

	double maxDist = 0;
	double minDist = std::numeric_limits<float>::max() ;
	for(int i = 0; i < flannMatches.size(); ++i) {
		double dist = flannMatches[i].distance;
		maxDist = max(maxDist, dist);
		minDist = min(minDist, dist);
	}

	for(int i = 0; i < flannMatches.size(); ++i) {
		double distThresh = kFlannMaxDistScale * minDist;
		if (flannMatches[i].distance <= max(distThresh, kFlannMaxDistThreshold)) {
			goodMatches.push_back(flannMatches[i]);
		}
	}

	for (const DMatch& match : goodMatches) {
		const Point2f& kptL = kptsL[match.queryIdx].pt;
		const Point2f& kptR = kptsR[match.trainIdx].pt;
		matchedPair.push_back(std::make_pair(kptL, kptR));
	}

}
void StitchingUtil::matchWithORB(
	const Mat &left, const Mat &right, std::vector<std::pair<Point2f, Point2f>> &matchedPair) {
	static const bool kUseGPU = false;
	static const float kMatchConfidence = 0.4;

	OrbFeaturesFinder finder;

	ImageFeatures imgFeaturesL;
	finder(left, imgFeaturesL);
	imgFeaturesL.img_idx = 0;

	ImageFeatures imgFeaturesR;
	finder(right, imgFeaturesR);
	imgFeaturesR.img_idx = 1;

	vector<ImageFeatures> features;
	features.push_back(imgFeaturesL);
	features.push_back(imgFeaturesR);
	vector<MatchesInfo> pairwiseMatches;
	BestOf2NearestMatcher matcher(kUseGPU, kMatchConfidence);
	matcher(features, pairwiseMatches);

	for (const MatchesInfo& matchInfo : pairwiseMatches) {
		if (matchInfo.src_img_idx != 0 || matchInfo.dst_img_idx != 1) {
			continue;
		}
		for (const DMatch& match : matchInfo.matches) {
			const Point2f& kptL = imgFeaturesL.keypoints[match.queryIdx].pt;
			const Point2f& kptR = imgFeaturesR.keypoints[match.trainIdx].pt;
			matchedPair.push_back(std::make_pair(kptL, kptR));
		}
	}
}

#ifdef OPENCV_3	/* AKAZE Required OPENCV 3.0+ */
void StitchingUtil::matchWithAKAZE(
	const Mat &left, const Mat &right, std::vector<std::pair<Point2f, Point2f>> &matchedPair) {
	static const int kFlannMaxDistScale = 3;
	static const double kFlannMaxDistThreshold = 0.04;

	Mat descL, descR;
	vector<KeyPoint> kptsL, kptsR;
	vector<DMatch> goodMatches;

	Ptr<AKAZE> akaze = AKAZE::create();
	akaze->detectAndCompute(left, noArray(), kptsL, descL);
	akaze->detectAndCompute(right, noArray(), kptsR, descR);

	// FlannBasedMatcher with KD-Trees needs CV_32
	descL.convertTo(descL, CV_32F);
	descR.convertTo(descR, CV_32F);

	// KD-Tree param: # of parallel kd-trees
	static const int kFlannNumTrees = 4;
	FlannBasedMatcher matcher(new flann::KDTreeIndexParams(kFlannNumTrees));
	vector<DMatch> flannMatches;
	matcher.match(descL, descR, flannMatches);

	double maxDist = 0;
	double minDist = std::numeric_limits<float>::max() ;
	for(int i = 0; i < flannMatches.size(); ++i) {
		double dist = flannMatches[i].distance;
		maxDist = max(maxDist, dist);
		minDist = min(minDist, dist);
	}

	for(int i = 0; i < flannMatches.size(); ++i) {
		double distThresh = kFlannMaxDistScale * minDist;
		if (flannMatches[i].distance <= max(distThresh, kFlannMaxDistThreshold)) {
			goodMatches.push_back(flannMatches[i]);
		}
	}

	for (const DMatch& match : goodMatches) {
		const Point2f& kptL = kptsL[match.queryIdx].pt;
		const Point2f& kptR = kptsR[match.trainIdx].pt;
		matchedPair.push_back(std::make_pair(kptL, kptR));
	}

}
#endif

void StitchingUtil::facebookKeyPointMatching(Mat &left, Mat &right, std::vector<std::pair<Point2f, Point2f>> &matchedPair) {
	vector<std::pair<Point2f, Point2f>> matchPointPairsLRAll;
	matchWithBRISK(left, right, matchPointPairsLRAll);
	matchWithORB(left, right, matchPointPairsLRAll);
#ifdef OPENCV_3
	matchWithAKAZE(left, right, matchPointPairsLRAll);
#endif


	// Remove duplicate keypoints
	sort(matchPointPairsLRAll.begin(), matchPointPairsLRAll.end());
	matchPointPairsLRAll.erase(
		std::unique(matchPointPairsLRAll.begin(), matchPointPairsLRAll.end()),
		matchPointPairsLRAll.end());

	// Apply RANSAC to filter weak matches (like, really weak)
	vector<Point2f> matchesL, matchesR;
	vector<uchar> inlinersMask;
	for(int i = 0; i < matchPointPairsLRAll.size(); ++i) {
		matchesL.push_back(matchPointPairsLRAll[i].first);
		matchesR.push_back(matchPointPairsLRAll[i].second);
	}

	static const int kRansacReprojThreshold = 100;
	findHomography(
		matchesL,
		matchesR,
		CV_RANSAC,
		kRansacReprojThreshold,
		inlinersMask);

	for (int i = 0; i < inlinersMask.size(); ++i) {
		if (inlinersMask[i]) {
			matchedPair.push_back(std::make_pair(matchesL[i], matchesR[i]));
		}
	}
}
