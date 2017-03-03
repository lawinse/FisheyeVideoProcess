#include "StitchingUtil.h"
#ifdef OPENCV_3
	#include <opencv2\stitching\detail\matchers.hpp>
	#include <opencv2\xfeatures2d\nonfree.hpp>
#else
	#include <opencv2\nonfree\features2d.hpp>
	#include <opencv2\nonfree\nonfree.hpp>
#endif
using namespace cv::detail;
void StitchingUtil::opencvSelfStitching(std::vector<Mat> &srcs, Mat &dstImage) {
	assert(srcs.size() == 2);
	Mat left = srcs[0].clone();
	Mat right = srcs[1].clone();
	static const bool kUseGPU = false;
	static const float kMatchConfidence = 0.4;

	detail::OrbFeaturesFinder finder;

	ImageFeatures imgFeaturesL;
	finder(left, imgFeaturesL, getMaskROI(left, true));
	imgFeaturesL.img_idx = 0;

	ImageFeatures imgFeaturesR;
	finder(right, imgFeaturesR, getMaskROI(right,false));
	imgFeaturesR.img_idx = 1;

	std::vector<ImageFeatures> features;
	features.push_back(imgFeaturesL);
	features.push_back(imgFeaturesR);
	std::vector<MatchesInfo> pairwiseMatches;
	BestOf2NearestMatcher matcher(kUseGPU, kMatchConfidence);
	matcher(features, pairwiseMatches);

	HomographyBasedEstimator hEst;
	std::vector<CameraParams> cameras;
	hEst(features, pairwiseMatches, cameras);

	Ptr<detail::BundleAdjusterBase> adjuster = new detail::BundleAdjusterRay();
	adjuster->setConfThresh(1.0f);
	Mat_<uchar> refine_mask = Mat::ones(3,3,CV_8U);
	adjuster->setRefinementMask(refine_mask);
	(*adjuster)(features, pairwiseMatches, cameras);

	std::vector<double> focals;
	for (int i=0; i<cameras.size(); ++i) {
		focals.push_back(cameras[i].focal);
		std::cout << "[Message] Camera #" << i << ": K=" << cameras[i].K() << ", f=" << cameras[i].focal << std::endl;
	}
	sort(focals.begin(), focals.end());

	double warped_image_size = (focals[focals.size()/2] + focals[(focals.size()-1)/2])*0.5;


	// wave coorect
	std::vector<Mat> rmats;
	for (int i=0; i<cameras.size(); ++i)
		rmats.push_back(cameras[i].R);
	waveCorrect(rmats, WAVE_CORRECT_HORIZ);
	for (int i=0; i<cameras.size(); ++i)
		cameras[i].R = rmats[i];

	// warp
	// TODO ...




}