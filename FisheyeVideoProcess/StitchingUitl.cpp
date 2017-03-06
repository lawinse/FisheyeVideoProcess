#include "StitchingUtil.h"
#include <opencv2\features2d\features2d.hpp>
#ifdef OPENCV_3
	#include <opencv2\stitching\detail\matchers.hpp>
	#include <opencv2\xfeatures2d\nonfree.hpp>
#else
	#include <opencv2\nonfree\features2d.hpp>
	#include <opencv2\nonfree\nonfree.hpp>
#endif

using namespace cv::detail;
using namespace cv;
#ifdef OPENCV_3
void StitchingUtil::matchWithBRISK(
	const Mat &left, const Mat &right, std::vector<std::pair<Point2f, Point2f>> &matchedPair) {

	std::vector<KeyPoint> kptsL, kptsR;
	Mat descL, descR;
	std::vector<DMatch> goodMatches;
	const int Thresh = 150, Octave = 3;
	const float PatternScales = 4.0f;
#ifdef OPENCV_3
	Ptr<BRISK> brisk = BRISK::create(Thresh, Octave, PatternScales);
	brisk->detectAndCompute(left, getMask(left,true), kptsL, descL);
	brisk->detectAndCompute(right, getMask(right,false), kptsR, descR);
#else
	//TOSOLVE: 2.4.9-style incovation, not sure it works
	cv::BRISK briskDetector(Thresh, Octave, PatternScales);
	briskDetector.create("Feature2D.BRISK");
	briskDetector.detect(left, kptsL);
	briskDetector.compute(left, kptsL, descL);
	briskDetector.detect(right, kptsR);
	briskDetector.compute(right, kptsR, descR);
#endif

	descL.convertTo(descL, CV_32F);
	descR.convertTo(descR, CV_32F);

	FlannBasedMatcher matcher(new flann::KDTreeIndexParams(kFlannNumTrees));
	std::vector<DMatch> flannMatches;
	matcher.match(descL, descR, flannMatches);

	double maxDist = 0;
	double minDist = std::numeric_limits<float>::max();
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
	showMatchingPair(left, kptsL, right, kptsR, goodMatches);


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

	for (const MatchesInfo& matchInfo : pairwiseMatches) {
		if (matchInfo.src_img_idx != 0 || matchInfo.dst_img_idx != 1) {
			continue;
		}
		for (const DMatch& match : matchInfo.matches) {
			const Point2f& kptL = imgFeaturesL.keypoints[match.queryIdx].pt;
			const Point2f& kptR = imgFeaturesR.keypoints[match.trainIdx].pt;
			matchedPair.push_back(std::make_pair(kptL, kptR));
		}
		showMatchingPair(left, imgFeaturesL.keypoints, right, imgFeaturesR.keypoints, matchInfo.matches);
	}
}


void StitchingUtil::matchWithAKAZE(
	const Mat &left, const Mat &right, std::vector<std::pair<Point2f, Point2f>> &matchedPair) {

	Mat descL, descR;
	std::vector<KeyPoint> kptsL, kptsR;
	std::vector<DMatch> goodMatches;

	Ptr<AKAZE> akaze = AKAZE::create();
	akaze->detectAndCompute(left, getMask(left, true), kptsL, descL);
	akaze->detectAndCompute(right, getMask(right, false), kptsR, descR);

	// FlannBasedMatcher with KD-Trees needs CV_32
	descL.convertTo(descL, CV_32F);
	descR.convertTo(descR, CV_32F);

	// KD-Tree param: # of parallel kd-trees
	FlannBasedMatcher matcher(new flann::KDTreeIndexParams(kFlannNumTrees));
	std::vector<DMatch> flannMatches;
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
	showMatchingPair(left, kptsL, right, kptsR, goodMatches);;
	for (const DMatch& match : goodMatches) {
		const Point2f& kptL = kptsL[match.queryIdx].pt;
		const Point2f& kptR = kptsR[match.trainIdx].pt;
		matchedPair.push_back(std::make_pair(kptL, kptR));
	}

}
#endif

void StitchingUtil::facebookKeyPointMatching(Mat &left, Mat &right, std::vector<std::pair<Point2f, Point2f>> &matchedPair) {
	// input Mat must be grayscale
	assert(left.channels() == 1);
	assert(right.channels() == 1);

	std::vector<std::pair<Point2f, Point2f>> matchPointPairsLRAll;
	//try {
	//	matchWithBRISK(left, right, matchPointPairsLRAll);  /* BRISK seems not so good */
	//} catch(...){
	//	std::cout << "[Warning] matchingWithBrisk failed.";
	//};
	matchWithORB(left, right, matchPointPairsLRAll);
#ifdef OPENCV_3
	matchWithAKAZE(left, right, matchPointPairsLRAll);
#endif


	// Remove duplicate keypoints
	sort(matchPointPairsLRAll.begin(), matchPointPairsLRAll.end(), _cmp_pp2f);
	matchPointPairsLRAll.erase(
		std::unique(matchPointPairsLRAll.begin(), matchPointPairsLRAll.end()),
		matchPointPairsLRAll.end());

	// Apply RANSAC to filter weak matches (like, really weak)
	std::vector<Point2f> matchesL, matchesR;
	std::vector<uchar> inlinersMask;
	for(int i = 0; i < matchPointPairsLRAll.size(); ++i) {
		matchesL.push_back(matchPointPairsLRAll[i].first);
		matchesR.push_back(matchPointPairsLRAll[i].second);
	}

	static const int kRansacReprojThreshold = 80;
	findHomography(
		matchesR,
		matchesL,
		CV_RANSAC,
		kRansacReprojThreshold,
		inlinersMask);

	for (int i = 0; i < inlinersMask.size(); ++i) {
		if (inlinersMask[i]) {
			matchedPair.push_back(std::make_pair(matchesL[i], matchesR[i]));
		}
	}
}

void StitchingUtil::selfKeyPointMatching(Mat &left, Mat &right, std::vector<std::pair<Point2f, Point2f>> &matchedPair, StitchingType sType) {
	// input Mat must be grayscale
	assert(left.channels() == 1);
	assert(right.channels() == 1);

	assert(sType == SELF_SIFT || sType == SELF_SURF);

	Mat desL, desR;
	std::vector<KeyPoint> kptL, kptR;
#if (defined OPENCV_3) && (defined  OPENCV3_CONTRIB)
	if (sType == SELF_SIFT) {
		Ptr<FeatureDetector> DE = cv::xfeatures2d::SIFT::create();
		DE->detectAndCompute(left, getMask(left, true), kptL, desL);
		DE->detectAndCompute(right, getMask(right, false), kptR, desR);
	} else {
		Ptr<FeatureDetector> DE = cv::xfeatures2d::SURF::create();
		DE->detectAndCompute(left, getMask(left, true), kptL, desL);
		DE->detectAndCompute(right, getMask(right, false), kptR, desR);
	}
#elif (defined OPENCV_2)
	if (sType == SELF_SIFT) {
		SiftFeatureDetector siftFD;
		SiftDescriptorExtractor siftDE;
		siftFD.detect(left, kptL);
		siftFD.detect(right, kptR);
		siftDE.compute(left, kptL, desL);
		siftDE.compute(right, kptR, desR);
	} else {
		SurfFeatureDetector surfFD(minHessian);
		SurfDescriptorExtractor surfDE;
		surfFD.detect(left, kptL);
		surfFD.detect(right, kptR);
		surfDE.compute(left, kptL, desL);
		surfDE.compute(right, kptR, desR);
	}
#endif
	FlannBasedMatcher matcher(new flann::KDTreeIndexParams(kFlannNumTrees));
	std::vector<DMatch> matches;
	matcher.match(desL, desR, matches);

	double maxDist = 0;
	double minDist = std::numeric_limits<float>::max();
	double dis;
	for (int i=0; i<desL.rows; ++i) {
		dis = matches[i].distance;
		minDist = min(dis, minDist);
		maxDist = max(dis, maxDist);
	}

	// USe only Good macthes (distance less than 3*minDist)
	std::vector<DMatch> goodMatches;
	for (int i=0; i<desL.rows; ++i) {
		if (matches[i].distance < max(kFlannMaxDistScale*minDist, kFlannMaxDistThreshold))
			goodMatches.push_back(matches[i]);
	}

	showMatchingPair(left, kptL, right, kptR, goodMatches);
	for (int i=0; i<goodMatches.size(); ++i) {
		matchedPair.push_back(std::make_pair(kptL[goodMatches[i].queryIdx].pt, kptR[goodMatches[i].trainIdx].pt));
	}
}

void StitchingUtil::selfStitchingSAfterMatching(
	Mat &left, Mat &right, Mat &leftOri, Mat &rightOri, std::vector<std::pair<Point2f, Point2f>> &matchedPair, Mat &dstImage) {
	// input mat should be colored ones
	std::vector<Point2f> matchedL, matchedR;
	unzipMatchedPair(matchedPair, matchedL, matchedR);

	const double ransacReprojThreshold = 5;
	OutputArray mask=noArray();
	const int maxIters = 2000;
	const double confidence = 0.95;

	Mat H = cv::findHomography(matchedR, matchedL, CV_RANSAC, ransacReprojThreshold, mask, maxIters, confidence);
	std::cout <<"[Message] Homography = \n" << H <<std::endl;
	warpPerspective(rightOri, dstImage, H, Size(leftOri.cols+rightOri.cols, leftOri.rows), INTER_LINEAR);
	Mat half(dstImage, Rect(0,0,leftOri.cols,leftOri.rows));
	leftOri.copyTo(half);

	// TOSOLVE: how to add blend manually
}

Stitcher StitchingUtil::opencvStitcherBuild(StitchingType sType) {
	Stitcher s = Stitcher::createDefault(0);
	if (sType == OPENCV_DEFAULT) return s;
	s.setRegistrationResol(0.3);					//0.6 by default, smaller the faster
	s.setPanoConfidenceThresh(1);					//1 by default, 0.6 or 0.4 worth a try
	s.setWaveCorrection(false);						//true by default, set false for acceleration
	const bool useORB = false;						// ORB is faster but less stable
	s.setFeaturesFinder(useORB ? (FeaturesFinder*)(new OrbFeaturesFinder()) : (FeaturesFinder*)(new SurfFeaturesFinder()));
	s.setFeaturesMatcher(new BestOf2NearestMatcher(false, 0.5f /* 0.65 by default */));
	s.setBundleAdjuster(new BundleAdjusterRay());	// faster
	s.setSeamFinder(new NoSeamFinder);
	s.setExposureCompensator(new NoExposureCompensator);
	s.setBlender(new FeatherBlender);				// multiBandBlender byb default, this is faster
	//s.setWarper(new cv::CylindricalWarper());
	return s;
}


void StitchingUtil::opencvStitching(std::vector<Mat> &srcs, Mat &dstImage, StitchingType sType) {
	assert(sType <= OPENCV_TUNED);
	static Stitcher s = opencvStitcherBuild(sType);
	Stitcher::Status status;
	switch (sType) {
	case OPENCV_DEFAULT:
		status = s.stitch(srcs, dstImage);
		
		if (Stitcher::OK != status) {
			std::cout << "[Error] Cannot stitch the image, errCode = " << status << std::endl;
			assert(false);
		}
		break;
	case OPENCV_TUNED:
		status = s.estimateTransform(srcs);
		if (Stitcher::OK != status) {
			std::cout << "[Error] Cannot stitch the image, error in estimateTranform, errCode = " << status << std::endl;
			assert(false);
		}
		status = s.composePanorama(dstImage);
				if (Stitcher::OK != status) {
			std::cout << "[Error] Cannot stitch the image, error in composePanorama, errCode = " << status << std::endl;
			assert(false);
		}
		break;
	default:
		assert(false);
	}
}

void StitchingUtil::_stitch(std::vector<Mat> &srcs, Mat &dstImage, StitchingType sType) {
	std::vector<Mat> srcsGrayScale;
	std::vector<std::pair<Point2f, Point2f>> matchedPair;
	Mat tmp, tmpGrayScale, tmp2;
	switch (sType) {
	case OPENCV_DEFAULT:
	case OPENCV_TUNED:
		opencvStitching(srcs, dstImage, sType);
		break;
	case OPENCV_SELF_DEV:
		opencvSelfStitching(srcs, dstImage);
		break;
	case FACEBOOK:
	case SELF_SURF:
	case SELF_SIFT:
		getGrayScale(srcs, srcsGrayScale);
		tmp = srcs[0].clone(), tmpGrayScale = srcsGrayScale[0].clone();
		for (int i=1; i<srcs.size(); ++i) {
			matchedPair.clear();
			sType == FACEBOOK
				? facebookKeyPointMatching(tmpGrayScale, srcsGrayScale[i], matchedPair)
				: selfKeyPointMatching(tmpGrayScale, srcsGrayScale[i], matchedPair, sType);
			selfStitchingSAfterMatching(tmpGrayScale, srcsGrayScale[i].clone(), tmp, srcs[i], matchedPair, tmp2);
			tmp = tmp2.clone();
			cvtColor(tmp, tmpGrayScale, CV_RGB2GRAY);
		}
		dstImage = tmp;
		break;
	default:
		assert(false);
	}
}

void StitchingUtil::doStitch(std::vector<Mat> &srcs, Mat &dstImage, StitchingPolicy sp, StitchingType sType) {
	// assumes srcs[0] is the front angle of view, so srcs[1] needs cut
	// TOSOLVE: Currently supports two srcs to stitch
	assert(srcs.size() == 2);		
	std::vector<Mat> matCut;
	Mat tmp, forshow;
	switch(sp) {
	case DIRECT:
		matCut.clear();
		matCut.push_back( // matCut[0]: left after cut
			srcs[1](Range(0, srcs[1].rows), Range(round(srcs[1].cols/2)+1, srcs[1].cols)).clone());
		matCut.push_back( // matCut[1]: right after cut
			srcs[1](Range(0, srcs[1].rows), Range(0, round(srcs[1].cols/2)+1)).clone());

		dstImage.create(srcs[0].rows, srcs[0].cols + matCut[1].cols + matCut[0].cols, srcs[0].type());
		srcs[0].copyTo(dstImage(Rect(matCut[0].cols, 0, srcs[0].cols, srcs[0].rows)));
		matCut[0].copyTo(dstImage(Rect(0, 0, matCut[0].cols, srcs[0].rows)));
		matCut[1].copyTo(dstImage(Rect(matCut[0].cols + srcs[0].cols, 0, matCut[1].cols, srcs[0].rows)));
		break;
	case STITCH_ONE_SIDE:
		//std::swap(srcs[0], srcs[1]);
		_stitch(srcs, dstImage, sType);
		//// stitch another side
		//matCut.clear();
		//matCut.push_back(
		//	tmp(Range(0, tmp.rows), Range(round(tmp.cols*2.0/3), tmp.cols)).clone());
		//resize(dstImage, forshow, Size(matCut[0].cols/2, matCut[0].rows/2));
		//imshow("windows",forshow);
		//matCut.push_back(
		//	tmp(Range(0, tmp.rows), Range(0, round(tmp.cols*2.0/3))).clone());
		//_stitch(matCut, dstImage, sType);
		break;
	default:
		assert(false);
	}
}


void StitchingUtil::getGrayScale(std::vector<Mat> &src, std::vector<Mat> &dst) {
	for (int i=0; i<src.size(); ++i) {
		Mat tmp;
		cvtColor(src[i], tmp, CV_RGB2GRAY);
		dst.push_back(tmp);
	}
	assert(src.size() == dst.size());
}

void StitchingUtil::unzipMatchedPair(
	std::vector< std::pair<Point2f, Point2f> > &matchedPair, std::vector<Point2f> &matchedL, std::vector<Point2f> &matchedR) {
		matchedL.clear(); matchedR.clear();
		for (int i=0; i<matchedPair.size(); ++i) {
			matchedL.push_back(matchedPair[i].first);
			matchedR.push_back(matchedPair[i].second);
		}
}

bool StitchingUtil::_cmp_pp2f(const std::pair<Point2f, Point2f> &a, const std::pair<Point2f, Point2f> &b) {
	return a.first == b.first ? _cmp_p2f(a.second, b.second) : _cmp_p2f(a.first, b.first);
}

bool StitchingUtil::_cmp_p2f(const Point2f &a, const Point2f &b) {
	return a.x == b.x ? a.y < b.y : a.x < b.x;
}

Mat StitchingUtil::getMask(const Mat &srcImage, bool isLeft) {
	Mat mask = Mat::zeros(srcImage.size(), CV_8U);
	Mat roi(mask, getMaskROI(srcImage,isLeft)[0]);
	roi = Scalar(255,255,255);
	return mask;
}
std::vector<Rect> StitchingUtil::getMaskROI(const Mat &srcImage, bool isLeft) {
	const double widthParam = 0.20, heightParam = 0.6;
	std::vector<Rect> ret;
	ret.push_back(isLeft
		? Rect(round(srcImage.cols*(1-widthParam)),0,round(srcImage.cols*widthParam),srcImage.rows*heightParam)
		: Rect(0,0,round(srcImage.cols*widthParam),srcImage.rows*heightParam));
	return ret;
}

void StitchingUtil::showMatchingPair(
	const Mat &left,
	const std::vector<KeyPoint> &kptL,
	const Mat &right,
	const std::vector<KeyPoint> &kptR,
	const std::vector<DMatch> &goodMatches) {
		Mat img_matches;
		drawMatches(left,kptL, right, kptR, goodMatches, img_matches);
		Mat forshow;
		resize(img_matches, forshow, Size(img_matches.cols/3, img_matches.rows/3));
		imshow("MatchSift", forshow);
		waitKey();
}
