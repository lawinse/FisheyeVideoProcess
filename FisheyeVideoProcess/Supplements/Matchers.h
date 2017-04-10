#include "..\Config.h"
#include <unordered_map>
#include <set>
#include <map>
#ifdef OPENCV3_CONTRIB
	#include <opencv2\stitching.hpp>
	#include <opencv2\stitching\detail\matchers.hpp>
	#include <opencv2\xfeatures2d\nonfree.hpp>
#endif

#pragma once
namespace supp {
	/* cv:detail::FeaturesFinder using AKAZE */
	class  AKAZEFeaturesFinder : public detail::FeaturesFinder {
	public:
		AKAZEFeaturesFinder(int descriptor_type = AKAZE::DESCRIPTOR_MLDB,
							int descriptor_size = 0,
							int descriptor_channels = 3,
							float threshold = 0.001f,
							int nOctaves = 4,
							int nOctaveLayers = 4,
							int diffusivity = KAZE::DIFF_PM_G2);

	private:
		void find(InputArray image, detail::ImageFeatures &features);

		Ptr<AKAZE> akaze;
	};

	/* cv::detail::FeaturesFinder using SIFT */
	class SIFTFeaturesFinder : public detail::FeaturesFinder {
	public:
		SIFTFeaturesFinder(int nfeatures = 0,
						   int nOctaveLayers = 3,
						   double contrastThreshold = 0.04,
						   double edgeThreshold = 10,
						   double sigma = 1.6);

	private:
		void find(InputArray image, detail::ImageFeatures &features);

		Ptr<cv::xfeatures2d::SIFT> sift;
	};

	/* Struct to wrap matcher.match input params */
	struct matchesTuple {
		std::vector<cv::detail::ImageFeatures> *pfeatures;
		std::vector<cv::detail::MatchesInfo> *pmatchesInfos;
		cv::UMat * pmask;
		matchesTuple() {pfeatures = NULL; pmatchesInfos = NULL; pmask = NULL;}
		matchesTuple(
			std::vector<cv::detail::ImageFeatures>&_f,
			std::vector<cv::detail::MatchesInfo> &_mi,
			cv::UMat &_m = cv::UMat()) {
				pfeatures = &_f;
				pmatchesInfos = &_mi;
				pmask = &_m;
		}
	};

	/* cv::detail::BestOf2NearestMatcher appended with matched pairs merging function */
	class MergeableBestOf2NearestMatcher : public cv::detail::BestOf2NearestMatcher {
	private:
		bool onlyFindMatches;
		bool isInMergeStatus;
		//std::unordered_map<std::pair<int,int>,std::pair<std::pair<int,int>,std::pair<int,int>>, pairhash> featureOwner;
		//std::unordered_map<std::pair<int,int>, cv::detail::ImageFeatures, pairhash> featuresData;

		int mergeVersionCode;	// To ensure consistency
		/* Get idx pairs needed to find matches */
		std::vector<std::pair<int,int>> getNearPairs(const std::vector<cv::detail::ImageFeatures> &features,const cv::UMat &mask);
	public:
		MergeableBestOf2NearestMatcher(bool try_use_gpu = false, float match_conf = 0.3f, int num_matches_thresh1 = 6,
			int num_matches_thresh2 = 6)
			:cv::detail::BestOf2NearestMatcher(try_use_gpu,match_conf,num_matches_thresh1,num_matches_thresh2){
			onlyFindMatches = isInMergeStatus = false;
		}
		/* Set whether only to find matched pairs without filtering inliers*/
		void setOnlyFindMatched(bool _b) {onlyFindMatches = _b;}
		/* Set whether it is contains multiple matched pairs from merging op */
		void setIsInMergeStatus(bool _b) {isInMergeStatus = _b;}
		/* Deprecated. Intend to verify consistence */
		bool verifyMergeVersion(int code) {return mergeVersionCode == code;}
		void match(
			const cv::detail::ImageFeatures &features1,
			const cv::detail::ImageFeatures &features2,
			cv::detail::MatchesInfo &matches_info,
			int pairIdx = -1);
		void match(const std::vector<cv::detail::ImageFeatures> &features, std::vector<cv::detail::MatchesInfo> &pairwise_matches, const cv::UMat &mask=cv::UMat());
		/* Merge op */
		int mergeMatchesTuple(std::vector<matchesTuple> &mtps, matchesTuple &);
	};
}
