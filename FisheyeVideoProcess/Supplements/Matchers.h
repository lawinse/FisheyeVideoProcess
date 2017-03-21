#include "..\Config.h"
#ifdef OPENCV3_CONTRIB
	#include <opencv2\stitching.hpp>
	#include <opencv2\stitching\detail\matchers.hpp>
	#include <opencv2\xfeatures2d\nonfree.hpp>
#endif

#pragma once
namespace supp {
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
}