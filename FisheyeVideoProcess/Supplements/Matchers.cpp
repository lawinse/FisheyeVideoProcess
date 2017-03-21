#include "Matchers.h"
using namespace supp;
AKAZEFeaturesFinder::AKAZEFeaturesFinder(int descriptor_type,
										 int descriptor_size,
										 int descriptor_channels,
										 float threshold,
										 int nOctaves,
										 int nOctaveLayers,
										 int diffusivity)
{
	akaze = AKAZE::create(descriptor_type, descriptor_size, descriptor_channels,
						  threshold, nOctaves, nOctaveLayers, diffusivity);
}

void AKAZEFeaturesFinder::find(InputArray image, detail::ImageFeatures &features)
{
	CV_Assert((image.type() == CV_8UC3) || (image.type() == CV_8UC1));
	Mat descriptors;
	UMat uimage = image.getUMat();
	akaze->detectAndCompute(uimage, UMat(), features.keypoints, descriptors);
	features.descriptors = descriptors.getUMat(ACCESS_READ);
}

SIFTFeaturesFinder::SIFTFeaturesFinder(int nfeatures,
									   int nOctaveLayers,
									   double contrastThreshold,
									   double edgeThreshold,
									   double sigma)
{
	sift = cv::xfeatures2d::SIFT::create(nfeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);
}

void SIFTFeaturesFinder::find(InputArray image, detail::ImageFeatures &features)
{
	UMat gray_image;
	CV_Assert((image.type() == CV_8UC3) || (image.type() == CV_8UC1));
	if(image.type() == CV_8UC3)
		cvtColor(image, gray_image, COLOR_BGR2GRAY);
	else
		gray_image = image.getUMat();

	UMat descriptors;
	sift->detectAndCompute(gray_image, Mat(), features.keypoints, descriptors);
	features.descriptors = descriptors.reshape(1, (int)features.keypoints.size());

}
