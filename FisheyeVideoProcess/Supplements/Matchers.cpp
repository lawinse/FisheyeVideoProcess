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

void MergeableBestOf2NearestMatcher::match(
	const cv::detail::ImageFeatures &features1, const cv::detail::ImageFeatures &features2, cv::detail::MatchesInfo &matches_info, int pairIdx){

		if (!isInMergeStatus) {
			(*impl_)(features1, features2, matches_info);
			// Check if it makes sense to find homography
			if (matches_info.matches.size() < static_cast<size_t>(num_matches_thresh1_))
				return;
			if (onlyFindMatches) return;
		}

		const cv::detail::ImageFeatures *pImageFeature1;
		const cv::detail::ImageFeatures *pImageFeature2;
		// Construct point-point correspondences for homography estimation
		Mat src_points(1, static_cast<int>(matches_info.matches.size()), CV_32FC2);
		Mat dst_points(1, static_cast<int>(matches_info.matches.size()), CV_32FC2);
		for (int i = 0; i < matches_info.matches.size(); ++i) {
			const DMatch& m = matches_info.matches[i];
			//if (isInMergeStatus) {
			//	pImageFeature1 = &featuresData[featureOwner[std::make_pair(pairIdx,i)].first];
			//	pImageFeature2 = &featuresData[featureOwner[std::make_pair(pairIdx,i)].second];
			//} else {
				pImageFeature1 = &features1;
				pImageFeature2 = &features2;
			//}

			Point2f p = pImageFeature1->keypoints[m.queryIdx].pt;
			p.x -= pImageFeature1->img_size.width * 0.5f;
			p.y -= pImageFeature1->img_size.height * 0.5f;
			src_points.at<Point2f>(0, static_cast<int>(i)) = p;

			p = pImageFeature2->keypoints[m.trainIdx].pt;
			p.x -= pImageFeature2->img_size.width * 0.5f;
			p.y -= pImageFeature2->img_size.height * 0.5f;
			dst_points.at<Point2f>(0, static_cast<int>(i)) = p;
		}

		matches_info.H = findHomography(src_points, dst_points, matches_info.inliers_mask, RANSAC);
		if (matches_info.H.empty() || std::abs(determinant(matches_info.H)) < std::numeric_limits<double>::epsilon())
			return;

		matches_info.num_inliers = 0;
		for (size_t i = 0; i < matches_info.inliers_mask.size(); ++i)
			if (matches_info.inliers_mask[i])
				matches_info.num_inliers++;

		matches_info.confidence = matches_info.num_inliers / (8 + 0.3 * matches_info.matches.size());

		matches_info.confidence = matches_info.confidence > 3. ? 0. : matches_info.confidence;

		if (matches_info.num_inliers < num_matches_thresh2_)
			return;

		src_points.create(1, matches_info.num_inliers, CV_32FC2);
		dst_points.create(1, matches_info.num_inliers, CV_32FC2);
		int inlier_idx = 0;
		for (int i = 0; i < matches_info.matches.size(); ++i) {
			if (!matches_info.inliers_mask[i])
				continue;

			const DMatch& m = matches_info.matches[i];
			//if (isInMergeStatus) {
			//	pImageFeature1 = &featuresData[featureOwner[std::make_pair(pairIdx,i)].first];
			//	pImageFeature2 = &featuresData[featureOwner[std::make_pair(pairIdx,i)].second];
			//} else {
				pImageFeature1 = &features1;
				pImageFeature2 = &features2;
			//}

			Point2f p = pImageFeature1->keypoints[m.queryIdx].pt;
			p.x -= pImageFeature1->img_size.width * 0.5f;
			p.y -= pImageFeature1->img_size.height * 0.5f;
			src_points.at<Point2f>(0, inlier_idx) = p;

			p = pImageFeature2->keypoints[m.trainIdx].pt;
			p.x -= pImageFeature2->img_size.width * 0.5f;
			p.y -= pImageFeature2->img_size.height * 0.5f;
			dst_points.at<Point2f>(0, inlier_idx) = p;

			inlier_idx++;
		}
		matches_info.H = findHomography(src_points, dst_points, RANSAC);
}

int MergeableBestOf2NearestMatcher::mergeMatchesTuple(std::vector<matchesTuple> &mtps, matchesTuple& ret) {
	if (mtps.size() == 1) {
		ret = (mtps[0]);
		return 0;
	}

	int sz = mtps.size();

	// Validate
	auto near_pairs = getNearPairs(*(mtps[0].pfeatures), *(mtps[0].pmask));
	for (int i=1; i<sz; ++i) {
		if (near_pairs != getNearPairs(*(mtps[i].pfeatures), *(mtps[i].pmask))) {
			ret = (mtps[0]);
			LOG_ERR("MergeableBestOf2NearestMatcher:Consistency validation failed in mergeMatchesTuple()");
			return -1;
		}
	}

	// Initialize
	//featureOwner.clear();
	//featuresData.clear();
	//for (int i=0; i<sz; ++i) {
	//	for (int j=0; j<(*(mtps[i].pfeatures)).size(); ++j)
	//		featuresData[std::make_pair(i,j)] = (*(mtps[i].pfeatures))[j];
	//}
	(*ret.pmatchesInfos).resize((*(mtps[0].pmatchesInfos)).size());
	(*ret.pfeatures).resize((*(mtps[0].pfeatures)).size());
	for (int i=0; i<(*ret.pfeatures).size(); ++i) {
		(*ret.pfeatures)[i].img_idx = (*(mtps[0].pfeatures))[i].img_idx;
		(*ret.pfeatures)[i].img_size = (*(mtps[0].pfeatures))[i].img_size;
		(*ret.pfeatures)[i].descriptors = cv::UMat();
	}
	(*ret.pmask) = (*(mtps[0].pmask)).clone();


	//Merge
	int num_images = (*(mtps[0].pfeatures)).size();
	for (int k = 0; k < sz; ++k) {
		for (int i = 0; i < near_pairs.size(); ++i) {
				int from = near_pairs[i].first;
				int to = near_pairs[i].second;
				int pair_idx = from*num_images + to;
				(*ret.pmatchesInfos)[pair_idx].matches.clear();


				for (int j=0; j<(*(mtps[k].pmatchesInfos))[pair_idx].matches.size(); ++j) {
					cv::DMatch m = (*(mtps[k].pmatchesInfos))[pair_idx].matches[j];

					// TODO: Decide to append or replace

					//featureOwner[std::make_pair(pair_idx,(*ret.pmatchesInfos).size())] = 
					//	std::make_pair(
					//		std::make_pair(k,from),
					//		std::make_pair(k,to)
					//		);
					m.queryIdx += (*ret.pfeatures)[from].keypoints.size();
					m.trainIdx += (*ret.pfeatures)[to].keypoints.size();
					(*ret.pmatchesInfos)[pair_idx].matches.push_back(m);
				}
			

		}
		for (int i=0; i<(*ret.pfeatures).size(); ++i) {
			(*ret.pfeatures)[i].keypoints.insert((*ret.pfeatures)[i].keypoints.end(),
				(*(mtps[k].pfeatures))[i].keypoints.begin(),
				(*(mtps[k].pfeatures))[i].keypoints.end());/*
			LOG_MESS("now k i len: " << (*(mtps[k].pfeatures))[i].keypoints.size());
			LOG_MESS("now ttl i len: " << (*ret.pfeatures)[i].keypoints.size());*/
		}

	}
	//system("pause");
	auto cur = time(0);
	return mergeVersionCode = rand() + 0x9e3779b9 + (cur << 6) + (cur >> 2);
}

void MergeableBestOf2NearestMatcher::match(
	const std::vector<cv::detail::ImageFeatures> &features,
	std::vector<cv::detail::MatchesInfo> &pairwise_matches,
	const cv::UMat &mask) {
		
		auto near_pairs = getNearPairs(features, mask);

		cv::RNG rng = cv::theRNG(); // save entry rng state
		const int num_images = static_cast<int>(features.size());
		for (int i = 0; i < near_pairs.size(); ++i) {
			cv::theRNG() = cv::RNG(rng.state + i); // force "stable" RNG seed for each processed pair

			int from = near_pairs[i].first;
			int to = near_pairs[i].second;
			int pair_idx = from*num_images + to;

			match(features[from], features[to], pairwise_matches[pair_idx],pair_idx);
			pairwise_matches[pair_idx].src_img_idx = from;
			pairwise_matches[pair_idx].dst_img_idx = to;

			size_t dual_pair_idx = to*num_images + from;

			pairwise_matches[dual_pair_idx] = pairwise_matches[pair_idx];
			pairwise_matches[dual_pair_idx].src_img_idx = to;
			pairwise_matches[dual_pair_idx].dst_img_idx = from;

			if (!pairwise_matches[pair_idx].H.empty())
				pairwise_matches[dual_pair_idx].H = pairwise_matches[pair_idx].H.inv();

			for (size_t j = 0; j < pairwise_matches[dual_pair_idx].matches.size(); ++j)
				std::swap(pairwise_matches[dual_pair_idx].matches[j].queryIdx,
							pairwise_matches[dual_pair_idx].matches[j].trainIdx);
		}

}

std::vector<std::pair<int,int>> MergeableBestOf2NearestMatcher::getNearPairs(const std::vector<cv::detail::ImageFeatures> &features,const cv::UMat &mask ) {
	const int num_images = static_cast<int>(features.size());

	Mat_<uchar> mask_(mask.getMat(ACCESS_READ));
	if (mask_.empty())
		mask_ = Mat::ones(num_images, num_images, CV_8U);

	std::vector<std::pair<int,int> > near_pairs;
	for (int i = 0; i < num_images - 1; ++i)
		for (int j = i + 1; j < num_images; ++j)
			if (features[i].keypoints.size() > 0 && features[j].keypoints.size() > 0 && mask_(i, j))
				near_pairs.push_back(std::make_pair(i, j));
	return near_pairs;
}