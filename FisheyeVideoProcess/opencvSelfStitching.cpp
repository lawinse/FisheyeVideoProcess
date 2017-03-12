#include "StitchingUtil.h"
using namespace cv::detail;
void StitchingUtil::opencvSelfStitching(const std::vector<Mat> &srcs, Mat &dstImage, std::pair<double, double> &maskRatio) {
	Size maxSize = srcs[0].size();
	for (auto src:srcs) {
		if (src.size().area() > maxSize.area()) maxSize = src.size();
	}
	opencvSelfStitching(srcs,dstImage, maxSize , maskRatio);
}

void StitchingUtil::opencvSelfStitching(
	const std::vector<Mat> &srcs, Mat &dstImage, const Size resizeSz,std::pair<double, double> &maskRatio) {



	int imgCnt = srcs.size();
	double work_scale = 1, seam_scale = 1, compose_scale = 1;
	LOG_MESS("Finding features... with MaskRatio (" << maskRatio.first << "," << maskRatio.second <<")");

	Ptr<FeaturesFinder> finder;
	finder = new SurfFeaturesFinder();

	Mat full_img1,full_img, img;
	std::vector<ImageFeatures> features(imgCnt);
	std::vector<Mat> images(imgCnt);
	std::vector<Size> full_img_sizes(imgCnt);
	double seam_work_aspect = 1;

	for (int i = 0; i < imgCnt; ++i) {
		full_img1 = srcs[i].clone();
		//assert(full_img1.size().width >= resizeSz[i].width && full_img1.size().height >= resizeSz[i].height);
		resize(full_img1,full_img, resizeSz);
		full_img_sizes[i] = full_img.size();
		work_scale = min(1.0, sqrt(osParam.workMegapix * 1e6 / full_img.size().area()));

		resize(full_img, img, Size(), work_scale, work_scale);
		seam_scale = min(1.0, sqrt(osParam.seamMegapix * 1e6 / full_img.size().area()));
		seam_work_aspect = seam_scale / work_scale;
		(*finder)(img, features[i],StitchingUtil::getMaskROI(img, i==0, maskRatio));
		features[i].img_idx = i;
		LOG_MESS("Features in image #" << i+1 << ": " << features[i].keypoints.size());
		resize(full_img, img, Size(), seam_scale, seam_scale);
		images[i] = img.clone();
	}

	finder->collectGarbage();
	full_img.release();
	img.release();

	LOG_MESS("Pairwise matching ...");
	std::vector<MatchesInfo> pairwise_matches;
	BestOf2NearestMatcher matcher(false, osParam.match_conf);
	matcher(features, pairwise_matches); 
	matcher.collectGarbage();

	HomographyBasedEstimator estimator;
	std::vector<CameraParams> cameras;
	estimator(features, pairwise_matches, cameras);

	for (size_t i = 0; i < cameras.size(); ++i) {
		Mat R;
		cameras[i].R.convertTo(R, CV_32F);
		cameras[i].R = R;
		LOG_MESS("Initial intrinsics #" << i+1 << ":\n" << cameras[i].K());
	}

	Ptr<detail::BundleAdjusterBase> adjuster;
	adjuster = new detail::BundleAdjusterRay();

	adjuster->setConfThresh(osParam.conf_thresh);
	Mat_<uchar> refine_mask = Mat::zeros(3, 3, CV_8U);
	refine_mask(0,0) = 1;
	refine_mask(0,1) = 1;
	refine_mask(0,2) = 1;
	refine_mask(1,1) = 1;
	refine_mask(1,2) = 1;
	adjuster->setRefinementMask(refine_mask);
	(*adjuster)(features, pairwise_matches, cameras);

	
	std::vector<double> focals;
	for (size_t i = 0; i < cameras.size(); ++i) {
		LOG_MESS("Camera #" << i+1 << ":\n" << cameras[i].K());
		focals.push_back(cameras[i].focal);
	}

	sort(focals.begin(), focals.end());
	float warped_image_scale =(focals[(focals.size()-1) / 2] + focals[focals.size() / 2]) * 0.5f; 

	std::vector<Mat> rmats;
	for (size_t i = 0; i < cameras.size(); ++i)
		rmats.push_back(cameras[i].R);
	waveCorrect(rmats, osParam.wave_correct);
	for (size_t i = 0; i < cameras.size(); ++i)
		cameras[i].R = rmats[i];


	LOG_MESS("Warping images ... ");


	std::vector<Point> corners(imgCnt);
	std::vector<UMat> masks_warped(imgCnt);
	std::vector<UMat> images_warped(imgCnt);
	std::vector<Size> sizes(imgCnt);
	std::vector<Mat> masks(imgCnt);

	for (int i = 0; i < imgCnt; ++i) {
		masks[i].create(images[i].size(), CV_8U);
		masks[i].setTo(Scalar::all(255));
		//masks[i] = getMask(images[i],i==0);
	}

	Ptr<WarperCreator> warper_creator;
	warper_creator = new cv::SphericalWarper();			//TOSOLVE: Not sure

	Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(warped_image_scale * seam_work_aspect));

	for (int i = 0; i < imgCnt; ++i) {
		Mat_<float> K;
		cameras[i].K().convertTo(K, CV_32F);
		float swa = (float)seam_work_aspect;
		K(0,0) *= swa; K(0,2) *= swa;
		K(1,1) *= swa; K(1,2) *= swa;

		corners[i] = warper->warp(images[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warped[i]);//Calculate the unite corner
		sizes[i] = images_warped[i].size();

		warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
	}

	std::vector<UMat> images_warped_f(imgCnt);
	for (int i = 0; i < imgCnt; ++i)
		images_warped[i].convertTo(images_warped_f[i], CV_32F);


	Ptr<ExposureCompensator> compensator = ExposureCompensator::createDefault(osParam.expos_comp_type);
	compensator->feed(corners, images_warped, masks_warped);

	Ptr<SeamFinder> seam_finder;
	seam_finder = new detail::GraphCutSeamFinder(GraphCutSeamFinderBase::COST_COLOR);
	seam_finder->find(images_warped_f, corners, masks_warped);

	images.clear();
	images_warped.clear();
	images_warped_f.clear();
	masks.clear();

	LOG_MESS("Compositing...");


	Mat img_warped, img_warped_s;
	Mat dilated_mask, seam_mask, mask, mask_warped;
	Ptr<Blender> blender;
	
	double compose_work_aspect = 1;

	for (int img_idx = 0; img_idx < imgCnt; ++img_idx) {
		LOG_MESS("Compositing image #" << img_idx+1);
		// reCalculate corner and mask since the former estimation is based on work_scale
		
		full_img1 = srcs[img_idx].clone();
		resize(full_img1,full_img, resizeSz);
		compose_scale = min(1.0, sqrt(osParam.composeMegapix * 1e6 / full_img.size().area()));
		compose_work_aspect = compose_scale / work_scale;
		warped_image_scale *= static_cast<float>(compose_work_aspect);
		warper = warper_creator->create(warped_image_scale);

		// Update
		for (int i = 0; i < imgCnt; ++i) {
			cameras[i].focal *= compose_work_aspect;
			cameras[i].ppx *= compose_work_aspect;
			cameras[i].ppy *= compose_work_aspect;

			Size sz = full_img_sizes[i];
			if (std::abs(compose_scale - 1) > 1e-1) {
				sz.width = round(full_img_sizes[i].width * compose_scale);
				sz.height = round(full_img_sizes[i].height * compose_scale);
			}

			Mat K;
			cameras[i].K().convertTo(K, CV_32F);
			Rect roi = warper->warpRoi(sz, K, cameras[i].R);
			corners[i] = roi.tl();
			sizes[i] = roi.size();
		}
	
		if (abs(compose_scale - 1) > 1e-1)
			resize(full_img, img, Size(), compose_scale, compose_scale);
		else
			img = full_img;
		full_img.release();
		Size img_size = img.size();
	
		Mat K;
		cameras[img_idx].K().convertTo(K, CV_32F);
		warper->warp(img, K, cameras[img_idx].R, INTER_LINEAR, BORDER_REFLECT, img_warped);
		mask.create(img_size, CV_8U);
		mask.setTo(Scalar::all(255));
		warper->warp(mask, K, cameras[img_idx].R, INTER_NEAREST, BORDER_CONSTANT, mask_warped);
		compensator->apply(img_idx, corners[img_idx], img_warped, mask_warped);

		img_warped.convertTo(img_warped_s, CV_16S);
		img_warped.release();
		img.release();
		mask.release();

		dilate(masks_warped[img_idx], dilated_mask, Mat());
		resize(dilated_mask, seam_mask, mask_warped.size());
		mask_warped = seam_mask & mask_warped;
		if (blender.empty()) {
			blender = Blender::createDefault(osParam.blend_type, false);
			Size dst_sz = resultRoi(corners, sizes).size();
			float blend_width = sqrt(static_cast<float>(dst_sz.area())) * osParam.blend_strength / 100.f;
			if (blend_width < 1.f)
				blender = Blender::createDefault(Blender::NO, false);
			else if (osParam.blend_type == Blender::MULTI_BAND) {
				MultiBandBlender * mb = dynamic_cast<MultiBandBlender*>(static_cast<Blender*>(blender));
				mb->setNumBands(static_cast<int>(ceil(log(blend_width)/log(2.0)) - 1.0));
				LOG_MESS("Multi-band blender, number of bands: " << mb->numBands());
			} else if (osParam.blend_type == Blender::FEATHER) {
				FeatherBlender *fb = dynamic_cast<FeatherBlender*>(static_cast<Blender*>(blender));
				fb->setSharpness(1.0/blend_width);
				LOG_MESS("Feather blender, sharpness " << fb->sharpness());
			}

			blender->prepare(corners, sizes);
		}

		blender->feed(img_warped_s, mask_warped, corners[img_idx]);
	}

	Mat result, result_mask, tmp;
	blender->blend(result, result_mask);
	result.convertTo(tmp, CV_8UC3);
	removeBlackPixelByBound(tmp, dstImage);
	LOG_MESS("Size of Pano:" << dstImage.size());
	return;
}
