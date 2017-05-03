#pragma once
#include "..\Config.h"
class ImageUtil {
public:
	/* USM sharpening process */
	static void USM(Mat &src, Mat &dst) {
		Mat blur, tmp2; 
		double amount=1.0;
		int thres=0;
		double sigma=3;

		GaussianBlur(src,blur,Size(),sigma, sigma);
		Mat lowContratsMask = abs(src-blur)<thres;
		tmp2 = src*(1+amount)+blur*(-amount);
		src.copyTo(tmp2, lowContratsMask);
		dst = tmp2;
	}

	/* Laplace enhancement process */
	static void LaplaceEnhannce(Mat &src, Mat &dst) {
		Mat k = (Mat_<int>(3,3) <<0,-1,0,-1,5,-1,0,-1,0);
		filter2D(src,dst,src.depth(),k);
	}

	/* Create an all-black 3-channels Mat of given size and type */
	static Mat createDummyMatRGB(Size sz, int src_type) {
		Mat m(sz, src_type,Scalar(255,255,255));
		return m;
	}

	/* Auto-adjust resize op */
	inline static void ImageUtil::resize(InputArray src, OutputArray dst, Size dsize, double fx = 0, double fy = 0) {
		src.size().area() > dsize.area()
			? cv::resize(src, dst, dsize, fx, fy, CV_INTER_AREA)
			: cv::resize(src, dst, dsize, fx, fy, CV_INTER_CUBIC);
	}

	inline static void ImageUtil::resizeHeightBased(Mat &src, Mat &dst, int height) {
		double ratio = height*1.0/src.size().height;
		int width = src.size().width*ratio;
		ImageUtil::resize(src,dst,Size(width,height));
	}

	static void batchResize(std::vector<Mat> &srcs, std::vector<Mat> &dsts, Size dsize) {
		dsts.resize(srcs.size());
		for (int i=0; i<srcs.size(); ++i) {
			ImageUtil::resize(srcs[i],dsts[i],dsize);
		}
	}

	static void batchResizeHeightBased(std::vector<Mat> &srcs, std::vector<Mat> &dsts, int height) {
		dsts.resize(srcs.size());
		for (int i=0; i<srcs.size(); ++i) {
			ImageUtil::resizeHeightBased(srcs[i],dsts[i],height);
		}
	}


	/* More conveneient way to use cv::imshow */
	static void imshow(char * winName, Mat img, double ratio = 1.0, bool holdon = false) {
		Mat tmp;
		ImageUtil::resize(img,tmp,Size(img.cols*ratio, img.rows*ratio));
		cv::imshow(winName, tmp);
		if (holdon) cvWaitKey();
	}

	/* BGR-Equalization process */
	static void equalizeHistBGR(Mat &src, Mat &dst) {
		if (src.channels()<3) {
			LOG_WARN("ImageUtil: failed in equalizeHistBGR (less than 3 channels)");
			dst = src.clone();
		} else {
			cvtColor(src, dst, COLOR_BGR2YCrCb);
			std::vector<Mat> channels;
			cv::split(dst, channels);
			cv::equalizeHist(channels[0], channels[0]);
			cv::merge(channels,dst);
			cvtColor(dst,dst,COLOR_YCrCb2BGR);
		}
	}

	/* Batch ops supported */
	static void batchOperation(std::vector<Mat> &srcs, std::vector<Mat> &dsts, void (*op)(Mat &src, Mat &dst)) {
		if (dsts.size() != srcs.size()) dsts = std::vector<Mat>(srcs.size());
		for (int i=0; i<srcs.size(); ++i) op(srcs[i],dsts[i]);
	}


};
