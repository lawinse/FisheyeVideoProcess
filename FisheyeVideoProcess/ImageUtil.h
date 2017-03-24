#pragma once
#include "Config.h"
class ImageUtil {
public:
	static void USM(Mat &src, Mat &dst, double amount=1.0, int thres=0, double sigma=3) {
		Mat blur, tmp2;
		GaussianBlur(src,blur,Size(),sigma, sigma);
		Mat lowContratsMask = abs(src-blur)<thres;
		tmp2 = src*(1+amount)+blur*(-amount);
		src.copyTo(tmp2, lowContratsMask);
		dst = tmp2;
	}

	static void LaplaceEnhannce(Mat &src, Mat &dst) {
		Mat k = (Mat_<int>(3,3) <<0,-1,0,-1,5,-1,0,-1,0);
		filter2D(src,dst,src.depth(),k);
	}

	static Mat createDummyMatRGB(Size sz, int src_type) {
		Mat m(sz, src_type,Scalar(255,255,255));
		return m;
	}

	inline static void ImageUtil::_resize_(InputArray src, OutputArray dst, Size dsize, double fx = 0, double fy = 0) {
		src.size().area() > dsize.area()
			? cv::resize(src, dst, dsize, fx, fy, CV_INTER_AREA)
			: cv::resize(src, dst, dsize, fx, fy, CV_INTER_LINEAR);
	}

	static void imshow(char * winName, Mat img, double ratio = 1.0) {
		Mat tmp;
		ImageUtil::_resize_(img,tmp,Size(img.cols*ratio, img.rows*ratio));
		cv::imshow(winName, tmp);
	}


};
