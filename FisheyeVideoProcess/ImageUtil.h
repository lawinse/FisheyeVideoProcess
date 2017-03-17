#pragma once
#include "Config.h"
class ImageUtil {
public:
	void USM(Mat &src, Mat &dst, double amount=1.0, int thres=0, double sigma=3) {
		Mat blur, tmp2;
		GaussianBlur(src,blur,Size(),sigma, sigma);
		Mat lowContratsMask = abs(src-blur)<thres;
		tmp2 = src*(1+amount)+blur*(-amount);
		src.copyTo(tmp2, lowContratsMask);
		dst = tmp2.clone();
	}

	void LaplaceEnhannce(Mat &src, Mat &dst) {
		Mat k = (Mat_<int>(3,3) <<0,-1,0,-1,5,-1,0,-1,0);
		filter2D(src,dst,src.depth(),k);
	}

};
