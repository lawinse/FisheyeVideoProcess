#pragma once
#include "Config.h"
class ImageUtil {
public:
	void USM(Mat &src, Mat &dst, double amount=0.0, int thres=0, double sigma=3) {
		Mat blur, tmp2;
		GaussianBlur(src,blur,Size(),sigma, sigma);
		Mat lowContratsMask = abs(src-blur)<thres;
		tmp2 = src*(1+amount)+blur*(-amount);
		src.copyTo(tmp2, lowContratsMask);
		dst = tmp2.clone();
	}
};
