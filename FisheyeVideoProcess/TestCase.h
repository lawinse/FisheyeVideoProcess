#pragma once
#include "Config.h"		
#include "StitchingUtil.h"
#include "OtherUtils\ImageUtil.h"
#include "Supplements\RewarpableWarper.h"
#include "MyLog.h"
#include <algorithm>
using namespace std;
using namespace cv;
using namespace cv::detail;

/* Free to add any testcases */
class TestCase {
public:
	void test1() {
		vector<Mat> srcs;
		Mat dst;
		int srcsCnt = 2;
		string srcsName[] = {RESOURCE_PATH + (string)"dstL.jpg", RESOURCE_PATH + (string)"dstR.jpg"};
		string dstName = RESOURCE_PATH + (string)"result123.jpg";

		for (int i=0; i<srcsCnt; ++i) {
			srcs.push_back(imread(srcsName[i]));
		}
		/*srcs[0] = srcs[0](Range(0, srcs[0].rows), Range(srcs[0].cols/2, srcs[0].cols)).clone();*/
		//srcs.push_back(srcs[1](Range(0, srcs[1].rows), Range(srcs[1].cols/2, srcs[1].cols)).clone());
		//srcs[1] = (srcs[1](Range(0, srcs[1].rows), Range(0,srcs[1].cols/2)).clone());

		//srcs.push_back(srcs[0](Range(0, srcs[0].rows), Range(0,srcs[0].cols/2)).clone());
		StitchingUtil().opencvSelfStitching(srcs, dst, StitchingInfo());
		//srcs.clear();
		//srcs.push_back(dst);
		//srcs.push_back(imread(srcsName[0]));
		
		//StitchingUtil::opencvSelfStitching(srcs, dst, sz);
		imwrite(dstName, dst);
	}

	void test2() {
		std::string s;
		GET_STR(1<<3<<"343",s);
		std::cout << s;
	}

	void test3() {
		cv::Mat input = cv::imread(RESOURCE_PATH+(std::string)"result1.jpg");
		Mat dst;
		StitchingUtil::removeBlackPixel(input, dst, StitchingInfo());
		imshow("dst",dst);
		cvWaitKey();
	}

	void test4() {
		float theta1 = 30*PI/180, theta2 = 35*PI/180;
		std::vector<Mat> rots;
		Mat rotx1 = (Mat_<float>(3,3)<<0.90684992, -4.7476134e-010, 0.42145374, 0.015350031, 0.99933648, -0.033028975, -0.42117411, 0.036421653, 0.90624827);
		//Mat rotx2 = (Mat_<float>(1,9)<<1.0,0.0,0.0,0.0,cos(theta2),-sin(theta2),0.0,sin(theta2),cos(theta2));
		//Mat rotx3 = (Mat_<float>(2,2)<<1,2,3,4);
		//rotx3.copyTo(rotx1(Range(0,2),Range(0,2)));
		//std::cout << rotx1 << std::endl;
		rots.push_back(rotx1.t());
		//rots.push_back(rotx1);
		//rots.push_back(rotx2);
		Mat ret;
		supp::_ProjectorBase::getAverRotationMatrix(rots, ret);
		std::cout << ret << std::endl;
	}

};
