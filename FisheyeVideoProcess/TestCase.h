#pragma once
#include "Config.h"		
#include "StitchingUtil.h"
#include "MyLog.h"
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
		string srcsName[] = {RESOURCE_PATH + (string)"resultll.jpg", RESOURCE_PATH + (string)"resultrr.jpg"};
		string dstName = RESOURCE_PATH + (string)"result123.jpg";

		for (int i=0; i<srcsCnt; ++i) {
			srcs.push_back(imread(srcsName[i]));
		}
		/*srcs[0] = srcs[0](Range(0, srcs[0].rows), Range(srcs[0].cols/2, srcs[0].cols)).clone();*/
		//srcs.push_back(srcs[1](Range(0, srcs[1].rows), Range(srcs[1].cols/2, srcs[1].cols)).clone());
		//srcs[1] = (srcs[1](Range(0, srcs[1].rows), Range(0,srcs[1].cols/2)).clone());

		//srcs.push_back(srcs[0](Range(0, srcs[0].rows), Range(0,srcs[0].cols/2)).clone());
		StitchingUtil().opencvSelfStitching(srcs, dst, Size(800,700));
		//srcs.clear();
		//srcs.push_back(dst);
		//srcs.push_back(imread(srcsName[0]));
		
		//StitchingUtil::opencvSelfStitching(srcs, dst, sz);
		imwrite(dstName, dst);
	}

	void test2() {
		LOG_ERR("dgsafgfsd" << "dsfafdasf" << "d34634634");
	}


};
