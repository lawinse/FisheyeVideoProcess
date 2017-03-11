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
		string srcsName[] = {RESOURCE_PATH + (string)"222.jpg", RESOURCE_PATH + (string)"111.jpg"};
		string dstName = RESOURCE_PATH + (string)"result.jpg";

		for (int i=0; i<srcsCnt; ++i) {
			srcs.push_back(imread(srcsName[i]));
		}
		StitchingUtil::opencvSelfStitching(srcs, dst);
		imwrite(dstName, dst);
	}

	void test2() {
		LOG_ERR("dgsafgfsd" << "dsfafdasf" << "d34634634");
	}


};
