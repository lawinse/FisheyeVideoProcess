#pragma once

#include "Config.h"

class Processor {
private:
	VideoCapture vCapture[2];	// 0 stands for front and 1 stands for back, maybe more cam
	VideoWriter vWriter;
	const int camCnt = 2;

	int radiusOfCircle;
	Point2i centerOfCircleBeforeResz;
	Point2i centerOfCircleAfterResz;
	
	
	void findFisheyeCircleRegion();
	void fisheyeShirnk(Mat &frm);
	void fisheyeCorrect(Mat &src, Mat &dst);
	void panoStitch(std::vector<Mat> srcs, Mat &dstImage);
public:
	Processor();
	void setPaths(std::string inputPaths[], int inputCnt, std::string outputPath);
	void process();
	


};