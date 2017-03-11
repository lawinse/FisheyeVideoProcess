#pragma once

#include "Config.h"
#include "StitchingUtil.h"
#include "CorrectingUtil.h"

class Processor {
private:
	VideoCapture vCapture[2];	// 0 stands for front and 1 stands for back, maybe more cam
	VideoWriter vWriter;
	#define camCnt 2

	int radiusOfCircle;
	Point2i centerOfCircleBeforeResz;
	Point2i centerOfCircleAfterResz;
	int fps;
	Size dstPanoSize;

	// Utils
	CorrectingUtil correctingUtil;
	StitchingUtil stitchingUtil;
	
	
	void findFisheyeCircleRegion();
	void fisheyeShirnk(Mat &frm);
	void fisheyeCorrect(Mat &src, Mat &dst);
	void panoStitch(std::vector<Mat> &srcs, Mat &dstImage);
	void panoRefine(Mat &, Mat &dstImage);
public:
	Processor();
	~Processor();
	void setPaths(std::string inputPaths[], int inputCnt, std::string outputPath);
	void process(int maxSecCnt = INT_MAX, int startSecond = 0);
	


};