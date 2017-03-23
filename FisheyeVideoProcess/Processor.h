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
	int ttlFrmsCnt;
	Size dstPanoSize;

	int curStitchingIdx;	// will have a gap between fIndex

	// Utils
	CorrectingUtil correctingUtil;
	StitchingUtil stitchingUtil;
	LocalStitchingInfoGroup *pLSIG;
	
	
	void findFisheyeCircleRegion();
	void fisheyeCorrect(Mat &src, Mat &dst);
	bool panoStitch(std::vector<Mat> &srcs, int frameIdx);
	void panoRefine(Mat &, Mat &dstImage);
	void calculateWind(int fidx, int &lidx, int &ridx);
	void persistPano();
public:
	Processor(LocalStitchingInfoGroup *);
	~Processor();
	void setPaths(std::string inputPaths[], int inputCnt, std::string outputPath);
	void process(int maxSecCnt = INT_MAX, int startSecond = 0);
	


};