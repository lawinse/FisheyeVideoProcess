#pragma once

#include "Config.h"
#include "StitchingUtil.h"
#include "CorrectingUtil.h"


#define OUTPUT_PANO_SIZE Size(2880,1440)
#define INPUT_FISHEYE_RESIZE Size(1440,1440)
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
	Size inputFisheyeResize;
	Size dstPanoSize;

	int curStitchingIdx;	// will have a gap between fIndex

	// Utils
	CorrectingUtil correctingUtil;
	StitchingUtil stitchingUtil;
	LocalStitchingInfoGroup *pLSIG;
	
	
	void findFisheyeCircleRegion(Mat &);
	void fisheyeCorrect(Mat &src, Mat &dst);
	void preProcess(Mat &src, Mat &dst);
	bool panoStitch(std::vector<Mat> &srcs, int frameIdx);
	void panoRefine(Mat &, Mat &dstImage);
	void calculateWinSz(int fidx, int &lidx, int &ridx);
	void persistPano(bool isFlush = false);
	void blackenOutsideRegion(Mat &);
public:
	Processor(LocalStitchingInfoGroup *);
	~Processor();
	void setPaths(std::string inputPaths[], int inputCnt, std::string outputPath);
	void process(int maxSecCnt = INT_MAX, int startSecond = 0);
	


};