#pragma once

#include "Config.h"
#include "StitchingUtil.h"
#include "CorrectingUtil.h"


#define INPUT_FISHEYE_RESIZE Size(INPUT_FISHEYE_LENGTH,INPUT_FISHEYE_LENGTH)
#define OUTPUT_PANO_SIZE Size(INPUT_FISHEYE_LENGTH*2,INPUT_FISHEYE_LENGTH)
class Processor {
#define camCnt 2
private:
	VideoCapture vCapture[camCnt];	// 0 stands for front and 1 stands for back, maybe more cam
	VideoWriter vWriter;
	
	int radiusOfCircle;
	Point2i centerOfCircleBeforeResz;
	Point2i centerOfCircleAfterResz;
	int fps;
	int ttlFrmsCnt;
	Size inputFisheyeResize;
	Size dstPanoSize;

	int curStitchingIdx;	// will have a delay gap between fIndex

	/* Main Utils */
	CorrectingUtil correctingUtil;
	StitchingUtil stitchingUtil;

	/* Pointer of <class LSIG> */
	LocalStitchingInfoGroup *pLSIG;
	
	/* Detect the region of interest of fisheye input */
	void findFisheyeCircleRegion(Mat &);
	/* Blacken the pixel outside fisheye ROI */
	void blackenOutsideRegion(Mat &);
	/* Calibrate fisheye distortedness */
	void fisheyeCorrect(Mat &src, Mat &dst);
	/* Apply some pre-process to input */
	void preProcess(Mat &src, Mat &dst);
	/* Stitch */
	bool panoStitch(std::vector<Mat> &srcs, int frameIdx);
	/* Apply some refinement to pano */
	void panoRefine(Mat &, Mat &dstImage);
	/* Calculate windows boundaries for given fidx */
	void calculateWinSz(int fidx, int &lidx, int &ridx);
	/* Persist final pano to disk */
	void persistPano(bool isFlush = false);

public:
	Processor(LocalStitchingInfoGroup *);
	~Processor();
	/* Set input/output path inpfomation and some initialization */
	void setPaths(std::string inputPaths[], int inputCnt, std::string outputPath);
	/* The whole process flow */
	void process(int maxSecCnt = INT_MAX, int startSecond = 0);
};