#include "Processor.h"
#include "CorrectingUtil.h"
#include "OtherUtils\ImageUtil.h"
#include "OtherUtils\FileUtil.h"

Processor::Processor(LocalStitchingInfoGroup *_pLSIG) {
	FileUtil::findOrCreateAllDirsNeeded();	// create or validate necessary folders and files
	correctingUtil = CorrectingUtil();
	stitchingUtil = StitchingUtil();
	pLSIG = _pLSIG;
	curStitchingIdx = 0;
	inputFisheyeResize = INPUT_FISHEYE_RESIZE;
	dstPanoSize = OUTPUT_PANO_SIZE;
}

Processor::~Processor() {
	FileUtil::deleteAllTemp();
}

// Calculate window size of given frameidx
void Processor::calculateWinSz(int fidx, int &lidx, int &ridx) {
#if (!LSIG_MOVING_WINDOWS) 
	fidx=0;
#endif
	lidx = max(0, fidx-LSIG_WINDOW_SIZE/2);
	ridx = min(ttlFrmsCnt, lidx + LSIG_WINDOW_SIZE);
	lidx = min(lidx, ridx-LSIG_WINDOW_SIZE);
}


void Processor::findFisheyeCircleRegion(Mat &frm) {
	// TOSOLVE: Simplest estimation,
	// but in fact circle region may move slightly from time to time
	radiusOfCircle = (int)round(frm.size().height/2);
	centerOfCircleBeforeResz.y = radiusOfCircle;
	centerOfCircleBeforeResz.x = (int)round(frm.size().width/2);
}

void Processor::setPaths(std::string inputPaths[], int inputCnt, std::string outputPath) {
	for (int i=0; i<inputCnt; ++i) vCapture[i].open(inputPaths[i]);
	assert(camCnt == inputCnt);
	
	
	vWriter = VideoWriter(
		outputPath, CV_FOURCC('D', 'I', 'V', 'X'),
		fps = vCapture[0].get(CV_CAP_PROP_FPS), dstPanoSize);

	std::cout << "[BASIC INFO]" << std::endl;
	std::cout << "INPUT: (FPS=" << fps << ")" <<  std::endl;
	for (int i=0; i<inputCnt; ++i) std::cout << "\t" << inputPaths[i] << std::endl;
	std::cout << "OUTPUT: " << std::endl; 
	std::cout << "\t" << outputPath << std::endl;
}

void Processor::fisheyeCorrect(Mat &src, Mat &dst) {
	//TODO: To apply different type of correction
	CorrectingParams cp = CorrectingParams(
		PERSPECTIVE_LONG_LAT_MAPPING_CAM_LENS_MOD_REVERSED,
		centerOfCircleAfterResz,
		radiusOfCircle,
		LONG_LAT);
	//cp.use_reMap = false;
	//cp.w = Point2d(90*PI/180, 90*PI/180);
	correctingUtil.doCorrect(src, dst, cp);
}

// Return value indicates whether curStitchingIdx in move forward
bool Processor::panoStitch(std::vector<Mat> &srcs, int frameIdx) {
	StitchingInfoGroup sInfoGIN;
	StitchingInfoGroup sInfoGOUT;

	StitchingPolicy sp = StitchingPolicy::STITCH_DOUBLE_SIDE;
	StitchingType sType = StitchingType::OPENCV_SELF_DEV;

	stitchingUtil.stitchingPolicy = sp;
	stitchingUtil.stitchingType = sType;

	pLSIG->addToWaitingBuff(frameIdx, srcs);
	std::vector<Mat> vmat, modifiedSrcs(srcs);
	//ImageUtil::batchOperation(modifiedSrcs, modifiedSrcs, &ImageUtil::equalizeHistBGR);
	Mat dummy, tmpDst;
	int leftIdx, rightIdx;
	calculateWinSz(curStitchingIdx, leftIdx, rightIdx);
#ifdef TRY_CATCH
	try {
#endif
		stitchingUtil.osParam.isRealStitching = false;
		if (!pLSIG->cover(leftIdx, rightIdx)) {
			sInfoGOUT = stitchingUtil.doStitch(
					modifiedSrcs, dummy, 
					sInfoGIN,
					sp,
					sType);
			pLSIG->push_back(frameIdx,sInfoGOUT);
		}

		
#ifdef TRY_CATCH
	} catch(cv::Exception e) {
		LOG_ERR("Stitching failed at " << frameIdx << " frame.");
		pLSIG->push_back(frameIdx,sInfoGOUT);
	}
#endif

	calculateWinSz(curStitchingIdx, leftIdx, rightIdx);
	if  (!pLSIG->cover(leftIdx, rightIdx)) {
		LOG_WARN("StitchingBuff does not cover the need. Required:" <<leftIdx<<"-"<<rightIdx <<\
			", Current:" << pLSIG->getCovered().first << "-" << pLSIG->getCovered().second);
		return false;
	} else {
		std::vector<int> selFrame;
		stitchingUtil.osParam.isRealStitching = true;
		do {
			bool b = pLSIG->getFromWaitingBuff(curStitchingIdx, vmat);
			assert(b);
			sInfoGIN = pLSIG->getAver(leftIdx, rightIdx, selFrame, stitchingUtil);
			LOG_MESS("Stitching "<< curStitchingIdx << " frame using " <<vec2str(selFrame) << "frames.");
			stitchingUtil.doStitch(
				vmat, tmpDst, 
				sInfoGIN,
				sp,
				sType);
			panoRefine(tmpDst, tmpDst);
			pLSIG->addToStitchedBuff(curStitchingIdx, tmpDst);
			LOG_MARK("Done stitching " << curStitchingIdx << " frame.");
			persistPano();
			calculateWinSz(++curStitchingIdx, leftIdx, rightIdx);
		} while(curStitchingIdx<ttlFrmsCnt
			&& pLSIG->cover(leftIdx, rightIdx)
			&& pLSIG->isExistInWaitingBuff(curStitchingIdx));
		return true;
	}
}

void Processor::panoRefine(Mat &srcImage, Mat &dstImage) {
	Mat tmp, tmp2;
	tmp = srcImage.clone();
	ImageUtil::resize(tmp, tmp, dstPanoSize,0,0);
	//ImageUtil::equalizeHistBGR(tmp,tmp);
	ImageUtil::brightnessAndContrastAuto(tmp,tmp);
	ImageUtil::USM(tmp,tmp);
	//ImageUtil::LaplaceEnhannce(tmp,tmp);
	dstImage = tmp.clone();
}

void Processor::process(int maxSecondsCnt, int startFrame) {
	std::vector<Mat> srcFrms(camCnt);
	std::vector<Mat> dstFrms(camCnt);
	ttlFrmsCnt = fps*(maxSecondsCnt)+startFrame;
	int fIndex = 0;
	while (fIndex < startFrame) {
		Mat tmp;
		for (int i=0; i<camCnt; ++i) {
			vCapture[i] >> tmp;
		}
		fIndex++;
	}

	while (fIndex < ttlFrmsCnt) {
		// frame by frame
		LOG_MARK("Processing " << fIndex  << "/" << ttlFrmsCnt-1 << " frame ...");
#ifdef TRY_CATCH
		try {
#endif
			std::vector<Mat> tmpFrms(camCnt);
			Mat dstImage;

			for (int i=0; i<camCnt; ++i) {
				vCapture[i] >> tmpFrms[i];
				if (tmpFrms[i].empty()) break;
				preProcess(tmpFrms[i], tmpFrms[i]);
		
				/* Restrict to square frame */
				srcFrms[i] = tmpFrms[i](
					/* row */
					Range(centerOfCircleBeforeResz.y-radiusOfCircle, centerOfCircleBeforeResz.y+radiusOfCircle),
					/* col */
					Range(centerOfCircleBeforeResz.x-radiusOfCircle, centerOfCircleBeforeResz.x+radiusOfCircle))
					.clone();	// must use clone()
		
			
				dstFrms[i].create(srcFrms[i].rows, srcFrms[i].cols, srcFrms[i].type());
			}

			// Hardcode: Use 1st to set centerOfCircleAfterResz
			static bool isSetCenter = false;
			if (!isSetCenter) {
				centerOfCircleAfterResz.x = srcFrms[0].cols/2;
				centerOfCircleAfterResz.y = srcFrms[0].rows/2;
				isSetCenter = true;
			}
			std::cout << "\tCorrecting ..." <<std::endl;
			for (int i=0; i<camCnt; ++i) {
				/*blackenOutsideRegion(srcFrms[i]);*/
				fisheyeCorrect(srcFrms[i], dstFrms[i]);
				//ImageUtil::imshow("dstFrm",dstFrms[i],0.5,true);
				
			}
			std::cout << "\tStitching ..." <<std::endl;
			panoStitch(dstFrms, fIndex);
#ifdef TRY_CATCH
		} catch (cv::Exception e) {
			
			LOG_ERR("process "<< fIndex  << "/" << ttlFrmsCnt << " frame: " <<e.what());
		} catch (...) {
			LOG_ERR("process "<< fIndex  << "/" << ttlFrmsCnt << " frame: UNKNOWN");
		}
#endif
		
		++fIndex;

	}
	persistPano(true);	//final flush
}

void Processor::persistPano(bool isFlush) {
	if (!pLSIG->isStitchedBuffFull() && !isFlush) return; 
	auto buf = pLSIG->getStitchedBuff();
	for (int dsti=0; dsti<buf->size(); ++dsti) {
		auto p = buf->at(dsti);
		int fidx = p.first;
		Mat dstImage = p.second;
				
#ifdef SHOW_IMAGE
		Mat forshow;
		ImageUtil::resize(dstImage, forshow, Size(1400, 700));
		imshow("windows11",forshow);
		cvWaitKey();
#endif
			
		std::string dstname;
		GET_STR(OUTPUT_PATH << fidx << ".jpg", dstname);
		LOG_MESS("Persisting " << fidx << ".jpg.");
		imwrite(dstname, dstImage);

		vWriter << dstImage;
	}
	pLSIG->clearStitchedBuff();
}

void Processor::preProcess(Mat &src, Mat &dst) {
	static bool isFoundFisheyeRegion = false;
	ImageUtil::resize(src, dst, inputFisheyeResize);
	if (!isFoundFisheyeRegion) {
		findFisheyeCircleRegion(dst);
		isFoundFisheyeRegion = true;
	}
}

void Processor::blackenOutsideRegion(Mat &src) {
	Mat_<Vec3b> tmpSrc =src;
	for (int i=0; i<tmpSrc.size().width; ++i) 
		for (int j=0; j<tmpSrc.size().height; ++j) {
			if (square(i-centerOfCircleAfterResz.x) + square(j-centerOfCircleAfterResz.y) > square(radiusOfCircle)*1.01)
				tmpSrc(Point(i,j)) = Vec3b(0,0,0);
		}

}
