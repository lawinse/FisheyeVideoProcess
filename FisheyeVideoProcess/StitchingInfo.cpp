#include "StitchingUtil.h"
#include "ImageUtil.h"
void StitchingInfo::clear() {
	imgCnt = 0;
	ranges.clear();
	cameras.clear();
	resultRois.clear();
	pltHelpers.clear();
}
StitchingInfo::StitchingInfo(const StitchingInfo &sinfo){
		imgCnt = sinfo.imgCnt, nonBlackRatio = sinfo.nonBlackRatio;
		srcType = sinfo.srcType;
		resizeSz = sinfo.resizeSz;
		maskRatio = sinfo.maskRatio;
		ranges.assign(sinfo.ranges.begin(), sinfo.ranges.end());
		cameras.assign(sinfo.cameras.begin(), sinfo.cameras.end());
		warpData = sinfo.warpData.clone();
		resultRois.assign(sinfo.resultRois.begin(), sinfo.resultRois.end());
		pltHelpers.assign(sinfo.pltHelpers.begin(), sinfo.pltHelpers.end());
}

StitchingInfo &StitchingInfo::operator = (const StitchingInfo &sinfo) {
		if (this == &sinfo) return *this;
		imgCnt = sinfo.imgCnt, nonBlackRatio = sinfo.nonBlackRatio;
		srcType = sinfo.srcType;
		resizeSz = sinfo.resizeSz;
		maskRatio = sinfo.maskRatio;
		ranges.assign(sinfo.ranges.begin(), sinfo.ranges.end());
		cameras.assign(sinfo.cameras.begin(), sinfo.cameras.end());
		warpData = sinfo.warpData.clone();
		resultRois.assign(sinfo.resultRois.begin(), sinfo.resultRois.end());
		pltHelpers.assign(sinfo.pltHelpers.begin(), sinfo.pltHelpers.end());
		return *this;
}



bool StitchingInfo::isNull() const{
	return imgCnt == 0;
}

void StitchingInfo::setRanges(const std::vector<Point> &corners, const std::vector<Size> &sizes) {
	assert(corners.size() == imgCnt);
	ranges = std::vector<Range>(imgCnt);
	const int lmost = corners[0].x;
	
	for (int i=0; i<imgCnt; ++i) {
		ranges[i].start = corners[i].x-lmost;
		ranges[i].end = ranges[i].start + sizes[i].width;
	}

	for (int i=1; i<imgCnt; ++i) {
		if (ranges[i-1].end >= ranges[i].start) {
			int mid = (ranges[i-1].end + ranges[i].start) / 2;
			ranges[i-1].end = ranges[i].start = mid;
		} else {
			LOG_ERR("Stitching is likely to have a seam.")
		}
	}
}

void StitchingInfo::setRanges(const Range &fullImgRange) {
	const int lmost = fullImgRange.start, rmost = fullImgRange.end;
	if (ranges[0].start <= lmost && ranges[0].end >= lmost
		&& ranges[imgCnt-1].start <= rmost && ranges[imgCnt-1].end >= rmost) {
			ranges[0].start = 0;
			for (int i=1; i<imgCnt; ++i) {
				ranges[i-1].end -= lmost;
				ranges[i].start -= lmost;
			}
			ranges[imgCnt-1].end = rmost-lmost;
	} else {
		LOG_ERR("Stitching failed. Will lose component.")
	}
}

std::ostream& operator <<(std::ostream& out, const StitchingInfo& sInfo) {
	out << "\n<STITCHING_INFO>" << std::endl;
	out << " imgCnt: " << sInfo.imgCnt << "\n";
	out << " resizeSz: " <<sInfo.resizeSz << "\n";
	out << " nonBlackRatio: " << sInfo.nonBlackRatio << "\n";
	//for (int i=0; i<sInfo.cameras.size(); ++i) {
	//	out << " Camera #" << i+1 << ":\n" << sInfo.cameras[i].K()<<"\n";
	//	out <<  "Range #" << i+1 << ":" << sInfo.ranges[i].start << "," << sInfo.ranges[i].end << "\n";
	//}
	out << "warpData:\n" << "\n";
	out << sInfo.warpData << "\n";
	/*out << (sInfo.cameras[1].focal-sInfo.cameras[0].focal)*1.0/sInfo.cameras[0].focal<<"\n";*/
	out << "<\\STITCHING_INFO>" << std::endl;
	return out;
}

bool StitchingInfo::isSuccess() const {
	if (isNull()) return false;
	if (cameras.size() == 2) {
		double relative_focal_ratio = abs((cameras[1].focal-cameras[0].focal)*1.0/cameras[0].focal);
		return relative_focal_ratio >= ERR && relative_focal_ratio < 0.1 && nonBlackRatio >= NONBLACK_REMAIN_FLOOR;
	} else {
		return nonBlackRatio >= NONBLACK_REMAIN_FLOOR;
	}
}

double StitchingInfo::evaluate() const{
	// TODO: to refine
	return isSuccess() ? nonBlackRatio : 0.0;
}

bool StitchingInfo::isSuccess(const StitchingInfoGroup &group) {
	bool ret = true;
	for (StitchingInfo info:group)
		if (!info.isSuccess()) return false;
	return ret;
}

double StitchingInfo::evaluate(const StitchingInfoGroup &group) {
	if (!StitchingInfo::isSuccess(group)) {
		return 0.0;
	} else {
		double sum_val = 0.0;
		for (StitchingInfo sinfo:group) sum_val += sinfo.evaluate();
		return sum_val/group.size();
	}
}


int LocalStitchingInfoGroup::push_back(StitchingInfoGroup g) {
	if (endIdx-startIdx<=wSize*2) {
		groups[endIdx] = std::make_pair(g, StitchingInfo::evaluate(g));
		endIdx++;
	} else {
		groups[endIdx] = std::make_pair(g, StitchingInfo::evaluate(g));
		endIdx++;
		startIdx++;
	}
	if (groups.size()>=wSize*4)
		for (int i=endIdx-wSize*4; i<startIdx; ++i)
			groups.erase(i);
	return startIdx;
}

StitchingInfoGroup LocalStitchingInfoGroup::getAver(int head, int tail, std::vector<int> &selectedFrameIdx, StitchingUtil &stitchingUtil) {
	std::vector<std::pair<int,double>> tmp(tail-head);
	for (int i=0; i<tmp.size(); ++i) {tmp[i] = std::make_pair(head + i, groups[head+i].second);}
	std::sort(tmp.begin(), tmp.end(),
		[](const std::pair<int,double> &a, const std::pair<int,double> &b) {return a.second>b.second;});
	int r = LSIG_BEST_NONBLACK_NUM-1;
	for (;r>=0 && tmp[r].second == 0;--r);
	if (r < 0) {LOG_ERR("The local stitching info is entirely bad.");}
	++r;

#define GET_GROUP(i) (groups[tmp[i].first].first)
	// Using the middle scale ones
	if (r > LSIG_SELECT_NUM) {
		for (int i=0; i<r; ++i) {
			if (GET_GROUP(i).size() == 4) {
				tmp[i].second = 
					//(GET_GROUP(i)[0].getLastScale()+GET_GROUP(i)[1].getLastScale())*GET_GROUP(i)[2].getLastScale()*GET_GROUP(i)[3].getLastScale();
					(GET_GROUP(i)[0].getAverFocal()+GET_GROUP(i)[1].getAverFocal())*GET_GROUP(i)[2].getAverFocal()*GET_GROUP(i)[3].getAverFocal();
			} else if (GET_GROUP(i).size() == 2) {
				tmp[i].second = 
					//(GET_GROUP(i)[0].getLastScale()*GET_GROUP(i)[1].getLastScale());
					(GET_GROUP(i)[0].getAverFocal()*GET_GROUP(i)[1].getAverFocal());
			}

		}
		std::sort(tmp.begin(), tmp.begin()+r,
			[](const std::pair<int,double> &a, const std::pair<int,double> &b) {return a.second<b.second;});
		tmp.assign(max(tmp.begin()+r/2-(LSIG_SELECT_NUM+1)/2,tmp.begin()), min(tmp.end(),tmp.begin()+r/2+(LSIG_SELECT_NUM)/2));
		r = tmp.size();
	}


	selectedFrameIdx.clear();
	for (int i=0; i<r; ++i) selectedFrameIdx.push_back(tmp[i].first);


	StitchingInfoGroup ret(GET_GROUP(0).size());
	

	for (int j=0; j<ret.size(); ++j) {
		ret[j].srcType = GET_GROUP(0)[j].srcType;
		ret[j].imgCnt = GET_GROUP(0)[j].imgCnt;
		ret[j].maskRatio = GET_GROUP(0)[j].maskRatio;
		ret[j].cameras = std::vector<cv::detail::CameraParams>(GET_GROUP(0)[j].cameras.size());
		ret[j].warpData = Mat::zeros(GET_GROUP(0)[j].warpData.size(),GET_GROUP(0)[j].warpData.type());
		for (int camidx = 0; camidx <ret[j].cameras.size(); ++camidx) {
			ret[j].cameras[camidx].aspect = 0;
			ret[j].cameras[camidx].focal = 0;
			ret[j].cameras[camidx].ppx = 0;
			ret[j].cameras[camidx].ppy = 0;
			ret[j].cameras[camidx].R = Mat::zeros(GET_GROUP(0)[j].cameras[camidx].R.size(), GET_GROUP(0)[j].cameras[camidx].R.type());
			ret[j].cameras[camidx].t = Mat::zeros(GET_GROUP(0)[j].cameras[camidx].t.size(),GET_GROUP(0)[j].cameras[camidx].t.type());
		}
		for (int i=0; i<r; ++i) {
			/*
			* float warpedImageScale;
			* Size resizeSz;
			* std::vector<cv::detail::CameraParams> cameras
			*/
			ret[j].resizeSz.width += GET_GROUP(i)[j].resizeSz.width;
			ret[j].resizeSz.height += GET_GROUP(i)[j].resizeSz.height;
			ret[j].warpData+= GET_GROUP(i)[j].warpData;
	

			for (int camidx = 0; camidx <ret[j].cameras.size(); ++camidx) {
				ret[j].cameras[camidx].focal += GET_GROUP(i)[j].cameras[camidx].focal;
				ret[j].cameras[camidx].aspect += GET_GROUP(i)[j].cameras[camidx].aspect;
				
				ret[j].cameras[camidx].ppx += GET_GROUP(i)[j].cameras[camidx].ppx;
				ret[j].cameras[camidx].ppy += GET_GROUP(i)[j].cameras[camidx].ppy;
				//LOG_ERR(ret[j].cameras[camidx].R.type() << " " << GET_GROUP(i)[j].cameras[camidx].R.type());
				//LOG_ERR(ret[j].cameras[camidx].t.type() << " " << GET_GROUP(i)[j].cameras[camidx].t.type());
				ret[j].cameras[camidx].R += GET_GROUP(i)[j].cameras[camidx].R;
				ret[j].cameras[camidx].t += GET_GROUP(i)[j].cameras[camidx].t;
			}

		}
		ret[j].resizeSz.width /= r;
		ret[j].resizeSz.height /= r;
		ret[j].warpData/= r;
		for (int camidx = 0; camidx <ret[j].cameras.size(); ++camidx) {
			ret[j].cameras[camidx].focal /= r;
			ret[j].cameras[camidx].aspect /= r;
			ret[j].cameras[camidx].ppx /= r;
			ret[j].cameras[camidx].ppy /= r;
			ret[j].cameras[camidx].R /= double(r);
			ret[j].cameras[camidx].t /= double(r);
		}
		//for (int i=0; i<ret[j].warpData.size(); ++i) {
		//	LOG_MESS(vec2str(ret[j].warpData[i]));
		//}
		//LOG_MESS("at getAver()");
		//system("pause");
	}

	// Adjust the PLT for StitchingInfoGroup
	adjustPltForLSIG(ret, selectedFrameIdx, stitchingUtil);

	return ret;

}

void LocalStitchingInfoGroup::addToWaitingBuff(int fidx, std::vector<Mat>&v) {
	std::vector<Mat> tmpV;
	for (Mat m:v) {
		tmpV.push_back(m.clone());
	}
	stitchingWaitingBuff[fidx] = tmpV;
	if (stitchingWaitingBuff.size() > wSize*4)
		for (int i=fidx-wSize*4; i<=fidx-wSize*2; ++i)
			stitchingWaitingBuff.erase(i);
}

bool StitchingInfo::setToCamerasInternalParam(std::vector<cv::detail::CameraParams> &_cameras) {
	if (_cameras.size() != 0) {LOG_WARN("cameraParams are not null !"); return false;}
	_cameras.assign(cameras.size(), cv::detail::CameraParams());
	for (int i=0; i<cameras.size(); ++i) {
		_cameras[i].focal = cameras[i].focal;
		_cameras[i].aspect = cameras[i].aspect;
		_cameras[i].ppx = cameras[i].ppx;
		_cameras[i].ppy = cameras[i].ppy;
		_cameras[i].R = cameras[i].R.clone();
		_cameras[i].t = cameras[i].t.clone();
	}

	return true;
}

void StitchingInfo::setFromCamerasInternalParam(std::vector<cv::detail::CameraParams> &_cameras) {
	cameras.assign(_cameras.size(), cv::detail::CameraParams());
	for (int i=0; i<cameras.size(); ++i) {
		cameras[i].focal = _cameras[i].focal;
		cameras[i].aspect = _cameras[i].aspect;
		cameras[i].ppx = _cameras[i].ppx;
		cameras[i].ppy = _cameras[i].ppy;
		cameras[i].R = _cameras[i].R.clone();
		cameras[i].t = _cameras[i].t.clone();
	}
}

float StitchingInfo::getWarpScale() const {
	std::vector<float> focals;
	for (int i = 0; i < cameras.size(); ++i) {
		focals.push_back(cameras[i].focal);
	}
	sort(focals.begin(), focals.end());
	return (focals[(focals.size()-1) / 2] + focals[focals.size() / 2]) * 0.5f; 
}

void LocalStitchingInfoGroup::adjustPltForLSIG(StitchingInfoGroup &group, const std::vector<int>&v, StitchingUtil &stitchingUtil) {
	std::unordered_set<int> tmp(v.begin(), v.end());
	if (tmp == resultRoisUsedFrameCur) {
		LOG_MESS("LSIG: resultRoisUsedFrameCur remains the same.")
		for (int i=0; i<group.size(); ++i) {
			group[i].pltHelpers = pltHelperGroup[i];
		}
	} else {
		if (resultRoisUsedFrameBase.empty()) {
			LOG_WARN("LSIG: resultRoisUsedFrameBase is empty, set to" << vec2str(v));
			resultRoisUsedFrameBase = tmp;
			resultRoisUsedFrameCur = resultRoisUsedFrameBase;
			// calc resultRois
			Mat dummydst;
			std::vector<Mat> dummysrcs(group[0].imgCnt,ImageUtil().createDummyMatRGB(group[0].resizeSz, group[0].srcType));
			StitchingInfoGroup out;
#ifdef TRY_CATCH
			try {
#endif
				out = stitchingUtil.doStitch(
						dummysrcs, dummydst, 
						group,
						stitchingUtil.stitchingPolicy,
						stitchingUtil.stitchingType);
#ifdef TRY_CATCH
			} catch(cv::Exception e) {
				LOG_ERR("LSIG: failed at fake doStitching, when Base empty.")
				throw e;
			}
#endif
			for (auto sinfo:out) {
				resultRoisBase.push_back(sinfo.resultRois);
			}
			pltHelperGroup = std::vector<std::vector<supp::PlaneLinearTransformHelper>>(resultRoisBase.size());
			for (int i=0; i<out.size(); ++i) {
				for (int j=0; j<out[i].resultRois.size(); ++j) {
					auto plt = supp::PlaneLinearTransformHelper();
					pltHelperGroup[i].push_back(plt);
					//LOG_MESS("plt: " << plt.ax << ","<< plt.bx << ","<< plt.ay << ","<< plt.by  );
					//system("pause");
				}
			}

		} else {
			LOG_WARN("LSIG: resultRoisUsedFrameCur changes, from" << \
				vec2str(std::vector<int>(resultRoisUsedFrameCur.begin(), resultRoisUsedFrameCur.end())) \
				<< " to" << vec2str(v));
			resultRoisUsedFrameCur = tmp;
			Mat dummydst;
			std::vector<Mat> dummysrcs(group[0].imgCnt,ImageUtil().createDummyMatRGB(group[0].resizeSz, group[0].srcType));
			StitchingInfoGroup out;
#ifdef TRY_CATCH
			try {
#endif
				out = stitchingUtil.doStitch(
						dummysrcs, dummydst, 
						group,
						stitchingUtil.stitchingPolicy,
						stitchingUtil.stitchingType);
#ifdef TRY_CATCH
			} catch(cv::Exception e) {
				LOG_ERR("LSIG: failed at fake doStitching, when Cur changes.")
				throw e;
			}
#endif
			assert(out.size() == resultRoisBase.size());
			pltHelperGroup = std::vector<std::vector<supp::PlaneLinearTransformHelper>>(resultRoisBase.size());
			for (int i=0; i<out.size(); ++i) {
				group[i].pltHelpers.clear();
				assert(out[i].resultRois.size() == resultRoisBase.size());
				for (int j=0; j<out[i].resultRois.size(); ++j) {
					//LOG_MESS("resultRoisBase: " <<  resultRoisBase[i][j].roi );
					//LOG_MESS("out: " <<  out[i].resultRois[j].roi );
					supp::PlaneLinearTransformHelper plt = supp::PlaneLinearTransformHelper::calcPLT(
						resultRoisBase[i][j].roi,out[i].resultRois[j].roi);
					group[i].pltHelpers.push_back(plt);
					pltHelperGroup[i].push_back(plt);
					//LOG_MESS("plt: " << plt.ax << ","<< plt.bx << ","<< plt.ay << ","<< plt.by  );
					//system("pause");
				}
			}

		}


		
	}


}
