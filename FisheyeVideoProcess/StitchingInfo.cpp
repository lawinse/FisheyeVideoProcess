#include "StitchingUtil.h"
#include "OtherUtils\ImageUtil.h"
#include "OtherUtils\FileUtil.h"
#include "Supplements\Matchers.h"



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
		projData = sinfo.projData.clone();
		resultRois.assign(sinfo.resultRois.begin(), sinfo.resultRois.end());
		pltHelpers.assign(sinfo.pltHelpers.begin(), sinfo.pltHelpers.end());
		features.assign(sinfo.features.begin(), sinfo.features.end());
}

StitchingInfo &StitchingInfo::operator = (const StitchingInfo &sinfo) {
		if (this == &sinfo) return *this;
		imgCnt = sinfo.imgCnt, nonBlackRatio = sinfo.nonBlackRatio;
		srcType = sinfo.srcType;
		resizeSz = sinfo.resizeSz;
		maskRatio = sinfo.maskRatio;
		ranges.assign(sinfo.ranges.begin(), sinfo.ranges.end());
		cameras.assign(sinfo.cameras.begin(), sinfo.cameras.end());
		projData = sinfo.projData.clone();
		resultRois.assign(sinfo.resultRois.begin(), sinfo.resultRois.end());
		pltHelpers.assign(sinfo.pltHelpers.begin(), sinfo.pltHelpers.end());
		features.assign(sinfo.features.begin(), sinfo.features.end());
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
	out << "projData:\n" << "\n";
	out << sInfo.projData << "\n";
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
		double sum_val = 1.0;
		if (group.size() == 4) {
			sum_val *= min(group[0].evaluate(),group[1].evaluate())*group[2].evaluate()*group[3].evaluate();
		} else {
			for (StitchingInfo sinfo:group) sum_val *= sinfo.evaluate();
		}
		return sum_val;
	}
}


void LocalStitchingInfoGroup::push_back(int fidx, StitchingInfoGroup& g) {
	groups.addCandidate(fidx, g);
}

StitchingInfoGroup LocalStitchingInfoGroup::getAver(int head, int tail, std::vector<int> &selectedFrameIdx, StitchingUtil &stitchingUtil) {
	std::vector<std::pair<int,double>> tmp = groups.getBestIdx(head, tail);
	int r = tmp.size()-1;
	for (;r>=0 && tmp[r].second == 0;--r);
	if (r < 0) {LOG_ERR("The local stitching info is entirely bad.");}
	++r;

#define GET_GROUP(i) (groups[tmp[i].first])
	 //Using the middle scale ones
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

	if (
#if LSIG_MOVING_WINDOWS
		selectedFrameIdx.empty() 
		|| !resultRoisUsedFrameCur.empty()
			&& std::unordered_set<int>(selectedFrameIdx.begin(), selectedFrameIdx.end()) == resultRoisUsedFrameCur
#else
		!preSuccessSIG.empty()
#endif
		) {
		LOG_MESS("LSIG: Reuse former SIG, since current selectedFrame is" << vec2str(selectedFrameIdx));
		selectedFrameIdx = std::vector<int>(resultRoisUsedFrameCur.begin(), resultRoisUsedFrameCur.end());
		return preSuccessSIG;
	} else {
		StitchingInfoGroup ret(GET_GROUP(0).size());
		std::vector<StitchingInfoGroup*> SIGs;
		for (int i=0; i<r; ++i) SIGs.push_back(&GET_GROUP(i));
		StitchingInfo::getAverageSIG(SIGs, ret);

		// Adjust the PLT for StitchingInfoGroup
		bool b = stitchingUtil.osParam.isRealStitching;
		stitchingUtil.osParam.isRealStitching = false;
		bool retOK = adjustPltForLSIG(ret, selectedFrameIdx, stitchingUtil);
		stitchingUtil.osParam.isRealStitching = b;

		if (retOK) {
			return preSuccessSIG=ret;
		} else {
			LOG_ERR("LSIG: Reuse former SIG, since current selectedFrame failed " << vec2str(selectedFrameIdx));
			selectedFrameIdx = std::vector<int>(resultRoisUsedFrameCur.begin(), resultRoisUsedFrameCur.end());
			return preSuccessSIG;
		}

	}

}

void LocalStitchingInfoGroup::addToWaitingBuff(int fidx, std::vector<Mat>&v) {
	std::vector<Mat> tmpV;
	for (Mat m:v) tmpV.push_back(m.clone());

	if (stitchingWaitingBuff.size() >= LSIG_MAX_WAITING_BUFF_SIZE) {
		stitchingWaitingBuffPersistedSize[fidx] = tmpV.size();
		FileUtil::persistFrameMats(fidx, tmpV, FileUtil::FILE_STORAGE_MAT_DEFAULT);
	} else
		stitchingWaitingBuff[fidx] = tmpV;
}


void LocalStitchingInfoGroup::dumpWaitingBuffToDisk() {
	for (auto src:stitchingWaitingBuff) {
		stitchingWaitingBuffPersistedSize[src.first] = src.second.size();
		FileUtil::persistFrameMats(src.first, src.second, FileUtil::FILE_STORAGE_MAT_DEFAULT);
	}
	stitchingWaitingBuff.clear();
	LOG_MESS("LocalStitchingInfoGroup: Dump WaitingBuff To Disk.")
}


bool LocalStitchingInfoGroup::getFromWaitingBuff(int fidx, std::vector<Mat>& v) {
	auto ret = stitchingWaitingBuff.find(fidx);
	auto ret1 = stitchingWaitingBuffPersistedSize.find(fidx);
	if (ret != stitchingWaitingBuff.end()) {
		v = (*ret).second; return true;
	} else if (ret1 != stitchingWaitingBuffPersistedSize.end()) {
		int sz = (*ret1).second;
		v = FileUtil::loadFrameMats(fidx, sz, FileUtil::FILE_STORAGE_MAT_DEFAULT);
		return true;
	} else {
		LOG_ERR("Cannot find " << fidx << " frame src data.")
		return false;
	}
}

bool LocalStitchingInfoGroup::isExistInWaitingBuff(int fidx) {
	return stitchingWaitingBuff.find(fidx) != stitchingWaitingBuff.end()
		|| stitchingWaitingBuffPersistedSize.find(fidx) != stitchingWaitingBuffPersistedSize.end();
}


bool LocalStitchingInfoGroup::removeFromWaitingBuff(int fidx) {
	if (stitchingWaitingBuff.find(fidx) != stitchingWaitingBuff.end()) {
		stitchingWaitingBuff.erase(fidx);
		return true;
	}
	else if (stitchingWaitingBuffPersistedSize.find(fidx) != stitchingWaitingBuffPersistedSize.end()) {
		FileUtil::deletePersistedFrameMats(fidx,stitchingWaitingBuffPersistedSize[fidx],FileUtil::FILE_STORAGE_MAT_DEFAULT);
		stitchingWaitingBuffPersistedSize.erase(fidx);
		return true;
	}
	return false;
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

bool LocalStitchingInfoGroup::adjustPltForLSIG(StitchingInfoGroup &group, const std::vector<int>&v, StitchingUtil &stitchingUtil) {
	std::unordered_set<int> tmp(v.begin(), v.end());
	if (tmp == resultRoisUsedFrameCur) {
		LOG_MESS("LSIG: resultRoisUsedFrameCur remains the same.")
		for (int i=0; i<group.size(); ++i) {
			group[i].pltHelpers = pltHelperGroup[i];
		}
		return true;
	} else {
		if (resultRoisUsedFrameBase.empty()) {
			LOG_WARN("LSIG: resultRoisUsedFrameBase is empty, set to" << vec2str(v));
			resultRoisUsedFrameBase = tmp;
			resultRoisUsedFrameCur = resultRoisUsedFrameBase;
			// calc resultRois
			Mat dummydst;
			std::vector<Mat> dummysrcs;//(group[0].imgCnt,ImageUtil::createDummyMatRGB(group[0].resizeSz, group[0].srcType));
			getFromWaitingBuff(v[0],dummysrcs);
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
			if (!StitchingInfo::isSuccess(out)) return false;
			for (int i=0; i<out.size(); ++i) {
				resultRoisBase.push_back(out[i].resultRois);
				group[i] = out[i];
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
			std::vector<Mat> dummysrcs;//(group[0].imgCnt,ImageUtil::createDummyMatRGB(group[0].resizeSz, group[0].srcType));
			getFromWaitingBuff(v[0],dummysrcs);
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
			if (!StitchingInfo::isSuccess(out)) return false;
			for (int i=0; i<out.size(); ++i) {
				group[i] = out[i];
			}
			assert(out.size() == resultRoisBase.size());
			pltHelperGroup = std::vector<std::vector<supp::PlaneLinearTransformHelper>>(resultRoisBase.size());
			for (int i=0; i<out.size(); ++i) {
				assert(out[i].resultRois.size() == resultRoisBase[i].size());
				group[i].pltHelpers.clear(); group[i].pltHelpers.resize(out[i].resultRois.size());
				pltHelperGroup[i].clear(); pltHelperGroup[i].resize(out[i].resultRois.size());
				std::vector<std::vector<std::pair<int,supp::ResultRoi>>> tmpV1(group[i].imgCnt), tmpV2(group[i].imgCnt);
				for (int j=0; j<out[i].resultRois.size(); ++j) {
					tmpV2[out[i].resultRois[j].imgIdx].push_back(std::make_pair(j,out[i].resultRois[j]));
					tmpV1[resultRoisBase[i][j].imgIdx].push_back(std::make_pair(j,resultRoisBase[i][j]));
				}
				for (int j=0; j<tmpV1[0].size(); ++j) {
					// We assume that the imgIdx increases from left to right
					Point2i tl1 = tmpV1[0][j].second.roi.tl(), br1=tmpV1[group[i].imgCnt-1][j].second.roi.br();
					Point2i tl2 = tmpV2[0][j].second.roi.tl(), br2=tmpV2[group[i].imgCnt-1][j].second.roi.br();
					for (int ic = 1; ic<group[i].imgCnt; ++ic) {
						tl1.x = min(tl1.x, tmpV1[ic][j].second.roi.tl().x);
						tl1.y = min(tl1.y, tmpV1[ic][j].second.roi.tl().y);
						br1.x = max(br1.x, tmpV1[ic][j].second.roi.br().x);
						br1.y = max(br1.y, tmpV1[ic][j].second.roi.br().y);

						tl2.x = min(tl2.x, tmpV2[ic][j].second.roi.tl().x);
						tl2.y = min(tl2.y, tmpV2[ic][j].second.roi.tl().y);
						br2.x = max(br2.x, tmpV2[ic][j].second.roi.br().x);
						br2.y = max(br2.y, tmpV2[ic][j].second.roi.br().y);
					}

					auto plt = supp::PlaneLinearTransformHelper::calcPLT(
						Rect(tl1, br1),Rect(tl2,br2));
					for (int ic=0; ic<group[i].imgCnt; ++ic) {
						group[i].pltHelpers[tmpV2[ic][j].first] = plt;
						pltHelperGroup[i][tmpV2[ic][j].first] = plt;
					}

				}


				//for (int j=0; j<out[i].resultRois.size(); ++j) {
				//	//LOG_MESS("resultRoisBase: " <<  resultRoisBase[i][j].roi );
				//	//LOG_MESS("out: " <<  out[i].resultRois[j].roi );
				//	auto plt = group[i].pltHelpers[j];
				//	LOG_MESS("plt: " << plt.ax << ","<< plt.bx << ","<< plt.ay << ","<< plt.by  );
				//	system("pause");
				//}
			}
		}
	}
	return true;
}

void LocalStitchingInfoGroup::addToStitchedBuff(int fidx, Mat& m) {
	stitchedBuff.push_back(std::make_pair(fidx,m.clone()));
	collectGarbage(fidx);
}

void LocalStitchingInfoGroup::collectGarbage(int fidx) {
	int del = groups.getRange().first-1;
	bool ret;
	do {
		ret = removeFromWaitingBuff(del);
		--del;
	}while(ret);

	ret = true;
	for (del=fidx; del>=groups.getRange().second && ret; --del) {
		ret = removeFromWaitingBuff(del);
	}
#if (!LSIG_MOVING_WINDOWS)
	// Tricky
	if (fidx == groups.getRange().second) dumpWaitingBuffToDisk();
#endif
}

void StitchingInfo::getAverageSIG(const std::vector<StitchingInfoGroup*> &pSIGs, StitchingInfoGroup &ret) {
	int r = pSIGs.size();
	if (r == 1) {
		ret.assign((*pSIGs[0]).begin(),(*pSIGs[0]).end()) ;
		return;
	}

	// Check consistency
	for (int i=0;i<ret.size(); ++i) {
		auto st = (*pSIGs[0])[i].srcType, ic = (*pSIGs[0])[i].imgCnt;
		auto csz = (*pSIGs[0])[i].cameras.size();
		auto mr = (*pSIGs[0])[i].maskRatio; 
		auto rs = (*pSIGs[0])[i].resizeSz;
		for (int j=1; j<r; ++j) {
			if (st != (*pSIGs[j])[i].srcType
				|| ic != (*pSIGs[j])[i].imgCnt
				|| csz != (*pSIGs[j])[i].cameras.size()
				|| mr != (*pSIGs[j])[i].maskRatio 
				|| rs != (*pSIGs[j])[i].resizeSz) {
					LOG_ERR("StitchingInfo: Inconsistency encountered in getAverageSIG()");
					ret.assign((*pSIGs[0]).begin(),(*pSIGs[0]).end());
					return;
			}
		}
	}

	// Averaging
	for (int j=0; j<ret.size(); ++j) {
		if (ret.size() == 4 && j>=2) {
			break;
		}
		ret[j].srcType = (*pSIGs[0])[j].srcType;
		ret[j].resizeSz = (*pSIGs[0])[j].resizeSz;
		ret[j].imgCnt = (*pSIGs[0])[j].imgCnt;
		ret[j].maskRatio = (*pSIGs[0])[j].maskRatio;
		ret[j].cameras = std::vector<cv::detail::CameraParams>((*pSIGs[0])[j].cameras.size());

		supp::MergeableBestOf2NearestMatcher matcher(false, OpenCVStitchParam().match_conf);
		matcher.setOnlyFindMatched(true);
		std::vector<supp::matchesTuple> mtps(r);
		std::vector<std::vector<cv::detail::MatchesInfo>> pairMatches(r);
		for (int i=0; i<r; ++i) {
			matcher((*pSIGs[i])[j].features, pairMatches[i]); 
			matcher.collectGarbage();
			mtps[i] = supp::matchesTuple((*pSIGs[i])[j].features, pairMatches[i]);
		}
		std::vector<cv::detail::ImageFeatures> retFeatures;
		std::vector<cv::detail::MatchesInfo> retMatchesInfo;
		supp::matchesTuple retMtp(retFeatures, retMatchesInfo);

		int version = matcher.mergeMatchesTuple(mtps,retMtp);
		if (matcher.verifyMergeVersion(version)) {
			matcher.setIsInMergeStatus(true);
			matcher.setOnlyFindMatched(false);
			matcher.match(*retMtp.pfeatures, *retMtp.pmatchesInfos, *retMtp.pmask);
			matcher.collectGarbage();

			auto estimator = cv::detail::HomographyBasedEstimator();
			estimator(*retMtp.pfeatures, *retMtp.pmatchesInfos, ret[j].cameras);

			//std::vector<std::vector<Mat>> rvec(ret[j].cameras.size());
			//for (int k=0; k<ret[j].cameras.size();++k) {
			//	for (int i=0; i<r; ++i) {
			//		rvec[k].push_back((*pSIGs[i])[j].cameras[k].R);
			//	}
			//	supp::_ProjectorBase::getAverRotationMatrix(rvec[k], ret[j].cameras[k].R);
			//}


			for (size_t i = 0; i < ret[j].cameras.size(); ++i) {
				Mat R;
				ret[j].cameras[i].R.convertTo(R, CV_32F);
				ret[j].cameras[i].R = R;
				//LOG_MESS("Initial intrinsics #" << i+1 << ":\n" << ret[j].cameras[i].K());
				//LOG_MESS("Initial intrinsics R #" << i+1 << ":\n" << ret[j].cameras[i].R);
				//LOG_MESS("Initial intrinsics t #" << i+1 << ":\n" << ret[j].cameras[i].t);
				//system("pause");
			}

			Ptr<detail::BundleAdjusterBase> adjuster;
			adjuster = new detail::BundleAdjusterRay();

			adjuster->setConfThresh(OpenCVStitchParam().conf_thresh);
			Mat_<uchar> refine_mask = Mat::zeros(3, 3, CV_8U);
			refine_mask(0,0) = 1;
			refine_mask(0,1) = 1;
			refine_mask(0,2) = 1;
			refine_mask(1,1) = 1;
			refine_mask(1,2) = 1;
			adjuster->setRefinementMask(refine_mask);
			(*adjuster)(*retMtp.pfeatures, *retMtp.pmatchesInfos, ret[j].cameras);

			std::vector<Mat> rmats;
			for (size_t i = 0; i < ret[j].cameras.size(); ++i)
				rmats.push_back(ret[j].cameras[i].R);
			cv::detail::waveCorrect(rmats, OpenCVStitchParam().wave_correct);
			for (size_t i = 0; i < ret[j].cameras.size(); ++i){
				ret[j].cameras[i].R = rmats[i];
			}
		} else {
			LOG_ERR("MergeableBestOf2NearestMatcher: Inconsistent merge version.");
			ret.assign((*pSIGs[0]).begin(),(*pSIGs[0]).end()) ;
			return;
		}
	}
}
