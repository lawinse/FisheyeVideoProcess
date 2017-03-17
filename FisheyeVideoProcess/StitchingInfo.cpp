#include "StitchingUtil.h"

void StitchingInfo::clear() {
	imgCnt = 0;
	ranges.clear();
	cameras.clear();
}

bool StitchingInfo::isNull() {
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
	for (int i=0; i<sInfo.cameras.size(); ++i) {
		out << " Camera #" << i+1 << ":\n" << sInfo.cameras[i].K()<<"\n";
		out <<  "Range #" << i+1 << ":" << sInfo.ranges[i].start << "," << sInfo.ranges[i].end << "\n";
	}
	/*out << (sInfo.cameras[1].focal-sInfo.cameras[0].focal)*1.0/sInfo.cameras[0].focal<<"\n";*/
	out << "<\\STITCHING_INFO>" << std::endl;
	return out;
}

bool StitchingInfo::isSuccess() const {
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
	if (endIdx-startIdx<=wSize) {
		groups[endIdx] = std::make_pair(g, StitchingInfo::evaluate(g));
		endIdx++;
	} else {
		groups[endIdx] = std::make_pair(g, StitchingInfo::evaluate(g));
		groups.erase(startIdx);
		endIdx++;
		startIdx++;
	}
	LOG_ERR(StitchingInfo::evaluate(g));
	return startIdx;
}

StitchingInfoGroup LocalStitchingInfoGroup::getAver(int head, int tail) {
	std::vector<std::pair<int,double>> tmp(tail-head);
	for (int i=0; i<tmp.size(); ++i) {tmp[i] = std::make_pair(head + i, groups[head+i].second);}
	std::sort(tmp.begin(), tmp.end(),
		[](const std::pair<int,double> &a, const std::pair<int,double> &b) {return a.second>b.second;});
	int r = LSIG_BEST_NUM-1;
	for (;r>=0 && tmp[r].second == 0;--r);
	if (r < 0) {LOG_ERR("The local stitching info is entirely bad.");}

#define GET_GROUP(i) (groups[tmp[(i)].first].first)
	StitchingInfoGroup ret(GET_GROUP(0).size());
	for (int j=0; j<ret.size(); ++j) {
		ret[j].imgCnt = GET_GROUP(0)[j].imgCnt;
		ret[j].maskRatio = GET_GROUP(0)[j].maskRatio;
		ret[j].cameras = std::vector<cv::detail::CameraParams>(GET_GROUP(0)[j].cameras.size());
		for (int camidx = 0; camidx <ret[j].cameras.size(); ++camidx) {
			ret[j].cameras[camidx].R = Mat::zeros(3,3,CV_32F);
			ret[j].cameras[camidx].t = Mat::zeros(3,1,CV_64F);
		}
		for (int i=0; i<=r; ++i) {
			/*
			* float warpedImageScale;
			* Size resizeSz;
			* std::vector<cv::detail::CameraParams> cameras
			*/
			ret[j].resizeSz.width += GET_GROUP(i)[j].resizeSz.width;
			ret[j].resizeSz.height += GET_GROUP(i)[j].resizeSz.height;

			ret[j].warpedImageScale += GET_GROUP(i)[j].warpedImageScale;

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
		LOG_WARN("Averaging StitchingInfoGroup...")
		ret[j].resizeSz.width /= (r+1);
		ret[j].resizeSz.height /= (r+1);
		ret[j].warpedImageScale /= (r+1);
		for (int camidx = 0; camidx <ret[j].cameras.size(); ++camidx) {
			ret[j].cameras[camidx].focal /= (r+1);
			ret[j].cameras[camidx].aspect /= (r+1);
			ret[j].cameras[camidx].ppx /= (r+1);
			ret[j].cameras[camidx].ppy /= (r+1);
			ret[j].cameras[camidx].R /= double(r+1);
			ret[j].cameras[camidx].t /= double(r+1);
		}
	}

	return ret;

}

void LocalStitchingInfoGroup::addToWaitingBuff(int fidx, std::vector<Mat>&v) {
	std::vector<Mat> tmpV;
	for (Mat m:v) {
		tmpV.push_back(m.clone());
	}
	stitchingWaitingBuff[fidx] = tmpV;
	if (stitchingWaitingBuff.size() > wSize*2) 
		stitchingWaitingBuff.erase(fidx-wSize);
}
