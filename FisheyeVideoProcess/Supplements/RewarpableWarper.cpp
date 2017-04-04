#include "RewarpableWarper.h"
using namespace supp;

void _ProjectorBase::setCameraParams(InputArray _K, InputArray _R, InputArray _T) {
	if (!isSetAllMatsData()) {
		cv::detail::ProjectorBase::setCameraParams(_K, _R, _T);
		autoSaveProjData();
	}
	else {
		setAllMats(projData[curProjIdx]);
		curProjIdx++;
	}/*
	for (int ii=0; ii<data.size(); ++ii) {
		LOG_MESS(vec2str(data[ii]));
	}
	system("pause");*/
}

void _ProjectorBase::autoSaveProjData() {
	projData.push_back(getAllMats());
	ttlProjTime = projData.size();
}


void _ProjectorBase::setAllMats(std::vector<float>& _d) {
	int cnt = 0;
	scale = _d[cnt++];
	for (int i=0; i<9; ++i) k[i] = _d[cnt++];
	for (int i=0; i<9; ++i) rinv[i] = _d[cnt++];
	for (int i=0; i<9; ++i) r_kinv[i] = _d[cnt++];
	for (int i=0; i<9; ++i) k_rinv[i] =_d[cnt++];
	for (int i=0; i<3; ++i) t[i] = _d[cnt++];
}


void _ProjectorBase::setAllMatsMultiple(std::vector<std::vector<float>> &_data) {
	LOG_MESS("Set projector matrix params manually");
	ttlProjTime = _data.size();
	setAllMatsData(true);
	projData = std::vector<std::vector<float>>(_data.size());
	for (int i=0; i<projData.size(); ++i) 
		projData[i].assign(_data[i].begin(), _data[i].end());
}

std::vector<std::vector<float>> _ProjectorBase::getAllMatsMultiple() {
	std::vector<std::vector<float>> _data;
	if (ttlProjTime > 0) {
		_data = std::vector<std::vector<float>>(projData.size());
		for (int i=0; i<projData.size(); ++i) 
			_data[i].assign(projData[i].begin(), projData[i].end());
	}
	return _data;
}

std::vector<float> _ProjectorBase::getAllMats() {
	std::vector<float> ret;
	ret.push_back(scale);
	for (int i=0; i<9; ++i) ret.push_back(k[i]);
	for (int i=0; i<9; ++i) ret.push_back(rinv[i]);
	for (int i=0; i<9; ++i) ret.push_back(r_kinv[i]);
	for (int i=0; i<9; ++i) ret.push_back(k_rinv[i]);
	for (int i=0; i<3; ++i) ret.push_back(t[i]);
	ret.push_back(curImageIdx);
	return ret;
}

std::vector<float> _ProjectorBase::reCalcCameraParamsAndGetAllMats(float _scale, InputArray K, InputArray R, InputArray T, int cImageIdx) {
	_ProjectorBase pb;
	pb.scale = _scale;
	pb.curImageIdx = cImageIdx;
	pb.setCameraParams(K,R,T);
	return pb.getAllMats();
}



void RewarpableSphericalWarper::detectResultRoi(Size src_size, Point &dst_tl, Point &dst_br) {
	detectResultRoiByBorder(src_size, dst_tl, dst_br);

	float tl_uf = static_cast<float>(dst_tl.x);
	float tl_vf = static_cast<float>(dst_tl.y);
	float br_uf = static_cast<float>(dst_br.x);
	float br_vf = static_cast<float>(dst_br.y);

	float x = projector_.rinv[1];
	float y = projector_.rinv[4];
	float z = projector_.rinv[7];
	if (y > 0.f) {
		float x_ = (projector_.k[0] * x + projector_.k[1] * y) / z + projector_.k[2];
		float y_ = projector_.k[4] * y / z + projector_.k[5];
		if (x_ > 0.f && x_ < src_size.width && y_ > 0.f && y_ < src_size.height) {
			tl_uf = std::min(tl_uf, 0.f); tl_vf = std::min(tl_vf, static_cast<float>(CV_PI * projector_.scale));
			br_uf = std::max(br_uf, 0.f); br_vf = std::max(br_vf, static_cast<float>(CV_PI * projector_.scale));
		}
	}

	x = projector_.rinv[1];
	y = -projector_.rinv[4];
	z = projector_.rinv[7];
	if (y > 0.f) {
		float x_ = (projector_.k[0] * x + projector_.k[1] * y) / z + projector_.k[2];
		float y_ = projector_.k[4] * y / z + projector_.k[5];
		if (x_ > 0.f && x_ < src_size.width && y_ > 0.f && y_ < src_size.height) {
			tl_uf = std::min(tl_uf, 0.f); tl_vf = std::min(tl_vf, static_cast<float>(0));
			br_uf = std::max(br_uf, 0.f); br_vf = std::max(br_vf, static_cast<float>(0));
		}
	}

	dst_tl.x = static_cast<int>(tl_uf);
	dst_tl.y = static_cast<int>(tl_vf);
	dst_br.x = static_cast<int>(br_uf);
	dst_br.y = static_cast<int>(br_vf);
}


Point RewarpableSphericalWarper::warp(InputArray src, InputArray K, InputArray R, int interp_mode, int border_mode, OutputArray dst) {
	UMat uxmap, uymap;
	Rect dst_roi = buildMaps(src.size(), K, R, uxmap, uymap);

	dst.create(dst_roi.height + 1, dst_roi.width + 1, src.type());
	remap(src, dst, uxmap, uymap, interp_mode, border_mode);

	return dst_roi.tl();
}

Point RewarpableCylindricalWarper::warp(InputArray src, InputArray K, InputArray R, int interp_mode, int border_mode, OutputArray dst) {
	UMat uxmap, uymap;
	Rect dst_roi = buildMaps(src.size(), K, R, uxmap, uymap);

	dst.create(dst_roi.height + 1, dst_roi.width + 1, src.type());
	remap(src, dst, uxmap, uymap, interp_mode, border_mode);

	return dst_roi.tl();
}

void _ProjectorBase::getAverRotationMatrix(std::vector<Mat> &rots, Mat & ret) {
	Size inputSize = rots[0].size();
	Mat tmpRot;
	double ttlthetaz=0, ttlthetay=0, ttlthetax=0;
	int cnt = 0;
	if (inputSize.area() == 9) {
		for (int i=0; i<rots.size(); ++i) {
			tmpRot = rots[i].clone().reshape(0,3);
			Mat_<float> it = tmpRot;
			double thetaz = atan2(it(1,0), it(0,0));
			double thetay = atan2(-1 * it(2,0), sqrt(it(2,1)*it(2,1) + it(2,2)*it(2,2)));
			double thetax = atan2(it(2,1), it(2,2));
			if (fabs (thetay) > (M_PI/2.0-ERR)) {
				LOG_ERR("_ProjectorBase: Pitch(Y) axis is paralleled to others. Couldn't solve." << rots[i]);
				continue;
			}
			cnt++;
			ttlthetaz+=thetaz, ttlthetay+=thetay, ttlthetax+=thetax;
		}
		if (cnt > 0) {
			ttlthetax/=cnt;
			ttlthetay/=cnt;
			ttlthetaz/=cnt;
			//Mat rotx = (Mat_<float>(3,3)<<1.0,0.0,0.0,0.0,cos(ttlthetax),-sin(ttlthetax),0.0,sin(ttlthetax),cos(ttlthetax));
			//Mat roty = (Mat_<float>(3,3)<<cos(ttlthetay),0.0,sin(ttlthetay),0.0,1.0,0.0,-sin(ttlthetay),0.0,cos(ttlthetay));
			//Mat rotz = (Mat_<float>(3,3)<<cos(ttlthetaz),-sin(ttlthetaz),0.0,sin(ttlthetaz),cos(ttlthetaz),0.0,0.0,0.0,1.0);
			//ret = rotz*roty*rotx;
			double ca1,cb1,cc1,sa1,sb1,sc1;
			ca1 = cos(ttlthetaz); sa1 = sin(ttlthetaz);
			cb1 = cos(ttlthetay); sb1 = sin(ttlthetay);
			cc1 = cos(ttlthetax); sc1 = sin(ttlthetax);
			tmpRot = (Mat_<double>(3,3)<<\
				ca1*cb1,	ca1*sb1*sc1 - sa1*cc1,	ca1*sb1*cc1 + sa1*sc1,\
				sa1*cb1,	sa1*sb1*sc1 + ca1*cc1,	sa1*sb1*cc1 - ca1*sc1,\
				-sb1,		cb1*sc1,				cb1*cc1);
			tmpRot.convertTo(tmpRot, rots[0].type());
			ret = tmpRot.reshape(0,rots[0].rows);
		} else {
			ret = rots[0].clone();
		}
	} else {
		LOG_ERR("_ProjectorBase: Wrong Size of Rotation matirx when averaging (not area 9).");
		ret = rots[0].clone();
	}


}
