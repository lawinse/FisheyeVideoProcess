#include "RewarpableWarper.h"
using namespace supp;

void _ProjectorBase::setCameraParams(InputArray _K, InputArray _R, InputArray _T) {
	if (!isSetAllMatsData()) {
		cv::detail::ProjectorBase::setCameraParams(_K, _R, _T);
		autoSaveProjData();
	}
	else {
		setAllMats(data[curProjIdx]);
		curProjIdx++;
	}/*
	for (int ii=0; ii<data.size(); ++ii) {
		LOG_MESS(vec2str(data[ii]));
	}
	system("pause");*/
}

void _ProjectorBase::autoSaveProjData() {
	data.push_back(getAllMats());
	ttlProjTime = data.size();
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
	data = std::vector<std::vector<float>>(_data.size());
	for (int i=0; i<data.size(); ++i) 
		data[i].assign(_data[i].begin(), _data[i].end());
}

std::vector<std::vector<float>> _ProjectorBase::getAllMatsMultiple() {
	std::vector<std::vector<float>> _data;
	if (ttlProjTime > 0) {
		_data = std::vector<std::vector<float>>(data.size());
		for (int i=0; i<data.size(); ++i) 
			_data[i].assign(data[i].begin(), data[i].end());
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
	return ret;
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
