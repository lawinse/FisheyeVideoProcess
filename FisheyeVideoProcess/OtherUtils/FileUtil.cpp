#include "FileUtil.h"
std::vector<std::string> FileUtil::waitToDeleteBuff = std::vector<std::string>();

bool FileUtil::findOrCreateDir(const char * path) {
	if (access(path,0) == -1) {
		int flg = mkdir(path);
		if (flg == 0) return true;
		else {
			LOG_ERR("FileUtil: Creating " << path << " failed.");
			return false;
		}
	} else {
		return true;
	}

}

bool FileUtil::deleteFile(const char * fn) {
	if(!_access(fn,0)) {
		if (remove(fn)) {
			LOG_ERR("FileUtil: Failed when deleting " << fn << " (UNKNOWN)");
			return false;
		} else return true;
	} else {
		LOG_ERR("FileUtil: Failed when deleting " << fn << " (file not exist)");
		return false;
	}
}

std::string FileUtil::getFileNameByFidx(int fidx, std::string elseInfo, std::string extension) {
	std::string rets;
	GET_STR(TEMP_PATH << runtimeHashCode << '_' << fidx <<elseInfo<<extension,rets);
	return rets;
}

std::string FileUtil::getMatNameByMatidx(int fidx, int midx) {
	std::string rets;
	GET_STR("M"<< runtimeHashCode<< fidx  << midx,rets);
	return rets;
}

bool FileUtil::findOrCreateAllDirsNeeded() {
	if (findOrCreateDir(RESOURCE_PATH)
		&& findOrCreateDir(OUTPUT_PATH)
		&& findOrCreateDir(TEMP_PATH)
		&& findOrCreateDir(LOG_PATH)) {
		return true;
	} else {
		LOG_ERR("FileUtil: Failed when findOrCreateAllDirsNeeded()");
		return false;
	}
}

void FileUtil::persistMats(int fidx, std::vector<Mat> &mats, FILE_STORAGE_TYPE fst) {
	if (fst == NORMAL) {
		std::string fn = getFileNameByFidx(fidx);
		cv::FileStorage storage(fn, cv::FileStorage::WRITE);
		for (int i=0; i<mats.size(); ++i) {
			storage << getMatNameByMatidx(fidx, i) << mats[i];	 
		}
		storage.release();
	} else if (fst == BIN) {
		for (int i=0; i<mats.size(); ++i) {
			SaveMatBinary(getFileNameByFidx(fidx,getMatNameByMatidx(fidx, i),".bin"),mats[i]);
		}
	}

}

std::vector<Mat> FileUtil::loadMats(int fidx, int sz, FILE_STORAGE_TYPE fst) {
	std::vector<Mat> v(sz);
	if (fst == NORMAL) {
		std::string fn = getFileNameByFidx(fidx);
		
		cv::FileStorage storage(fn, cv::FileStorage::READ);
		for (int i=0; i<sz; ++i) {
			storage[getMatNameByMatidx(fidx, i)] >> v[i];	 
		}
		storage.release();
	} else if (fst == BIN) {
		for (int i=0; i<v.size(); ++i) {
			LoadMatBinary(getFileNameByFidx(fidx,getMatNameByMatidx(fidx, i),".bin"),v[i]);
		}
	}
	return v;
}

void FileUtil::deletePersistedMats(int fidx, int sz, FILE_STORAGE_TYPE fst) {
	if (fst == NORMAL) {
		deleteFile(getFileNameByFidx(fidx).c_str());
	}else if (fst == BIN) {
		for (int i=0; i<sz; ++i) {
			deleteFile(getFileNameByFidx(fidx,getMatNameByMatidx(fidx, i),".bin").c_str());
		}
	}
}

bool FileUtil::deleteAllTemp() {
	bool ret = true;
	for (std::string n : FileUtil::waitToDeleteBuff) {
		ret &= deleteFile(n.c_str());
	}
	return ret;
}

bool FileUtil::writeMatBinary(std::ofstream& ofs, const cv::Mat& out_mat) {
	if(!ofs.is_open()){
		return false;
	}
	if(out_mat.empty()){
		int s = 0;
		ofs.write((const char*)(&s), sizeof(int));
		return true;
	}
	int type = out_mat.type();
	ofs.write((const char*)(&out_mat.rows), sizeof(int));
	ofs.write((const char*)(&out_mat.cols), sizeof(int));
	ofs.write((const char*)(&type), sizeof(int));
	ofs.write((const char*)(out_mat.data), out_mat.elemSize() * out_mat.total());

	return true;
}


bool FileUtil::SaveMatBinary(const std::string& filename, const cv::Mat& output) {
	std::ofstream ofs(filename, std::ios::binary);
	return writeMatBinary(ofs, output);
}


bool FileUtil::readMatBinary(std::ifstream& ifs, cv::Mat& in_mat) {
	if(!ifs.is_open()){
		return false;
	}
	
	int rows, cols, type;
	ifs.read((char*)(&rows), sizeof(int));
	if(rows==0){
		return true;
	}
	ifs.read((char*)(&cols), sizeof(int));
	ifs.read((char*)(&type), sizeof(int));

	in_mat.release();
	in_mat.create(rows, cols, type);
	ifs.read((char*)(in_mat.data), in_mat.elemSize() * in_mat.total());

	return true;
}


bool FileUtil::LoadMatBinary(const std::string& filename, cv::Mat& output) {
	std::ifstream ifs(filename, std::ios::binary);
	return readMatBinary(ifs, output);
}