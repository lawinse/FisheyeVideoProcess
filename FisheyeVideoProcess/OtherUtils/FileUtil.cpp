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

std::string FileUtil::getFileNameByFidx(int fidx) {
	std::string rets;
	GET_STR(TEMP_PATH << runtimeHashCode << '_' << fidx << ".xml",rets);
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

void FileUtil::persistMats(int fidx, std::vector<Mat> &mats) {
	std::string fn = getFileNameByFidx(fidx);
	cv::FileStorage storage(fn, cv::FileStorage::WRITE);
	for (int i=0; i<mats.size(); ++i) {
		storage << getMatNameByMatidx(fidx, i) << mats[i];	 
	}
	storage.release(); 
}

std::vector<Mat> FileUtil::loadMats(int fidx, int sz) {
	std::string fn = getFileNameByFidx(fidx);
	std::vector<Mat> v(sz);
	cv::FileStorage storage(fn, cv::FileStorage::READ);
	for (int i=0; i<sz; ++i) {
		storage[getMatNameByMatidx(fidx, i)] >> v[i];	 
	}
	storage.release();
	return v;
}

void FileUtil::deletePersistedMats(int fidx) {
	//waitToDeleteBuff.push_back(getFileNameByFidx(fidx));
	deleteFile(getFileNameByFidx(fidx).c_str());
}

bool FileUtil::deleteAllTemp() {
	bool ret = true;
	for (std::string n : FileUtil::waitToDeleteBuff) {
		ret &= deleteFile(n.c_str());
	}
	return ret;
}
