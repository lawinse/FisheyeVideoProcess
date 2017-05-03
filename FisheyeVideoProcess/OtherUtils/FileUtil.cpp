#include "FileUtil.h"
std::unordered_set<std::string> FileUtil::waitToDeleteBuff = std::unordered_set<std::string>();

FILE_STORAGE_TYPE FileUtil::FILE_STORAGE_MAT_DEFAULT = FILE_STORAGE_TYPE::BIN;

std::string FileUtil::getExtension(FILE_STORAGE_TYPE fst) {
	switch (fst) {
	case NORMAL:
		return ".xml";
	case BIN:
		return ".bin";
	case LOSSY:
		return ".jpg";
	default:
		return "";
	}
}

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

bool FileUtil::deleteFile(const char * fn, bool delay) {
	if (delay) {
		waitToDeleteBuff.insert(std::string(fn));
		LOG_WARN("Fileutil: " << fn << " is pushed into waitToDeleteBuff.")
		return false;
	}
	if(!access(fn,0)) {
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
#ifdef FU_COMPRESS_FLAG
			if (access(FU_RAROBJ,0)) system("copy /Y \"C:\\Program Files\\WinRAR\\Rar.exe\" .\\rar.exe");
			if (access(FU_RAROBJ,0)) system("copy /Y \"C:\\Program Files (x86)\\WinRAR\\Rar.exe\" .\\rar.exe");
#endif
			return true;
	} else {
		LOG_ERR("FileUtil: Failed when findOrCreateAllDirsNeeded()");
		return false;
	}
}

void FileUtil::persistFrameMats(int fidx, std::vector<Mat> &mats, FILE_STORAGE_TYPE fst) {
	std::string fn;
	if (fst == NORMAL) {
		fn = getFileNameByFidx(fidx);
		cv::FileStorage storage(fn, cv::FileStorage::WRITE);
		for (int i=0; i<mats.size(); ++i) {
			storage << getMatNameByMatidx(fidx, i) << mats[i];	 
		}
		storage.release();
	#ifdef FU_COMPRESS_FLAG
		compress(fn);
	#endif
	} else if (fst == BIN) {
		for (int i=0; i<mats.size(); ++i) {
			SaveMatBinary(fn=getFileNameByFidx(fidx,getMatNameByMatidx(fidx, i),getExtension(fst)),mats[i]);
		#ifdef FU_COMPRESS_FLAG
			compress(fn);
		#endif
		}
	} else if (fst == LOSSY) {
		std::vector<int>p(2);p[0] = CV_IMWRITE_JPEG_QUALITY,p[1]=100;
		for (int i=0; i<mats.size(); ++i) {
			imwrite(fn=getFileNameByFidx(fidx,getMatNameByMatidx(fidx, i),getExtension(fst)),mats[i],p);
		#ifdef FU_COMPRESS_FLAG
			compress(fn);
		#endif
		}
	}

}

std::vector<Mat> FileUtil::loadFrameMats(int fidx, int sz, FILE_STORAGE_TYPE fst) {
	std::vector<Mat> v(sz);
	std::string fn;
	if (fst == NORMAL) {
		fn = getFileNameByFidx(fidx);
	#ifdef FU_COMPRESS_FLAG
		decompress(fn);
	#endif
		cv::FileStorage storage(fn, cv::FileStorage::READ);
		for (int i=0; i<sz; ++i) {
			storage[getMatNameByMatidx(fidx, i)] >> v[i];	 
		}
		storage.release();
	#ifdef FU_COMPRESS_FLAG
		deleteFile(fn.c_str());
	#endif
	} else if (fst == BIN) {
		for (int i=0; i<v.size(); ++i) {
			fn = getFileNameByFidx(fidx,getMatNameByMatidx(fidx, i),getExtension(fst));
		#ifdef FU_COMPRESS_FLAG
			decompress(fn);
		#endif
			LoadMatBinary(fn,v[i]);
		#ifdef FU_COMPRESS_FLAG
			deleteFile(fn.c_str());
		#endif
		}
	} else if (fst == LOSSY) {
		for (int i=0; i<v.size(); ++i) {
			fn = getFileNameByFidx(fidx,getMatNameByMatidx(fidx, i),getExtension(fst));
		#ifdef FU_COMPRESS_FLAG
			decompress(fn);
		#endif
			v[i] = imread(fn,CV_LOAD_IMAGE_UNCHANGED);
		#ifdef FU_COMPRESS_FLAG
			deleteFile(fn.c_str());
		#endif
		}
	}
	return v;
}

void FileUtil::deletePersistedFrameMats(int fidx, int sz, FILE_STORAGE_TYPE fst, bool delay) {
	std::string elseInfo = "";
#ifdef FU_COMPRESS_FLAG
	elseInfo = FU_COMPRESS_EXTENSION;
#endif
	if (fst == NORMAL) {
		deleteFile((getFileNameByFidx(fidx)+elseInfo).c_str(),delay);
	} else if (fst == BIN) {
		for (int i=0; i<sz; ++i) {
			deleteFile((getFileNameByFidx(fidx,getMatNameByMatidx(fidx, i),getExtension(fst))+elseInfo).c_str(), delay);
		}
	} else if (fst == LOSSY) {
		for (int i=0; i<sz; ++i) {
			deleteFile((getFileNameByFidx(fidx,getMatNameByMatidx(fidx, i),getExtension(fst))+elseInfo).c_str(), delay);
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

void FileUtil::compress(const std::string&fname) {
	if(access(FU_RAROBJ,0)) return;
	std::string cmd = std::string(FU_RAROBJ) + " a -df -m5 -idcdpq "+ fname+FU_COMPRESS_EXTENSION + " " + fname;
	waitToDeleteBuff.insert(fname+FU_COMPRESS_EXTENSION);
	system(cmd.c_str());
}

void FileUtil::decompress(const std::string&fname) {
	if(access(FU_RAROBJ,0)) return;
	std::string cmd = std::string(FU_RAROBJ) + " x -idcdpq "+ fname+FU_COMPRESS_EXTENSION;               
	system(cmd.c_str());
}
