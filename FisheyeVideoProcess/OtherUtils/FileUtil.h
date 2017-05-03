#include "..\Config.h"
#include <direct.h>
#include <io.h>
#include <vector>
#include <unordered_set>
#include <fstream>

#pragma once
extern std::string runtimeHashCode;

enum FILE_STORAGE_TYPE {
	NORMAL,
	BIN,
	LOSSY		//
};

class FileUtil {
#define FU_COMPRESS_FLAG 
#define FU_RAROBJ ".\\rar.exe "
#define FU_COMPRESS_EXTENSION ".cmprs "
private:
	static std::unordered_set<std::string> waitToDeleteBuff;
	static bool findOrCreateDir(const char * path);
	static bool deleteFile(const char * fn, bool delay=false);
	static std::string getFileNameByFidx(int fidx, std::string elseInfo="",std::string extension=getExtension(NORMAL));
	static std::string getMatNameByMatidx(int fidx, int midx);

	// referred from https://github.com/takmin/BinaryCvMat
	static bool writeMatBinary(std::ofstream& ofs, const cv::Mat& out_mat);
	static bool readMatBinary(std::ifstream& ifs, cv::Mat& in_mat);
	

	static std::string getExtension(FILE_STORAGE_TYPE fst);
public:
	static FILE_STORAGE_TYPE FILE_STORAGE_MAT_DEFAULT;
	static bool SaveMatBinary(const std::string& filename, const cv::Mat& output);
	static bool LoadMatBinary(const std::string& filename, cv::Mat& output);
	static bool findOrCreateAllDirsNeeded();
	static void persistFrameMats(int fidx, std::vector<cv::Mat> &mats, FILE_STORAGE_TYPE fst=NORMAL);
	static std::vector<cv::Mat> loadFrameMats(int fidx, int sz, FILE_STORAGE_TYPE fst=NORMAL);
	static void deletePersistedFrameMats(int fidx, int sz, FILE_STORAGE_TYPE fst=NORMAL, bool delay=false);
	static bool deleteAllTemp();
	static void compress(const std::string& fname);
	static void decompress(const std::string&fname);
};