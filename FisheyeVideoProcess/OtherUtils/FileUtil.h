#include "..\Config.h"
#include <direct.h>
#include <io.h>
#include <vector>
#include <fstream>

extern std::string runtimeHashCode;

enum FILE_STORAGE_TYPE {
	NORMAL,
	BIN,
	IMG
};

class FileUtil {
private:
	static std::vector<std::string> waitToDeleteBuff;
	static bool findOrCreateDir(const char * path);
	static bool deleteFile(const char * fn);
	static std::string getFileNameByFidx(int fidx, std::string elseInfo="",std::string extension=".xml");
	static std::string getMatNameByMatidx(int fidx, int midx);

	// referred from https://github.com/takmin/BinaryCvMat
	static bool writeMatBinary(std::ofstream& ofs, const cv::Mat& out_mat);
	static bool SaveMatBinary(const std::string& filename, const cv::Mat& output);
	static bool readMatBinary(std::ifstream& ifs, cv::Mat& in_mat);
	static bool LoadMatBinary(const std::string& filename, cv::Mat& output);
public:
	static FILE_STORAGE_TYPE FILE_STORAGE_MAT_DEFAULT;


	static bool findOrCreateAllDirsNeeded();
	static void persistMats(int fidx, std::vector<Mat> &mats, FILE_STORAGE_TYPE fst=NORMAL);
	static std::vector<Mat> loadMats(int fidx, int sz, FILE_STORAGE_TYPE fst=NORMAL);
	static void deletePersistedMats(int fidx, int sz, FILE_STORAGE_TYPE fst=NORMAL);
	static bool deleteAllTemp();
};