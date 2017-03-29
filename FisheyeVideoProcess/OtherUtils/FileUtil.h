#include "..\Config.h"
#include <direct.h>
#include <io.h>
#include <vector>

extern std::string runtimeHashCode;

class FileUtil {
private:
	static std::vector<std::string> waitToDeleteBuff;
	static bool findOrCreateDir(const char * path);
	static bool deleteFile(const char * fn);
	static std::string getFileNameByFidx(int fidx);
	static std::string getMatNameByMatidx(int fidx, int midx);

public:

	static bool findOrCreateAllDirsNeeded();
	static void persistMats(int fidx, std::vector<Mat> &mats);
	static std::vector<Mat> loadMats(int fidx, int sz);
	static void deletePersistedMats(int fidx);
	static bool deleteAllTemp();
};