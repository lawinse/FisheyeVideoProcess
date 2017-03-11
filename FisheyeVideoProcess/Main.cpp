#include "Config.h"
#include "Processor.h"
#include <time.h>

#ifdef RUN_TEST
	#include "TestCase.h"
#endif
std::string runtimeHashCode;
inline std::string getruntimeHashCode() {
	char hash[20];
	sprintf(hash, "%x", std::hash<int>()(clock()));
	return (std::string) hash;
}


#ifdef NEED_LOG
std::stringstream sslog;
FILE *fplog;
#endif
/* Global variables */
Processor processor;


int main(int argc, char ** args) {
	runtimeHashCode = getruntimeHashCode();
#ifdef RUN_MAIN
	std::string oriSrc[] = {
		RESOURCE_PATH + (std::string)"back1.mp4",
		RESOURCE_PATH + (std::string)"front1.mp4"
	};
	processor.setPaths(oriSrc,sizeof(oriSrc)/sizeof(std::string),OUTPUT_PATH + (std::string)"test.avi"); //TOSOLVE: ouput must be avi format??
	processor.process(1,0);
#elif defined(RUN_TEST)
	TestCase tc;
	tc.test2();
	system("pause");
#endif
}
