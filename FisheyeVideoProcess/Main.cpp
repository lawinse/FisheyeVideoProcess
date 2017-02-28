#include "Config.h"
#include "Processor.h"

/* Global variables */
Processor processor;

int main(int argc, char ** args) {
	std::string oriSrc[] = {
		RESOURCE_PATH + (std::string)"o1.mp4",
		RESOURCE_PATH + (std::string)"o2.mp4"
	};
	processor.setPaths(oriSrc,2,OUTPUT_PATH + (std::string)"test.avi"); //TOSOLVE: ouput must be avi format??
	processor.process();
}
