#include "Config.h"
#include "Processor.h"

/* Global variables */
Processor processor;

int main(int argc, char ** args) {
	std::string oriSrc[] = {
		RESOURCE_PATH + (std::string)"front.mp4",
		RESOURCE_PATH + (std::string)"back.mp4"
	};
	processor.setPaths(oriSrc,sizeof(oriSrc)/sizeof(std::string),OUTPUT_PATH + (std::string)"test.avi"); //TOSOLVE: ouput must be avi format??
	processor.process(3);
}
