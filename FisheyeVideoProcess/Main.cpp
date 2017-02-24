#include "Config.h"
#include "Processor.h"

/* Global variables */
Processor processor;

int main(int argc, char ** args) {
	std::string oriSrc[] = {"1.mp4","2.mp4"};
	processor.setPaths(oriSrc,2,"test.avi"); //TOSOLVE: ouput must be avi format£¿
	processor.process();
}
