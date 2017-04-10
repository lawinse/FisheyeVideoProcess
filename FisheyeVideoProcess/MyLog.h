#pragma once
#include <sstream>
#include "Config.h"
#define NEED_LOG
extern std::string runtimeHashCode;
extern std::stringstream sslog;
extern FILE *fplog;

/* LOG to screen and files */

#ifdef NEED_LOG
	#define WRITE_LOG(msg,fname) {					\
		fopen_s(&fplog,(fname).c_str(), "a");\
		sslog.str("");\
		sslog <<"["<<timetodate(time(0))<<"]"<< msg;\
		fprintf(fplog,sslog.str().c_str());\
		fclose(fplog);}

	#define LOG_ERR(msg)          {           \
		std::cout << "[Error] " << msg << std::endl;\
		WRITE_LOG(msg<<std::endl,(LOG_PATH+runtimeHashCode+(std::string)".err"));}

	#define LOG_WARN(msg)          {           \
		std::cout << "[Warning] " << msg << std::endl;\
		WRITE_LOG(msg<<std::endl,(LOG_PATH+runtimeHashCode+(std::string)".warn"));}

	#define LOG_MESS(msg)          {          \
		std::cout << "[Message] " << msg << std::endl;\
		WRITE_LOG(msg<<std::endl,(LOG_PATH+runtimeHashCode+(std::string)".mess"));}

	#define LOG_MARK(msg)         {            \
		std::cout << ">>>>> " << msg << std::endl;\
		WRITE_LOG(">>>>> " << msg<<std::endl,(LOG_PATH+runtimeHashCode+(std::string)".mess"));\
		WRITE_LOG(">>>>> " << msg<<std::endl,(LOG_PATH+runtimeHashCode+(std::string)".warn"));\
		WRITE_LOG(">>>>> " << msg<<std::endl,(LOG_PATH+runtimeHashCode+(std::string)".err"));}
#else
	#define LOG_ERR(msg)                     \
		std::cout << "[Error] " << msg << std::endl;

	#define LOG_WARN(msg)                     \
		std::cout << "[Warning] " << msg << std::endl;

	#define LOG_MESS(msg)                     \
		std::cout << "[Message] " << msg << std::endl;
#endif
