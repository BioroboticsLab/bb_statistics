#pragma once

#include <string> // std::string

namespace CONFIGPARAM {
static const std::string CONFIGURATION_FILE = "./configStatistics.json";
}

namespace IMSTATISTICS {

	namespace BUFFERCONF{
	static const std::string CAMID 				= "CAMID";
	static const std::string SERIAL 			= "SERIAL";
	static const std::string ENABLED 			= "ENABLED";
	static const std::string VIDEO_WIDTH 			= "VIDEO_WIDTH";
	static const std::string VIDEO_HEIGHT 			= "VIDEO_HEIGHT";
	}

	static const std::string BUFFER					= "IMSTATISTICS.BUFFER";
}


