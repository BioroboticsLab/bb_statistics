#pragma once

#include <string> // std::string

namespace CONFIGPARAM
{
    static const std::string CONFIGURATION_FILE = "./configStatistics.json";
}

namespace IMSTATISTICS
{

    namespace BUFFERCONF
    {
        static const std::string CAMID        = "CAMID";
        static const std::string SERIAL       = "SERIAL";
        static const std::string ENABLED      = "ENABLED";
        static const std::string VIDEO_WIDTH  = "VIDEO_WIDTH";
        static const std::string VIDEO_HEIGHT = "VIDEO_HEIGHT";
    }

    static const std::string BUFFER = "IMSTATISTICS.BUFFER";

    static const std::string DOANALYSIS         = "IMSTATISTICS.DOANALYSIS";
    static const std::string DOIMSAVE           = "IMSTATISTICS.DOIMSAVE";
    static const std::string ANALYSISFILE       = "IMSTATISTICS.ANALYSISFILE";
    static const std::string ANALYSISFILEMIRROR = "IMSTATISTICS.ANALYSISFILEMIRROR";
    static const std::string SCALINGFACTOR      = "IMSTATISTICS.SCALINGFACTOR";
    static const std::string CRWINDOWSIZE       = "IMSTATISTICS.CRWINDOWSIZE";
    static const std::string TARGET1            = "IMSTATISTICS.TARGET1";
    static const std::string TARGET2            = "IMSTATISTICS.TARGET2";
    static const std::string TARGET3            = "IMSTATISTICS.TARGET3";
    static const std::string TARGET4            = "IMSTATISTICS.TARGET4";
}
