/****************************************************************************
 **
 ** Contact: moenck@zedat.fu-berlin.de
 **
 ** Main file of the image viewer
 **
 ** Major parts taken from:
 ** http://doc.qt.io/qt-5/qtwidgets-widgets-imageviewer-main-cpp.html
 **
 ****************************************************************************/

#include "statistics.h"
#include "Settings/Settings.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <unistd.h>

#include <stdlib.h>     /* system, NULL, EXIT_FAILURE */

int main(int argc, char *argv[]) {
    // QApplication app(argc, argv);
    SettingsIAC::setConf("configStatistics.json");
    SettingsIAC *set = SettingsIAC::getInstance();
    EncoderQualityConfig qconf[4];
    std::string outfileStr = "analysis.txt";

    for (int i = 0; i < 4; i++)
        qconf[i] = set->getBufferConf(i);

    cv::Mat ref;

    std::string outFlip[4];
    outFlip[0]      = set->getValueOfParam<std::string>(IMSTATISTICS::TARGET1);
    outFlip[1]      = set->getValueOfParam<std::string>(IMSTATISTICS::TARGET2);
    outFlip[2]      = set->getValueOfParam<std::string>(IMSTATISTICS::TARGET3);
    outFlip[3]      = set->getValueOfParam<std::string>(IMSTATISTICS::TARGET4);
    int doanalysis  = set->getValueOfParam<int>(IMSTATISTICS::DOANALYSIS);
    int doimsave    = set->getValueOfParam<int>(IMSTATISTICS::DOIMSAVE);
    outfileStr      = set->getValueOfParam<std::string>(IMSTATISTICS::ANALYSISFILE);

    FILE* fp = fopen("refIm.jpg", "r");

    beeStatistics::Statistics *stat[4];
    for (int i = 0; i < 4; i++)
        stat[i] = new beeStatistics::Statistics();

    if (fp) {
        ref = cv::imread("refIm.jpg", CV_LOAD_IMAGE_GRAYSCALE);
        fclose(fp);
    } else {
        std::cout << "Warning: not found reference image refIm.jpg."
                << std::endl;
    }

    for (int i = 0; i < 4; i++)
        if (qconf[i].enabled == 1)
            stat[i]->configShdMem(&qconf[i]);

    while (true) {
        for (int i = 0; i < 4; i++) 
            if (qconf[i].enabled == 1)
                stat[i]->grabRecentImage();

        for (int i = 0; i < 4; i++) {
            if (qconf[i].enabled == 1) {

                if (doimsave == 1) {
                    stat[i]->saveImage(outFlip[i]);
                    //Ok, this is cheap, but it works...
                    std::string cmd = "chmod 755 " + outFlip[i];
                    system(cmd.c_str());
                }

                if (doanalysis == 1)
                    stat[i]->analyse(&ref, outfileStr);
            }
        }
        sleep(60);
    }

    return 0;
}
