
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

int main(int argc, char *argv[])
{
   // QApplication app(argc, argv);
	SettingsIAC::setConf("configStatistics.json");
	SettingsIAC *set = SettingsIAC::getInstance();
	EncoderQualityConfig qconf[4];
	std::string outfileStr = "analysis.txt";

	for (int i=0; i<4; i++)
		qconf[i] = set->getBufferConf(i);
    
	cv::Mat 			ref;

	FILE*				outfile = fopen(outfileStr.c_str(),"ab");
	FILE* 				fp 		= fopen("refIm.jpg", "r");

	beeStatistics::Statistics *stat[4];
	for (int i=0; i<4; i++)
		stat[i] = new beeStatistics::Statistics();

	if (fp) {
		ref = cv::imread( "refIm.jpg", CV_LOAD_IMAGE_GRAYSCALE );
		fclose(fp);
	} else {
		std::cout << "Warning: not found reference image refIm.jpg."<<std::endl;
	}

	for (int i=0; i<4; i++)
		if (qconf[i].enabled == 1)
			stat[i]->configShdMem(&qconf[i]);

	while (true){
		//Pulse(5) signals analysis thread is alive
		//_Dog->pulse(5);
		for (int i=0; i<4; i++)
			if (qconf[i].enabled == 1)
				stat[i]->analyse(&ref,outfile);
		//TODO: sleep
	}

	//Well, just in case...
	fclose(outfile);

    return 0;//app.exec();
}
