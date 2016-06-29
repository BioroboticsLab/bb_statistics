/****************************************************************************
**
** Contact: moenck@zedat.fu-berlin.de
**
** Image statistics main functionality (header)
**
****************************************************************************/

#ifndef STATISTICS_H
#define STATISTICS_H

#include <opencv2/opencv.hpp>
#include <string>
#include "Settings/Settings.h"

#include <boost/interprocess/sync/interprocess_mutex.hpp>

namespace beeStatistics {

class Statistics
{

public:
	Statistics();

	int curID;

	////////////////////////SHARED MEMORY//////////////////////
	int width;
	int height;
	int lockpos;
	int memsize;

	void analyse(cv::Mat *ref, FILE *outfile);
	void configShdMem(EncoderQualityConfig *p_qconf);

private:
	
	key_t key;
	int shmid;
	char *data;
	unsigned char *image;
	int mode;
	boost::interprocess::interprocess_mutex *mutex;
	EncoderQualityConfig *qconf;

	void grabRecentImage();
	void getContrastRatio(cv::Mat &image);
	double getVariance(cv::Mat &image);
	double sumModulusDifference(cv::Mat *image);
	double DCT(double k1, double k2, int m, int n, cv::Mat &image);
	double SSF(double d);
	double STilde(int m, int n,cv::Mat &image);
	double S_PSM(cv::Mat *image);
	cv::Mat getHist(cv::Mat *M);
	double avgHistDifference(cv::Mat reference, cv::Mat measure);
	double noiseEstimate(cv::Mat image);
	std::string getTimestamp();
};

}

#endif

