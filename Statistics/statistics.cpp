/****************************************************************************
**
** Contact: moenck@zedat.fu-berlin.de
**
** Image viewer main functionality
**
** Major parts taken from:
** http://doc.qt.io/qt-5/qtwidgets-widgets-imageviewer-imageviewer-cpp.html
**
****************************************************************************/

#include "statistics.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <iostream>
#include <unistd.h> //usleep

#include <math.h>       /* cos */
#include <vector>
#include <algorithm>
#include <iostream>
#ifdef WIN32
#include <windows.h>
#include <stdint.h>
#endif
#define PI 3.14159265

//Getting the timestamp
#include <time.h>
#if __linux__
#include <sys/time.h>
#else
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include "windows.h"
#endif

namespace beeStatistics {
using namespace cv;

Statistics::Statistics()
{
    curID 	= 0;
    shmid 	= 0;
}

void Statistics::configShdMem(EncoderQualityConfig *p_qconf){

    qconf = p_qconf;
    curID = qconf->camid;
    //_Dog = p_dog;

    //SettingsIAC* set 		= SettingsIAC::getInstance();
    //EncoderQualityConfig cfg 	= set->getBufferConf(cid,0);

    ////////////////////////SHARED MEMORY//////////////////////
    std::string ftopkFile 	= "memory"+std::to_string(qconf->camid)+".txt";
    width 			= qconf->width;
    height 			= qconf->height;
    lockpos 			= height*width; //32 is w,h,camid ; 64 is timestamp
    memsize 			= height*width + 32 + 64+sizeof(boost::interprocess::interprocess_mutex);

    image = (char*) malloc (width*height);

    /* make the key: */
    if ((key = ftok(ftopkFile.c_str(), 'R')) == -1) /*Here the file must exist */
    {
        perror("ftok");
        std::exit(1);
    }

    if(shmid!=0){ //TODO maybe the gap in the existence of data is a problem?
        //detach from the segment:
	if (shmdt(data) == -1) {
		perror("shmdt");
		std::exit(1);
	}
    }

    /*  create the segment: */
    if ((shmid = shmget(key, memsize, 0644 | IPC_CREAT)) == -1) {
        perror("shmget");
        std::exit(1);
    }

    /* attach to the segment to get a pointer to it: */
    data = shmat(shmid, (void *)0,0);
    if (data == (char *)(-1)) {
        perror("shmat");
        std::exit(1);
    }

    mutex = (boost::interprocess::interprocess_mutex*)(data+lockpos);
    ///////////////////////////////////////////////////////////

    /////////////////////////CREATING IMAGE////////////////////

    //const uchar *bits = image.bits();
    memcpy(image,(const unsigned char *)data, width*height);
}

void Statistics::grabRecentImage(){
	mutex->lock();
        memcpy(image,(const uchar *)data, width*height);
        mutex->unlock();
}

void Statistics::analyse(cv::Mat *ref, FILE *outfile) {
	char outstr[512];
	int w = qconf->width;
	int h = qconf->height;
	cv::Mat mat(h, w, cv::DataType<uint8_t>::type);
	mat.data = image;
	double smd = sumModulusDifference(&mat);
	double variance = getVariance(mat);
	double contrast = avgHistDifference(*ref, mat);
	double noise = noiseEstimate(mat);
	std::string ts = getTimestamp();
	sprintf(outstr, "Cam_%d_%s: %f,\t%f,\t%f,\t%f,\t%f\n", qconf->camid, ts.c_str(), smd, variance, contrast, noise);
	fwrite(outstr,sizeof(char), strlen(outstr),outfile);
	fflush(outfile);
}

void Statistics::getContrastRatio(Mat &image){
	uint8_t min = 255;
	uint8_t max = 0;
	for( int y = 0; y < image.rows; y++ ){
		for( int x = 0; x < image.cols; x++ ){
			uint8_t val = image.at<uint8_t>(y, x);
			val < min ? min = val : 0 ;
			val > max ? max = val : 0 ;
		}
	}
	double ratio = ((double)min)/((double)max);
	//printf("%d  %d, ratio: %f\n",max,min,ratio);
}

double Statistics::getVariance(Mat &image){
	//http://www.lfb.rwth-aachen.de/bibtexupload/pdf/GRO10a.pdf
	Mat out(image.size(),cv::DataType<double>::type);
	Mat squared(image.size(),cv::DataType<double>::type);
	Mat in(image.size(),cv::DataType<double>::type);
	image.assignTo(in,CV_64F);

	double s = cv::sum( in )[0];
	double frac = 1.0 / ((double)(in.rows * in.cols));
	double sfrac = -1.0 * s*frac;
	add(in,sfrac,out);
	//square: http://docs.opencv.org/2.4/modules/core/doc/operations_on_arrays.html
	//multiply - Calculates the per-element scaled product of two arrays.
	multiply(out,out,squared);
	double var = frac * cv::sum( squared )[0];

	//printf("Variance: %f\n",var);
	return var;
}

double Statistics::sumModulusDifference(Mat *image){
	//http://www.lfb.rwth-aachen.de/bibtexupload/pdf/GRO10a.pdf
	double smd = 0.0;
	Mat in(image->size(),cv::DataType<double>::type);
	Mat out1(image->size(),cv::DataType<double>::type);
	Mat out2(image->size(),cv::DataType<double>::type);
	Mat res(image->size(),cv::DataType<double>::type);
	image->assignTo(in,CV_64F);
	Mat vkernel = Mat::ones( 3, 1, CV_64F )/ (double)3.0;
	Mat hkernel = Mat::ones( 1, 3, CV_64F )/ (double)3.0;

	vkernel.at<double>(0, 0) = -1;
	vkernel.at<double>(1, 0) = 1;
	vkernel.at<double>(2, 0) = 0;
	hkernel.at<double>(0, 0) = -1;
	hkernel.at<double>(0, 1) = 1;
	hkernel.at<double>(0, 2) = 0;
	Point anchor = Point( -1, -1 ); //center
	double delta = 0;
	int ddepth = -1;
	filter2D(in, out1, ddepth , hkernel, anchor, delta, BORDER_DEFAULT );
	filter2D(in, out2, ddepth , vkernel, anchor, delta, BORDER_DEFAULT );
	out1 = abs(out1);
	out2 = abs(out2);
	add(out1,out2,res);
	smd = (cv::sum( res )[0])/(double)(image->rows * image->cols);

	//printf("Sum modulus difference: %f\n",smd);
	return smd;
}

double Statistics::DCT(double k1, double k2, int m, int n, Mat &image){
	double sum = 0.0;
	for (double i=0 ; i < 8; i++){
		for (double j=0 ; j < 8; j++){
			sum += image.at<double>(i+m, j+n)*cos(PI/8.0 * (i+0.5)*k1) * cos(PI/8.0 * (j+0.5)*k2);
		}
	}
	return sum;
}

double Statistics::SSF(double d){
	return (pow(d, 0.269) * (-3.533+3.533*d) * exp(-0.548*d));
}

double Statistics::STilde(int m, int n,Mat &image){
	double sum = 0.0;
	double Bn_ijOneOne = image.at<double>(m+1, n+1);
	for (int d=0 ; d < 8; d++){
		double ssf = SSF(d);// / Bn_ijOneOne;
		sum += ssf*(DCT(0,d-1,m,n,image)+ DCT(d-1,0,m,n,image));
	}
	return sum;
}

double Statistics::S_PSM(Mat *image){
	Mat in(image->size(),cv::DataType<double>::type);
	image->assignTo(in,CV_64F);

	double sum = 0.0;
	for (int i=0 ; i < image->rows-8; i+=8){
		for (int j=0 ; j < image->cols-8; j+=8){
			double st = STilde(i,j,in);
			sum += st;
			//printf("st: %lf\n",st);
		}
	}
	double frac = 1.0 / ((double)(in.rows * in.cols));
	return frac * sum;
}

cv::Mat Statistics::getHist(cv::Mat *M){

	  /// Establish the number of bins
	  int histSize = 256;

	  /// Set the ranges ( for B,G,R) )
	  float range[] = { 0, 256 } ;
	  const float* histRange = { range };

	  bool uniform = true;
	  bool accumulate = false;

	  Mat grey_hist;

	  /// Compute the histogram:
	  calcHist( M, 1, 0, Mat(), grey_hist, 1, &histSize, &histRange, uniform, accumulate );

	  // Draw the histograms for B, G and R
	  int hist_w = 512;
	  int hist_h = 400;
	  int bin_w = cvRound( (double) hist_w/histSize );

	  Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

	  /// Normalize the result to [ 0, histImage.rows ]
	  //normalize(grey_hist, grey_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

	  return grey_hist;
}

double Statistics::avgHistDifference(Mat reference, Mat measure){
	Mat ideal = getHist(&reference);
	Mat noisy = getHist(&measure);
	Mat dif(noisy.size(),cv::DataType<double>::type);
	subtract(ideal,noisy,dif);
	double total = cv::sum(cv::abs(dif))[0] / 256.0; //TODO SSD!?
	//std::cout << total << std::endl;
	return total;
}

double Statistics::noiseEstimate(Mat image){
	Mat blur(image.size(),cv::DataType<double>::type);
	Mat dif(image.size(),cv::DataType<double>::type);
	cv::medianBlur(image,blur,5);
	subtract(image,blur,dif);
	double total = cv::sum(cv::abs(dif))[0] / (image.rows * image.cols); //TODO SSD!?
	//std::cout << total << std::endl;
	return total;
}

/*
 * Timestamp creation from system clock
 */
std::string Statistics::getTimestamp(){
#if __linux__
	struct tm 	*timeinfo;
	char		timeresult[15];
	struct timeval 	tv;
	struct timezone tz;
	struct tm 	*tm;

	gettimeofday(&tv, &tz);
	timeinfo=localtime(&tv.tv_sec);


	sprintf(timeresult, "%d%.2d%.2d%.2d%.2d%.2d_%06d",
			timeinfo -> tm_year + 1900,
			timeinfo -> tm_mon  + 1,
			timeinfo -> tm_mday,
			timeinfo -> tm_hour,
			timeinfo -> tm_min,
			timeinfo -> tm_sec,
			tv.tv_usec);
	std::string r(timeresult);
	return r;
#else
	char		timeresult[20];
	SYSTEMTIME	stime;
	//structure to store system time (in usual time format)
	FILETIME	ltime;
	//structure to store local time (local time in 64 bits)
	FILETIME	ftTimeStamp;
	GetSystemTimeAsFileTime(&ftTimeStamp); //Gets the current system time

	FileTimeToLocalFileTime(&ftTimeStamp, &ltime);//convert in local time and store in ltime
	FileTimeToSystemTime(&ltime, &stime);//convert in system time and store in stime
	
	sprintf(timeresult, "%d%.2d%.2d%.2d%.2d%.2d_%06d",
		stime.wYear,
		stime.wMonth,
		stime.wDay,
		stime.wHour,
		stime.wMinute,
		stime.wSecond,
		stime.wMilliseconds);
	std::string r(timeresult);
	return r;
#endif
}

} /* namespace beeCompress */


