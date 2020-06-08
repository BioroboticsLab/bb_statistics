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
#include "Settings/Settings.hpp"

#include <boost/interprocess/sync/interprocess_mutex.hpp>

namespace beeStatistics
{

    class Statistics
    {

    public:
        Statistics();

        int _curID;

        ////////////////////////SHARED MEMORY//////////////////////

        //! Width of the image in shared memory
        int _width;

        //! Height of the image in shared memory
        int _height;

        /**
         * @brief Position of the IP mutex in shared memory
         *
         * It is located after the image, so _height * _width
         */
        int _lockpos;

        //! Total size of the shared memory.
        int _memsize;

        //! pointer to the image data
        unsigned char* _image;

        /**
         * @brief Do all image analysis: Noise, SMD, contrast, variance
         *
         * @param Reference image for contrast calculation
         * @param Analysis output file
         * @param Mirror of the output
         */
        void analyse(cv::Mat* ref, std::string outfileStr, std::string outfileMirrorStr);

        /**
         * @brief Sets up the shared memory enviroment.
         *
         * Opens all 4 shared memory chunks, one for each camera.
         * Unused chunks may remain blank.
         *
         * @param Configuration of the shared memory
         */
        void configShdMem(EncoderQualityConfig* p_qconf);

        /**
         * @brief Saves the local image to a file.
         *
         * Source image is _image. To update the local
         * image call grabRecentImage()
         *
         *
         */
        void saveImage(std::string target);

        /**
         * @brief Copy most recent image from shared to local memory.
         */
        void grabRecentImage();

    private:
        //! Interprocess mutex
        boost::interprocess::interprocess_mutex* _mutex;

        //! Shared memory key
        key_t _key;

        //! Shared memory id
        int _shmid;

        //! Pointer to the memory segment
        char* _data;

        EncoderQualityConfig* _qconf;

        // Timestamp used on analysis
        std::string _timestamp;

        /**
         * @brief Applies 'getContrastRatio' to 5 sections of an image
         *
         * Sections are in the dice-5 pattern, each 200px^2. Minimum image size is
         * 800x800px.
         *
         * @param The input image
         * @param (output) The difference between highest and lowest segment's contrast
         * @return Contrast ratio or -1 if image to small.
         */
        double getContrastRatioSegmented(cv::Mat* image, double* minMax);

        /**
         * @brief Gets the contrast ratio of a Matrix
         *
         * This might be useless for large real images, as
         * the ratio will always be maximum. Pick image sections.
         *
         * @param The input image
         * @return Contrast ratio
         */
        double getContrastRatio(cv::Mat& image);

        /**
         * @brief Gets the variance of an image.
         *
         * @param The image
         * @return The variance
         */
        double getVariance(cv::Mat& image);

        /**
         * @brief Gets the Sum Modulus Difference of an image (SMD).
         *
         * See the following papers for info:
         * Practical issues in pixel-based autofocusing for machine vision
         * Echtzeitfhige Extraktion scharfer Standbilder in der Video-Koloskopie
         *
         * @param The image
         * @return The SMD
         */
        double sumModulusDifference(cv::Mat* image);

        /**
         * @brief A helper function of S_PSM.
         *
         * See paper for detail:
         * Echtzeitfhige Extraktion scharfer Standbilder in der Video-Koloskopie
         * Parameters are as per paper
         */
        double DCT(double k1, double k2, int m, int n, cv::Mat& image);

        /**
         * @brief A helper function of S_PSM.
         *
         * See paper for detail:
         * Echtzeitfhige Extraktion scharfer Standbilder in der Video-Koloskopie
         * Parameters are as per paper
         */
        double SSF(double d);

        /**
         * @brief A helper function of S_PSM.
         *
         * See paper for detail:
         * Echtzeitfhige Extraktion scharfer Standbilder in der Video-Koloskopie
         * Parameters are as per paper
         */
        double STilde(int m, int n, cv::Mat& image);

        /**
         * @brief Calculates the Perceptual Sharpness Metric
         *
         * See paper for detail:
         * Echtzeitfhige Extraktion scharfer Standbilder in der Video-Koloskopie
         *
         * @param The image
         * @return The PSM
         */
        double S_PSM(cv::Mat* image);

        /**
         * @brief Gets a colour histogram. Helper of avgHistDifference
         *
         * Use grayscale images only.
         *
         * @param The image to get the histogram from
         * @param The histogram
         */
        cv::Mat getHist(cv::Mat* M);

        /**
         * @brief Calculates the average difference of colour histograms.
         *
         * @param First image
         * @param Second image
         * @return Avg difference
         */
        double avgHistDifference(cv::Mat reference, cv::Mat measure);

        /**
         * @brief Estimates the noise in an image
         *
         * Applies a median filter and gets the SSD.
         * See thesis for details:
         * Masters Thesis Hauke Moenck
         *
         * @param The image to analyse
         * @return Noise estimate
         */
        double noiseEstimate(cv::Mat image);

        /**
         * @brief Gets current timestamp as YYYYMMDDhhmmss_iiiiii
         *
         * @return The timestamp
         */
        std::string getTimestamp();
    };

}

#endif
