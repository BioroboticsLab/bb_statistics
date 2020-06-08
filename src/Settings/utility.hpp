/*
 * utility.h
 *
 *  Created on: Nov 11, 2015
 *      Author: hauke
 */

#ifndef UTILITY_H_
#define UTILITY_H_

#include <string>
#include <QString>

/**
 * @brief Generates a log message to log.txt in the given path.
 *
 * @param Path to the log.txt file
 * @param Message to emit
 */
void generateLog(QString path, QString message);

/**
 * @brief Wrapper around the QString version
 *
 * @param Path to the log.txt file
 * @param Message to emit
 */
void generateLog(std::string path, std::string message);

/**
 * @brief Sets file to 0755 and logs errors
 *
 * @param Path to the file
 */
int simpleChmod(std::string file);

// This file is a mirror from bb_imgAcquisition
// https://git.imp.fu-berlin.de/bioroboticslab/bb_imgacquisition/blob/nvEncoder/ImgAcquisition/settings/utility.h

std::string get_utc_offset_string();
std::string getTimestamp();

#endif /* UTILITY_H_ */
