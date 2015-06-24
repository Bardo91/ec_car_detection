///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	EUROARMS:	Bar Task
//		Author:	Pablo Ramon Soria
//		Date:	2015-MAR-09
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//

#ifndef OPENCV_IMAGE_FILE_SENSOR_H_
#define OPENCV_IMAGE_FILE_SENSOR_H_

#include "OpencvSensor.h"

#include <opencv2/opencv.hpp>
#include <mutex>
#include <thread>

class OpencvImageFileSensor : public OpencvSensor{
public:
	OpencvImageFileSensor(std::string _nameTemplate, unsigned _counterInit) : mNameTemplate(_nameTemplate), mCounter(_counterInit){};
	OpencvImageFileSensor(OpencvImageFileSensor&){};
	~OpencvImageFileSensor(){/*666 TODO stop thread*/ };

	cv::Mat get() override {
		std::stringstream fileName;
		fileName << mNameTemplate.substr(0, mNameTemplate.find("%d"));
		fileName << mCounter;
		fileName << mNameTemplate.substr(mNameTemplate.find("%d") + 2, mNameTemplate.npos);
		mCounter++;
		return cv::imread(fileName.str());
	}


private:
	std::string			mNameTemplate;
	unsigned			mCounter;
};

#endif	//	OPENCV_IMAGE_FILE_SENSOR_H_