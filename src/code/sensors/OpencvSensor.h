///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	EUROARMS:	Bar Task
//		Author:	Pablo Ramon Soria
//		Date:	2015-MAY-19
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//

#ifndef OPENCV_IMAGE_SENSOR_H_
#define OPENCV_IMAGE_SENSOR_H_

#include "SensorManager.h"

#include <opencv2/opencv.hpp>
#include <mutex>

typedef SensorTrait<SensorType::eVision, cv::Mat> OpencvTrait;

class OpencvSensor : public Sensor<OpencvTrait>{
public:
	OpencvSensor() { };
	OpencvSensor(OpencvSensor&){};

	cv::Mat get(){
		cv::Mat copyImg;
		mMutex.lock();
		mImage.copyTo(copyImg);
		mMutex.unlock();
		return copyImg;
	}

protected:
	cv::Mat	mImage;
	std::mutex mMutex;
};

#endif	//	OPENCV_CAMERA_SENSOR_H_