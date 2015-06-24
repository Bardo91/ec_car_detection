///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	EUROARMS:	Bar Task
//		Author:	Pablo Ramon Soria
//		Date:	2015-FEB-26
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//

#ifndef OPENCV_CAMERA_SENSOR_H_
#define OPENCV_CAMERA_SENSOR_H_

#include "OpencvSensor.h"

#include <opencv2/opencv.hpp>
#include <mutex>
#include <thread>

class OpencvCameraSensor : public OpencvSensor{
public:
	OpencvCameraSensor() : mCamera(0), mWatchThread(&OpencvCameraSensor::callback, this){ };
	OpencvCameraSensor(OpencvCameraSensor&){};
	~OpencvCameraSensor(){/*666 TODO stop thread*/ };

private:
	void callback(){
		for (;;){
			cv::Mat frame;
			mCamera >> frame;
			mMutex.lock();
			frame.copyTo(mImage);
			mMutex.unlock();

		}
	}

private:
	cv::VideoCapture	mCamera;
	std::thread			mWatchThread;
};

#endif	//	OPENCV_CAMERA_SENSOR_H_