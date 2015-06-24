///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	EUROARMS:	Bar Task
//		Author:	Pablo Ramon Soria
//		Date:	2015-FEB-28
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//

#ifndef OPENCV_VIDEO_SENSOR_H_
#define OPENCV_VIDEO_SENSOR_H_

#include "OpencvSensor.h"

#include <opencv2/opencv.hpp>
#include <mutex>
#include <thread>

class OpencvVideoSensor : public OpencvSensor{
public:
	OpencvVideoSensor() :	mCamera("C:/programming/datasets/multiple_bar_manual_test.mp4"), 
							mNuFrames(unsigned(mCamera.get(CV_CAP_PROP_FRAME_COUNT))),
							mCurrentFrame(0),
							mWatchThread(&OpencvVideoSensor::callback, this) {};
	OpencvVideoSensor(OpencvVideoSensor&){};
	~OpencvVideoSensor(){ mCamera.release();  /*666 TODO stop thread*/ };

private:
	void callback(){
		for (;;){
			if (mCurrentFrame > mNuFrames)
				return;	// 666 Stop callback.

			mCamera.set(CV_CAP_PROP_POS_FRAMES, mCurrentFrame);
			cv::Mat frame;
			mCamera >> frame;
			mCurrentFrame++;
			mMutex.lock();
			frame.copyTo(mImage);
			mMutex.unlock();

		}
	}

private:
	cv::VideoCapture	mCamera;
	unsigned			mNuFrames;
	unsigned			mCurrentFrame;
	std::thread			mWatchThread;
};

#endif	//	OPENCV_VIDEO_SENSOR_H_