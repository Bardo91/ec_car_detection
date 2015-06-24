///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	EUROARMS:	Bar Task
//		Author:	Pablo Ramon Soria
//		Date:	2015-MAY-05
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//

#ifndef DETECTION_TASK_H_
#define DETECTION_TASK_H_

#include "Task.h"
#include "sensors/OpencvVideoSensor.h"
#include "sensors/OpencvImageFileSensor.h"
#include "sensors/OpencvCameraSensor.h"
#include "CarParticleFilter.h"
#include "visualization/Map.h"

#include <fstream>

#include <opencv2/opencv.hpp>

#include <algorithms/segmentation/color_clustering/types/ColorClusterSpace.h>
#include <core/types/BasicTypes.h>

#include <array>

class DetectionTask : public Task<DetectionTask>{
public:
	DetectionTask(Json &_data);
	void run() override;

private:
	void init();

	std::vector<std::pair<cv::Point2f, cv::Point2f> >	featuresAndFlow(const cv::Mat &_prev, const cv::Mat &_current);
	std::vector<BOViL::ImageObject>						segmentate(cv::Mat &_img);
	std::vector<std::pair<cv::Point2f, cv::Point2f> >	filterFeatures(std::vector<std::pair<cv::Point2f, cv::Point2f> > &_features, std::vector<BOViL::ImageObject> &_objects);
	std::vector<std::array<double, 2>>					transformParticles(std::vector<std::pair<cv::Point2f, cv::Point2f> > &_features);

private:
	OpencvSensor		*mVisionSensor;
	ImuSimulatorSensor			*mImuSensor;

	BOViL::ColorClusterSpace	mCcs;
	CarParticleFilter			mParticleFilter;
	Map							mRepresentation;

	CameraInfo					mCameraInfo;

private: // Temporal 
	cv::Mat frame;
	cv::Mat Transform;
	cv::Mat Transform_avg = cv::Mat::eye(2, 3, CV_64FC1);
	cv::Mat warped;

private:	// Analysis.
	std::ofstream globalMovementFile;
	std::ofstream nFeaturesFile;
	std::ofstream particleEstimationFile;
	std::ofstream timeFile;
	unsigned filecounter = 0;

	unsigned frameCounter = 0;
	
};

#include "DetectionTask.inl"

#endif	//	DETECTION_TASK_H_