///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	EUROARMS:	Bar Task
//		Author:	Pablo Ramon Soria
//		Date:	2015-MAY-05
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//


#include <algorithms/segmentation/color_clustering/types/ColorSpaceHSV8.h>
#include <algorithms/segmentation/color_clustering/types/ccsCreation.h>
#include <algorithms/segmentation/color_clustering/ColorClustering.h>
#include <algorithms/state_estimators/GMMEM.h>

#include <Eigen/Eigen>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>

#include <core/time/time.h>

#include <functional>
#include <iostream>
#include <sstream>
#include <fstream>


#ifdef _WIN32
#include <Windows.h>
inline void do_mkdir(std::string _filename) {
	CreateDirectory(_filename.c_str(), NULL);
}
#elif __linux__
#include <sys/stat.h>
#include <sys/types.h>
inline void do_mkdir(std::string _filename) {
	mkdir(_filename.c_str(), 0700);
}
#endif

using namespace cv;
using namespace BOViL;
using namespace std;

//---------------------------------------------------------------------------------------------------------------------
DetectionTask::DetectionTask(Json &_data): mCcs(createSingleClusteredSpace(0, 180, 0, 50, 180, 255, 180, 255, 255, 36)),
								mParticleFilter(1000),
								mRepresentation("map", 400, 400, _data["map"]){

	Json cameraData = _data["camera"];
	mCameraInfo.mU0			= cameraData["u0"].asFloat();
	mCameraInfo.mV0			= cameraData["v0"].asFloat();
	mCameraInfo.mWidth		= cameraData["width"].asInt();
	mCameraInfo.mHeight		= cameraData["height"].asInt();
	mCameraInfo.mFocal		= cameraData["f"].asFloat();

	CarParticle::setParams(_data["map"]);
}

//---------------------------------------------------------------------------------------------------------------------
void DetectionTask::run(){
	init();

	Mat curImg, prevImg;
	
	bool first = true;
	bool condition = true;
	double t0, t1, t2, t3, t4, t5, t6;
	while (condition){
		t0 = STime::get()->getTime();
		curImg = mVisionSensor->get();

		if (curImg.rows == 0)
			continue;

		if (first){
			first = false;
			curImg.copyTo(prevImg);
			continue;
		}
		t1 = STime::get()->getTime();

		// Get features and flow of them.
		std::vector<std::pair<Point2f, Point2f>> features = featuresAndFlow(prevImg, curImg);
		t2 = STime::get()->getTime();

		// Transform particles
		vector<array<double, 2>> transformedFeatures = transformParticles(features);
		t3 = STime::get()->getTime();

		mParticleFilter.reinitialize(0.1);
		mParticleFilter.step(transformedFeatures);
		t4 = STime::get()->getTime();

		// Filter PF with GMMEM
		vector<GaussianParameters> gaussians;
		array<double, 2> mean1 = { double(rand()) / RAND_MAX - 2, double(rand()) / RAND_MAX - 2 };
		array<double, 2> mean2 = { double(rand()) / RAND_MAX - 2, double(rand()) / RAND_MAX - 2 };
		array<double, 2> mean3 = { double(rand()) / RAND_MAX - 2, double(rand()) / RAND_MAX - 2 };
		array<double, 4> cov = { 1.0, 0.0, 0.0, 1.0 };
		gaussians.push_back(GaussianParameters(0.5, mean1, cov));
		gaussians.push_back(GaussianParameters(0.5, mean2, cov));
		gaussians.push_back(GaussianParameters(0.5, mean3, cov));

		vector<array<double, 2>> particlesGmmem;
		for (CarParticle particle : mParticleFilter.particles()){
			particlesGmmem.push_back(particle.position());
		}

		GMMEM gmmem(particlesGmmem, gaussians);
		if (!gmmem.iterate()){
			continue;
		}

		gaussians = gmmem.result();
		t5 = STime::get()->getTime();

		// Store previous img.
		curImg.copyTo(prevImg);

		// Visualization.
		for (std::pair<Point2f, Point2f> feature : features){
			if (feature.second.x > 0) {
				circle(curImg, feature.first, 5, Scalar(255, 0, 0), 3, 1, 0);
				line(curImg, feature.first, feature.first + feature.second, Scalar(0, 0, 255), 2, 1, 0);

			}
			else {
				circle(curImg, feature.first, 5, Scalar(255, 0, 0), 3, 1, 0);
				line(curImg, feature.first, feature.first + feature.second, Scalar(0, 255, 0), 2, 1, 0);
			}
		}

		imshow("image", curImg);

		// Draw particles
		vector<array<double, 2>> particles;
		for (CarParticle particle : mParticleFilter.particles()){
			particles.push_back(particle.position());
		}
		mRepresentation.addPoints(particles, eColor::eBlue);
		mRepresentation.addPoints(transformedFeatures, eColor::eRed);

		// GMMEM representation.
		vector<array<double, 5> > ellipses;
		for (GaussianParameters gaussian : gaussians){
			array<double, 5> ellipse = { gaussian.mean[0], gaussian.mean[1], 0.0, 0.0, 0.0 };

			Eigen::Matrix2d cov;
			cov << gaussian.covariance[0], gaussian.covariance[1], gaussian.covariance[2], gaussian.covariance[3];
			Eigen::EigenSolver<Eigen::Matrix2d> eigenSolver(cov);

			// Supossing that are always real.
			double value1 = eigenSolver.eigenvalues()[0].real();
			double value2 = eigenSolver.eigenvalues()[1].real();

			ellipse[2] = 4.8953*sqrt(value1); // 95% confidence.
			ellipse[3] = 4.8953*sqrt(value2);

			if (ellipse[2] <= 0 || ellipse[3] <= 0){
				write2Log("Ellipse de eje cero o menor: " + std::to_string(ellipse[2]) + ", " + std::to_string(ellipse[3]));
				particleEstimationFile << 99999.9 << "\t" << 99999.9 << "\t";
				continue;
			}
			else if (ellipse[2] >= 3.0 || ellipse[3] >= 3.0){
				write2Log("Sigma demasiado grande: " + std::to_string(ellipse[2]) + ", " + std::to_string(ellipse[3]));
				particleEstimationFile << 99999.9 << "\t" << 99999.9 << "\t";
				continue;
			}

			// ellipse[4] = angle;

			ellipses.push_back(ellipse);
			particleEstimationFile << ellipse[0] << "\t" << ellipse[1] << "\t";
		}
		
		particleEstimationFile << std::endl;

		mRepresentation.addEllipse(ellipses, eColor::eGreen);

		mRepresentation.show();

		//Time storage
		t6 = STime::get()->getTime();
		cout << "FPS: " << 1 / (t5 - t0) << endl;
		cout << "Total Time: " << t6 - t0 << endl;
		cout << "\t PT Camera acquisition: " << t1 - t0 << endl;
		cout << "\t PT Feature detection: " << t2 - t1 << endl;
		cout << "\t PT Particle adaptation: " << t3 - t2 << endl;
		cout << "\t PT Particle filter: " << t4 - t3 << endl;
		cout << "\t PT GMMEM: " << t5 - t4 << endl;
		cout << "\t PT Visualization: " << t6 - t5 << endl;
		timeFile << t5 - t0 << "\t" << t1 - t0 << "\t" << t2 - t1 << "\t" << t3 - t2 << "\t" << t4 - t3 << "\t" << t5 - t4 << "\t" << t6 - t5 << std::endl;
		mRepresentation.clean();
		waitKey(1);
	}	
}

//---------------------------------------------------------------------------------------------------------------------
// Private methods
void DetectionTask::init(){
	mVisionSensor	= getSensor<OpencvSensor>();
	mImuSensor		= getSensor<ImuSensor>();

	do_mkdir("dataAnalysis");
	do_mkdir("subImages");

	globalMovementFile.open("dataAnalysis/globalMovement");
	assert(globalMovementFile.is_open());
	nFeaturesFile.open("dataAnalysis/nFeatures");
	assert(nFeaturesFile.is_open());
	particleEstimationFile.open("dataAnalysis/particleEstimation");
	assert(particleEstimationFile.is_open());
	timeFile.open("dataAnalysis/time");
	assert(timeFile.is_open());
}

//---------------------------------------------------------------------------------------------------------------------

class clusters{
	public:
	bool operator()(const pair<Point2f, Point2f> _a, const pair<Point2f, Point2f> _b){
		const double cMaxErrPos = 60.0;
		const double cMaxErrVel = 10.0/180*M_PI;

		double dist = sqrt(pow(_a.first.x - _b.first.x,2) + pow(_a.first.y - _b.first.y,2));

		if (dist< cMaxErrPos){
			if (abs(atan2(_a.second.y, _a.second.x) - atan2(_b.second.y, _b.second.x))<cMaxErrVel){
				return true;
			}
		}

		
		return false;
	}

};


vector<pair<Point2f, Point2f>>	DetectionTask::featuresAndFlow(const cv::Mat &_prev, const cv::Mat &_current){
	Mat prev, current;
	vector<Point2f> featuresPrevious;
	vector<Point2f> featuresNextPos;
	vector<uchar> featuresFound;
	Mat err;
	Mat cflow;

	cvtColor(_prev, prev, CV_BGR2GRAY);
	cvtColor(_current, current, CV_BGR2GRAY);

	// Global motion estimation
	//Transform = estimateRigidTransform(current, prev, 0);
	//if (Transform.rows == 0){
	//	return vector<pair<Point2f, Point2f> >();
	//}

	// Track Features.
	std::vector<cv::KeyPoint> keypoints;
	FAST(prev, keypoints, 9);
	cv::KeyPointsFilter::retainBest(keypoints, 400);
	for (cv::KeyPoint keypoint : keypoints){
		featuresPrevious.push_back(Point2f(keypoint.pt.x, keypoint.pt.y));
	}


	if (featuresPrevious.size() == 0)
		return vector<pair<Point2f, Point2f> >();

	calcOpticalFlowPyrLK(prev, current, featuresPrevious, featuresNextPos, featuresFound, err);

	vector<pair<Point2f, Point2f> > features;

	//Point2f globalMovement = Point2f(float(*((double *)(Transform.data) + 2)), float(*((double *)(Transform.data) + 5)));
	//globalMovementFile << globalMovement.x << "\t" << globalMovement.y << std::endl;

	std::ofstream featuresFile("dataAnalysis/features" + to_string(filecounter++));
	for (unsigned i = 0; i < featuresPrevious.size(); i++){	
		Point2f featureMovement = (featuresNextPos[i] - featuresPrevious[i]);// +globalMovement; // Relative movement - global movement
		features.push_back(pair<Point2f, Point2f>(featuresNextPos[i], featureMovement));
		featuresFile << featuresNextPos[i].x << "\t" << featuresNextPos[i].y << "\t" << featureMovement.x << "\t" << featureMovement.y << std::endl;
	}

	featuresFile.close();

	nFeaturesFile << features.size() << std::endl;


	vector<int> labels;
	int nClasses = partition<pair<Point2f, Point2f>, clusters>(features, labels);

	std::vector<vector<pair<Point2f, Point2f> >> clusters;
	clusters.resize(nClasses);

	for (unsigned i = 0; i < features.size(); i++){
		clusters[labels[i]].push_back(features[i]);
	}
	
	///// EXTRACT IMAGE USING CLUSTERS
	/*
	const unsigned cMinFeatures = 6;
	features.clear();
	int clusterCounter = 0;
	for (vector<pair<Point2f, Point2f> > cluster : clusters){
		if (cluster.size() < cMinFeatures){
			continue;
		}
		features.insert(features.end(), cluster.begin(), cluster.end());

		float minX = 99999.9f, minY = 99999.9f, maxX = -99999.9f, maxY = -99999.9f;
		for (unsigned i = 0; i < cluster.size(); i++){
			float x, y;
			if ((x = cluster[i].first.x) < minX)
				minX = x;
			if ((x = cluster[i].first.x) > maxX)
				maxX = x;
			if ((y = cluster[i].first.y) < minY)
				minY = y;
			if ((y = cluster[i].first.y) > maxY)
				maxY = y;
		}
		Mat subImage = _current.colRange(int(minX < 0 ? 0 : minX), int(maxX)).rowRange(int(minY< 0 ? 0 : minY), int(maxY + 1));
		imwrite("subImages/subImage_" + std::to_string(frameCounter) + "_" + std::to_string(clusterCounter)+".png", subImage);
		++clusterCounter;
	}
	++frameCounter;
	*/
	return  features;
}


//---------------------------------------------------------------------------------------------------------------------
vector<array<double, 2>> DetectionTask::transformParticles(vector<pair<Point2f, Point2f> > &_features){
	ImuData imuData = mImuSensor->get();

	vector<array<double, 2>> quad;
	array<double, 2> quadWrapper = { imuData.mPos[0], imuData.mPos[1] };
	quad.push_back(quadWrapper);
	mRepresentation.addPoints(quad, eColor::eGreen);

	// 666 TODO check manually transformation.
	Eigen::Matrix<double, 3, 1> camPos;
	Eigen::Matrix<double, 3, 1> pos;
	Eigen::Matrix<double, 3, 3> ori;

	// Quad rotation related to testbed
	ori =
		Eigen::AngleAxisd(imuData.mEulerAngles[0], Eigen::Vector3d::UnitX())*
		Eigen::AngleAxisd(imuData.mEulerAngles[1], Eigen::Vector3d::UnitY())*
		Eigen::AngleAxisd(imuData.mEulerAngles[2], Eigen::Vector3d::UnitZ());

	//std::cout << ori << std::endl;
	// Camera rotation related to quad
	// First rotation, local Z edge.
	Eigen::Vector3d localZ = ori.col(2);
	ori = Eigen::AngleAxisd(3*M_PI / 4, localZ)*ori;
	// Second rotation, local X edge.
	Eigen::Vector3d localX = ori.col(0);
	ori = Eigen::AngleAxisd(M_PI * 165 / 180, localX)*ori;
	//std::cout << ori << std::endl;
	camPos << imuData.mPos[0], imuData.mPos[1], imuData.mAltitude;

	vector<array<double, 2> > possibleTargets;
	for (pair<Point2f, Point2f> feature : _features){
		feature.first.y = feature.first.y;
		feature.first.x = feature.first.x;

		double zc = (0.0 - camPos[2]) / 
									(	-ori(2, 0) * (feature.first.x - mCameraInfo.mU0) / mCameraInfo.mFocal + 
										-ori(2, 1) *  (feature.first.y - mCameraInfo.mV0) / mCameraInfo.mFocal +
										ori(2, 2));
		double xc = -(feature.first.x - mCameraInfo.mU0) / mCameraInfo.mFocal*zc;
		double yc = -(feature.first.y - mCameraInfo.mV0) / mCameraInfo.mFocal*zc;

		Eigen::Vector3d targetC;
		targetC << xc, yc, zc;

		Eigen::Vector3d targetFromC;
		targetFromC = ori*targetC;

		pos = camPos + targetFromC;

		array<double, 2> tempPos = { pos[0], pos[1] };
		possibleTargets.push_back(tempPos);
	}
	

	return possibleTargets;
}