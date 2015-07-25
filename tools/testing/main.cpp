///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	EUROARMS:	Bar Task
//		Author:	Pablo Ramon Soria
//		Date:	2015-02-05
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <DroneApplication.h>

#include <algorithms/machine_learning/NeuronalNetwork.h>

#include <implementations/sensors/OpencvImageFileSensor.h>

#include <iostream>
#include <fstream>

using namespace std;
using namespace Eigen;
using namespace BOViL::algorithms;
using namespace cv;

double colorMedValue(const Mat &_frame, int _x, int _y, int _r, int _channel);
int numKeyArround(const KeyPoint &_keypoint, const std::vector<KeyPoint> &_keypoints, double _range);


int main(int _argc, char** _argv){
	_argc; _argv;

	DroneApplication app;

	// Get Neuronal network params
	std::array<Eigen::MatrixXd, 1 + 2 - 1>  params;
	params[0] = MatrixXd(16,17);
	params[1] = MatrixXd(1, 17);
	for (unsigned i = 0; i < params.size(); i++) {
		ifstream paramFile("params" + to_string(i) + ".txt");
		paramFile.read( (char *) params[i].data(), params[i].rows() * params[i].cols() * sizeof(double) );
	}

	std::pair<Eigen::Matrix<double, 1, 16>, Eigen::Matrix<double, 1, 16>> normParams;
	ifstream nuFile("nuParams.txt");
	nuFile.read( (char *) normParams.first.data(), normParams.first.rows() * normParams.first.cols() * sizeof(double) );	
	ifstream sigmaFile("sigmaParams.txt");
	sigmaFile.read( (char *) normParams.second.data(), normParams.second.rows() * normParams.second.cols() * sizeof(double) );

	NeuronalNetwork<16, 1, 16, 1> nn;
	nn.parameters(params);
	nn.normalParams(normParams);
	
	// Test it
	OpencvImageFileSensor sensor("C:/programming/datasets/images_500mm/image%d.jpg", 829);

	std::vector<KeyPoint> keypoints;
	for (;;) {
		Mat ori, display;
		Mat frame = sensor.get();
		cvtColor(frame, ori, CV_BGR2HSV);
		if(frame.rows == 0)
			break;

		frame.copyTo(display);
		cvtColor(frame, frame, CV_BGR2GRAY);
		FAST(frame, keypoints, 5);

		for (unsigned i = 0; i < keypoints.size(); i++) {
			Matrix<double, 16,1> x;
			x << colorMedValue(ori, int(keypoints[i].pt.x), int(keypoints[i].pt.y), 4, 0),
				 colorMedValue(ori, int(keypoints[i].pt.x), int(keypoints[i].pt.y), 8, 0) ,
				 colorMedValue(ori, int(keypoints[i].pt.x), int(keypoints[i].pt.y), 16, 0),
				 colorMedValue(ori, int(keypoints[i].pt.x), int(keypoints[i].pt.y), 32, 0),
				 colorMedValue(ori, int(keypoints[i].pt.x), int(keypoints[i].pt.y), 4, 1) ,
				 colorMedValue(ori, int(keypoints[i].pt.x), int(keypoints[i].pt.y), 8, 1) ,
				 colorMedValue(ori, int(keypoints[i].pt.x), int(keypoints[i].pt.y), 16, 1),
				 colorMedValue(ori, int(keypoints[i].pt.x), int(keypoints[i].pt.y), 32, 1),
				 colorMedValue(ori, int(keypoints[i].pt.x), int(keypoints[i].pt.y), 4, 2) ,
				 colorMedValue(ori, int(keypoints[i].pt.x), int(keypoints[i].pt.y), 8, 2) ,
				 colorMedValue(ori, int(keypoints[i].pt.x), int(keypoints[i].pt.y), 16, 2),
				 colorMedValue(ori, int(keypoints[i].pt.x), int(keypoints[i].pt.y), 32, 2),
				 double(numKeyArround(keypoints[i], keypoints, 4) ),
				 double(numKeyArround(keypoints[i], keypoints, 8) ), 
				 double(numKeyArround(keypoints[i], keypoints, 16)),
				 double(numKeyArround(keypoints[i], keypoints, 32));

			Matrix<double, 1, 1> y = nn.evaluate(x);
			if (y(0, 0) > 0.6) {
				circle(display, keypoints[i].pt, 2, Scalar(255, 0, 0), 4);
			}
			else {
				circle(display, keypoints[i].pt, 2, Scalar(255, 255, 255), 1);
			}
		}

		imshow("display", display);
		waitKey(3);

		keypoints.clear();
	}
}




double colorMedValue(const Mat &_frame, int _x, int _y, int _r, int _channel) {
	double medVal = 0;
	int accum = 0;
	for (int i = -_r; i < _r; i++) {
		for (int j = -_r; j < _r; j++) {
			if (sqrt(i ^ 2 + j ^ 2) < _r) {
				if(_x + i < 0 ||_y + j < 0 || _x + i > _frame.cols || _y + j > _frame.rows)
					continue;

				medVal += _frame.data[(_x + i) + (_y + j)*_frame.cols + _channel*_frame.cols*_frame.rows];
				accum ++;
			}
		}
	}

	return medVal/accum;
}

int numKeyArround(const KeyPoint &_keypoint, const std::vector<KeyPoint> &_keypoints, double _range) {
	int accum = -1;
	for (KeyPoint keypoint : _keypoints) {
		if (sqrt(pow(keypoint.pt.x - _keypoint.pt.x, 2) + pow(keypoint.pt.y - _keypoint.pt.y, 2))< _range) {
			accum ++;
		}
	}

	return accum;
}