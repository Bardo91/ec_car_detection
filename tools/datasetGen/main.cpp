///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	EUROARMS:	Bar Task
//		Author:	Pablo Ramon Soria
//		Date:	2015-02-05
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This is the main of the program.This program is an implementation to solve the trouble of catching bars
// in order to build afterwards structures.

#include <core/time/time.h>
#include <DroneApplication.h>
#include <implementations/sensors/OpencvImageFileSensor.h>
#include <implementations/sensors/ImuSimulatorSensor.h>

#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>

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

using namespace std;
using namespace cv;

void mouseCallback(int event, int x, int y, int flags, void* userdata);
double colorMedValue(const Mat &_frame, int _x, int _y, int _r, int _channel);
int numKeyArround(const KeyPoint &_keypoint, const std::vector<KeyPoint> &_keypoints, double _range);

vector<KeyPoint> keypoints;	// To be used with mouse callback
vector<bool> yClass;
Mat frame, display;
const string winName = "Display";

int main(int _argc, char** _argv){
	_argc; _argv;
	do_mkdir("results");
	ofstream dataset("results/dataset.txt");
	
	namedWindow(winName, 1);
	setMouseCallback(winName, mouseCallback, NULL);


	DroneApplication mainApp;

	OpencvImageFileSensor vSensor("C:/programming/datasets/images_500mm/image%d.jpg",829);

	Mat descriptors, ori;

	for (;;) {
		// Clear data for next step
		keypoints.clear();
		yClass.clear();

		// Get image
		frame = vSensor.get();
		if(frame.rows == 0)
			break;

		// To gray scale
		frame.copyTo(ori);
		cvtColor(frame, frame, CV_BGR2GRAY);
		
		// Compute features
		FAST(frame, keypoints, 5);
		yClass.resize(keypoints.size());

		// First display
		frame.copyTo(display);
		for (unsigned i = 0 ; i < keypoints.size(); i++) {
			circle(display, keypoints[i].pt, 5, Scalar(255,255,255));
			yClass[i] = false;
		}

		imshow(winName, display);

		// Wait until esc
		int key;
		while ((key = waitKey()) != 27) {	}
		
		// Store Data
		for (unsigned i = 0; i < keypoints.size(); i++) {
			dataset << yClass[i] << "," <<
					colorMedValue(ori, keypoints[i].pt.x, keypoints[i].pt.y, 4, 0) << "," << 
					colorMedValue(ori, keypoints[i].pt.x, keypoints[i].pt.y, 8, 0) << "," <<
					colorMedValue(ori, keypoints[i].pt.x, keypoints[i].pt.y, 16, 0) << "," <<
					colorMedValue(ori, keypoints[i].pt.x, keypoints[i].pt.y, 32, 0) << "," << 
					colorMedValue(ori, keypoints[i].pt.x, keypoints[i].pt.y, 4, 1) << "," << 
					colorMedValue(ori, keypoints[i].pt.x, keypoints[i].pt.y, 8, 1) << "," <<
					colorMedValue(ori, keypoints[i].pt.x, keypoints[i].pt.y, 16, 1) << "," <<
					colorMedValue(ori, keypoints[i].pt.x, keypoints[i].pt.y, 32, 1) << "," <<
					colorMedValue(ori, keypoints[i].pt.x, keypoints[i].pt.y, 4, 2) << "," << 
					colorMedValue(ori, keypoints[i].pt.x, keypoints[i].pt.y, 8, 2) << "," <<
					colorMedValue(ori, keypoints[i].pt.x, keypoints[i].pt.y, 16, 2) << "," <<
					colorMedValue(ori, keypoints[i].pt.x, keypoints[i].pt.y, 32, 2) << "," <<
					numKeyArround(keypoints[i], keypoints, 4) << "," <<
					numKeyArround(keypoints[i], keypoints, 8) << "," << 
					numKeyArround(keypoints[i], keypoints, 16) << "," <<
					numKeyArround(keypoints[i], keypoints, 32) << "\n";
		}
		dataset.flush();
	}

	dataset.close();
	//system("PAUSE");
	
}

bool pressed = false;
Point2i lastClick;
void mouseCallback(int _event, int _x, int _y, int _flags, void* _userdata) {
	if (_event == EVENT_LBUTTONDOWN && !pressed) {
		pressed = !pressed;
		lastClick = Point2i(_x, _y);
	}
	else if (_event == EVENT_LBUTTONUP && pressed) {
		pressed = !pressed;
		int lbx = lastClick.x < _x ? lastClick.x : _x;
		int lby = lastClick.y < _y ? lastClick.y : _y;
		int rtx = lastClick.x > _x ? lastClick.x : _x;
		int rty = lastClick.y > _y ? lastClick.y : _y;

		for (int i = 0; i < keypoints.size(); i++) {
			KeyPoint keypoint = keypoints[i];
			if (keypoint.pt.x > lbx && keypoint.pt.x < rtx) {
				if (keypoint.pt.y > lby && keypoint.pt.y < rty) {
					yClass[i] = !yClass[i];
				}
			}

			if (yClass[i]) {
				circle(display, keypoint.pt, 5, Scalar(0,0,255));
			}
			else {
				circle(display, keypoint.pt, 5, Scalar(255,255,255));
			}
		}

		imshow(winName, display);
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