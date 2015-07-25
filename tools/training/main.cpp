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

void loadDataset(MatrixXd &_xSet, MatrixXd &_ySet, std::string _file) ;

double colorMedValue(const Mat &_frame, int _x, int _y, int _r, int _channel);
int numKeyArround(const KeyPoint &_keypoint, const std::vector<KeyPoint> &_keypoints, double _range);


int main(int _argc, char** _argv){
	_argc; _argv;

	DroneApplication app;

	// Train Neuronal network
	MatrixXd xSet;
	MatrixXd ySet;
	loadDataset(xSet, ySet, "dataset.txt");

	MatrixXd trainSetX = xSet.block(0,0, xSet.rows()*60/100, xSet.cols());
	MatrixXd trainSetY = ySet.block(0,0, ySet.rows()*60/100, ySet.cols());

	MatrixXd cvSetX = xSet.block(xSet.rows()*60/100 + 1,0, xSet.rows()*20/100, xSet.cols());
	MatrixXd cvSetY = ySet.block(ySet.rows()*60/100 + 1,0, ySet.rows()*20/100, ySet.cols());

	MatrixXd testSetX = xSet.block(xSet.rows()*80/100 + 1 ,0, xSet.rows()*20/100-1, xSet.cols());
	MatrixXd testSetY = ySet.block(ySet.rows()*80/100 + 1 ,0, ySet.rows()*20/100-1, ySet.cols());

	NeuronalNetwork<16, 1, 16, 1> nn;
	nn.train(trainSetX, trainSetY, 0.15, 0.5, 300);

	double tp = 0, tn = 0, fp = 0, fn = 0;
	for (int i = 0; i < cvSetX.rows(); i++) {
		MatrixXd y = nn.evaluate(cvSetX.block(i,0, 1, cvSetX.cols()).transpose());
		int yi = y(0,0)> 0.5?1:0;
		if (yi == int(ySet(i, 0))) {
			if(yi == 1)
				tp++;
			else
				tn++;
		}
		else {
			if(yi == 1)
				fp++;
			else
				fn++;
		}
	}

	double accuracy = (tp + tn)/(tp+tn+fp+fn);
	double precision = tp/(tp + fp);
	double recall = tp/(tp + fn);
	double Fscore = 2*precision*recall/(precision + recall);

	auto params = nn.parameters();
	for (unsigned i = 0; i < params.size(); i++) {
		ofstream paramFile("params" + to_string(i) + ".txt");
		paramFile.write( (char *) params[i].data(), params[i].rows() * params[i].cols() * sizeof(double) );
		paramFile.flush();
	}

	auto normParams = nn.normalParams();
	ofstream nuFile("nuParams.txt");
	nuFile.write( (char *) normParams.first.data(), normParams.first.rows() * normParams.first.cols() * sizeof(double) );
	nuFile.flush();
	ofstream sigmaFile("sigmaParams.txt");
	sigmaFile.write( (char *) normParams.second.data(), normParams.second.rows() * normParams.second.cols() * sizeof(double) );
	sigmaFile.flush();
	
	ofstream results("Results.txt");
	results << "tp \t tn \t fp \t fn \t accuracy \t precision \t recall \t Fscore \n";
	results << tp << "\t" << tn << "\t" <<fp << "\t" <<fn << "\t" <<accuracy << "\t" << precision << "\t" << recall << "\t" <<Fscore << "\n";
	results.flush();
	results.close();

	std::cout << "Finished" << std::endl;
}


void loadDataset(MatrixXd &_xSet, MatrixXd &_ySet, std::string _file) {
	std::ifstream dataFile(_file);
	assert(dataFile.is_open());
	std::string line;
	unsigned i = 0;
	_xSet = MatrixXd(20000, 16);
	_ySet = MatrixXd::Zero(20000,1);
	while(std::getline(dataFile, line) && i < 20000) {
		// Get result
		int index = line.find(",");
		_ySet(i,0) = atoi(line.substr(0,index).c_str());
		line = line.substr(index+1);

		// Get inputs
		int j = 0;
		do {
			int index = line.find(",");
			_xSet(i,j) = atof(line.substr(0,index).c_str());
			line = line.substr(index+1);
			j++;
		}while(line.find(",") != std::string::npos);
		_xSet(i,j) = atof(line.c_str());
		i++;
	}
	dataFile.close();
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