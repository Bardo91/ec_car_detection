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
void plotCost(const std::vector<double> &_cost);

int main(int _argc, char** _argv){
	_argc; _argv;

	DroneApplication app;

	// Train Neuronal network
	MatrixXd xSet;
	MatrixXd ySet;
	loadDataset(xSet, ySet, "dataset.txt");
	std::cout << "Loaded Dataset" << std::endl;

	MatrixXd trainSetX = xSet.block(0,0, xSet.rows()*60/100, xSet.cols());
	MatrixXd trainSetY = ySet.block(0,0, ySet.rows()*60/100, ySet.cols());

	MatrixXd cvSetX = xSet.block(xSet.rows()*60/100 + 1,0, xSet.rows()*20/100, xSet.cols());
	MatrixXd cvSetY = ySet.block(ySet.rows()*60/100 + 1,0, ySet.rows()*20/100, ySet.cols());

	MatrixXd testSetX = xSet.block(xSet.rows()*80/100 + 1 ,0, xSet.rows()*20/100-1, xSet.cols());
	MatrixXd testSetY = ySet.block(ySet.rows()*80/100 + 1 ,0, ySet.rows()*20/100-1, ySet.cols());

	NeuronalNetwork<16, 1, 16, 1> nn;
	std::cout << "Start training" << std::endl;
	nn.train(trainSetX, trainSetY, 1, 2, 1500);
	std::cout << "End training" << std::endl;

	double tp = 0, tn = 0, fp = 0, fn = 0;
	for (int i = 0; i < cvSetX.rows(); i++) {
		MatrixXd y = nn.evaluate(cvSetX.block(i,0, 1, cvSetX.cols()).transpose());
		int yi = y(0,0)> 0.2?1:0;
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

	plotCost(nn.costHistory());

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
	waitKey();
}

void plotCost(const std::vector<double> &_cost) {
	Mat display(480,640,CV_8UC3);

	double minCost = *std::min_element(_cost.begin(), _cost.end());
	double maxCost = *std::max_element(_cost.begin(), _cost.end());

	const int xMin = 20, xMax = 620, yMin = 20, yMax = 460;
	for (unsigned i = 0; i < _cost.size() - 1; i++) {
		int x1 = (i)*(xMax - xMin)/_cost.size() + xMin;
		int y1 = 480 -  int((_cost[i] - minCost)*(yMax - yMin)/(maxCost - minCost) + yMin);

		int x2 = (i+1)*(xMax - xMin)/_cost.size() + xMin;
		int y2 = 480 - int((_cost[i+1] - minCost)*(yMax - yMin)/(maxCost - minCost) + yMin);


		line(display, Point2i(x1,y1), Point2i(x2, y2), Scalar(0,0,255), 2);
	}

	line(display, Point2i(xMin,480 - yMin), Point2i(xMin, 480 - yMax), Scalar(255,255,255), 2);
	line(display, Point2i(xMin,480 - yMin), Point2i(xMax, 480 - yMin), Scalar(255,255,255), 2);
	putText(display, to_string(minCost), Point2i(0, 480 - yMin), FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, Scalar(255,255,255));
	putText(display, to_string(maxCost), Point2i(0, 480 - yMax), FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, Scalar(255,255,255));
	imshow("Cost Function", display);
}


void loadDataset(MatrixXd &_xSet, MatrixXd &_ySet, std::string _file) {
	std::ifstream dataFile(_file);
	assert(dataFile.is_open());
	std::string line;
	unsigned i = 0;
	_xSet = MatrixXd(10000, 16);
	_ySet = MatrixXd::Zero(10000,1);
	while(std::getline(dataFile, line) && i < 10000) {
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
