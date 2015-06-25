///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	EUROARMS:	Bar Task
//		Author:	Pablo Ramon Soria
//		Date:	2015-02-05
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This is the main of the program.This program is an implementation to solve the trouble of catching bars
// in order to build afterwards structures.

#include "DroneApplication.h"
#include "tasks/DetectionTask.h"

#include <implementations/sensors/OpencvVideoSensor.h>
#include <implementations/sensors/OpencvCameraSensor.h>
#include <implementations/sensors/OpencvImageFileSensor.h>
#include <implementations/sensors/ImuSimulatorSensor.h>

#include <core/types/file/file.h>
#include <core/types/file/json.h>
#include <core/time/time.h>

#include <iostream>
#include <sstream>

using namespace std;

Json loadData(std::string _path);

int main(int _argc, char** _argv){
	_argc; _argv;
	DroneApplication mainApp;
	Json data = loadData("data.json");
	
	// Init Vision sensor
	//OpencvVideoSensor	vSensor;
	//OpencvCameraSensor	vSensor;
	OpencvImageFileSensor vSensor(data["dataset"]["name"].asText(), data["dataset"]["index"].asInt());
	
	mainApp.registerSensor(*((OpencvSensor*)&vSensor));
	ImuSimulatorSensor imuSensor(data["dataset"]["vicon"].asText());
	mainApp.registerSensor(imuSensor);

	// Task
	DetectionTask process(data);
	mainApp.attachProcess(process);
	process.start();

	for (;;){
		mainApp.step();
		STime::get()->delay(100);
	}

	//system("PAUSE");

}

Json loadData(std::string _path){
	File * dataFile = File::openExisting(_path);
	assert(dataFile);
	dataFile->readAll();
	Json loadedData(dataFile->bufferAsText());
	delete dataFile;
	return loadedData;
}