///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		EC-SAFEMOBIL: Particle Filter
//			Author: Pablo Ramon Soria
//			Date:	2015-FEB-01
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////ç
// 


#ifndef MAP_H_
#define MAP_H_

#include <core/types/BasicTypes.h>
#include <core/types/file/json.h>

#include <array>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

enum eColor {eRed, eGreen, eBlue, ePurple};

class Map{
public:
	Map(std::string _name, unsigned _resX, unsigned _resY, Json &_info);

	bool loadMap(Json &_info);
	void addPoints(const std::vector<std::array<double, 2>>  &_objects, eColor _color);
	void addEllipse(const std::vector<std::array<double, 5>> &_data, eColor _color);
	void clean();

	void show();
	void store(std::string _filename = "");

private:
	void draw();

private:
	std::string mName;
	cv::Mat mMap;
	int  mMaxX = 0, mMaxY = 0, mMinX = 0, mMinY = 0;
	unsigned mResX = 0, mResY = 0;

	std::vector<std::pair<std::array<double, 2>, eColor> > mObjects;
	std::vector<std::pair<std::array<double, 5>, eColor> > mEllipses;
};

#endif	//	MAP_H_