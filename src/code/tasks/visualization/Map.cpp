///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		EC-SAFEMOBIL: Particle Filter
//			Author: Pablo Ramon Soria
//			Date:	2015-FEB-01
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////ç
//

#include "Map.h"

#include <core/types/BasicTypes.h>
#include <sstream>
#include <time.h>

using namespace cv;
using namespace std;
using namespace BOViL;

//---------------------------------------------------------------------------------------------------------------------
// Public interface
Map::Map(std::string _name, unsigned _resX, unsigned _resY, Json &_info) : mResX(_resX), mResY(_resY){
	//	Clean Map
	loadMap(_info);
	clean();
	mName = _name;
}

//---------------------------------------------------------------------------------------------------------------------
bool Map::loadMap(Json &_info){
	//	Get Map dimmensions.
	mMinX = _info["minx"].asInt();
	mMinY = _info["miny"].asInt();
	mMaxX = _info["maxx"].asInt();
	mMaxY = _info["maxy"].asInt();

	//	Reserve window name.
	//namedWindow("Map Representation", CV_WINDOW_AUTOSIZE);

	return true;
}
//---------------------------------------------------------------------------------------------------------------------
void Map::addPoints(const vector<array<double,2>> &_objects, eColor _color){
	//	Add objects to private list.
	for (array<double, 2> obj : _objects)
		mObjects.push_back(pair<array<double, 2>, eColor>(obj, _color));
}

//---------------------------------------------------------------------------------------------------------------------
void Map::addEllipse(const vector<array<double, 5>> &_data, eColor _color){
	//	Add objects to private list.
	for (array<double, 5> data : _data)
		mEllipses.push_back(pair<array<double, 5>, eColor>(data, _color));
}

//---------------------------------------------------------------------------------------------------------------------
void Map::clean(){
	//	Set image to black.
	mMap = Mat::zeros(mResX, mResY, CV_8UC3);

	//	Erase objects.
	mObjects.clear();
	mEllipses.clear();
}

//---------------------------------------------------------------------------------------------------------------------
void Map::show(){
	//	Show image
	draw();
	imshow(mName, mMap);

	waitKey(1);

}

//---------------------------------------------------------------------------------------------------------------------
void Map::store(std::string _filename){
	if (_filename.size() == 0){	//	If there's no name, fill it.
		time_t t = time(nullptr);
		stringstream ss;
		ss << "map_representation_" << t << ".jpg";

		_filename = ss.str();
	}

	//	Save image.
	imwrite(_filename, mMap);

}

//---------------------------------------------------------------------------------------------------------------------
//	Private interface
void Map::draw(){
	//	Draw coordinates Edges	666 TODO
	int x0 = int((0.0 - mMinX) / (mMaxX - mMinX)*mResX);
	int y0 = int((0.0 - mMinY) / (mMaxY - mMinY)*mResY);
	Point2i plv1(x0, 0);
	Point2i plv2(x0, mResX);
	Point2i plh1(0,			y0);
	Point2i plh2(mResY,		y0);

	line(mMap, plv1, plv2, Scalar(255, 255, 255));
	line(mMap, plh1, plh2, Scalar(255, 255, 255));

	putText(mMap, "(0,0)", Point2i(x0, y0), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255));	// Oriting
	putText(mMap, "(0," + to_string(mMinY) + ")", Point2i(x0, 10), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255));	// Margins
	putText(mMap, "(0," + to_string(mMaxY) + ")", Point2i(x0, mResY), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255));
	putText(mMap, "(" + to_string(mMinX) + ",0)", Point2i(0, y0), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255));
	putText(mMap, "(" + to_string(mMaxX) + ",0)", Point2i(mResX-35, y0), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255));

	//	Draw targes
	for (pair<array<double, 2>, eColor> obj : mObjects){
		Point2i p(	int((obj.first[0] - mMinX) / (mMaxX - mMinX)*mResX),
					int((obj.first[1] - mMinY) / (mMaxY - mMinY)*mResY));
		
		Scalar col;
		int size = 1;
		switch (obj.second) {
		case eColor::eRed:
			col = Scalar(0, 0, 255);
			break;
		case eColor::eGreen:
			col = Scalar(0, 255, 0);
			size = 5;
			break;
		case eColor::eBlue:
			col = Scalar(255, 0, 0);
			break;
		case eColor::ePurple:
			col = Scalar(255, 0, 255);
			size = 5;
			break;
		default:
			break;
		}

		circle(	mMap, 
				p, 
				1, col, size);
	}
	//	Draw Ellipses
	for (pair<array<double, 5>, eColor> data : mEllipses){
		Point2i p(int((data.first[0]- mMinX) / (mMaxX - mMinX)*mResX),
			int((data.first[1] - mMinY) / (mMaxY - mMinY)*mResY));


		Size_<double> size(data.first[2] / (mMaxX - mMinX) *mResX, 
					data.first[3] / (mMaxY - mMinY) * mResY);

		double angle = data.first[4]*180 / 3.141598  + 180;

		Scalar col;
		switch (data.second) {
		case eColor::eRed:
			col = Scalar(0, 0, 255);
			break;
		case eColor::eGreen:
			col = Scalar(0, 255, 0);
			break;
		case eColor::eBlue:
			col = Scalar(255, 0, 0);
			break;
		default:
			break;
		}

		ellipse(mMap, p, size, angle, 0, 360, col);
		circle(mMap,
			p,
			1, col, 3);
	}


}

//---------------------------------------------------------------------------------------------------------------------
