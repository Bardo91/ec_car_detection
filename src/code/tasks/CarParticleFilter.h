///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	EUROARMS:	Bar Task
//		Author:	Pablo Ramon Soria
//		Date:	2015-MAY-05
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//

#ifndef CAR_PARTICLE_FILTER_H_
#define CAR_PARTICLE_FILTER_H_

#include <algorithms/state_estimators/ParticleFilterCPU.h>
#include <core/types/file/json.h>

#include <array>

struct CameraInfo{
	double mU0;
	double mV0;
	unsigned mWidth;
	unsigned mHeight;
	double mFocal;
};

class CarParticle;

typedef ParticleFilterCPU<CarParticle, std::vector<std::array<double,2> > > CarParticleFilter;

class CarParticle :public CarParticleFilter::Particle{
public:
	static void setParams(Json &_params);

	CarParticle();
	void simulate();
	void calcWeigh(std::vector<std::array<double, 2> > &_data);

	std::array<double, 2> position();
	std::array<double, 2> speed();

private:
	std::array<double, 2> mPosition;
	std::array<double, 2> mSpeed;

	static double sMinX;
	static double sMinY;
	static double sMaxX;
	static double sMaxY;
};



#endif	//	CAR_PARTICLE_FILTER_H_