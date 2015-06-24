///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	EUROARMS:	Bar Task
//		Author:	Pablo Ramon Soria
//		Date:	2015-MAY-05
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//

#include "CarParticleFilter.h"

#include <Eigen/Eigen>
#include <random>
#include <time.h>

using namespace std;

//---------------------------------------------------------------------------------------------------------------------
// Static data definition
double CarParticle::sMinX = -5.0;
double CarParticle::sMinY = -5.0;
double CarParticle::sMaxX = 5.0;
double CarParticle::sMaxY = 5.0;

std::random_device	sRandomDevice;
std::mt19937		sUniformGenerator(sRandomDevice());

//---------------------------------------------------------------------------------------------------------------------
// "Private funcions"
double gaussian(const double & _nu, const double & _sigma, const double & _x);
double randomGaussian(const double & _nu, const double & _sigma);
double gaussian2d(const Eigen::Vector2d& _nu, const Eigen::Matrix2d& _cov, const Eigen::Vector2d& _x);


//---------------------------------------------------------------------------------------------------------------------
void CarParticle::setParams(Json &_params){
	srand(unsigned(time(NULL)));
	
	sMinX = _params["minx"].asInt();
	sMinY = _params["miny"].asInt();
	sMaxX = _params["maxx"].asInt();
	sMaxY = _params["maxy"].asInt();
}

//---------------------------------------------------------------------------------------------------------------------
CarParticle::CarParticle(){
	mPosition[0] = (sMaxX - sMinX)*double(rand()) / RAND_MAX + sMinX;
	mPosition[1] = (sMaxY - sMinY)*double(rand()) / RAND_MAX + sMinY;
	mSpeed[0] = double(rand())/RAND_MAX*0.1;
	mSpeed[1] = double(rand())/RAND_MAX*0.1;
}

//---------------------------------------------------------------------------------------------------------------------
void CarParticle::simulate(){
	const double cIncT = 0.03;	// Approx 30 fps of camera.
	mPosition[0] += mSpeed[0] * cIncT + randomGaussian(0.0, 0.05);
	mPosition[1] += mSpeed[1] * cIncT + randomGaussian(0.0, 0.05);
	mSpeed[0] += randomGaussian(0.0, 0.025);
	mSpeed[1] += randomGaussian(0.0, 0.025);
}

//---------------------------------------------------------------------------------------------------------------------
void CarParticle::calcWeigh(std::vector<std::array<double, 2> > &_data){
	double prob = 0.0;
	unsigned index = 0;
	Eigen::Vector2d nu;
	Eigen::Matrix2d cov;
	Eigen::Vector2d particlePos;
	particlePos << mPosition[0], mPosition[1];

	for (unsigned i = 0; i < _data.size(); i++){
		nu << _data[i][0], _data[i][1];
		cov << 0.2, 0.0, 0.0, 0.2;
		double auxProb = gaussian2d(nu, cov, particlePos);
		if (auxProb > prob){
			index = i;
			prob = auxProb;
		}
	}

	mWeight = std::pair<unsigned, double>(index, prob);
}


//---------------------------------------------------------------------------------------------------------------------
std::array<double, 2> CarParticle::position(){
	return mPosition;
}
//---------------------------------------------------------------------------------------------------------------------
std::array<double, 2> CarParticle::speed(){
	return mSpeed;
}


//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
double gaussian(const double & _nu, const double & _sigma, const double & _x) {
	return exp(-(pow(_x - _nu, 2)) * 0.5 / (pow(_sigma, 2))) / sqrt(2.0 * 3.1416) / _sigma;
}

//---------------------------------------------------------------------------------------------------------------------
double randomGaussian(const double & _nu, const double & _sigma) {
	std::normal_distribution<> d(_nu, _sigma);

	return d(sUniformGenerator);
}

//---------------------------------------------------------------------------------------------------------------------
double gaussian2d(const Eigen::Vector2d& _nu, const Eigen::Matrix2d& _cov, const Eigen::Vector2d& _x){
	double res = 1 / (sqrt(pow(2 * M_PI, 2))*_cov.determinant());
	Eigen::Matrix2d invCov = _cov.inverse();
	Eigen::Vector2d difX = _x - _nu;
	res *= exp(-0.5* (_x - _nu).transpose()*invCov*(_x - _nu));
	//res *= exp(-0.5 *difX(0)*(difX(0)*invCov(0, 0) + difX(1)*invCov(1, 0)) + difX(1)*(difX(0)*invCov(0, 1) + difX(1)*invCov(1, 1)));
	return res;
}