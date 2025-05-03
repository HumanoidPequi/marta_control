/*
 * MathUtils.cpp
 *
 *  Created on: Aug 16, 2015
 *      Author: root
 */

#include "MathUtils.h"

namespace itandroids_lib {
namespace math {

boost::mt19937 MathUtils::generator;

MathUtils::MathUtils() {
	// TODO Auto-generated constructor stub

}

MathUtils::~MathUtils() {
	// TODO Auto-generated destructor stub
}

double MathUtils::degreesToRadians(double degrees) {
	return (degrees * (boost::math::constants::pi<double>())) / 180.0;
}

/**
 *
 * @param radians
 * @return degrees in double
 */

double MathUtils::radiansToDegrees(double radians) {
	return (radians * 180.0) / (boost::math::constants::pi<double>());
}

double MathUtils::radiansPerSecondToRpm(double radiansSecond) {
	return radiansSecond * 30 / M_PI;
}

double MathUtils::saturate(double value, double min, double max) {
	if (value > max)
		return max;
	else if (value < min)
		return min;
	return value;
}

std::vector<double> MathUtils::tridiagonalSolution(std::vector<double> a, std::vector<double> b, std::vector<double> c, std::vector<double> d){
	c[0] = c[0] / b[0];
	d[0] = d[0] / b[0];
	int length = a.size();
	std::vector<double> retVector(length);
	for (int i = 1; i < length; i++){
		c[i] = c[i] / (b[i] - a[i]*c[i-1]);
		d[i] = (d[i] - a[i]*d[i-1]) / (b[i] - a[i] * c[i-1]);
	}
	retVector[length -1] = d[length-1];
	for (int i = length-2; i>=0; i--){
		retVector[i] = d[i] - c[i] * retVector[i+1];
	}
	return retVector;
}

double MathUtils::sign(double value) {
	if (value > 0)
		return 1.0f;
	else if (value < 0)
		return -1.0f;
	else
		return 0.0f;
}

double MathUtils::linearInterpol(double begin , double end, double t){
	LinearInterpolator interpolator(begin, end);
	return interpolator.interpolate(t);
}



void MathUtils::lerp(double* begin, double* end, double t, int n,
		double* result) {
	for (int i = 0; i < n; ++i) {
		LinearInterpolator interpolator(begin[i],end[i]);
		result[i] = interpolator.interpolate(t);
	}
}

double MathUtils::sigmoid(double value) {
	return 1.0f / (1.0f + exp(-value));
}

double MathUtils::clamp(double value, double min, double max) {
	return std::max(min, std::min(max, value));
}

double MathUtils::clamp(int value, int min, int max) {
	return std::max(min, std::min(max, value));
}

double MathUtils::normalizeAngle(double radians) {
	while (radians < -M_PI)
		radians += 2 * M_PI;
	while (radians >= M_PI)
		radians -= 2 * M_PI;
	return radians;
}

double MathUtils::normalizeSumOfAngles(std::vector<double> anglesRadians) {
    double sumSin = 0.0;
    double sumCos = 0.0;
    for(auto a : anglesRadians) {
        sumSin += sin(a);
        sumCos += cos(a);
    }

    return atan2(sumSin, sumCos);
}

boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > MathUtils::getNormalRandomGenerator(
		double mean, double sigma) {
	boost::normal_distribution<> normal(mean, sigma);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > normalDistribution(
			generator, normal);
	return normalDistribution;
}

boost::variate_generator<boost::mt19937&, boost::uniform_real<> > MathUtils::getUniformRandomGenerator(
		double min, double max) {
	boost::uniform_real<> uniform(min, max);
	boost::variate_generator<boost::mt19937&, boost::uniform_real<> > uniformDistribution(
			generator, uniform);
	return uniformDistribution;
}

} /* namespace math */
} /* namespace itandroids_lib */
