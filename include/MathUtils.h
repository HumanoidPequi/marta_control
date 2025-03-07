/*
 * MathUtils.h
 *
 *  Created on: Aug 16, 2015
 *      Author: root
 */

#ifndef SOURCE_MATH_MATHUTILS_H_
#define SOURCE_MATH_MATHUTILS_H_

#include <boost/random.hpp>
#include <boost/math/constants/constants.hpp>
#include "LinearInterpolator.h"

namespace itandroids_lib {
namespace math {

class MathUtils {
public:
	MathUtils();
	virtual ~MathUtils();
	static double sign(double value);
	static double linearInterpol(double begin, double end, double t);
	/**
	 * Interpolate two points linearly
	 * @param begin
	 * @param end
	 * @param time t
	 * @return
	 */
	static void lerp(double* begin, double* end, double t, int n,
			double*result);
	static double sigmoid(double value);
	static double clamp(double value, double min, double max);
	static double clamp(int value, int min, int max);
	static double normalizeAngle(double radians);
	static double normalizeSumOfAngles(std::vector<double> anglesRadians);
	static double degreesToRadians(double degrees);
	static double radiansToDegrees(double radians);
	static double radiansPerSecondToRpm(double radiansSecond);
	static double saturate(double value, double min, double max);
	static std::vector<double> tridiagonalSolution(std::vector<double> a,
			std::vector<double> b, std::vector<double> c,
			std::vector<double> d);
	static boost::variate_generator<boost::mt19937&,
			boost::normal_distribution<> > getNormalRandomGenerator(double mean,
			double sigma);
	static boost::variate_generator<boost::mt19937&, boost::uniform_real<> > getUniformRandomGenerator(
			double min, double max);

	static boost::mt19937 generator;

private:

};

} /* namespace math */
} /* namespace itandroids_lib */

#endif /* SOURCE_MATH_MATHUTILS_H_ */
