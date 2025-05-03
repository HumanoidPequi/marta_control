/*
 * LinearInterpolator.h
 *
 *  Created on: Oct 16, 2015
 *      Author: root
 */

#ifndef SOURCE_MATH_LINEARINTERPOLATOR_H_
#define SOURCE_MATH_LINEARINTERPOLATOR_H_

#include "Interpolator.h"

namespace itandroids_lib{
namespace math {

class LinearInterpolator: public Interpolator {
public:
	LinearInterpolator(double initialPosition, double finalPosition);
	virtual ~LinearInterpolator();
	double interpolate(double fraction);
private:
	double initialPosition;
	double finalPosition;
};

} /* namespace keyframe */
} /* namespace control */

#endif /* SOURCE_MATH_LINEARINTERPOLATOR_H_ */
