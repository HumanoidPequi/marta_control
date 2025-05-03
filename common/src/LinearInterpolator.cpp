/*
 * LinearInterpolator.cpp
 *
 *  Created on: Oct 16, 2015
 *      Author: root
 */

#include "LinearInterpolator.h"

namespace itandroids_lib {
namespace math {

LinearInterpolator::LinearInterpolator(double initialPosition,
		double finalPosition) :
		initialPosition(initialPosition), finalPosition(finalPosition) {
	// TODO Auto-generated constructor stub

}

LinearInterpolator::~LinearInterpolator() {
	// TODO Auto-generated destructor stub
}

double LinearInterpolator::interpolate(double fraction){
	return initialPosition + (finalPosition - initialPosition)*fraction;
}

} /* namespace keyframe */
} /* namespace control */
