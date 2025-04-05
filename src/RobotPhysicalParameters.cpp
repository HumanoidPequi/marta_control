/*
 * RobotPhysicalParameters.cpp
 *
 *  Created on: Sep 6, 2014
 *      Author: mmaximo
 */

#include "RobotPhysicalParameters.h"

#include <math.h>
#include <stdio.h>

namespace representations {

RobotPhysicalParameters RobotPhysicalParameters::getMartaPhysicalParameters() {
    RobotPhysicalParameters physicalParameters;

    physicalParameters.torsoToLeftHip = itandroids_lib::math::Vector3<double>(0.027349, 0.040504, -0.059509);
    physicalParameters.leftAnkleToLeftFoot = itandroids_lib::math::Vector3<double>(0.0, 0.0, -0.13305);

    physicalParameters.armLength = 0.314815;

    physicalParameters.upperLegLength = 0.098202;
    physicalParameters.lowerLegLength = 0.0974;
    double legHeight0 = 0.195602;
    double a = physicalParameters.upperLegLength;
    double b = physicalParameters.lowerLegLength;
    double c = legHeight0;

    double gamma0 = acos((a * a + b * b - c * c) / (2.0 * a * b));
    double beta0 = acos((a * a + c * c - b * b) / (2.0 * a * c));
    double alpha0 = acos((b * b + c * c - a * a) / (2.0 * b * c));

    physicalParameters.hipPitch0 = beta0;
    physicalParameters.kneePitch0 = -(M_PI - gamma0);
    physicalParameters.anklePitch0 = alpha0;

    return physicalParameters;
}

} /* namespace representations */
