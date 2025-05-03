/*
 * InverseKinematics.cpp
 *
 *  Created on: Mar 2, 2014
 *      Author: itandroids
 */

#include "InverseKinematics.h"

#include <stdio.h>
#include "math.h"



namespace control {

namespace walking {

InverseKinematics::InverseKinematics(RobotPhysicalParameters physicalParameters) :
		upperLegLength(physicalParameters.upperLegLength), lowerLegLength(
				physicalParameters.lowerLegLength), hipPitch0(
				physicalParameters.hipPitch0), kneePitch0(
				physicalParameters.kneePitch0), anklePitch0(
				physicalParameters.anklePitch0), physicalParameters(
				physicalParameters) {
}

InverseKinematics::~InverseKinematics() {
}

void InverseKinematics::computeJointsTargetsOneLeg(double x, double y, double z,
		double theta, double* hipPitch, double* kneePitch, double* anklePitch,
		double* hipRoll, double* ankleRoll, double *hipYaw) {
	double yz = sqrt(y * y + z * z);
	double c = sqrt(x * x + y * y + z * z);
	double a = upperLegLength;
	double b = lowerLegLength;

	double gamma = acos((a * a + b * b - c * c) / (2.0f * a * b));
	double beta = acos((a * a + c * c - b * b) / (2.0f * a * c));
	double alpha = acos((b * b + c * c - a * a) / (2.0f * b * c));

	//printf("alpha: %f, beta: %f, gamma: %f\n", alpha * 180.0f / M_PI, beta * 180.0f / M_PI, gamma * 180.0f / M_PI);

	*hipPitch = -(atan2(x, yz) + beta) - hipPitch0;
	*kneePitch = M_PI - gamma - kneePitch0;
	*anklePitch = atan2(x, yz) - alpha - anklePitch0;
	*hipRoll = atan2(y, z);
	*ankleRoll = -*hipRoll;
	*hipYaw = theta;



	//if(has == 1.0){
	//	appendToCSV(m_filename, std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z));
	//	appendToCSV(j_filename, std::to_string(*hipPitch) + "," + std::to_string(*kneePitch) + "," + std::to_string(*anklePitch) + "," + std::to_string(*hipRoll) + "," + std::to_string(*ankleRoll) + "," + std::to_string(*hipYaw));
	//}

}

void InverseKinematics::computeJointsTargets(double xLeft, double yLeft,
		double zLeft, double thetaLeft, double xRight, double yRight, double zRight,
		double thetaRight, Joints& jointsTargets, float &Xscale_) {
	xLeft -= physicalParameters.torsoToLeftHip.x * Xscale_;
	yLeft -= physicalParameters.torsoToLeftHip.y;
	xRight -= physicalParameters.torsoToLeftHip.x * Xscale_;
	yRight += physicalParameters.torsoToLeftHip.y;

	zLeft += physicalParameters.torsoToLeftHip.z;
	zRight += physicalParameters.torsoToLeftHip.z;

	double xLeftAux = xLeft;
	double yLeftAux = yLeft;
	xLeft = xLeftAux * cos(thetaLeft) + yLeftAux * sin(thetaLeft);
	yLeft = -xLeftAux * sin(thetaLeft) + yLeftAux * cos(thetaLeft);

	double xRightAux = xRight;
	double yRightAux = yRight;
	xRight = xRightAux * cos(thetaRight) + yRightAux * sin(thetaRight);
	yRight = -xRightAux * sin(thetaRight) + yRightAux * cos(thetaRight);

	xLeft -= physicalParameters.leftAnkleToLeftFoot.x;
	yLeft -= physicalParameters.leftAnkleToLeftFoot.y;
	xRight -= physicalParameters.leftAnkleToLeftFoot.x;
	yRight += physicalParameters.leftAnkleToLeftFoot.y;

	zLeft += physicalParameters.leftAnkleToLeftFoot.z;
	zRight += physicalParameters.leftAnkleToLeftFoot.z;

	computeJointsTargetsOneLeg(xLeft, yLeft, zLeft, thetaLeft,
			&(jointsTargets.leftHipPitch), &(jointsTargets.leftKneePitch),
			&(jointsTargets.leftAnklePitch), &(jointsTargets.leftHipRoll),
			&(jointsTargets.leftAnkleRoll), &(jointsTargets.leftHipYaw));
	computeJointsTargetsOneLeg(xRight, yRight, zRight, thetaRight,
			&(jointsTargets.rightHipPitch), &(jointsTargets.rightKneePitch),
			&(jointsTargets.rightAnklePitch), &(jointsTargets.rightHipRoll),
			&(jointsTargets.rightAnkleRoll), &(jointsTargets.rightHipYaw));
}

} /* namespace walking */

} /* namespace control */
