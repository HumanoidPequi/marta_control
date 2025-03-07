/*
 * InverseKinematics.h
 *
 *  Created on: Mar 2, 2014
 *      Author: itandroids
 */

#ifndef INVERSEKINEMATICS_H_
#define INVERSEKINEMATICS_H_

#include "Joints.h"
#include "RobotPhysicalParameters.h"

namespace control {

namespace walking {

using namespace representations;

class InverseKinematics {
private:
    double upperLegLength;
    double lowerLegLength;
    double hipPitch0;
    double kneePitch0;
    double anklePitch0;

    RobotPhysicalParameters physicalParameters;

public:
    InverseKinematics(RobotPhysicalParameters physicalParameters);

    virtual ~InverseKinematics();

    void computeJointsTargetsOneLeg(double x, double y, double z, double theta,
                                    double *hipPitch, double *kneePitch, double *anklePitch,
                                    double *hipRoll, double *ankleRoll, double *hipYaw);

    void computeJointsTargets(double xLeft, double yLeft, double zLeft,
                              double thetaLeft, double xRight, double yRight, double zRight,
                              double thetaRight, Joints &jointsTargets, float &Xscale_);
};

} /* namespace walking */

} /* namespace control */

#endif /* INVERSEKINEMATICS_H_ */
