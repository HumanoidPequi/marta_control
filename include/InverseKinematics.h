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
    double hipYawSign = 1.0;   ///< ver setHipYawSign()

    RobotPhysicalParameters physicalParameters;

public:
    InverseKinematics(RobotPhysicalParameters physicalParameters);

    virtual ~InverseKinematics();

    void computeJointsTargetsOneLeg(double x, double y, double z, double theta,
                                    double *hipPitch, double *kneePitch, double *anklePitch,
                                    double *hipRoll, double *ankleRoll, double *hipYaw);

    /**
     * Sinal entre o yaw desejado do pe (em relacao ao torso, + = anti-horario
     * visto de cima) e o angulo da junta hip_yaw. No URDF da simulacao as duas
     * hip_yaw tem eixo -z, entao o sinal correto la e -1: com +1 (o historico)
     * um comando de rotacao para a esquerda faz o robo girar para a DIREITA
     * (medido em 2026-10-07). No robo real depende do sentido dos servos e
     * precisa ser verificado; por isso o default continua +1.
     */
    void setHipYawSign(double sign) { hipYawSign = sign < 0.0 ? -1.0 : 1.0; }

    void computeJointsTargets(double xLeft, double yLeft, double zLeft,
                              double thetaLeft, double xRight, double yRight, double zRight,
                              double thetaRight, Joints &jointsTargets, float &Xscale_);
};

} /* namespace walking */

} /* namespace control */

#endif /* INVERSEKINEMATICS_H_ */
