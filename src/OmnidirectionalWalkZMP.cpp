/*
 * OmnidirectionalWalkZMP.cpp
 *
 *  Created on: Sep 14, 2014
 *      Author: mmaximo
 */

#include "OmnidirectionalWalkZMP.h"

#include <stdio.h>
#include <math.h>
#include <cfloat>
#include <Vector3.h>
#include <fstream>

#include <MathUtils.h>

namespace control {

namespace walking {

using itandroids_lib::math::MathUtils;

OmnidirectionalWalkZMPParams::OmnidirectionalWalkZMPParams() :
        period(0.0), zCom(0.0), zStep(0.0), doubleSupportRatio(0.0), ySeparation(
        0.0), armsCompensationFactor(0.0), comSwingFactor(0.0) {
    
}

OmnidirectionalWalkZMPParams OmnidirectionalWalkZMPParams::getDefaultOmnidirecionalWalkZMPParams() {
    OmnidirectionalWalkZMPParams params;
    params.period = 0.8;
    params.doubleSupportRatio = 0.4;          // Proporção de tempo em que ambos os pés estão no chão
    params.ySeparation = 0.08;                // Separação lateral entre os pés.
    params.zCom = 0.297509;                   // Altura do centro de massa (CoM - Center of Mass)
    params.zStep = 0.04;                      // Altura do movimento oscilatório do pé durante o passo
    params.armsCompensationFactor = 1.0;      // Fator de compensação do movimento dos braços para equilibrar o movimento do torso
    params.comSwingFactor = 1.0;              // Fator que ajusta a amplitude do movimento do CoM
    return params;
}

OmnidirectionalWalkZMP::OmnidirectionalWalkZMP(
        OmnidirectionalWalkZMPParams walkParams,
        RobotPhysicalParameters physicalParams,
        InverseKinematics *inverseKinematics) :
        GRAVITY(9.80665), walkParams(walkParams), physicalParams(
        physicalParams), inverseKinematics(inverseKinematics), walkTime(
        0.0) {
    tZMP = sqrt(walkParams.zCom * walkParams.comSwingFactor / GRAVITY);
    restart();

    //appendToCSV(j_filename, "q1,q2,q3,q4");
}

OmnidirectionalWalkZMP::~OmnidirectionalWalkZMP() {
}

// TODO: make the walking works with Pose2D instead of Vector3
void OmnidirectionalWalkZMP::setDesiredVelocity(itandroids_lib::math::Pose2D velocity) {
    this->velocity.x = velocity.translation.x;
    this->velocity.y = velocity.translation.y;
    this->velocity.z = velocity.rotation;
}

void OmnidirectionalWalkZMP::restart() {
    walkTime = 0.0;
    leftFootIsSwing = false;
    torsoBegin.x = 0.0;
    torsoBegin.y = -walkParams.ySeparation;
    torsoBegin.z = 0.297509;
    torsoEnd.x = 0.0;
    torsoEnd.y = -walkParams.ySeparation;
    torsoEnd.z = 0.0;
    swingFootBegin.x = 0.0;
    swingFootBegin.y = torsoBegin.y - walkParams.ySeparation;
    swingFootBegin.z = 0.0;
    swingFootEnd.x = 0.0;
    swingFootEnd.y = torsoEnd.y - walkParams.ySeparation;
    swingFootEnd.z = 0.0;
    odometry.x = 0.0;
    odometry.y = 0.0;
    odometry.z = 0.0;
    previousTorso = torsoBegin;

    stoppingState = NONE;
}

void OmnidirectionalWalkZMP::update(double elapsedTime, Joints &joints, float &Xscale_) {
    walkTime += elapsedTime;

    if (stoppingState == STOPPED)
        walkTime = 0.0;

    if (walkTime >= walkParams.period) {
        walkTime -= walkParams.period;
        computeNewStep();
    }

    double t = walkTime / walkParams.period;

    double aPx;
    double aNx;
    double aPy;
    double aNy;

    solveZMP(0.0, torsoBegin.x, torsoEnd.x, torsoBegin.x, torsoEnd.x, aPx,
             aNx);
    solveZMP(0.0, torsoBegin.y, torsoEnd.y, torsoBegin.y, torsoEnd.y, aPy,
             aNy);

    double comX = getCoM(t, aPx, aNx, 0.0, torsoBegin.x, torsoEnd.x);
    double comY = getCoM(t, aPy, aNy, 0.0, torsoBegin.y, torsoEnd.y);

    double xFrac = computeXFrac(t);
    double yFrac = computeYFrac(t);
    double zFrac = computeZFrac(t);
    double thetaFrac = computeThetaFrac(t);

    double thetaTorso = torsoBegin.z + (torsoEnd.z - torsoBegin.z) * thetaFrac;

    torso.x = comX;
    torso.y = comY;
    torso.z = thetaTorso;

    odometry = poseRelative(torso, previousTorso);

    previousTorso = torso;

    //std::cout << "torso: " << torso.x << " " << torso.y << " " << torso.z << "\n";
    //itandroids_lib::math::Vector3<double> torso(comX, comY, thetaTorso);

    double xSwingGlobal = swingFootBegin.x
                          + (swingFootEnd.x - swingFootBegin.x) * xFrac;
    double ySwingGlobal = swingFootBegin.y
                          + (swingFootEnd.y - swingFootBegin.y) * yFrac;
    double thetaSwingGlobal = swingFootBegin.z
                              + (swingFootEnd.z - swingFootBegin.z) * thetaFrac;

    itandroids_lib::math::Vector3<double> swingFoot(xSwingGlobal, ySwingGlobal,
                                                    thetaSwingGlobal);

    swingFoot = poseRelative(swingFoot, torso);

    itandroids_lib::math::Vector3<double> supportFoot = poseRelative(itandroids_lib::math::Vector3<double>(), torso);

    double zSupport = walkParams.zCom;
    double zSwing = walkParams.zCom - zFrac * walkParams.zStep;

    itandroids_lib::math::Vector3<double> leftFoot;
    itandroids_lib::math::Vector3<double> rightFoot;
    double zLeft;
    double zRight;
    if (leftFootIsSwing) {
        leftFoot = swingFoot;
        rightFoot = supportFoot;
        zLeft = zSwing;
        zRight = zSupport;
    } else {
        leftFoot = supportFoot;
        rightFoot = swingFoot;
        zLeft = zSupport;
        zRight = zSwing;
    }

    double xLeftArm = rightFoot.x * walkParams.armsCompensationFactor;
    double xRightArm = leftFoot.x * walkParams.armsCompensationFactor;

    joints.leftShoulderPitch = - asin(xLeftArm / physicalParams.armLength);
    joints.rightShoulderPitch = - asin(xRightArm / physicalParams.armLength);
    joints.leftShoulderRoll = MathUtils::degreesToRadians(-50.0);
    joints.rightShoulderRoll = MathUtils::degreesToRadians(50.0);


	//appendToCSV(j_filename, std::to_string(joints.leftShoulderPitch) + "," + std::to_string(joints.rightShoulderPitch) + "," + std::to_string(joints.leftShoulderRoll) + "," + std::to_string(joints.rightShoulderRoll));


    inverseKinematics->computeJointsTargets(leftFoot.x, leftFoot.y, zLeft,
                                            leftFoot.z, rightFoot.x, rightFoot.y, zRight, rightFoot.z, joints, Xscale_);
    joints.leftHipPitch -= 5.0 * M_PI / 180.0;
    joints.rightHipPitch -= 5.0 * M_PI / 180.0;

    leftFoot_ = leftFoot;
    rightFoot_ = rightFoot;
    zLeft_ = zLeft;
    zRight_ = zRight;
}

void OmnidirectionalWalkZMP::appendToCSV(const std::string& filename, const std::string& data) {
    std::ofstream file;
    file.open(filename, std::ios::app); // Abre em modo append (adiciona linhas)
    if (file.is_open()) {
        file << data << "\n";
        file.close();
    } else {
        std::cerr << "Erro ao abrir arquivo: " << filename << std::endl;
    }
}

void OmnidirectionalWalkZMP::computeNewStep() {
    if (stoppingState == STOP_REQUESTED)
        stoppingState = ADJUSTING_1ST;
    else if (stoppingState == ADJUSTING_1ST)
        stoppingState = ADJUSTING_2ND;
    else if (stoppingState == ADJUSTING_2ND)
        stoppingState = STOPPED;

    //printf("stoppingState: %d", stoppingState);

    if (stoppingState == ADJUSTING_1ST || stoppingState == ADJUSTING_2ND) {
        velocity.x = 0.0;
        velocity.y = 0.0;
        velocity.z = 0.0;
    }

    switchSwingFoot();

    torsoBegin = poseRelative(torsoEnd, swingFootEnd);
    itandroids_lib::math::Vector3<double> supportFoot(0.0, 0.0, 0.0);
    swingFootBegin = poseRelative(supportFoot, swingFootEnd);
    previousTorso = poseRelative(previousTorso, swingFootEnd); // for odometry
    itandroids_lib::math::Vector3<double> torsoDisplacement = getDesiredTorsoDisplacement(
            torsoBegin);
    torsoEnd = limitTorso(torsoBegin, torsoDisplacement);

    itandroids_lib::math::Vector3<double> torsoTwoSteps = torsoEnd
                                                          + getDesiredTorsoDisplacement(torsoEnd);

    bool oY = ((torsoDisplacement.y >= 0 && leftFootIsSwing)
               || (torsoDisplacement.y < 0 && !leftFootIsSwing));
    bool oPsi = ((torsoDisplacement.z >= 0 && leftFootIsSwing)
                 || (torsoDisplacement.z < 0 && !leftFootIsSwing));

    itandroids_lib::math::Vector3<double> p;

    if (oPsi) {
        p = torsoTwoSteps;
    } else {
        p = torsoEnd;
    }

    swingFootEnd.z = 0.0;

    itandroids_lib::math::Vector3<double> torsoEndRelativeP = poseRelative(torsoEnd, p);
    itandroids_lib::math::Vector3<double> torsoTwoStepsRelativeP = poseRelative(
            torsoTwoSteps, p);

    double swingFootDisplacementY;
    if (leftFootIsSwing) {
        swingFootDisplacementY = walkParams.ySeparation;
    } else {
        swingFootDisplacementY = -walkParams.ySeparation;
    }

    if (oY) {
        swingFootEnd.y = torsoTwoStepsRelativeP.y + swingFootDisplacementY;
    } else {
        swingFootEnd.y = torsoEndRelativeP.y + swingFootDisplacementY;
    }

    double dx;
    double a = 2.0 * (velocity.x / velocity.z)
               * sin(velocity.z * walkParams.period / 2.0);
    double b = 2.0 * (velocity.y / velocity.z)
               * sin(velocity.z * walkParams.period / 2.0);
    double cossine = cos(velocity.z * walkParams.period / 2.0);
    double sine = sin(velocity.z * walkParams.period / 2.0);

    if (oPsi) {
        dx = a * cossine + b * sine;
    } else {
        dx = a * cossine - b * sine;
    }

    if (velocity.z < DBL_EPSILON) {
        dx = walkParams.period * velocity.x;
    }

    swingFootEnd.x = torsoEndRelativeP.x + dx / 2.0;

    swingFootEnd = poseGlobal(swingFootEnd, p);

}

itandroids_lib::math::Vector3<double> OmnidirectionalWalkZMP::limitTorso(
        itandroids_lib::math::Vector3<double> beginTorso,
        itandroids_lib::math::Vector3<double> torsoDisplacement) {
    itandroids_lib::math::Vector3<double> torsoEnd = torsoBegin + torsoDisplacement;

    if (leftFootIsSwing) {
        torsoEnd.y = std::max(walkParams.ySeparation, torsoEnd.y);
        torsoEnd.z = std::max(0.0, torsoEnd.z);
    } else {
        torsoEnd.y = std::min(-walkParams.ySeparation, torsoEnd.y);
        torsoEnd.z = std::min(0.0, torsoEnd.z);
    }

    return torsoEnd;
}

void OmnidirectionalWalkZMP::switchSwingFoot() {
    leftFootIsSwing = !leftFootIsSwing;
}

itandroids_lib::math::Vector3<double> OmnidirectionalWalkZMP::getDesiredTorsoDisplacement(
        itandroids_lib::math::Vector3<double> torsoBegin) {
    itandroids_lib::math::Vector3<double> displacement;
    if (velocity.z < DBL_EPSILON) {
        displacement = velocity * walkParams.period;
    } else {
        double a = 2.0 * (velocity.x / velocity.z)
                   * sin(velocity.z * walkParams.period / 2.0);
        double b = 2.0 * (velocity.y / velocity.z)
                   * sin(velocity.z * walkParams.period / 2.0);
        double cossine = cos(torsoBegin.z + velocity.z * walkParams.period / 2.0);
        double sine = sin(torsoBegin.z + velocity.z * walkParams.period / 2.0);
        displacement.x = a * cossine - b * sine;
        displacement.y = a * sine + b * cossine;
        displacement.z = walkParams.period * velocity.z;
    }
    return displacement;
}

void OmnidirectionalWalkZMP::solveZMP(double zs, double z1, double z2, double x1,
                                      double x2, double &aP, double &aN) {
    double T1 = walkParams.period * walkParams.doubleSupportRatio / 2.0;
    double T2 = walkParams.period - T1;

    double m1 = (zs - z1) / T1;
    double m2 = -(zs - z2) / (walkParams.period - T2);

    double c1 = x1 - z1 + tZMP * m1 * sinh(-T1 / tZMP);
    double c2 = x2 - z2 + tZMP * m2 * sinh((walkParams.period - T2) / tZMP);
    double expTStep = exp(walkParams.period / tZMP);
    aP = (c2 - c1 / expTStep) / (expTStep - 1 / expTStep);
    aN = (c1 * expTStep - c2) / (expTStep - 1 / expTStep);
}

// TODO: Organize this code
double OmnidirectionalWalkZMP::getCoM(double t, double aP, double aN, double zs,
                                      double z1, double z2) {
    double t1 = walkParams.doubleSupportRatio / 2.0;
    double t2 = 1.0 - t1;

    double expT = exp(walkParams.period * t / tZMP);
    double com = aP * expT + aN / expT;

    double m1 = (zs - z1) / (t1 * walkParams.period);
    double m2 = -(zs - z2) / (walkParams.period - (t2 * walkParams.period));

    if (t < t1) {
        com += z1 + m1 * walkParams.period * t// - t1)
               - tZMP * m1 * sinh(walkParams.period * (t - t1) / tZMP);
    } else if (t < t2) {
        com += zs;
    } else {
        com += zs + m2 * walkParams.period * (t - t2)
                - tZMP * m2 * sinh(walkParams.period * (t - t2) / tZMP);
    }

    return com;
}

itandroids_lib::math::Vector3<double> OmnidirectionalWalkZMP::poseGlobal(
        itandroids_lib::math::Vector3<double> poseRelative,
        itandroids_lib::math::Vector3<double> pose) {
    double ca = cos(pose.z);
    double sa = sin(pose.z);
    double x = pose.x + ca * poseRelative.x - sa * poseRelative.y;
    double y = pose.y + sa * poseRelative.x + ca * poseRelative.y;
    double z = pose.z + poseRelative.z;

    return itandroids_lib::math::Vector3<double>(x, y, MathUtils::normalizeAngle(z));
}

itandroids_lib::math::Vector3<double> OmnidirectionalWalkZMP::poseRelative(
        itandroids_lib::math::Vector3<double> poseGlobal, itandroids_lib::math::Vector3<double> pose) {
    double ca = cos(pose.z);
    double sa = sin(pose.z);
    double px = poseGlobal.x - pose.x;
    double py = poseGlobal.y - pose.y;
    double pa = poseGlobal.z - pose.z;
    return itandroids_lib::math::Vector3<double>(ca * px + sa * py, -sa * px + ca * py,
                                                 MathUtils::normalizeAngle(pa));
}

double OmnidirectionalWalkZMP::computeXFrac(double t) {
    double t1 = walkParams.doubleSupportRatio / 2.0;
    double t2 = 1.0 - t1;

    if (t < t1)
        return 0.0;
    if (t < t2) {
        return 0.5f * (1.0 - cos(M_PI * (t - t1) / (t2 - t1)));
    }
    return 1.0;
}

double OmnidirectionalWalkZMP::computeYFrac(double t) {
    double t1 = walkParams.doubleSupportRatio / 2.0;
    double t2 = 1.0 - t1;

    if (t < t1)
        return 0.0;
    if (t < t2) {
        return 0.5f * (1.0 - cos(M_PI * (t - t1) / (t2 - t1)));
    }
    return 1.0;
}

double OmnidirectionalWalkZMP::computeZFrac(double t) {
    double t1 = walkParams.doubleSupportRatio / 2.0;
    double t2 = 1.0 - t1;

    if (t > t1 && t < t2)
        return 0.5f * (1.0 - cos(2.0 * M_PI * (t - t1) / (t2 - t1)));
    return 0.0;
}

double OmnidirectionalWalkZMP::computeThetaFrac(double t) {
    double t1 = walkParams.doubleSupportRatio / 2.0;
    double t2 = 1.0 - t1;

    if (t < t1)
        return 0.0;
    if (t < t2) {
        return 0.5f * (1.0 - cos(M_PI * (t - t1) / (t2 - t1)));
    }
    return 1.0;
}

itandroids_lib::math::Vector3<double> OmnidirectionalWalkZMP::getOdometry() {
    return odometry;
}

void OmnidirectionalWalkZMP::requestStop() {
    if (stoppingState == NONE)
        stoppingState = STOP_REQUESTED;
}

bool OmnidirectionalWalkZMP::hasStopped() {
    return (stoppingState == STOPPED);
}

} /* namespace walking */

} /* namespace control */

