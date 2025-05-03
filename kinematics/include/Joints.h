/*
 * Joints.h
 *
 *  Created on: 16/02/2014
 *      Author: mmaximo
 */

#ifndef JOINTS_H_
#define JOINTS_H_

#include <iostream>
#include <sstream>
#include <map>
#include <vector>

namespace representations {

struct Joints {
    enum HINGE_JOINTS {
        NECK_PITCH = 0,
        NECK_YAW,
        LEFT_SHOULDER_PITCH,
        LEFT_SHOULDER_YAW,
        LEFT_ELBOW_YAW,
        LEFT_HIP_YAW,
        LEFT_HIP_ROLL,
        LEFT_HIP_PITCH,
        LEFT_KNEE_PITCH,
        LEFT_ANKLE_PITCH,
        LEFT_ANKLE_ROLL,
        RIGHT_SHOULDER_PITCH,
        RIGHT_SHOULDER_YAW,
        RIGHT_ELBOW_YAW,
        RIGHT_HIP_YAW,
        RIGHT_HIP_ROLL,
        RIGHT_HIP_PITCH,
        RIGHT_KNEE_PITCH,
        RIGHT_ANKLE_PITCH,
        RIGHT_ANKLE_ROLL,
        NUM_JOINTS,
        INVALID,
    };

    double neckPitch;
    double neckYaw;

    double leftShoulderPitch;
    double leftShoulderRoll;
    double leftElbowYaw;

    double rightShoulderPitch;
    double rightShoulderRoll;
    double rightElbowYaw;

    double leftHipYaw;
    double leftHipRoll;
    double leftHipPitch;
    double leftKneePitch;
    double leftAnklePitch;
    double leftAnkleRoll;

    double rightHipYaw;
    double rightHipRoll;
    double rightHipPitch;
    double rightKneePitch;
    double rightAnklePitch;
    double rightAnkleRoll;

    std::string PescocoYawTopic;
	std::string CabecaPitchTopic;

	std::string PeEsquerdoPitchTopic;
	std::string PeEsquerdoRollTopic;
	std::string QuadrilEsquerdoPitchTopic;
	std::string QuadrilEsquerdoRollTopic;
	std::string QuadrilEsquerdoYawTopic;
	std::string JoelhoEsquerdoPitchTopic;


	std::string PeDireitoPitchTopic;
	std::string PeDireitoRollTopic;
	std::string QuadrilDireitoPitchTopic;
	std::string QuadrilDireitoRollTopic;
	std::string QuadrilDireitoYawTopic;
	std::string JoelhoDireitoPitchTopic;


	std::string CotoveloEsquerdoPitchTopic;
	std::string OmbroEsquerdoPitchTopic;
	std::string OmbroEsquerdoRollTopic;

	std::string CotoveloDireitoPitchTopic;
	std::string OmbroDireitoPitchTopic;
	std::string OmbroDireitoRollTopic;

    std::string right_leg;
    std::string left_leg;
    std::string arm_r;
    std::string arm_l_head;

    Joints() :
            neckPitch(0.0),
            neckYaw(0.0),

            leftShoulderPitch(0.0),
            leftShoulderRoll(0.0),
            leftElbowYaw(0.0),

            rightShoulderPitch(0.0),
            rightShoulderRoll(0.0),
            rightElbowYaw(0.0),

            leftHipYaw(0.0),
            leftHipRoll(0.0),
            leftHipPitch(0.0),
            leftKneePitch(0.0),
            leftAnklePitch(0.0),
            leftAnkleRoll(0.0),

            rightHipYaw(0.0),
            rightHipRoll(0.0),
            rightHipPitch(0.0),
            rightKneePitch(0.0),
            rightAnklePitch(0.0),
            rightAnkleRoll(0.0) {}

    Joints(std::vector<double> joints);

    void clear();

    void StandardNames();

    Joints operator+(const Joints &other);

    Joints operator-(const Joints &other);

    Joints operator*(const Joints &other);

    void scale(double scalar);

    double lInfiniteNorm();

    static Joints getDarwinDirectionsFixing();

    double getValue(HINGE_JOINTS joint);

    void setValue(HINGE_JOINTS joint, double value);

    static Joints getSymmetricJoints(Joints joints);

    std::map<Joints::HINGE_JOINTS, double> getAsMap();

};

Joints operator*(double scalar, const Joints &jointsTargets);

} /* representations */

#endif /* JOINTS_H_ */
