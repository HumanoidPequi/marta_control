#ifndef KICK_NODE_H
#define KICK_NODE_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <unistd.h>
#include <cmath>
#include <vector>
#include "Joints.h"
#include "RobotPhysicalParameters.h"
#include "OmnidirectionalWalkZMP.h"
#include "InverseKinematics.h"
#include "Vector3.h"
#include "Pose2D.h"

namespace control {
namespace walking {

class KickNode {
public:
    KickNode(Joints &joints);

    virtual ~KickNode();

    void publishJoints(Joints &joints, float scalar);

    void publishJointsReal(Joints &joints, float scalar);

    itandroids_lib::math::Vector3<double> leftFoot_;
    itandroids_lib::math::Vector3<double> rightFoot_;
    double zLeft_;
    double zRight_;

private:
    ros::Publisher pubPescocoYaw_;
    ros::Publisher pubCabecaPitch_;
    ros::Publisher pubPeEsquerdoPitch_;
    ros::Publisher pubPeEsquerdoRoll_;
    ros::Publisher pubQuadrilEsquerdoPitch_;
    ros::Publisher pubQuadrilEsquerdoRoll_;
    ros::Publisher pubQuadrilEsquerdoYaw_;
    ros::Publisher pubJoelhoEsquerdoPitch_;
    ros::Publisher pubPeDireitoPitch_;
    ros::Publisher pubPeDireitoRoll_;
    ros::Publisher pubQuadrilDireitoPitch_;
    ros::Publisher pubQuadrilDireitoRoll_;
    ros::Publisher pubQuadrilDireitoYaw_;
    ros::Publisher pubJoelhoDireitoPitch_;
    ros::Publisher pubCotoveloEsquerdoPitch_;
    ros::Publisher pubOmbroEsquerdoPitch_;
    ros::Publisher pubOmbroEsquerdoRoll_;
    ros::Publisher pubCotoveloDireitoPitch_;
    ros::Publisher pubOmbroDireitoPitch_;
    ros::Publisher pubOmbroDireitoRoll_;

    ros::Publisher pub_right_leg;
    ros::Publisher pub_left_leg;
    ros::Publisher pub_right_arm;
    ros::Publisher pub_left_arm_head;

    // Nó ROS
    ros::NodeHandle nh_;
};

}  // namespace walking
}  // namespace control

#endif  // Kick_NODE_H
