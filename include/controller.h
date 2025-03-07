#ifndef CONTROLLER_NODE_H
#define CONTROLLER_NODE_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <unistd.h>
#include "Joints.h"
#include "RobotPhysicalParameters.h"
#include "OmnidirectionalWalkZMP.h"
#include "InverseKinematics.h"
#include <thread>
#include <std_srvs/Trigger.h>

namespace control {
namespace walking {

class ControllerNode {
public:
    ControllerNode(Joints &joints);

    virtual ~ControllerNode();

    void publishJoints(Joints &joints, float scalar);

    void publishJointsReal(Joints &joints, float scalar);

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

    bool get_angle_x_axis_available_ = true;
    float dt;
    float Xscale_ = 1.0;
    ros::Timer timer_;
    ros::ServiceClient client_;
    std::thread functionThread_;

    // Nó ROS
    ros::NodeHandle nh_;

    void timerCallback(const ros::TimerEvent &event);
    void Update(Joints &joints);
    float normalize(float x);
};

}  // namespace walking
}  // namespace control

#endif  // CONTROLLER_NODE_H
