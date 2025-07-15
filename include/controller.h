#ifndef CONTROLLER_NODE_H
#define CONTROLLER_NODE_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <unistd.h>
#include <deque>
#include <numeric>
#include "Joints.h"
#include "RobotPhysicalParameters.h"
#include "OmnidirectionalWalkZMP.h"
#include "InverseKinematics.h"
#include <thread>
#include <std_srvs/Trigger.h>
#include <marta_msgs/Levantar.h>
#include <sensor_msgs/Imu.h>

namespace control {
namespace walking {

class ControllerNode {
public:
    ControllerNode(Joints &joints);

    virtual ~ControllerNode();

    void publishJoints(Joints &joints, float scalar);

    void Agacha(Joints &joints);



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

    ros::Subscriber imu_sub_;

    std::deque<double> imuXAccBuffer_;
    bool get_angle_x_axis_available_ = true;
    bool isReal_ = false;
    float dt;
    float Xscale_ = 1.0;
    float imuXAcc_ = 0.0;
    ros::Timer timer_;
    ros::ServiceClient clientAngles_;
    ros::ServiceClient client_frente_;
    ros::ServiceClient client_tras_;
    std::thread functionThread_;

    // Nó ROS
    ros::NodeHandle nh_;

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void timerCallback(const ros::TimerEvent &event);
    void Update(Joints &joints);
    float normalize(float x);
};

}  // namespace walking
}  // namespace control

#endif  // CONTROLLER_NODE_H
