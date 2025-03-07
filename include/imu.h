#ifndef IMU_H
#define IMU_H
#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include <std_srvs/Trigger.h>

struct IMUData {
    ros::Time timestamp;
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d linear_acceleration;

    IMUData() : timestamp(0, 0), angular_velocity(Eigen::Vector3d::Zero()), linear_acceleration(Eigen::Vector3d::Zero()) {}
};

class IMUFilterNode {
public:
    IMUFilterNode(ros::NodeHandle& nh);
    std::vector<IMUData> imu_data_buffer_;

private:
    bool getAngle(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    float getAngle2XAxis();

    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Timer timer_;
    Eigen::VectorXd state_vector_;
    Eigen::MatrixXd P_;
    float dt;
    IMUData last_imu_data_;
    double gravity_ = 9.80665;
    ros::ServiceServer service_;

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void timerCallback(const ros::TimerEvent&);
};

#endif // IMU_H
