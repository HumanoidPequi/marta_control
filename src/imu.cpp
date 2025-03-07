#include <vector>
#include "imu.h"

IMUFilterNode::IMUFilterNode(ros::NodeHandle& nh) : nh_(nh) {
    imu_sub_ = nh_.subscribe("/marta/imu", 400, &IMUFilterNode::imuCallback, this);
    dt = (float)1/400;
    timer_ = nh_.createTimer(ros::Duration(dt), &IMUFilterNode::timerCallback, this);
    state_vector_ = Eigen::VectorXd::Zero(9); // [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, ang_x, ang_y, ang_z]
    P_ = Eigen::MatrixXd::Identity(9, 9);

    service_ = nh.advertiseService("get_angle_x_axis", &IMUFilterNode::getAngle, this);
}

void IMUFilterNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    IMUData imu_data;
    
    imu_data.timestamp = msg->header.stamp;
    imu_data.angular_velocity << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    imu_data.linear_acceleration << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

    last_imu_data_ = imu_data;

    // Removing gravity --------------------------------------- It's not working yet
    imu_data.linear_acceleration[2] -= gravity_;
    imu_data_buffer_.push_back(imu_data);

}

void IMUFilterNode::timerCallback(const ros::TimerEvent&) {
    if (imu_data_buffer_.empty()) {
        //ROS_WARN("Nenhum dado do IMU disponivel no buffer.");
        return;
    }

    IMUData imu_data = imu_data_buffer_.front();
    imu_data_buffer_.erase(imu_data_buffer_.begin());

    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(9, 9);
    A(0, 3) = dt; A(1, 4) = dt; A(2, 5) = dt;
    
    Eigen::VectorXd B(9);

    B(0) = 0.5 * imu_data.linear_acceleration[0] * dt * dt;
    B(1) = 0.5 * imu_data.linear_acceleration[1] * dt * dt;
    B(2) = 0.5 * imu_data.linear_acceleration[2] * dt * dt;
    B(3) = imu_data.linear_acceleration[0] * dt;
    B(4) = imu_data.linear_acceleration[1] * dt;
    B(5) = imu_data.linear_acceleration[2] * dt;
    B(6) = imu_data.angular_velocity[0] * dt;
    B(7) = imu_data.angular_velocity[1] * dt;
    B(8) = imu_data.angular_velocity[2] * dt;
    
    state_vector_ = A * state_vector_ + B;

    Eigen::MatrixXd Q = 0.01 * Eigen::MatrixXd::Identity(9, 9);
    P_ = A * P_ * A.transpose() + Q;
    
    //ROS_INFO_STREAM("Estado atualizado: " << state_vector_.transpose());
}

bool IMUFilterNode::getAngle(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    float angle = getAngle2XAxis();

    if (angle == -1) {
        res.success = false;
        res.message = "IMU is not ready yet.";
    } else {
        res.success = true;
        res.message = std::to_string(angle);
    }
    
    return true;
}

float IMUFilterNode::getAngle2XAxis(){
    if((last_imu_data_.timestamp.isZero())){
        return -1;
    }

    float ax = last_imu_data_.linear_acceleration[0];
    float ay = last_imu_data_.linear_acceleration[1];
    float az = last_imu_data_.linear_acceleration[2];

    if (az == 0) {  // Evita divisão por zero
        return -1;
    }

    float angle = std::atan2(-ax, sqrt(ay * ay + az * az));

    return angle * 180 / M_PI;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_filter_node");
    ros::NodeHandle nh;
    IMUFilterNode imu_filter(nh);
    ros::spin();
    return 0;
}
