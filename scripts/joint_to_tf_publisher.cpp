#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <vector>

class JointToTFPublisher {
private:
    ros::NodeHandle nh_;
    tf::TransformBroadcaster tf_broadcaster_;

    ros::Subscriber right_leg_sub_;
    ros::Subscriber left_leg_sub_;
    ros::Subscriber arm_r_sub_;
    ros::Subscriber arm_l_head_sub_;

    // Lista de frames para as juntas (esquerda, direita e cabeça)
    std::vector<std::string> left_leg_frames_ = {
        "l_waist_1", "l_thigh_1", "l_shin_1", "l_ankle_1", "l_feet_1"};
    std::vector<std::string> right_leg_frames_ = {
        "r_waist_1", "r_thigh_1", "r_shin_1", "r_ankle_1", "r_feet_1"};
    std::vector<std::string> left_arm_frames_ = {
        "l_shoulder_1", "l_arm_1"};
    std::vector<std::string> right_arm_frames_ = {
        "r_shoulder_1", "r_arm_1"};
    std::vector<std::string> head_frames_ = {
        "neck_1", "head_1"};

public:
    JointToTFPublisher() {
        // Inicializa os subscribers para os tópicos das juntas
        right_leg_sub_ = nh_.subscribe("/marta/right_leg/state", 10, &JointToTFPublisher::rightLegCallback, this);
        left_leg_sub_ = nh_.subscribe("/marta/left_leg/state", 10, &JointToTFPublisher::leftLegCallback, this);
        arm_r_sub_ = nh_.subscribe("/marta/arm_r/state", 10, &JointToTFPublisher::rightArmCallback, this);
        arm_l_head_sub_ = nh_.subscribe("/marta/arm_l_head/state", 10, &JointToTFPublisher::leftArmHeadCallback, this);
    }

    void publishTF(const std::string &parent_frame, const std::string &child_frame, double angle, double x, double y, double z) {
        tf::Transform transform;
        tf::Quaternion q;

        // Converte o ângulo para quaternion (rotação no eixo Z)
        q.setRPY(0, 0, angle);
        transform.setRotation(q);

        // Configura a posição
        transform.setOrigin(tf::Vector3(x, y, z));

        // Publica a transformação
        tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame, child_frame));
    }

    void processJointData(const std_msgs::Float64MultiArray &msg, const std::vector<std::string> &frames) {
        if (msg.data.size() != frames.size() - 1) {
            ROS_WARN("Mismatched joint data size and frame count for frames starting with %s", frames[0].c_str());
            return;
        }

        double x = 0.0, y = 0.0, z = 0.1; // Deslocamento inicial (ajustar conforme necessário)

        for (size_t i = 0; i < msg.data.size(); ++i) {
            publishTF(frames[i], frames[i + 1], msg.data[i], x, y, z);

            // Ajuste o deslocamento z entre as juntas (modifique se necessário)
            z += 0.1;
        }
    }

    void rightLegCallback(const std_msgs::Float64MultiArray &msg) {
        processJointData(msg, right_leg_frames_);
    }

    void leftLegCallback(const std_msgs::Float64MultiArray &msg) {
        processJointData(msg, left_leg_frames_);
    }

    void rightArmCallback(const std_msgs::Float64MultiArray &msg) {
        processJointData(msg, right_arm_frames_);
    }

    void leftArmHeadCallback(const std_msgs::Float64MultiArray &msg) {
        processJointData(msg, left_arm_frames_);
        processJointData(msg, head_frames_); // Para o pescoço e cabeça
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "joint_to_tf_publisher");

    JointToTFPublisher joint_to_tf_publisher;

    ros::spin();
    return 0;
}
