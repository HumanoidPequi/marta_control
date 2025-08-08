#include <iostream>
#include "controller.h"

using namespace control::walking;
using itandroids_lib::math::Pose2D;

ControllerNode::ControllerNode(Joints &joints) : nh_("~") {
    joints.StandardNames();
    pubPescocoYaw_ = nh_.advertise<std_msgs::Float64>(joints.PescocoYawTopic, 10);
    pubCabecaPitch_ = nh_.advertise<std_msgs::Float64>(joints.CabecaPitchTopic, 10);

    pubPeEsquerdoPitch_ = nh_.advertise<std_msgs::Float64>(joints.PeEsquerdoPitchTopic, 10);
    pubPeEsquerdoRoll_ = nh_.advertise<std_msgs::Float64>(joints.PeEsquerdoRollTopic, 10);
    pubQuadrilEsquerdoPitch_ = nh_.advertise<std_msgs::Float64>(joints.QuadrilEsquerdoPitchTopic, 10);
    pubQuadrilEsquerdoRoll_ = nh_.advertise<std_msgs::Float64>(joints.QuadrilEsquerdoRollTopic, 10);
    pubQuadrilEsquerdoYaw_ = nh_.advertise<std_msgs::Float64>(joints.QuadrilEsquerdoYawTopic, 10);
    pubJoelhoEsquerdoPitch_ = nh_.advertise<std_msgs::Float64>(joints.JoelhoEsquerdoPitchTopic, 10);

    pubPeDireitoPitch_ = nh_.advertise<std_msgs::Float64>(joints.PeDireitoPitchTopic, 10);
    pubPeDireitoRoll_ = nh_.advertise<std_msgs::Float64>(joints.PeDireitoRollTopic, 10);
    pubQuadrilDireitoPitch_ = nh_.advertise<std_msgs::Float64>(joints.QuadrilDireitoPitchTopic, 10);
    pubQuadrilDireitoRoll_ = nh_.advertise<std_msgs::Float64>(joints.QuadrilDireitoRollTopic, 10);
    pubQuadrilDireitoYaw_ = nh_.advertise<std_msgs::Float64>(joints.QuadrilDireitoYawTopic, 10);
    pubJoelhoDireitoPitch_ = nh_.advertise<std_msgs::Float64>(joints.JoelhoDireitoPitchTopic, 10);

    pubCotoveloEsquerdoPitch_ = nh_.advertise<std_msgs::Float64>(joints.CotoveloEsquerdoPitchTopic, 10);
    pubOmbroEsquerdoPitch_ = nh_.advertise<std_msgs::Float64>(joints.OmbroEsquerdoPitchTopic, 10);
    pubOmbroEsquerdoRoll_ = nh_.advertise<std_msgs::Float64>(joints.OmbroEsquerdoRollTopic, 10);

    pubCotoveloDireitoPitch_ = nh_.advertise<std_msgs::Float64>(joints.CotoveloDireitoPitchTopic, 10);
    pubOmbroDireitoPitch_ = nh_.advertise<std_msgs::Float64>(joints.OmbroDireitoPitchTopic, 10);
    pubOmbroDireitoRoll_ = nh_.advertise<std_msgs::Float64>(joints.OmbroDireitoRollTopic, 10);

    pub_right_leg = nh_.advertise<std_msgs::Float64MultiArray>(joints.right_leg, 10);
    pub_left_leg = nh_.advertise<std_msgs::Float64MultiArray>(joints.left_leg, 10);
    pub_right_arm = nh_.advertise<std_msgs::Float64MultiArray>(joints.arm_r, 10);
    pub_left_arm_head = nh_.advertise<std_msgs::Float64MultiArray>(joints.arm_l_head, 10);

    hasBallTimer_ = ros::Time::now();

    // Service client to get the angle of the x axis
    clientAngles_ = nh_.serviceClient<std_srvs::Trigger>("/get_angle_x_axis");
    ros::Duration timeout(5.0);
    if (!clientAngles_.waitForExistence(timeout)) {
        ROS_ERROR("The service 'get_angle_x_axis' is not available!");
        get_angle_x_axis_available_ = false;
    }

    if(get_angle_x_axis_available_){
        dt = (float)1/200;
        timer_ = nh_.createTimer(ros::Duration(dt), &ControllerNode::timerCallback, this);
    }

    client_frente_ = nh_.serviceClient<marta_msgs::Levantar>("/levantar_para_frente");
    if (!client_frente_.waitForExistence(timeout)) {
        ROS_ERROR("The service 'levantar_para_frente' is not available!");
    }

    client_tras_ = nh_.serviceClient<marta_msgs::Levantar>("/levantar_para_tras");
    if (!client_tras_.waitForExistence(timeout)) {
        ROS_ERROR("The service 'levantar_para_tras' is not available!");
    }

    functionThread_ = std::thread(std::bind(&ControllerNode::Update, this, joints));
    calculateVelocity_ = std::thread(std::bind(&ControllerNode::calculateVelocity, this));

    // Subscribers
    imu_sub_ = nh_.subscribe("/marta/imu", 400, &ControllerNode::imuCallback, this);
    ball_distance_sub_ = nh_.subscribe("/marta/ball_distance", 10, &ControllerNode::ballDistanceCallback, this);
}

void ControllerNode::ballDistanceCallback(const std_msgs::Float32::ConstPtr& msg) {
    hasBallDistance_ = true;
    hasBallTimer_ = ros::Time::now();
    ROS_INFO("Ball distance: %f", msg->data);
}

void ControllerNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // Adiciona novo valor ao buffer
    imuXAccBuffer_.push_back(msg->linear_acceleration.x);
    
    // Mantém o buffer com tamanho máximo de 20
    if (imuXAccBuffer_.size() > 20) {
        imuXAccBuffer_.pop_front();
    }
    
    // Calcula a média
    imuXAcc_ = std::accumulate(imuXAccBuffer_.begin(), imuXAccBuffer_.end(), 0.0) / imuXAccBuffer_.size();
}

ControllerNode::~ControllerNode() {
    if (functionThread_.joinable()) {
            functionThread_.join();
        }
}

float ControllerNode::normalize(float x) {
    return ((x + 45) / 90.0) * 0.8 + 0.6;
}

void ControllerNode::timerCallback(const ros::TimerEvent &event) {
    std_srvs::Trigger srv;

    if (clientAngles_.call(srv)) {
        if (srv.response.success) {
            float angle = std::stof(srv.response.message);

            if(angle < - 45){
                angle = -45;
            } else if(angle > 45){
                angle = 45;
            }

            Xscale_ = normalize(angle);

        } else {
            ROS_WARN("Fail: %s", srv.response.message.c_str());
        }
    } else {
        ROS_ERROR("Error calling service 'get_angle_x_axis'!");
    }

}

void ControllerNode::publishJoints(Joints &joints, float scalar) {
    if(isReal_) {
        std_msgs::Float64 msg;
        std_msgs::Float64MultiArray msg_array;

        msg_array.data = {0, 0, joints.leftShoulderPitch * scalar, joints.leftShoulderRoll * scalar, 0};
        pub_left_arm_head.publish(msg_array);

        msg_array.data = {joints.rightShoulderPitch * scalar, joints.rightShoulderRoll * scalar, 0};
        pub_right_arm.publish(msg_array);

        msg_array.data = {joints.leftHipYaw * scalar, joints.leftHipRoll * scalar, joints.leftHipPitch* scalar, joints.leftKneePitch* scalar , joints.leftAnklePitch * scalar, joints.leftAnkleRoll * scalar};
        pub_left_leg.publish(msg_array);

        msg_array.data = {joints.rightHipYaw * scalar, joints.rightHipRoll * scalar, joints.rightHipPitch * scalar, joints.rightKneePitch * scalar, joints.rightAnklePitch * scalar, joints.rightAnkleRoll * scalar};
        pub_right_leg.publish(msg_array);
    }else{

        std_msgs::Float64 msg;

        //msg.data = joints.neckPitch * scalar;
        //pubCabecaPitch_.publish(msg);

        //msg.data = joints.neckYaw * scalar;
        //pubPescocoYaw_.publish(msg);

        msg.data = joints.leftShoulderPitch * scalar;
        pubOmbroEsquerdoPitch_.publish(msg);

        msg.data = joints.leftShoulderRoll * scalar;
        pubOmbroEsquerdoRoll_.publish(msg);

        msg.data = joints.rightShoulderPitch * scalar;
        pubOmbroDireitoPitch_.publish(msg);

        msg.data = joints.rightShoulderRoll * scalar;
        pubOmbroDireitoRoll_.publish(msg);

        msg.data = joints.leftHipYaw * scalar;
        pubQuadrilEsquerdoYaw_.publish(msg);

        msg.data = joints.leftHipRoll * scalar;
        pubQuadrilEsquerdoRoll_.publish(msg);

        msg.data = joints.leftHipPitch * scalar;
        pubQuadrilEsquerdoPitch_.publish(msg);

        msg.data = joints.leftKneePitch * scalar;
        pubJoelhoEsquerdoPitch_.publish(msg);

        msg.data = joints.leftAnklePitch * scalar;
        pubPeEsquerdoPitch_.publish(msg);

        msg.data = joints.leftAnkleRoll * scalar;
        pubPeEsquerdoRoll_.publish(msg);

        msg.data = joints.rightHipYaw * scalar;
        pubQuadrilDireitoYaw_.publish(msg);

        msg.data = joints.rightHipRoll * scalar;
        pubQuadrilDireitoRoll_.publish(msg);

        msg.data = joints.rightHipPitch * scalar;
        pubQuadrilDireitoPitch_.publish(msg);

        msg.data = joints.rightKneePitch * scalar;
        pubJoelhoDireitoPitch_.publish(msg);

        msg.data = joints.rightAnklePitch * scalar;
        pubPeDireitoPitch_.publish(msg);

        msg.data = joints.rightAnkleRoll * scalar;
        pubPeDireitoRoll_.publish(msg);
    }
};

void ControllerNode::Agacha(Joints &joints) {
    int N = 100;
    for (int i = 1; i <= N; ++i) {
        publishJoints(joints, (float)i/N);
        usleep(100000);
    }
}

void ControllerNode::Update(Joints &joints){
    // Configuração dos parâmetros físicos do robô
    representations::RobotPhysicalParameters physicalParams = representations::RobotPhysicalParameters::getMartaPhysicalParameters();

    // Configuração dos parâmetros do andar
    OmnidirectionalWalkZMPParams walkParams = OmnidirectionalWalkZMPParams::getDefaultOmnidirecionalWalkZMPParams();


    // Instância de cinemática inversa
    InverseKinematics inverseKinematics(physicalParams);

    // Instância da classe OmnidirectionalWalkZMP
    OmnidirectionalWalkZMP walker(walkParams, physicalParams, &inverseKinematics);

    double elapsedTime = 0.01; // Tempo de iteração
    double elapsedTimeMic = elapsedTime * 1000000; // Tempo em microssegundos

    itandroids_lib::math::Pose2D desiredVelocity;

    // Configure os valores da velocidade
    desiredVelocity.translation.x = translationX_; // Velocidade no eixo x
    desiredVelocity.translation.y = translationY_; // Velocidade no eixo y
    desiredVelocity.rotation = rotationZ_;         // Velocidade de rotação (yaw)

    // Chame a função setDesiredVelocity
    walker.setDesiredVelocity(desiredVelocity);
    
    // Fazendo a Marta agachar lentamente
    walker.update(elapsedTime, joints, Xscale_);
    Agacha(joints);
    
    while (ros::ok()) {

        if(imuXAcc_ > 8){
            ROS_WARN("Marta esta deitada para frente, Tentando levantar");

            marta_msgs::Levantar srv;

            client_frente_.call(srv);

            if(srv.response.success){
                ROS_INFO("%s", srv.response.message.c_str());
            }else{
                ROS_ERROR("%s", srv.response.message.c_str());
            }
            walker.restart();

        }else if(imuXAcc_ < -8){
            ROS_WARN("Marta esta deitada para tras. Tentando levantar");

            marta_msgs::Levantar srv;
            client_tras_.call(srv);

            if(srv.response.success){
                ROS_INFO("%s", srv.response.message.c_str());
            }else{
                ROS_ERROR("%s", srv.response.message.c_str());
            }
            walker.restart();
            
        }else{
            desiredVelocity.translation.x = translationX_; // Velocidade no eixo x
            desiredVelocity.translation.y = translationY_; // Velocidade no eixo y
            desiredVelocity.rotation = rotationZ_;         // Velocidade de rotação (yaw)

            // Chame a função setDesiredVelocity
            walker.setDesiredVelocity(desiredVelocity);
            
            walker.update(elapsedTime, joints, Xscale_);
            publishJoints(joints, 1);
            usleep(elapsedTimeMic);
        }
    }

}

void ControllerNode::calculateVelocity(){
    while (ros::ok()) {
        ROS_INFO("hasBallTimer_: %f", hasBallTimer_.toSec() );
        ROS_INFO("ros::Time::now(): %f", ros::Time::now().toSec() );

        // Diferença de tempo corrido em segundos:
        double dt = (ros::Time::now().toSec() - hasBallTimer_.toSec());
        
        if (dt > 5.0){
            ROS_INFO("Marta nao tem a bola, parando de andar");
            translationX_ = 0.1;
            translationY_ = 0.0;
            rotationZ_ = 0.0;
        }else{
            ROS_INFO("Marta tem a bola, andando");
            translationX_ = 0.1;
            translationY_ = 0.0;
            rotationZ_ = 0.0;
        }

        usleep(100000);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_node");

    Joints joints;
    ControllerNode node(joints);
    ros::spin();
    return 0;
}
