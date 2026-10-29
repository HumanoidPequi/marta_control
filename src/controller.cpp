#include <iostream>
#include "controller.h"

using namespace control::walking;
using itandroids_lib::math::Pose2D;

ControllerNode::ControllerNode(Joints &joints) :
        nh_("~"),
        stabilizer_(stabilization::TorsoStabilizerParams::getDefaultTorsoStabilizerParams()) {

    // marta_params.yaml e carregado no namespace global (/isReal), mas nh_ e
    // privado (~isReal). Le o global como default e deixa o privado sobrescrever,
    // para que "rosrun ... _isReal:=true" continue funcionando.
    bool isRealGlobal = false;
    ros::param::param<bool>("/isReal", isRealGlobal, false);
    nh_.param<bool>("isReal", isReal_, isRealGlobal);
    ROS_INFO("[Controller] isReal = %s", isReal_ ? "true" : "false");

    nh_.param<bool>("use_xscale_service", use_xscale_service_, false);

    bool walkEnabled = true;
    nh_.param<bool>("walk_enabled", walkEnabled, true);
    walk_enabled_.store(walkEnabled);
    nh_.param<bool>("fall_recovery_enabled", fall_recovery_enabled_, true);
    nh_.param<double>("walk_velocity_x", walk_velocity_x_, 0.1);
    nh_.param<double>("walk_velocity_y", walk_velocity_y_, 0.0);
    nh_.param<double>("walk_velocity_yaw", walk_velocity_yaw_, 0.0);
    ROS_INFO("[Controller] walk_enabled=%s (vx=%.3f vy=%.3f wz=%.3f)",
             walkEnabled ? "true" : "false", walk_velocity_x_,
             walk_velocity_y_, walk_velocity_yaw_);

    stabilizer_.setParams(loadStabilizerParams());

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

    stabilizer_debug_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
            "/marta/stabilizer_debug", 10);
    walk_reference_pub_ = nh_.advertise<marta_msgs::WalkReference>(
            "/marta/walk/reference", 10);

    hasBallTimeSec_.store(ros::Time::now().toSec());

    ros::Duration timeout(5.0);

    // Realimentacao antiga de inclinacao (Xscale_). Desligada por default:
    // ver comentario em controller.h. Quando desligada, Xscale_ fica fixo em
    // 1.0 e a IK usa o offset nominal do quadril.
    if (use_xscale_service_) {
        ROS_WARN("[Controller] use_xscale_service=true: a realimentacao por "
                 "Xscale_ vai concorrer com o estabilizador de pitch.");
        clientAngles_ = nh_.serviceClient<std_srvs::Trigger>("/get_angle_x_axis");
        if (!clientAngles_.waitForExistence(timeout)) {
            ROS_ERROR("The service 'get_angle_x_axis' is not available!");
            get_angle_x_axis_available_ = false;
        }

        if (get_angle_x_axis_available_) {
            dt = (float) 1 / 200;
            timer_ = nh_.createTimer(ros::Duration(dt), &ControllerNode::timerCallback, this);
        }
    } else {
        get_angle_x_axis_available_ = false;
        Xscale_.store(1.0f);
        ROS_INFO("[Controller] Xscale_ fixo em 1.0 (realimentacao antiga de "
                 "inclinacao desativada).");
    }

    if (fall_recovery_enabled_) {
        client_frente_ = nh_.serviceClient<marta_msgs::Levantar>("/levantar_para_frente");
        if (!client_frente_.waitForExistence(timeout)) {
            ROS_ERROR("The service 'levantar_para_frente' is not available!");
        }

        client_tras_ = nh_.serviceClient<marta_msgs::Levantar>("/levantar_para_tras");
        if (!client_tras_.waitForExistence(timeout)) {
            ROS_ERROR("The service 'levantar_para_tras' is not available!");
        }
    } else {
        ROS_INFO("[Controller] deteccao de queda desativada "
                 "(fall_recovery_enabled=false).");
    }

    // Subscribers (criados antes das threads, para que elas ja partam com dados)
    imu_sub_ = nh_.subscribe("/marta/imu", 100, &ControllerNode::imuCallback, this);
    ball_distance_sub_ = nh_.subscribe("/marta/ball_distance", 10, &ControllerNode::ballDistanceCallback, this);
    std::string attitude_topic;
    nh_.param<std::string>("attitude_topic", attitude_topic, "/marta/torso_attitude");
    attitude_sub_ = nh_.subscribe(attitude_topic, 10, &ControllerNode::attitudeCallback, this);

    functionThread_ = std::thread(std::bind(&ControllerNode::Update, this, joints));
    calculateVelocity_ = std::thread(std::bind(&ControllerNode::calculateVelocity, this));
}

void ControllerNode::ballDistanceCallback(const std_msgs::Float32::ConstPtr& msg) {
    hasBallDistance_ = true;
    hasBallTimeSec_.store(ros::Time::now().toSec());
    ROS_INFO_THROTTLE(1.0, "Ball distance: %f", msg->data);
}

void ControllerNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // Aceleração crua no frame do sensor, usada apenas para detecção de queda.
    // A atitude filtrada vem do torso_attitude_node, em attitudeCallback().
    imuXAccBuffer_.push_back(msg->linear_acceleration.x);

    // Mantém o buffer com tamanho máximo de 20
    if (imuXAccBuffer_.size() > 20) {
        imuXAccBuffer_.pop_front();
    }

    // Calcula a média
    imuXAcc_.store(static_cast<float>(
            std::accumulate(imuXAccBuffer_.begin(), imuXAccBuffer_.end(), 0.0)
            / imuXAccBuffer_.size()));
}

/**
 * Recebe a atitude do tronco estimada pelo torso_attitude_node:
 *   data = [pitch, pitch_rate, roll, roll_rate], em rad e rad/s.
 * Convenção: pitch > 0 tombando para frente, roll > 0 tombando para a direita.
 *
 * O pitch e o pitch_rate alimentam o estabilizador na thread Update(); o roll
 * fica armazenado para a Fase 3.
 */
void ControllerNode::attitudeCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
    if (msg->data.size() < 4) {
        ROS_WARN_THROTTLE(5.0, "[Controller] /marta/torso_attitude com %zu "
                          "elementos; esperado 4.", msg->data.size());
        return;
    }

    torsoPitch_.store(static_cast<float>(msg->data[0]));
    torsoPitchRate_.store(static_cast<float>(msg->data[1]));
    torsoRoll_.store(static_cast<float>(msg->data[2]));
    torsoRollRate_.store(static_cast<float>(msg->data[3]));
    hasAttitude_.store(true);

    // yaw e yaw_rate vieram depois, no fim do array.
    if (msg->data.size() >= 6) {
        torsoYaw_.store(static_cast<float>(msg->data[4]));
        hasYaw_.store(true);
    }
}

ControllerNode::~ControllerNode() {
    if (functionThread_.joinable()) {
        functionThread_.join();
    }
    if (calculateVelocity_.joinable()) {
        calculateVelocity_.join();
    }
}

float ControllerNode::normalize(float x) {
    return ((x + 45) / 90.0) * 0.8 + 0.6;
}

control::stabilization::TorsoStabilizerParams ControllerNode::loadStabilizerParams() {
    stabilization::TorsoStabilizerParams params =
            stabilization::TorsoStabilizerParams::getDefaultTorsoStabilizerParams();

    nh_.param<bool>("stabilizer_enabled", params.enabled, params.enabled);
    nh_.param<double>("kp_pitch", params.kpPitch, params.kpPitch);
    nh_.param<double>("kd_pitch", params.kdPitch, params.kdPitch);
    nh_.param<double>("pitch_ref", params.pitchRef, params.pitchRef);
    nh_.param<double>("max_pitch_correction", params.maxPitchCorrection,
                      params.maxPitchCorrection);
    nh_.param<double>("support_blend_time", params.supportBlendTime,
                      params.supportBlendTime);

    ROS_INFO("[Stabilizer] %s  kp_pitch=%.4f  kd_pitch=%.4f  ref=%.4f rad  "
             "saturacao=%.4f rad (%.1f graus)  blend=%.3f s",
             params.enabled ? "ATIVO" : "DESATIVADO",
             params.kpPitch, params.kdPitch, params.pitchRef,
             params.maxPitchCorrection, params.maxPitchCorrection * 180.0 / M_PI,
             params.supportBlendTime);

    return params;
}

void ControllerNode::refreshHeadingParams() {
    const bool wasEnabled = headingEnabled_;
    const double kp = kpHeading_, sat = maxHeadingRate_;
    nh_.getParamCached("heading_enabled", headingEnabled_);
    nh_.getParamCached("kp_heading", kpHeading_);
    nh_.getParamCached("max_heading_rate", maxHeadingRate_);
    if (headingEnabled_ != wasEnabled || kp != kpHeading_ || sat != maxHeadingRate_) {
        ROS_INFO("[Rumo] malha %s  kp=%.3f rad/s/rad  sat=%.3f rad/s",
                 headingEnabled_ ? "ATIVA" : "DESATIVADA", kpHeading_, maxHeadingRate_);
    }
}

double ControllerNode::headingCorrection(double rotationCommand, double dt) {
    if (!headingEnabled_ || !walk_enabled_.load() || !hasYaw_.load()) {
        // Parado ou desligado: a referencia e retomada no proximo inicio.
        headingRefSet_ = false;
        lastHeadingError_ = 0.0;
        lastHeadingCorrection_ = 0.0;
        return 0.0;
    }
    const double yaw = torsoYaw_.load();
    if (!headingRefSet_) {
        headingRef_ = yaw;
        headingRefSet_ = true;
    }
    // Uma rotacao comandada move a referencia junto: a malha corrige o DESVIO
    // em relacao ao rumo pedido, nao impede o robo de curvar.
    headingRef_ += rotationCommand * dt;

    const double error = std::atan2(std::sin(yaw - headingRef_), std::cos(yaw - headingRef_));
    // yaw > referencia = girou para a esquerda -> rotacao negativa (a direita).
    double correction = -kpHeading_ * error;
    correction = std::max(-maxHeadingRate_, std::min(maxHeadingRate_, correction));

    lastHeadingError_ = error;
    lastHeadingCorrection_ = correction;
    return correction;
}

void ControllerNode::refreshStabilizerParams() {
    stabilization::TorsoStabilizerParams params = stabilizer_.getParams();
    const stabilization::TorsoStabilizerParams previous = params;

    // getParamCached evita uma ida ao parameter server a cada consulta: o mestre
    // avisa quando o valor muda. Permite sintonizar com "rosparam set" com o
    // robô em pé, sem reiniciar o nó.
    nh_.getParamCached("stabilizer_enabled", params.enabled);
    nh_.getParamCached("kp_pitch", params.kpPitch);
    nh_.getParamCached("kd_pitch", params.kdPitch);
    nh_.getParamCached("pitch_ref", params.pitchRef);
    nh_.getParamCached("max_pitch_correction", params.maxPitchCorrection);
    nh_.getParamCached("support_blend_time", params.supportBlendTime);

    const bool changed = params.enabled != previous.enabled
            || params.kpPitch != previous.kpPitch
            || params.kdPitch != previous.kdPitch
            || params.pitchRef != previous.pitchRef
            || params.maxPitchCorrection != previous.maxPitchCorrection
            || params.supportBlendTime != previous.supportBlendTime;

    if (changed) {
        stabilizer_.setParams(params);
        ROS_INFO("[Stabilizer] ganhos atualizados: %s  kp=%.4f  kd=%.4f  "
                 "ref=%.4f  sat=%.4f rad",
                 params.enabled ? "ATIVO" : "DESATIVADO", params.kpPitch,
                 params.kdPitch, params.pitchRef, params.maxPitchCorrection);
    }
}

/**
 * Publica o plano do gerador de marcha: pose do torso, pes, fase, estado de
 * apoio e os parametros do LIPM. E a metade "referencia" da telemetria; a
 * metade "realidade" e montada pelo walk_telemetry_node a partir da verdade de
 * solo do Gazebo.
 */
void ControllerNode::publishWalkReference(const OmnidirectionalWalkZMP &walker,
                                          SupportState support) {
    const OmnidirectionalWalkZMPParams &wp = walker.getParams();

    marta_msgs::WalkReference msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "odom";

    // Atencao: o Vector3 do gerador guarda (x, y, theta), entao o campo z de
    // cada Point carrega YAW, nao altura. Documentado tambem no .msg.
    msg.com.x = walker.torso.x;
    msg.com.y = walker.torso.y;
    msg.com.z = walker.torso.z;

    msg.left_foot.x = walker.leftFoot_.x;
    msg.left_foot.y = walker.leftFoot_.y;
    msg.left_foot.z = walker.leftFoot_.z;
    msg.right_foot.x = walker.rightFoot_.x;
    msg.right_foot.y = walker.rightFoot_.y;
    msg.right_foot.z = walker.rightFoot_.z;

    msg.z_left = walker.zLeft_;
    msg.z_right = walker.zRight_;

    msg.phase = walker.getPhase();
    msg.support_state = static_cast<uint8_t>(support);

    msg.z_com = wp.zCom;
    msg.t_c = walker.getTimeConstant();
    msg.step_period = wp.period;
    msg.double_support_ratio = wp.doubleSupportRatio;

    walk_reference_pub_.publish(msg);
}

void ControllerNode::publishStabilizerDebug(
        const stabilization::TorsoStabilizerState &state, SupportState support,
        double pitch, double pitchRate) {
    std_msgs::Float64MultiArray msg;
    msg.layout.dim.resize(1);
    // heading_error e heading_correction (malha de rumo) vao no fim, para nao
    // deslocar os indices que o walk_test.py ja le.
    msg.layout.dim[0].label = "delta_pitch,left_hip_delta,right_hip_delta,"
                              "pitch,pitch_rate,support,saturated,"
                              "heading_error,heading_correction";
    msg.layout.dim[0].size = 9;
    msg.layout.dim[0].stride = 9;
    msg.data = {state.deltaPitch,
                state.leftHipDelta,
                state.rightHipDelta,
                pitch,
                pitchRate,
                static_cast<double>(support),  // 0=duplo, 1=esquerdo, 2=direito
                state.saturated ? 1.0 : 0.0,
                lastHeadingError_,
                lastHeadingCorrection_};
    stabilizer_debug_pub_.publish(msg);
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

            Xscale_.store(normalize(angle));

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

        msg_array.data = {-10, -10, joints.leftShoulderPitch * scalar, joints.leftShoulderRoll * scalar, 0};
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
    ros::Duration step(0.1);
    for (int i = 1; i <= N && ros::ok(); ++i) {
        publishJoints(joints, (float)i/N);
        step.sleep();
    }
}

void ControllerNode::Update(Joints &joints){
    // Configuração dos parâmetros físicos do robô
    representations::RobotPhysicalParameters physicalParams = representations::RobotPhysicalParameters::getMartaPhysicalParameters();
    // x do quadril em relacao ao tronco na IK. O default (codigo) foi ajustado
    // empiricamente com o joelho cedendo; com a compensacao de gravidade o
    // geometrico do URDF (0,0036) volta a fazer sentido (ajustes_marcha.md).
    nh_.param<double>("hip_offset_x", physicalParams.torsoToLeftHip.x,
                      physicalParams.torsoToLeftHip.x);
    ROS_INFO("[Controller] hip_offset_x (tronco -> quadril, IK) = %.4f m",
             physicalParams.torsoToLeftHip.x);

    // Configuração dos parâmetros do andar
    OmnidirectionalWalkZMPParams walkParams = OmnidirectionalWalkZMPParams::getDefaultOmnidirecionalWalkZMPParams();

    // comSwingFactor entra em UM lugar so: tZMP = sqrt(zCom*comSwingFactor/g),
    // a constante de tempo do LIPM. A telemetria mostrou que o zCom do gerador
    // (0.2975 m, distancia torso-pe da IK) nao e a altura real do CoM
    // (~0.218 m), um erro de 36% em g/zc. comSwingFactor = 0.734 alinha os dois.
    // Exposto como parametro para que isso possa ser MEDIDO, nao so corrigido.
    nh_.param<double>("com_swing_factor", walkParams.comSwingFactor,
                      walkParams.comSwingFactor);
    // Demais parametros do gerador, para experimentos de postura e de marcha
    // sem recompilar (plano_comparacao_darwin.md). Defaults = os do gerador.
    // z_com e a altura do tronco usada pela IK: subir z_com estica a perna.
    nh_.param<double>("z_com", walkParams.zCom, walkParams.zCom);
    nh_.param<double>("z_step", walkParams.zStep, walkParams.zStep);
    nh_.param<double>("step_period", walkParams.period, walkParams.period);
    nh_.param<double>("double_support_ratio", walkParams.doubleSupportRatio,
                      walkParams.doubleSupportRatio);
    nh_.param<double>("y_separation", walkParams.ySeparation, walkParams.ySeparation);
    double hipPitchOffsetDeg = walkParams.hipPitchOffset * 180.0 / M_PI;
    nh_.param<double>("hip_pitch_offset_deg", hipPitchOffsetDeg, hipPitchOffsetDeg);
    walkParams.hipPitchOffset = hipPitchOffsetDeg * M_PI / 180.0;
    ROS_INFO("[Controller] hip_pitch_offset = %.2f graus", hipPitchOffsetDeg);
    ROS_INFO("[Controller] zCom=%.4f zStep=%.3f period=%.3f ds=%.2f ySep=%.3f",
             walkParams.zCom, walkParams.zStep, walkParams.period,
             walkParams.doubleSupportRatio, walkParams.ySeparation);
    ROS_INFO("[Controller] comSwingFactor=%.4f -> tZMP=%.4f s",
             walkParams.comSwingFactor,
             sqrt(walkParams.zCom * walkParams.comSwingFactor / 9.80665));


    // Instância de cinemática inversa
    InverseKinematics inverseKinematics(physicalParams);
    double hipYawSign = 1.0;
    nh_.param<double>("hip_yaw_sign", hipYawSign, 1.0);
    inverseKinematics.setHipYawSign(hipYawSign);
    ROS_INFO("[Controller] hip_yaw_sign = %+.0f", hipYawSign < 0.0 ? -1.0 : 1.0);

    // Instância da classe OmnidirectionalWalkZMP
    OmnidirectionalWalkZMP walker(walkParams, physicalParams, &inverseKinematics);

    // Pe que balanca primeiro (teste do espelho do desvio de direcao). O
    // default reproduz o comportamento historico: o direito primeiro.
    bool startLeftSwing = false;
    nh_.param<bool>("start_left_swing", startLeftSwing, false);
    walker.setStartWithLeftSwing(startLeftSwing);
    int startRampSteps = 0;
    nh_.param<int>("start_ramp_steps", startRampSteps, 0);
    walker.setStartRampSteps(startRampSteps);
    walker.restart();
    ROS_INFO("[Controller] primeiro passo: pe %s balanca; partida suave em %d passos",
             startLeftSwing ? "ESQUERDO" : "direito", startRampSteps);

    // Frequência do laço de controle. O gerador de marcha integra o tempo, então
    // ele precisa do dt REAL de cada iteração: com usleep e dt fixo o relógio da
    // marcha desliza em relação ao relógio real, e um ganho derivativo calculado
    // com dt errado tem ganho efetivo errado.
    const double CONTROL_RATE_HZ = 100.0;
    const double NOMINAL_DT = 1.0 / CONTROL_RATE_HZ;
    const double MAX_DT = 0.1;  // acima disso, o ciclo perdeu o passo

    ros::Rate loopRate(CONTROL_RATE_HZ);

    // Releitura dos ganhos do estabilizador a cada 0,5 s, para sintonia com
    // "rosparam set" durante o teste, sem reiniciar o nó.
    const int PARAM_REFRESH_CYCLES = static_cast<int>(0.5 * CONTROL_RATE_HZ);
    int paramRefreshCounter = 0;

    itandroids_lib::math::Pose2D desiredVelocity;

    // Configure os valores da velocidade
    desiredVelocity.translation.x = translationX_.load(); // Velocidade no eixo x
    desiredVelocity.translation.y = translationY_.load(); // Velocidade no eixo y
    desiredVelocity.rotation = rotationZ_.load();         // Velocidade de rotação (yaw)

    // Chame a função setDesiredVelocity
    walker.setDesiredVelocity(desiredVelocity);

    // Compensacao de gravidade: modelo do robot_description e Kp dos servos.
    gravityComp_.init(isReal_);

    // Fazendo a Marta agachar lentamente
    float xscale = Xscale_.load();
    walker.update(NOMINAL_DT, joints, xscale);
    Agacha(joints);

    bool walkingPrev = walk_enabled_.load();
    if (!walkingPrev) {
        // Leva o gerador ao estado STOPPED: postura estática de apoio duplo,
        // sem ciclo de passo. O estabilizador continua atuando normalmente.
        // requestStop() da ainda ~3 passos no lugar antes de parar, o que logo
        // apos o Agacha() arrasta os pes; skip_initial_stop_steps para direto
        // (investigacao do desvio de direcao).
        bool skipStopSteps = false;
        nh_.param<bool>("skip_initial_stop_steps", skipStopSteps, false);
        if (skipStopSteps) {
            walker.stopNow();
        } else {
            walker.requestStop();
        }
        ROS_INFO("[Controller] walk_enabled=false: parando a marcha "
                 "(teste estatico de perturbacao).");
    }

    ros::Time previousTime = ros::Time::now();

    while (ros::ok()) {

        // dt medido, com guarda contra saltos do /clock e ciclos perdidos
        const ros::Time now = ros::Time::now();
        double elapsedTime = (now - previousTime).toSec();
        previousTime = now;
        if (!(elapsedTime > 0.0) || elapsedTime > MAX_DT) {
            ROS_WARN_THROTTLE(2.0, "[Controller] dt do laco fora da faixa (%.4f s); "
                              "usando %.4f s.", elapsedTime, NOMINAL_DT);
            elapsedTime = NOMINAL_DT;
        }

        const bool fallenForward = fall_recovery_enabled_ && imuXAcc_.load() > 8;
        const bool fallenBackward = fall_recovery_enabled_ && imuXAcc_.load() < -8;

        if(fallenForward){
            ROS_WARN_THROTTLE(2.0, "Marta esta deitada para frente, Tentando levantar");

            marta_msgs::Levantar srv;

            if (client_frente_.call(srv)) {
                if (srv.response.success) {
                    ROS_INFO("%s", srv.response.message.c_str());
                } else {
                    ROS_ERROR("Levantar para frente falhou: %s",
                              srv.response.message.c_str());
                }
            } else {
                // Sem o servico no ar, a chamada falha e a resposta vem vazia:
                // reportar a falha da chamada em vez de imprimir string vazia.
                ROS_ERROR_THROTTLE(5.0, "Servico /levantar_para_frente indisponivel.");
            }
            walker.restart();
            stabilizer_.reset();
            gravityComp_.reset();

        }else if(fallenBackward){
            ROS_WARN_THROTTLE(2.0, "Marta esta deitada para tras. Tentando levantar");

            marta_msgs::Levantar srv;
            if (client_tras_.call(srv)) {
                if (srv.response.success) {
                    ROS_INFO("%s", srv.response.message.c_str());
                } else {
                    ROS_ERROR("Levantar para tras falhou: %s",
                              srv.response.message.c_str());
                }
            } else {
                ROS_ERROR_THROTTLE(5.0, "Servico /levantar_para_tras indisponivel.");
            }
            walker.restart();
            stabilizer_.reset();
            gravityComp_.reset();

        }else{
            desiredVelocity.translation.x = translationX_.load(); // Velocidade no eixo x
            desiredVelocity.translation.y = translationY_.load(); // Velocidade no eixo y
            // Malha de rumo (yaw): soma uma correcao ao comando de rotacao.
            {
                const double rotationCommand = rotationZ_.load();
                desiredVelocity.rotation =
                        rotationCommand + headingCorrection(rotationCommand, elapsedTime);
            }

            // Chame a função setDesiredVelocity
            walker.setDesiredVelocity(desiredVelocity);

            // Cópia local: walker.update recebe float& e Xscale_ é atômico.
            xscale = Xscale_.load();
            walker.update(elapsedTime, joints, xscale);

            // ── Estabilizador de atitude (Fase 2) ────────────────────────
            // Entra aqui, entre a IK e o despacho: o gerador de marcha
            // permanece em malha aberta e a realimentação é uma camada
            // separada, como em Kajita 4.5.1.
            if (++paramRefreshCounter >= PARAM_REFRESH_CYCLES) {
                paramRefreshCounter = 0;
                refreshStabilizerParams();
                refreshHeadingParams();
                gravityComp_.refreshParams();

                // walk_enabled pode ser chaveado em tempo de execução, para
                // assentar o robô em pé e só então mandar andar.
                bool walkingNow = walkingPrev;
                nh_.getParamCached("walk_enabled", walkingNow);
                if (walkingNow != walkingPrev) {
                    walk_enabled_.store(walkingNow);
                    if (walkingNow) {
                        // restart() devolve o gerador ao estado NONE; sem isso
                        // ele ficaria preso em STOPPED e nunca daria um passo.
                        walker.restart();
                        ROS_INFO("[Controller] marcha INICIADA.");
                    } else {
                        walker.requestStop();
                        ROS_INFO("[Controller] marcha: parada solicitada.");
                    }
                    walkingPrev = walkingNow;
                }
            }

            const SupportState support = walker.getSupportState();

            // Compensacao de gravidade (feedforward): soma tau/Kp aos alvos de
            // perna da IK, antes da correcao do estabilizador de pitch.
            {
                double wLeft, wRight;
                walker.getLoadWeights(wLeft, wRight);
                gravityComp_.apply(joints, wLeft, wRight, elapsedTime);
            }

            // Referencia de marcha: publicada ANTES do bloco que depende da
            // atitude, de proposito. A telemetria do plano nao pode ficar muda
            // so porque o estimador de atitude nao subiu.
            publishWalkReference(walker, support);

            if (hasAttitude_.load()) {
                const double pitch = torsoPitch_.load();
                const double pitchRate = torsoPitchRate_.load();

                const stabilization::TorsoStabilizerState &stabState =
                        stabilizer_.update(pitch, pitchRate, support, elapsedTime,
                                           joints);

                publishStabilizerDebug(stabState, support, pitch, pitchRate);

                if (stabState.saturated) {
                    ROS_WARN_THROTTLE(1.0, "[Stabilizer] correcao saturada em "
                                      "%.1f graus (pitch=%.1f graus). Ganhos altos "
                                      "ou o robo esta caindo.",
                                      stabState.deltaPitch * 180.0 / M_PI,
                                      pitch * 180.0 / M_PI);
                }

                ROS_INFO_THROTTLE(1.0,
                        "[Controller] pitch=%.2f deg  pitch_rate=%.2f rad/s  "
                        "roll=%.2f deg  apoio=%s  delta_hip=%.2f deg  dt=%.4f s",
                        pitch * 180.0 / M_PI, pitchRate,
                        torsoRoll_.load() * 180.0 / M_PI,
                        support == DOUBLE_SUPPORT ? "duplo"
                                : (support == LEFT_SUPPORT ? "esquerdo" : "direito"),
                        stabState.deltaPitch * 180.0 / M_PI, elapsedTime);
            } else {
                // Sem estimativa de atitude não há realimentação possível:
                // segue em malha aberta em vez de aplicar correção de lixo.
                stabilizer_.reset();
                ROS_WARN_THROTTLE(5.0, "[Controller] sem dados em "
                                  "/marta/torso_attitude; estabilizador inativo. "
                                  "O torso_attitude_node esta rodando?");
            }

            publishJoints(joints, 1);
        }

        loopRate.sleep();
    }

}

void ControllerNode::calculateVelocity(){
    ros::Rate rate(10.0);

    while (ros::ok()) {
        // Diferença de tempo corrido em segundos:
        double dt = ros::Time::now().toSec() - hasBallTimeSec_.load();

        if (!walk_enabled_.load()) {
            // Modo de teste estático: robô em pé, só o estabilizador atuando.
            translationX_.store(0.0f);
            translationY_.store(0.0f);
            rotationZ_.store(0.0f);
        } else if (dt > 5.0){
            ROS_INFO_THROTTLE(2.0, "Marta nao tem a bola, parando de andar");
            translationX_.store(static_cast<float>(walk_velocity_x_));
            translationY_.store(static_cast<float>(walk_velocity_y_));
            rotationZ_.store(static_cast<float>(walk_velocity_yaw_));
        }else{
            ROS_INFO_THROTTLE(2.0, "Marta tem a bola, andando");
            translationX_.store(static_cast<float>(walk_velocity_x_));
            translationY_.store(static_cast<float>(walk_velocity_y_));
            rotationZ_.store(static_cast<float>(walk_velocity_yaw_));
        }

        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_node");

    Joints joints;
    ControllerNode node(joints);
    ros::spin();
    return 0;
}
