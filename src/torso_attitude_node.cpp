/*
 * torso_attitude_node.cpp
 *
 * Ver TorsoAttitudeNode.h para os topicos, o servico e os parametros.
 */

#include "TorsoAttitudeNode.h"

#include <cmath>
#include <vector>

using control::estimation::TorsoAttitude;
using control::estimation::TorsoAttitudeParams;

namespace {

// dt fora desta faixa e descartado: protege contra saltos do /clock em
// simulacao, mensagens fora de ordem e pausas do Gazebo.
const double MIN_VALID_DT = 1e-4;
const double MAX_VALID_DT = 0.1;

}  // namespace

TorsoAttitudeNode::TorsoAttitudeNode(ros::NodeHandle &nh) :
        nh_(nh),
        estimator_(TorsoAttitudeParams::getDefaultTorsoAttitudeParams()),
        has_last_stamp_(false),
        nominal_dt_(0.01) {
    ros::NodeHandle pnh("~");

    double expected_rate_hz = 100.0;
    pnh.param<double>("expected_rate_hz", expected_rate_hz, 100.0);
    if (expected_rate_hz > 0.0) {
        nominal_dt_ = 1.0 / expected_rate_hz;
    }

    estimator_.setParams(loadParams());
    logCrossover();

    std::string imu_topic;
    std::string attitude_topic;
    pnh.param<std::string>("imu_topic", imu_topic, "/marta/imu");
    pnh.param<std::string>("attitude_topic", attitude_topic, "/marta/torso_attitude");

    attitude_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(attitude_topic, 10);
    imu_sub_ = nh_.subscribe(imu_topic, 100, &TorsoAttitudeNode::imuCallback, this);

    angle_service_ = nh_.advertiseService("get_angle_x_axis",
                                          &TorsoAttitudeNode::getAngle, this);

    ROS_INFO("[TorsoAttitude] imu=%s  attitude=%s  taxa esperada=%.1f Hz",
             imu_topic.c_str(), attitude_topic.c_str(), expected_rate_hz);
}

TorsoAttitudeParams TorsoAttitudeNode::loadParams() {
    ros::NodeHandle pnh("~");

    TorsoAttitudeParams params = TorsoAttitudeParams::getDefaultTorsoAttitudeParams();

    pnh.param<double>("alpha", params.alpha, params.alpha);
    pnh.param<double>("gyro_cutoff_hz", params.gyroCutoffHz, params.gyroCutoffHz);
    pnh.param<double>("accel_trust_band", params.accelTrustBand, params.accelTrustBand);
    pnh.param<double>("gravity", params.gravity, params.gravity);

    if (!(params.alpha > 0.0 && params.alpha < 1.0)) {
        ROS_WARN("[TorsoAttitude] alpha=%.4f fora de (0,1); usando 0.998.", params.alpha);
        params.alpha = 0.998;
    }

    // Rotacao do frame do sensor para o frame do corpo. Default [0, pi, 0]:
    // e a montagem declarada no URDF (sensor com rpy = (pi, 0, pi) em relacao a
    // base_link, o que equivale a R_y(pi)). No robo real, medir e ajustar aqui.
    std::vector<double> mount_rpy;
    if (pnh.getParam("imu_mount_rpy", mount_rpy)) {
        if (mount_rpy.size() == 3) {
            params.imuToBody = control::estimation::TorsoAttitudeEstimator::
                    rotationFromRPY(mount_rpy[0], mount_rpy[1], mount_rpy[2]);
            ROS_INFO("[TorsoAttitude] montagem da IMU rpy = [%.3f, %.3f, %.3f] rad",
                     mount_rpy[0], mount_rpy[1], mount_rpy[2]);
        } else {
            ROS_WARN("[TorsoAttitude] imu_mount_rpy precisa de 3 elementos; "
                     "mantendo o default R_y(pi).");
        }
    }

    ROS_INFO("[TorsoAttitude] alpha=%.4f  corte do giro=%.1f Hz  "
             "faixa de aceite do acel=%.2f m/s2",
             params.alpha, params.gyroCutoffHz, params.accelTrustBand);

    return params;
}

void TorsoAttitudeNode::logCrossover() const {
    // tau = alpha * dt / (1 - alpha) e a constante de tempo do crossover do
    // filtro complementar. Abaixo dela manda o acelerometro (e a aceleracao do
    // corpo vaza para o angulo); acima dela manda o giroscopio (e a deriva do
    // giro vaza). Na marcha, tau precisa ser bem maior que o periodo do passo.
    const TorsoAttitudeParams &params = estimator_.getParams();
    const double tau = params.alpha * nominal_dt_ / (1.0 - params.alpha);
    ROS_INFO("[TorsoAttitude] crossover do filtro complementar: tau = %.2f s "
             "(alpha=%.4f a %.0f Hz)", tau, params.alpha, 1.0 / nominal_dt_);
    if (tau < 1.0) {
        ROS_WARN("[TorsoAttitude] tau = %.2f s e curto para a marcha: a "
                 "aceleracao do corpo vai aparecer como inclinacao falsa. "
                 "Considere alpha >= 0.99.", tau);
    }
}

void TorsoAttitudeNode::imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    // Prefere o carimbo da mensagem; cai para o relogio do no se ele vier zerado.
    ros::Time stamp = msg->header.stamp;
    if (stamp.isZero()) {
        stamp = ros::Time::now();
    }

    double dt = nominal_dt_;
    if (has_last_stamp_) {
        const double measured = (stamp - last_imu_stamp_).toSec();
        if (measured >= MIN_VALID_DT && measured <= MAX_VALID_DT) {
            dt = measured;
        } else {
            ROS_WARN_THROTTLE(2.0, "[TorsoAttitude] dt invalido (%.4f s); "
                              "usando o nominal (%.4f s). Verifique a taxa de "
                              "publicacao da IMU e /use_sim_time.", measured, nominal_dt_);
        }
    }
    last_imu_stamp_ = stamp;
    has_last_stamp_ = true;

    Eigen::Vector3d accel(msg->linear_acceleration.x,
                          msg->linear_acceleration.y,
                          msg->linear_acceleration.z);
    Eigen::Vector3d gyro(msg->angular_velocity.x,
                         msg->angular_velocity.y,
                         msg->angular_velocity.z);

    estimator_.update(accel, gyro, dt);

    if (!estimator_.isInitialized()) {
        ROS_WARN_THROTTLE(2.0, "[TorsoAttitude] aguardando uma amostra confiavel "
                          "do acelerometro para inicializar.");
        return;
    }

    if (!estimator_.lastAccelAccepted()) {
        // Normal em impactos rapidos. Persistindo, a faixa de aceite esta
        // apertada demais e o pitch vai derivar so com o giroscopio.
        ROS_WARN_THROTTLE(5.0, "[TorsoAttitude] acelerometro fora da faixa de "
                          "confianca; integrando apenas o giroscopio.");
    }

    const TorsoAttitude &attitude = estimator_.getAttitude();

    std_msgs::Float64MultiArray out;
    out.layout.dim.resize(1);
    // yaw e yaw_rate vao no FIM, para nao deslocar os indices que os
    // consumidores antigos ja leem (eles checam size >= 4).
    out.layout.dim[0].label = "pitch,pitch_rate,roll,roll_rate,yaw,yaw_rate";
    out.layout.dim[0].size = 6;
    out.layout.dim[0].stride = 6;
    out.data = {attitude.pitch, attitude.pitchRate, attitude.roll, attitude.rollRate,
                attitude.yaw, attitude.yawRate};
    attitude_pub_.publish(out);
}

bool TorsoAttitudeNode::getAngle(std_srvs::Trigger::Request &req,
                                 std_srvs::Trigger::Response &res) {
    (void) req;

    if (!estimator_.isInitialized()) {
        res.success = false;
        res.message = "IMU is not ready yet.";
        return true;
    }

    const double pitch_deg = estimator_.getAttitude().pitch * 180.0 / M_PI;
    res.success = true;
    res.message = std::to_string(pitch_deg);
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "torso_attitude_node");
    ros::NodeHandle nh;
    TorsoAttitudeNode node(nh);
    ros::spin();
    return 0;
}
