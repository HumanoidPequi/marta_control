/*
 * TorsoAttitudeNode.h
 *
 * No ROS que le /marta/imu, estima a atitude do tronco com
 * control::estimation::TorsoAttitudeEstimator e publica
 * (pitch, pitch_rate, roll, roll_rate) em um topico dedicado.
 *
 * Substitui o antigo IMUFilterNode (imu.h / imu.cpp), cuja predicao de Kalman
 * de 9 estados nao tinha etapa de correcao e cujo estado nunca era lido.
 *
 * Publica:
 *   ~/../torso_attitude (default /marta/torso_attitude), std_msgs/Float64MultiArray
 *       data[0] = pitch      [rad]  positivo = tombando para frente
 *       data[1] = pitch_rate [rad/s]
 *       data[2] = roll       [rad]  positivo = tombando para a direita
 *       data[3] = roll_rate  [rad/s]
 *
 * Servico:
 *   get_angle_x_axis (std_srvs/Trigger) - mantido por compatibilidade com o
 *       Xscale_ do controller_node. ATENCAO: agora devolve o pitch filtrado em
 *       graus na convencao acima, enquanto a versao antiga devolvia o angulo
 *       cru do acelerometro no frame do sensor, que tem o sinal oposto.
 *
 * Parametros privados (~):
 *   imu_topic        (string, /marta/imu)
 *   attitude_topic   (string, /marta/torso_attitude)
 *   alpha            (double, 0.998)  peso do giro no filtro complementar
 *   gyro_cutoff_hz   (double, 15.0)   corte do Butterworth de 2a ordem do giro
 *   accel_trust_band (double, 2.0)    faixa de aceite do acelerometro [m/s^2]
 *   imu_mount_rpy    (double[3], [0, pi, 0]) rotacao sensor -> corpo
 */

#ifndef TORSO_ATTITUDE_NODE_H_
#define TORSO_ATTITUDE_NODE_H_
#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Trigger.h>

#include <string>

#include "TorsoAttitudeEstimator.h"

class TorsoAttitudeNode {
public:
    explicit TorsoAttitudeNode(ros::NodeHandle &nh);

private:
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Publisher attitude_pub_;
    ros::ServiceServer angle_service_;

    control::estimation::TorsoAttitudeEstimator estimator_;

    ros::Time last_imu_stamp_;
    bool has_last_stamp_;
    double nominal_dt_;

    /** Le os parametros privados e monta a configuracao do estimador. */
    control::estimation::TorsoAttitudeParams loadParams();

    /** Registra a constante de tempo equivalente do filtro complementar. */
    void logCrossover() const;

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);

    /** Compatibilidade: devolve o pitch filtrado em graus. */
    bool getAngle(std_srvs::Trigger::Request &req,
                  std_srvs::Trigger::Response &res);
};

#endif  // TORSO_ATTITUDE_NODE_H_
