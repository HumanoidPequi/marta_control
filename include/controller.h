#ifndef CONTROLLER_NODE_H
#define CONTROLLER_NODE_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <unistd.h>
#include <atomic>
#include <deque>
#include <numeric>
#include <string>
#include "Joints.h"
#include "RobotPhysicalParameters.h"
#include "OmnidirectionalWalkZMP.h"
#include "InverseKinematics.h"
#include "TorsoStabilizer.h"
#include "LegGravityCompensation.h"
#include <thread>
#include <std_srvs/Trigger.h>
#include <marta_msgs/Levantar.h>
#include <marta_msgs/WalkReference.h>
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

    /** Telemetria do estabilizador, para rqt_plot e rosbag. */
    ros::Publisher stabilizer_debug_pub_;

    /** Plano do gerador de marcha, metade "referencia" da telemetria. */
    ros::Publisher walk_reference_pub_;

    ros::Subscriber imu_sub_;
    ros::Subscriber ball_distance_sub_;
    ros::Subscriber attitude_sub_;

    // Acessado apenas dentro do callback da IMU (thread de callbacks do ROS).
    std::deque<double> imuXAccBuffer_;

    bool get_angle_x_axis_available_ = true;
    bool isReal_ = false;
    bool hasBallDistance_ = false;

    /**
     * Habilita a realimentacao antiga de inclinacao por Xscale_, que consulta
     * /get_angle_x_axis por chamada de servico sincrona e desloca o offset do
     * quadril dentro da IK. Desligada por default: o caminho de realimentacao
     * de pitch passa a ser o estabilizador PD (Fase 2), e as duas malhas
     * atuando juntas na mesma grandeza brigariam entre si.
     */
    bool use_xscale_service_ = false;

    /**
     * Com walk_enabled=false o robô fica em pé sobre os dois pés em vez de
     * caminhar: as velocidades vão a zero e o gerador de marcha recebe
     * requestStop(). É o modo usado nos testes de perturbação da Fase 2, em que
     * só o estabilizador deve estar atuando.
     *
     * É relido a cada ciclo de refresh, então dá para mandar o robô começar e
     * parar de andar com "rosparam set /controller_node/walk_enabled true"
     * sem reiniciar o nó. Atômico porque calculateVelocity() também o lê.
     */
    std::atomic<bool> walk_enabled_{true};

    /**
     * Habilita a deteccao de queda e a chamada dos servicos de levantar.
     * Desligada nos testes de perturbacao, onde a recuperacao atrapalharia a
     * medicao da resposta do estabilizador.
     */
    bool fall_recovery_enabled_ = true;

    /** Velocidade de marcha usada quando walk_enabled_ é verdadeiro. */
    double walk_velocity_x_ = 0.1;
    double walk_velocity_y_ = 0.0;
    double walk_velocity_yaw_ = 0.0;

    float dt;

    // ── Estado compartilhado entre os callbacks do ROS e a thread Update() ──
    // Todos atomicos: sao escritos em uma thread e lidos na outra a cada ciclo.
    std::atomic<float> Xscale_{1.0f};
    std::atomic<float> imuXAcc_{0.0f};
    std::atomic<float> translationX_{0.0f};
    std::atomic<float> translationY_{0.0f};
    std::atomic<float> rotationZ_{0.0f};

    // Atitude do tronco publicada pelo torso_attitude_node, na convencao
    // pitch > 0 = tombando para frente, roll > 0 = tombando para a direita.
    // O pitch e o pitch_rate alimentam o estabilizador (Fase 2); o roll fica
    // disponivel para a Fase 3.
    std::atomic<float> torsoPitch_{0.0f};
    std::atomic<float> torsoPitchRate_{0.0f};
    std::atomic<float> torsoRoll_{0.0f};
    std::atomic<float> torsoRollRate_{0.0f};
    std::atomic<bool> hasAttitude_{false};

    // Malha de RUMO (yaw). O yaw vem do torso_attitude_node por integracao do
    // giroscopio: sem referencia absoluta, so a variacao desde o inicio da
    // marcha e usada. Lidos e escritos so na thread Update(), exceto os atomicos.
    std::atomic<float> torsoYaw_{0.0f};
    std::atomic<bool> hasYaw_{false};
    bool headingEnabled_ = false;
    double kpHeading_ = 1.0;          ///< rad/s de rotacao por rad de erro
    double maxHeadingRate_ = 0.3;     ///< saturacao da correcao [rad/s]
    double headingRef_ = 0.0;
    bool headingRefSet_ = false;
    double lastHeadingError_ = 0.0;
    double lastHeadingCorrection_ = 0.0;

    ros::Timer timer_;

    // Instante da última detecção de bola, em segundos. Escrito no callback e
    // lido na thread calculateVelocity().
    std::atomic<double> hasBallTimeSec_{0.0};

    ros::ServiceClient clientAngles_;
    ros::ServiceClient client_frente_;
    ros::ServiceClient client_tras_;
    std::thread functionThread_;
    std::thread calculateVelocity_;

    // Nó ROS
    ros::NodeHandle nh_;

    // Estabilizador de atitude do tronco (Fase 2: pitch). Vive na thread
    // Update(), que é a única a chamá-lo.
    stabilization::TorsoStabilizer stabilizer_;

    // Compensacao de gravidade nas pernas (feedforward, plano_fase3_roll.md
    // §4.2). Declarada depois de nh_, que ela usa.
    stabilization::LegGravityCompensation gravityComp_{nh_};

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void attitudeCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void timerCallback(const ros::TimerEvent &event);
    void ballDistanceCallback(const std_msgs::Float32::ConstPtr &msg);
    void Update(Joints &joints);
    void calculateVelocity();
    float normalize(float x);

    /** Lê os ganhos do estabilizador dos parâmetros ROS (chamado na partida). */
    stabilization::TorsoStabilizerParams loadStabilizerParams();

    /**
     * Relê os ganhos via getParamCached, permitindo sintonia com
     * "rosparam set" sem reiniciar o nó. Chamada periodicamente pelo laço.
     */
    void refreshStabilizerParams();

    /** Rele heading_enabled, kp_heading e max_heading_rate. */
    void refreshHeadingParams();

    /**
     * Correcao de rotacao [rad/s] a somar ao comando, para manter o rumo do
     * inicio da marcha (avancado pela rotacao comandada). Zero se a malha
     * estiver desligada, sem yaw ou parado.
     */
    double headingCorrection(double rotationCommand, double dt);

    /** Publica o plano do gerador de marcha (pose, pes, fase, LIPM). */
    void publishWalkReference(const OmnidirectionalWalkZMP &walker,
                              SupportState support);

    /** Publica a telemetria do estabilizador para rqt_plot / rosbag. */
    void publishStabilizerDebug(const stabilization::TorsoStabilizerState &state,
                                SupportState support, double pitch, double pitchRate);
};

}  // namespace walking
}  // namespace control

#endif  // CONTROLLER_NODE_H
