/*
 * LegGravityCompensation.h
 *
 * Camada ROS da compensacao de gravidade (plano_fase3_roll.md §4.2): monta o
 * modelo do robot_description, le o Kp de cada servo do controlador que de
 * fato roda (/marta/<junta>_position/pid/p) e o limite de torque do URDF, e
 * soma Delta = clamp(tau)/Kp ao alvo das juntas de perna, logo depois da IK.
 *
 *   tau_perna = ganho * [ w * tau_apoio + (1 - w) * tau_balanco * (balanco ? 1 : 0) ]
 *
 * com w o peso de carga da perna (OmnidirectionalWalkZMP::getLoadWeights).
 * Parametros (privados do controller_node, relidos a cada ~0,5 s):
 *   gravity_comp_enabled  (false)
 *   gravity_comp_joints   ("hip_roll,knee")  juntas compensadas; Maximo usa estas
 *   gravity_comp_gain     (1.0)   o modelo estatico preve ~57% do torque medio
 *                                 do joelho (validacao E1)
 *   max_gravity_comp      (0.35 rad)
 *   gravity_comp_swing    (false) compensa tambem o peso da perna em balanco
 * Desligada no robo real (isReal): o Kp do servo real nao e o do Gazebo.
 * Diagnostico em /marta/gravity_comp (Float64MultiArray):
 *   [w_esq, w_dir, rampa, Delta x 12 juntas de perna, tau x 12]
 *   ordem das juntas: esq hip_yaw, hip_roll, hip_pitch, knee, ank_pitch,
 *   ank_roll, depois as mesmas da direita.
 */
#ifndef LEG_GRAVITY_COMPENSATION_H_
#define LEG_GRAVITY_COMPENSATION_H_

#include <ros/ros.h>

#include <string>
#include <vector>

#include "GravityCompensator.h"
#include "Joints.h"

namespace control {
namespace stabilization {

class LegGravityCompensation {
public:
    explicit LegGravityCompensation(ros::NodeHandle &nh);

    /** Carrega o modelo e os ganhos. false (e desliga) se algo faltar. */
    bool init(bool isReal);

    /** Rele os parametros via getParamCached. */
    void refreshParams();

    /** Soma a compensacao aos alvos de perna em joints (no lugar). */
    void apply(representations::Joints &joints, double wLeft, double wRight, double dt);

    /** Zera a rampa (apos queda/reinicio), para reentrar suave. */
    void reset() { ramp_ = 0.0; }

    bool enabled() const { return enabled_ && ready_; }

private:
    struct LegJoint {
        double representations::Joints::*field;
        std::string urdf;      // nome da junta no URDF
        std::string key;       // hip_yaw, hip_roll, ... (mascara)
        bool left;
        int index = -1;        // no modelo
        double kp = 0.0;
        double effort = 0.0;
        bool active = false;
    };
    struct OtherJoint {
        double representations::Joints::*field;
        std::string urdf;
        int index = -1;
    };

    void parseMask(const std::string &mask);

    ros::NodeHandle &nh_;
    ros::Publisher pub_;
    GravityCompensator comp_;
    std::vector<LegJoint> legs_;
    std::vector<OtherJoint> others_;
    int leftFoot_ = -1, rightFoot_ = -1;
    bool ready_ = false;
    bool enabled_ = false;
    bool swing_ = false;
    double gain_ = 1.0;
    double maxDelta_ = 0.35;
    double ramp_ = 0.0;
    std::string mask_;
    static constexpr double RAMP_TIME = 1.0;   // s para entrar por inteiro
};

}  // namespace stabilization
}  // namespace control

#endif  // LEG_GRAVITY_COMPENSATION_H_
