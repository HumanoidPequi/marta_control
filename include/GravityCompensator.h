/*
 * GravityCompensator.h
 *
 * Compensacao de gravidade por feedforward nas juntas das pernas (Maximo,
 * doutorado, §4.3, p. 78-81; plano_fase3_roll.md §4.2).
 *
 * Um servo de posicao so produz torque com erro: tau = Kp*(alvo - q). Sob
 * carga ele cede tau/Kp. Se o torque de gravidade da junta e conhecido, somar
 * tau/Kp ao alvo faz a junta terminar onde a IK queria.
 *
 * O modelo e um arvore de corpos rigidos (montada do URDF por quem usa a
 * classe; aqui nao ha dependencia de ROS). Cinematica direta no frame do
 * tronco, tronco suposto vertical (g = (0, 0, -g) nesse frame).
 *   - Perna de APOIO (pe fixo): a junta k sustenta tudo o que nao esta entre
 *     ela e o pe:  tau_k = [sum_{i fora da subarvore do filho} (r_i - p_k) x m_i g] . e_k
 *   - Perna em BALANCO (pendurada): a junta k sustenta a subarvore do filho:
 *     tau_k = -[sum_{i na subarvore} (r_i - p_k) x m_i g] . e_k
 * tau e o torque do atuador no link filho em torno de +e_k, a convencao do
 * comando do effort_controllers. Validacao com dados: plano_fase3_roll.md E1
 * (hip_roll confere; no joelho o sinal confere e o modelo estatico preve ~57%
 * do torque medio, por isso o ganho configuravel no controller_node).
 */
#ifndef GRAVITY_COMPENSATOR_H_
#define GRAVITY_COMPENSATOR_H_

#include <Eigen/Dense>
#include <map>
#include <string>
#include <vector>

namespace control {
namespace stabilization {

struct RigidBodyModel {
    struct Link {
        std::string name;
        double mass = 0.0;
        Eigen::Vector3d com = Eigen::Vector3d::Zero();   // no frame do link
        int parentJoint = -1;
        std::vector<int> childJoints;
    };
    struct Joint {
        std::string name;
        int parentLink = -1;
        int childLink = -1;
        Eigen::Vector3d originXyz = Eigen::Vector3d::Zero();
        Eigen::Matrix3d originRot = Eigen::Matrix3d::Identity();
        Eigen::Vector3d axis = Eigen::Vector3d::UnitX();  // unitario
        bool movable = true;
    };
    std::vector<Link> links;
    std::vector<Joint> joints;
    int root = 0;

    int linkIndex(const std::string &name) const;
    int jointIndex(const std::string &name) const;
};

class GravityCompensator {
public:
    explicit GravityCompensator(double gravity = 9.80665);

    /** Prepara as estruturas derivadas (subarvores, massas). false se invalido. */
    bool setModel(const RigidBodyModel &model);
    const RigidBodyModel &model() const { return model_; }
    double totalMass() const { return totalMass_; }

    /**
     * Torques de gravidade nas juntas da cadeia raiz -> footLink.
     * @param q       angulo por junta (indice do modelo); juntas fixas ignoradas
     * @param footLink link do pe
     * @param support true = pe fixo no chao (perna de apoio); false = pendurada
     * @return mapa indice da junta -> torque [N.m]
     */
    std::map<int, double> legTorques(const std::vector<double> &q, int footLink,
                                     bool support) const;

    /** Juntas da raiz ate footLink, na ordem raiz -> pe. */
    std::vector<int> chain(int footLink) const;

private:
    void forwardKinematics(const std::vector<double> &q,
                           std::vector<Eigen::Vector3d> &linkPos,
                           std::vector<Eigen::Matrix3d> &linkRot,
                           std::vector<Eigen::Vector3d> &jointPos,
                           std::vector<Eigen::Vector3d> &jointAxis) const;
    void collectSubtree(int link, std::vector<int> &out) const;

    RigidBodyModel model_;
    Eigen::Vector3d g_;
    double totalMass_;
    std::vector<std::vector<int>> subtreeOfJoint_;   // links na subarvore do filho
};

}  // namespace stabilization
}  // namespace control

#endif  // GRAVITY_COMPENSATOR_H_
