// Verificacao offline do GravityCompensator (plano_fase3_roll.md §4.4).
//
// Compilado pelo catkin_make (alvo test_gravity_compensator). Rodar:
//   rosrun marta_controller test_gravity_compensator <marta.urdf>
// (o URDF gerado pelo xacro com os defaults; sem argumento roda so a parte 1)
//
// 1. Pendulo de 2 links com torques calculados a mao: pe fixo e perna
//    pendurada, sinais inclusive.
// 2. URDF da Marta: massa total 4,781 kg e torques iguais aos do modelo de
//    referencia em Python (experiments/gravidade/modelo_gravidade.py), escrito
//    de forma independente, para uma pose com as duas pernas dobradas.
// 3. Pesos de carga do gerador: somam 1, valem 1 no apoio simples e sao
//    continuos na troca de passo.

#include <urdf/model.h>

#include <cmath>
#include <cstdio>
#include <map>
#include <string>

#include "GravityCompensator.h"
#include "OmnidirectionalWalkZMP.h"
#include "RobotModelLoader.h"

using control::stabilization::GravityCompensator;
using control::stabilization::RigidBodyModel;

static int failures = 0;

static void check(bool ok, const char *what, double got, double want) {
    std::printf("  [%s] %-46s %+.6f (esperado %+.6f)\n", ok ? " ok " : "FALHA", what, got, want);
    if (!ok)
        ++failures;
}

static void near(double got, double want, double tol, const char *what) {
    check(std::fabs(got - want) <= tol, what, got, want);
}

static void testPendulum() {
    std::printf("1. pendulo de 2 links\n");
    const double g = 9.81;
    RigidBodyModel m;
    m.links.resize(3);
    m.links[0].name = "torso"; m.links[0].mass = 2.0; m.links[0].com = {0.1, 0, 0};
    m.links[1].name = "leg";   m.links[1].mass = 1.0; m.links[1].com = {0, 0, -0.2};
    m.links[2].name = "foot";  m.links[2].mass = 0.5;
    m.joints.resize(2);
    m.joints[0].name = "hip";   m.joints[0].parentLink = 0; m.joints[0].childLink = 1;
    m.joints[0].axis = Eigen::Vector3d::UnitY();
    m.joints[1].name = "ankle"; m.joints[1].parentLink = 1; m.joints[1].childLink = 2;
    m.joints[1].axis = Eigen::Vector3d::UnitY(); m.joints[1].originXyz = {0, 0, -0.4};
    m.links[1].parentJoint = 0; m.links[2].parentJoint = 1;
    m.links[0].childJoints = {0}; m.links[1].childJoints = {1};
    GravityCompensator c(g);
    if (!c.setModel(m)) { std::printf("  modelo invalido\n"); ++failures; return; }

    // Pe fixo, q = 0: tronco (2 kg em x = 0,1) pesa sobre as duas juntas.
    auto t = c.legTorques({0.0, 0.0}, 2, true);
    near(t[0], 0.2 * g, 1e-9, "apoio: quadril sustenta o tronco");
    near(t[1], 0.2 * g, 1e-9, "apoio: tornozelo (perna vertical)");
    // Pendurado, quadril a 90 graus: perna e pe na horizontal em -x.
    t = c.legTorques({M_PI / 2, 0.0}, 2, false);
    near(t[0], 0.4 * g, 1e-9, "balanco: quadril segura perna + pe");
    near(t[1], 0.0, 1e-9, "balanco: tornozelo (pe pontual)");
}

static void testMartaUrdf(const char *path) {
    std::printf("2. URDF da Marta (%s)\n", path);
    urdf::Model u;
    if (!u.initFile(path)) { std::printf("  nao li o URDF\n"); ++failures; return; }
    RigidBodyModel m;
    std::string err;
    GravityCompensator c(9.81);
    if (!control::stabilization::buildRigidBodyModel(u, m, err) || !c.setModel(m)) {
        std::printf("  modelo invalido: %s\n", err.c_str()); ++failures; return;
    }
    near(c.totalMass(), 4.781104, 1e-4, "massa total [kg]");

    const std::map<std::string, double> pose = {
        {"l_hip_roll_link", 0.1}, {"l_hip_pitch_link", -0.6}, {"l_knee_link", 1.1},
        {"l_ank_pitch_link", -0.5}, {"l_ank_roll_link", -0.1}, {"r_hip_roll_link", 0.05},
        {"r_hip_pitch_link", 0.5}, {"r_knee_link", -1.0}, {"r_ank_pitch_link", 0.45},
        {"r_sho_pitch_link", 0.3}};
    std::vector<double> q(m.joints.size(), 0.0);
    for (const auto &kv : pose)
        q[m.jointIndex(kv.first)] = kv.second;

    // Referencia: modelo_gravidade.py, mesma pose e g = 9,81.
    const std::map<std::string, double> apoioEsq = {
        {"l_hip_roll_link", 1.871653}, {"l_hip_pitch_link", -0.227696}, {"l_knee_link", -2.704857},
        {"l_ank_pitch_link", -0.581145}, {"l_ank_roll_link", 2.603812}};
    const std::map<std::string, double> balancoDir = {
        {"r_hip_roll_link", 0.028076}, {"r_hip_pitch_link", 0.065598}, {"r_knee_link", -0.091429},
        {"r_ank_pitch_link", 0.022686}, {"r_ank_roll_link", 0.000969}};
    auto t = c.legTorques(q, m.linkIndex("l_feet_1"), true);
    for (const auto &kv : apoioEsq)
        near(t[m.jointIndex(kv.first)], kv.second, 1e-5, ("apoio esq: " + kv.first).c_str());
    t = c.legTorques(q, m.linkIndex("r_feet_1"), false);
    for (const auto &kv : balancoDir)
        near(t[m.jointIndex(kv.first)], kv.second, 1e-5, ("balanco dir: " + kv.first).c_str());
}

static void testLoadWeights() {
    std::printf("3. pesos de carga do gerador\n");
    using namespace control::walking;
    representations::RobotPhysicalParameters phys =
            representations::RobotPhysicalParameters::getMartaPhysicalParameters();
    InverseKinematics ik(phys);
    OmnidirectionalWalkZMP w(OmnidirectionalWalkZMPParams::getDefaultOmnidirecionalWalkZMPParams(),
                             phys, &ik);
    itandroids_lib::math::Pose2D v;
    v.translation.x = 0.1;
    w.setDesiredVelocity(v);
    representations::Joints j;
    float xs = 1.0f;
    double soma = 0.0, maxSalto = 0.0, minSimples = 1.0, pl = -1, pr = -1;
    for (int i = 0; i < 600; ++i) {
        w.update(0.01, j, xs);
        double l, r;
        w.getLoadWeights(l, r);
        soma = std::max(soma, std::fabs(l + r - 1.0));
        if (pl >= 0)
            maxSalto = std::max(maxSalto, std::max(std::fabs(l - pl), std::fabs(r - pr)));
        if (!w.isDoubleSupport())
            minSimples = std::min(minSimples, std::max(l, r));
        pl = l; pr = r;
    }
    check(soma < 1e-12, "pesos somam 1 (maior desvio)", soma, 0.0);
    check(minSimples > 1.0 - 1e-12, "apoio simples: perna de apoio com peso 1", minSimples, 1.0);
    // Passo de 0,01 s num apoio duplo de 0,09 s: o peso anda ~0,056 por ciclo.
    check(maxSalto < 0.07, "continuo (maior salto entre ciclos)", maxSalto, 0.056);
}

int main(int argc, char **argv) {
    testPendulum();
    if (argc > 1)
        testMartaUrdf(argv[1]);
    testLoadWeights();
    std::printf("\n%s (%d falha(s))\n", failures == 0 ? "TODOS OS TESTES PASSARAM"
                                                     : "HA FALHAS", failures);
    return failures == 0 ? 0 : 1;
}
