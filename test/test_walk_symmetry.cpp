// Verificacao offline da SIMETRIA do OmnidirectionalWalkZMP. Nao depende de ROS.
//
// Rodar a partir da raiz do pacote marta_control:
//   g++ -std=c++14 -Iinclude -I/usr/include/eigen3 \
//       test/test_walk_symmetry.cpp src/OmnidirectionalWalkZMP.cpp \
//       src/InverseKinematics.cpp src/RobotPhysicalParameters.cpp \
//       src/Joints.cpp src/MathUtils.cpp src/LinearInterpolator.cpp \
//       -o /tmp/test_walk_symmetry && /tmp/test_walk_symmetry
//
// Pergunta (hipotese H4 da investigacao do desvio de direcao): o gerador, com
// rotationZ = 0, pode ser a FONTE do giro? Ele e deterministico, entao uma
// execucao decide. Trava tres coisas:
//   1. nenhum yaw e comandado: theta dos pes, do torso e as duas hip_yaw sao
//      exatamente zero durante toda a marcha;
//   2. espelhamento: o passo com a perna esquerda em balanco e a imagem
//      espelhada (y -> -y) do passo seguinte, com a direita em balanco - nos
//      pes, nas juntas da perna e nos bracos;
//   3. pegadas em linha reta: compondo os passos planejados, o rumo e o
//      desvio lateral acumulados ficam em zero.

#include "OmnidirectionalWalkZMP.h"

#include <cmath>
#include <cstdio>
#include <vector>

using control::walking::InverseKinematics;
using control::walking::OmnidirectionalWalkZMP;
using control::walking::OmnidirectionalWalkZMPParams;
using itandroids_lib::math::Pose2D;
using itandroids_lib::math::Vector3;
using representations::Joints;
using representations::RobotPhysicalParameters;

static const double DT = 0.01;           // laco de 100 Hz, como no controller_node
static const double RAD2DEG = 180.0 / M_PI;
static const int STEPS = 100;
static const int WARMUP_STEPS = 4;       // transiente de partida de restart()

int failures = 0;

void check(const char *name, double got, double expected, double tol) {
    const bool ok = std::fabs(got - expected) <= tol;
    if (!ok) ++failures;
    std::printf("%-58s got=%11.3e  esperado=%9.3e  %s\n", name, got, expected,
                ok ? "OK" : "FALHOU");
}

// Uma amostra do gerador num instante da fase.
struct Sample {
    bool leftSwing;
    double lx, ly, lth, rx, ry, rth, torsoTh;
    Joints j;
};

// Pose do pe que pousa em relacao ao pe de apoio, no fim do passo.
struct Step {
    bool leftSwing;
    Vector3<double> landing;   // (x, y, theta)
};

int main() {
    RobotPhysicalParameters phys = RobotPhysicalParameters::getMartaPhysicalParameters();
    OmnidirectionalWalkZMPParams params =
            OmnidirectionalWalkZMPParams::getDefaultOmnidirecionalWalkZMPParams();
    InverseKinematics ik(phys);
    OmnidirectionalWalkZMP walker(params, phys, &ik);

    Pose2D v;
    v.translation.x = 0.1;   // o mesmo do walk_test.py
    v.translation.y = 0.0;
    v.rotation = 0.0;
    walker.setDesiredVelocity(v);

    const int ticksPerStep = static_cast<int>(std::lround(params.period / DT));
    std::vector<std::vector<Sample>> perStep;
    std::vector<Step> steps;

    Joints j;
    float xscale = 1.0f;
    double maxYaw = 0.0;       // do segundo passo em diante
    double maxYawStart = 0.0;  // so o primeiro passo depois de restart()
    double yawFirstTick = 0.0;
    bool prevSwing = walker.leftFootIsSwing;
    Vector3<double> prevEnd = walker.getSwingFootEnd();

    for (int k = 0; k < STEPS * ticksPerStep; ++k) {
        walker.update(DT, j, xscale);

        if (walker.leftFootIsSwing != prevSwing) {
            // computeNewStep rodou: o passo anterior terminou com o pe em
            // prevEnd (frame do pe de apoio daquele passo).
            steps.push_back({prevSwing, prevEnd});
            perStep.emplace_back();
            prevSwing = walker.leftFootIsSwing;
        }
        prevEnd = walker.getSwingFootEnd();

        const double yaw = std::max({std::fabs(walker.leftFoot_.z),
                                     std::fabs(walker.rightFoot_.z),
                                     std::fabs(walker.torso.z),
                                     std::fabs(j.leftHipYaw), std::fabs(j.rightHipYaw)});
        if (steps.empty()) {
            maxYawStart = std::max(maxYawStart, yaw);
            if (k == 0) yawFirstTick = walker.torso.z;
        } else {
            maxYaw = std::max(maxYaw, yaw);
        }

        if (!perStep.empty()) {
            perStep.back().push_back({walker.leftFootIsSwing,
                                      walker.leftFoot_.x, walker.leftFoot_.y,
                                      walker.leftFoot_.z, walker.rightFoot_.x,
                                      walker.rightFoot_.y, walker.rightFoot_.z,
                                      walker.torso.z, j});
        }
    }

    std::printf("%d passos, %d amostras por passo\n\n", (int)steps.size(), ticksPerStep);

    // --- 1. nenhum yaw comandado -------------------------------------------
    std::printf("1. yaw comandado (rotationZ = 0)\n");
    // restart() inicializava torsoBegin.z (o YAW do torso, nao a altura) com
    // 0.297509, o valor de zCom: o primeiro passo comandava um giro de ~17
    // graus nas duas hip_yaw. O passo de partida e checado separado por isso.
    std::printf("  theta do torso no 1o tick apos restart(): %.4f rad (%.1f graus)\n",
                yawFirstTick, yawFirstTick * RAD2DEG);
    check("  passo de partida: max |theta pes/torso, hip_yaw| [rad]", maxYawStart, 0.0, 0.0);
    check("  regime: max |theta pes/torso, hip_yaw| [rad]", maxYaw, 0.0, 0.0);

    // --- 2. espelhamento entre passos consecutivos --------------------------
    // Passo com a esquerda em balanco no instante i  <->  passo seguinte, com a
    // direita em balanco, no mesmo instante i, com y trocado de sinal.
    std::printf("\n2. espelhamento esquerda <-> direita (apos %d passos)\n", WARMUP_STEPS);
    double dFootX = 0, dFootY = 0, dPitch = 0, dRoll = 0, dArm = 0;
    int pares = 0;
    for (size_t s = WARMUP_STEPS; s + 1 < perStep.size(); ++s) {
        const auto &a = perStep[s];
        const auto &b = perStep[s + 1];
        if (a.empty() || b.empty() || !a[0].leftSwing || b[0].leftSwing) continue;
        const size_t n = std::min(a.size(), b.size());
        for (size_t i = 0; i < n; ++i) {
            dFootX = std::max({dFootX, std::fabs(a[i].lx - b[i].rx),
                               std::fabs(a[i].rx - b[i].lx)});
            dFootY = std::max({dFootY, std::fabs(a[i].ly + b[i].ry),
                               std::fabs(a[i].ry + b[i].ly)});
            const Joints &ja = a[i].j, &jb = b[i].j;
            dPitch = std::max({dPitch,
                               std::fabs(ja.leftHipPitch - jb.rightHipPitch),
                               std::fabs(ja.leftKneePitch - jb.rightKneePitch),
                               std::fabs(ja.leftAnklePitch - jb.rightAnklePitch)});
            // A IK faz hipRoll = atan2(y, z): espelhar y troca o sinal do roll.
            dRoll = std::max({dRoll,
                              std::fabs(ja.leftHipRoll + jb.rightHipRoll),
                              std::fabs(ja.leftAnkleRoll + jb.rightAnkleRoll)});
            dArm = std::max(dArm, std::fabs(ja.leftShoulderPitch - jb.rightShoulderPitch));
        }
        ++pares;
    }
    std::printf("  pares de passos comparados: %d\n", pares);
    const double TOL = 1e-9;
    check("  max |x_pe(esq, passo k) - x_pe(dir, passo k+1)| [m]", dFootX, 0.0, TOL);
    check("  max |y_pe(esq, passo k) + y_pe(dir, passo k+1)| [m]", dFootY, 0.0, TOL);
    check("  max |pitch/joelho/tornozelo esq - dir| [rad]", dPitch, 0.0, TOL);
    check("  max |roll esq + roll dir| [rad]", dRoll, 0.0, TOL);
    check("  max |ombro esq - ombro dir| [rad]", dArm, 0.0, TOL);

    // --- 3. pegadas em linha reta ------------------------------------------
    // Compoe as pegadas planejadas: cada uma esta no frame do pe de apoio do
    // proprio passo, que e a pegada anterior.
    std::printf("\n3. pegadas compostas no mundo\n");
    // As pegadas alternam em y; o rumo e reto se cada fileira (esq, dir) fica
    // num y constante. Mede a deriva de cada fileira entre o inicio e o fim.
    double x = 0, y = 0, th = 0, maxTh = 0;
    double firstL = NAN, lastL = NAN, firstR = NAN, lastR = NAN;
    for (size_t s = 0; s < steps.size(); ++s) {
        const Vector3<double> &p = steps[s].landing;
        x += std::cos(th) * p.x - std::sin(th) * p.y;
        y += std::sin(th) * p.x + std::cos(th) * p.y;
        th += p.z;
        maxTh = std::max(maxTh, std::fabs(th));
        if (s < WARMUP_STEPS) continue;
        if (steps[s].leftSwing) { if (std::isnan(firstL)) firstL = y; lastL = y; }
        else { if (std::isnan(firstR)) firstR = y; lastR = y; }
    }
    std::printf("  avanco total %.4f m em %d passos (%.4f m/passo)\n", x,
                (int)steps.size(), x / steps.size());
    check("  rumo acumulado max |theta| [graus]", maxTh * RAD2DEG, 0.0, 0.0);
    check("  deriva lateral da fileira esquerda [m]", lastL - firstL, 0.0, TOL);
    check("  deriva lateral da fileira direita [m]", lastR - firstR, 0.0, TOL);
    std::printf("  separacao entre fileiras %.4f m (2 x ySeparation = %.4f)\n",
                lastL - lastR, 2.0 * params.ySeparation);

    // --- 4. partida espelhada (setStartWithLeftSwing) ------------------------
    // Com o esquerdo balancando primeiro, cada tick tem de ser o espelho exato
    // (y -> -y, esquerda <-> direita) da partida padrao, desde o primeiro.
    std::printf("\n4. partida pelo pe esquerdo = espelho da partida padrao\n");
    {
        InverseKinematics ikA(phys), ikB(phys);
        OmnidirectionalWalkZMP a(params, phys, &ikA), b(params, phys, &ikB);
        b.setStartWithLeftSwing(true);
        a.restart();
        b.restart();
        a.setDesiredVelocity(v);
        b.setDesiredVelocity(v);
        Joints ja, jb;
        float xa = 1.0f, xb = 1.0f;
        double dPe = 0, dJunta = 0;
        bool trocou = true;
        for (int k = 0; k < 20 * ticksPerStep; ++k) {
            a.update(DT, ja, xa);
            b.update(DT, jb, xb);
            trocou = trocou && (a.leftFootIsSwing != b.leftFootIsSwing);
            dPe = std::max({dPe, std::fabs(a.leftFoot_.x - b.rightFoot_.x),
                            std::fabs(a.leftFoot_.y + b.rightFoot_.y),
                            std::fabs(a.rightFoot_.x - b.leftFoot_.x),
                            std::fabs(a.rightFoot_.y + b.leftFoot_.y)});
            dJunta = std::max({dJunta,
                               std::fabs(ja.leftHipPitch - jb.rightHipPitch),
                               std::fabs(ja.leftKneePitch - jb.rightKneePitch),
                               std::fabs(ja.leftHipRoll + jb.rightHipRoll),
                               std::fabs(ja.rightHipRoll + jb.leftHipRoll),
                               std::fabs(ja.leftShoulderPitch - jb.rightShoulderPitch),
                               std::fabs(jb.leftHipYaw), std::fabs(jb.rightHipYaw)});
        }
        check("  pe que balanca sempre oposto (1 = sim)", trocou ? 1.0 : 0.0, 1.0, 0.0);
        check("  max |pe padrao - espelho do pe espelhado| [m]", dPe, 0.0, TOL);
        check("  max |junta padrao - espelho da junta espelhada| [rad]", dJunta, 0.0, TOL);
    }

    // --- 5. partida suave (setStartRampSteps) --------------------------------
    std::printf("\n5. partida suave em 4 passos\n");
    {
        const int N = 4;
        InverseKinematics ik0(phys), ik1(phys), ik2(phys);
        OmnidirectionalWalkZMP base(params, phys, &ik0), suave(params, phys, &ik1),
                espelho(params, phys, &ik2);
        suave.setStartRampSteps(N);
        espelho.setStartRampSteps(N);
        espelho.setStartWithLeftSwing(true);
        for (OmnidirectionalWalkZMP *w : {&base, &suave, &espelho}) {
            w->restart();
            w->setDesiredVelocity(v);
        }
        Joints j0, j1, j2;
        float x0 = 1.0f, x1 = 1.0f, x2 = 1.0f;
        double vyBase = 0, vySuave = 0, prevBase = 0, prevSuave = 0;
        double dRegime = 0, dEspelho = 0, zMinPrimeiro = 1e9, zMinBase = 1e9;
        for (int k = 0; k < (N + 6) * ticksPerStep; ++k) {
            base.update(DT, j0, x0);
            suave.update(DT, j1, x1);
            espelho.update(DT, j2, x2);
            // velocidade lateral do torso em relacao ao pe de apoio, 1o passo
            if (k > 0 && k <= 3) {
                vyBase = std::max(vyBase, std::fabs(base.torso.y - prevBase) / DT);
                vySuave = std::max(vySuave, std::fabs(suave.torso.y - prevSuave) / DT);
            }
            prevBase = base.torso.y;
            prevSuave = suave.torso.y;
            if (k < ticksPerStep) {
                zMinPrimeiro = std::min({zMinPrimeiro, suave.zLeft_, suave.zRight_});
                zMinBase = std::min({zMinBase, base.zLeft_, base.zRight_});
            }
            // Terminada a rampa (+2 passos de folga), os pes em relacao ao
            // torso tem de ser os mesmos da marcha sem rampa.
            if (k >= (N + 2) * ticksPerStep) {
                dRegime = std::max({dRegime,
                                    std::fabs(base.leftFoot_.x - suave.leftFoot_.x),
                                    std::fabs(base.leftFoot_.y - suave.leftFoot_.y),
                                    std::fabs(base.rightFoot_.x - suave.rightFoot_.x),
                                    std::fabs(base.rightFoot_.y - suave.rightFoot_.y)});
            }
            dEspelho = std::max({dEspelho,
                                 std::fabs(suave.leftFoot_.x - espelho.rightFoot_.x),
                                 std::fabs(suave.leftFoot_.y + espelho.rightFoot_.y),
                                 std::fabs(suave.zLeft_ - espelho.zRight_)});
        }
        std::printf("  velocidade lateral do torso nos 1os ticks: sem rampa %.3f m/s, "
                    "com rampa %.4f m/s\n", vyBase, vySuave);
        check("  com rampa, |vy| inicial < 5% da sem rampa (1 = sim)",
              vySuave < 0.05 * vyBase ? 1.0 : 0.0, 1.0, 0.0);
        std::printf("  pe oscilante no 1o passo sobe ate %.4f m (sem rampa: %.4f m) abaixo de zCom\n",
                    params.zCom - zMinPrimeiro, params.zCom - zMinBase);
        check("  apos a rampa: max |pe suave - pe sem rampa| [m]", dRegime, 0.0, TOL);
        check("  partida suave espelhada: max |diferenca do espelho| [m]", dEspelho, 0.0, TOL);
    }

    // --- 6. curva a esquerda = espelho da curva a direita ---------------------
    // Pre-requisito da malha de rumo: ela comanda rotationZ dos dois sinais.
    // O teste antigo "velocity.z < DBL_EPSILON" mandava toda rotacao NEGATIVA
    // para a aproximacao de linha reta.
    std::printf("\n6. curva a esquerda (partida dir) = espelho da curva a direita (partida esq)\n");
    {
        InverseKinematics ikE(phys), ikD(phys);
        OmnidirectionalWalkZMP esq(params, phys, &ikE), dir(params, phys, &ikD);
        dir.setStartWithLeftSwing(true);
        esq.restart();
        dir.restart();
        Pose2D ve = v, vd = v;
        ve.rotation = 0.3;
        vd.rotation = -0.3;
        esq.setDesiredVelocity(ve);
        dir.setDesiredVelocity(vd);
        Joints je, jd;
        float xe = 1.0f, xd = 1.0f;
        double dPe = 0, dYaw = 0, maxYawE = 0;
        for (int k = 0; k < 20 * ticksPerStep; ++k) {
            esq.update(DT, je, xe);
            dir.update(DT, jd, xd);
            dPe = std::max({dPe, std::fabs(esq.leftFoot_.x - dir.rightFoot_.x),
                            std::fabs(esq.leftFoot_.y + dir.rightFoot_.y),
                            std::fabs(esq.leftFoot_.z + dir.rightFoot_.z)});
            dYaw = std::max({dYaw, std::fabs(je.leftHipYaw + jd.rightHipYaw),
                             std::fabs(je.rightHipYaw + jd.leftHipYaw)});
            maxYawE = std::max(maxYawE, std::fabs(je.leftHipYaw));
        }
        std::printf("  hip_yaw comandado na curva: ate %.1f graus\n", maxYawE * RAD2DEG);
        check("  max |pe (x, y, theta) - espelho| ", dPe, 0.0, TOL);
        check("  max |hip_yaw - espelho| [rad]", dYaw, 0.0, TOL);
    }

    std::printf("\n%s\n", failures == 0 ? "TODOS OS TESTES PASSARAM"
                                         : "HA FALHAS");
    return failures == 0 ? 0 : 1;
}
