// Verificacao offline da POSTURA e do BALANCO dos bracos no OmnidirectionalWalkZMP
// (docs/bracos.md). Nao depende de ROS.
//
// Rodar a partir da raiz do pacote marta_control:
//   g++ -std=c++14 -Iinclude -I/usr/include/eigen3 \
//       test/test_arm_posture.cpp src/OmnidirectionalWalkZMP.cpp \
//       src/InverseKinematics.cpp src/RobotPhysicalParameters.cpp \
//       src/Joints.cpp src/MathUtils.cpp src/LinearInterpolator.cpp \
//       -o /tmp/test_arm_posture && /tmp/test_arm_posture
//
// Trava:
//   1. defaults = comportamento historico (codigo ate nov/2026): ombro roll
//      -50/+50 graus, cotovelos 0, pitch = -asin(x do pe oposto / armLength);
//   2. postura configurada: roll, pitch fixo e flexao do cotovelo chegam as
//      juntas com os sinais do URDF, espelhados entre os lados;
//   3. s = 0 desliga o balanco (pitch constante); s = 2 dobra a amplitude;
//   4. balanco em antifase: o braco esquerdo vai para frente quando o pe
//      direito vai para frente.

#include "OmnidirectionalWalkZMP.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

using control::walking::InverseKinematics;
using control::walking::OmnidirectionalWalkZMP;
using control::walking::OmnidirectionalWalkZMPParams;
using itandroids_lib::math::Pose2D;
using representations::Joints;
using representations::RobotPhysicalParameters;

static const double DT = 0.01;
static const double D2R = M_PI / 180.0;
static const int TICKS = 3000;   // 30 s
static const int WARMUP = 300;   // 3 s

int failures = 0;

void check(const char *name, double got, double expected, double tol) {
    const bool ok = std::fabs(got - expected) <= tol;
    if (!ok) ++failures;
    std::printf("%-60s got=%11.4e  esperado=%11.4e  %s\n", name, got, expected,
                ok ? "OK" : "FALHOU");
}

struct Resumo {
    double maxRollErr = 0, maxElbowL = 0, maxElbowR = 0;
    double minPitchL = 1e9, maxPitchL = -1e9, minPitchR = 1e9, maxPitchR = -1e9;
    double maxFormulaErr = 0;   // so no caso default
    double corrAnti = 0;        // soma de pitchL * xPeDireito (antifase => < 0)
};

Resumo roda(const OmnidirectionalWalkZMPParams &params, double vx,
            double rollEsperado, double elbowEsperado, bool conferirFormula) {
    RobotPhysicalParameters phys = RobotPhysicalParameters::getMartaPhysicalParameters();
    InverseKinematics ik(phys);
    OmnidirectionalWalkZMP walker(params, phys, &ik);
    Pose2D v;
    v.translation.x = vx;
    v.translation.y = 0.0;
    v.rotation = 0.0;
    walker.setDesiredVelocity(v);
    Joints j;
    float xscale = 1.0f;
    Resumo r;
    for (int k = 0; k < TICKS; ++k) {
        walker.update(DT, j, xscale);
        if (k < WARMUP) continue;
        r.maxRollErr = std::max({r.maxRollErr, std::fabs(j.leftShoulderRoll + rollEsperado),
                                 std::fabs(j.rightShoulderRoll - rollEsperado)});
        r.maxElbowL = std::max(r.maxElbowL, std::fabs(j.leftElbowYaw + elbowEsperado));
        r.maxElbowR = std::max(r.maxElbowR, std::fabs(j.rightElbowYaw + elbowEsperado));
        r.minPitchL = std::min(r.minPitchL, j.leftShoulderPitch);
        r.maxPitchL = std::max(r.maxPitchL, j.leftShoulderPitch);
        r.minPitchR = std::min(r.minPitchR, j.rightShoulderPitch);
        r.maxPitchR = std::max(r.maxPitchR, j.rightShoulderPitch);
        r.corrAnti += j.leftShoulderPitch * walker.rightFoot_.x;
        if (conferirFormula) {
            // Formula historica, literal (OmnidirectionalWalkZMP.cpp ate nov/2026).
            const double hl = -std::asin(walker.rightFoot_.x * 1.0 / phys.armLength);
            const double hr = -std::asin(walker.leftFoot_.x * 1.0 / phys.armLength);
            r.maxFormulaErr = std::max({r.maxFormulaErr, std::fabs(j.leftShoulderPitch - hl),
                                        std::fabs(j.rightShoulderPitch - hr)});
        }
    }
    return r;
}

int main() {
    const OmnidirectionalWalkZMPParams def =
            OmnidirectionalWalkZMPParams::getDefaultOmnidirecionalWalkZMPParams();

    std::printf("1. defaults = comportamento historico (vx 0,1)\n");
    check("  default armShoulderRoll [rad]", def.armShoulderRoll, 50.0 * D2R, 1e-12);
    check("  default armShoulderPitchOffset [rad]", def.armShoulderPitchOffset, 0.0, 0.0);
    check("  default armElbowFlexion [rad]", def.armElbowFlexion, 0.0, 0.0);
    check("  default armsCompensationFactor", def.armsCompensationFactor, 1.0, 0.0);
    Resumo d = roda(def, 0.1, 50.0 * D2R, 0.0, true);
    check("  max |roll - (-50/+50 graus)| [rad]", d.maxRollErr, 0.0, 1e-12);
    check("  max |cotovelo esq| [rad]", d.maxElbowL, 0.0, 0.0);
    check("  max |cotovelo dir| [rad]", d.maxElbowR, 0.0, 0.0);
    check("  max |pitch - formula historica| [rad]", d.maxFormulaErr, 0.0, 1e-12);
    const double ampDef = 0.5 * (d.maxPitchL - d.minPitchL);
    std::printf("  amplitude do balanco (s = 1): +-%.2f graus\n", ampDef / D2R);
    check("  balanco existe (amplitude > 1 grau) [rad]", ampDef > 1.0 * D2R ? 1.0 : 0.0, 1.0, 0.0);
    check("  antifase: soma(pitch_esq * x_pe_dir) < 0", d.corrAnti < 0 ? 1.0 : 0.0, 1.0, 0.0);

    std::printf("\n2. postura contraida: roll 15, pitch +10, cotovelo 90 graus, s = 0\n");
    OmnidirectionalWalkZMPParams p = def;
    p.armShoulderRoll = 15.0 * D2R;
    p.armShoulderPitchOffset = 10.0 * D2R;
    p.armElbowFlexion = 90.0 * D2R;
    p.armsCompensationFactor = 0.0;
    Resumo c = roda(p, 0.1, 15.0 * D2R, 90.0 * D2R, false);
    check("  max |roll - (-15/+15 graus)| [rad]", c.maxRollErr, 0.0, 1e-12);
    check("  max |cotovelo esq - (-90 graus)| [rad]", c.maxElbowL, 0.0, 1e-12);
    check("  max |cotovelo dir - (-90 graus)| [rad]", c.maxElbowR, 0.0, 1e-12);
    check("  s = 0: pitch esq constante (max - min) [rad]", c.maxPitchL - c.minPitchL, 0.0, 1e-12);
    check("  s = 0: pitch dir constante (max - min) [rad]", c.maxPitchR - c.minPitchR, 0.0, 1e-12);
    check("  pitch fixo esq = -10 graus [rad]", c.maxPitchL, -10.0 * D2R, 1e-12);
    check("  pitch fixo dir = -10 graus [rad]", c.maxPitchR, -10.0 * D2R, 1e-12);

    std::printf("\n3. s = 2 dobra a amplitude do balanco\n");
    OmnidirectionalWalkZMPParams p2 = def;
    p2.armsCompensationFactor = 2.0;
    Resumo s2 = roda(p2, 0.1, 50.0 * D2R, 0.0, false);
    const double amp2 = 0.5 * (s2.maxPitchL - s2.minPitchL);
    std::printf("  amplitude s = 2: +-%.2f graus\n", amp2 / D2R);
    // asin nao e linear: o esperado exato e asin(2 sin(A1)) / A1 (2,02 em +-8 graus).
    check("  razao de amplitude s2/s1 (exata, com o asin)", amp2 / ampDef,
          std::asin(2.0 * std::sin(ampDef)) / ampDef, 1e-3);

    std::printf("\n%s (%d falha%s)\n", failures ? "HA FALHAS" : "TODOS OK", failures,
                failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
