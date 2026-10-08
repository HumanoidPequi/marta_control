// Verificacao offline do TorsoStabilizer (Fase 2). Nao depende de ROS.
//
// Rodar a partir da raiz do pacote marta_control:
//   g++ -std=c++14 -Iinclude -I/usr/include/eigen3 \
//       test/test_torso_stabilizer.cpp src/TorsoStabilizer.cpp src/Joints.cpp \
//       -o /tmp/test_torso_stabilizer && /tmp/test_torso_stabilizer
//
// Cobre o que mais custa errar: o SINAL da correcao, a saturacao, e a garantia
// de que a perna oscilante nao e tocada.

#include "TorsoStabilizer.h"

#include <cmath>
#include <cstdio>

using control::stabilization::TorsoStabilizer;
using control::stabilization::TorsoStabilizerParams;
using control::stabilization::TorsoStabilizerState;
using control::walking::DOUBLE_SUPPORT;
using control::walking::LEFT_SUPPORT;
using control::walking::RIGHT_SUPPORT;
using representations::Joints;

static const double DT = 0.01;
static const double RAD2DEG = 180.0 / M_PI;

int failures = 0;

void check(const char *name, double got, double expected, double tol) {
    const bool ok = std::fabs(got - expected) <= tol;
    if (!ok) ++failures;
    std::printf("%-56s got=%8.4f  esperado=%8.4f  %s\n", name, got, expected,
                ok ? "OK" : "FALHOU");
}

// Deixa os pesos das pernas assentarem no estado de apoio dado.
void settle(TorsoStabilizer &stab, control::walking::SupportState support) {
    Joints j;
    for (int i = 0; i < 200; ++i) {
        stab.update(0.0, 0.0, support, DT, j);
    }
}

int main() {
    TorsoStabilizerParams base = TorsoStabilizerParams::getDefaultTorsoStabilizerParams();
    std::printf("kp=%.3f  kd=%.3f  saturacao=%.4f rad (%.1f graus)\n\n",
                base.kpPitch, base.kdPitch, base.maxPitchCorrection,
                base.maxPitchCorrection * RAD2DEG);

    // --- 1. SINAL: tombando para frente -> correcao POSITIVA no hip pitch --
    // Somar delta > 0 ao hip pitch gira o tronco para TRAS (ver derivacao no
    // cabecalho de TorsoStabilizer.h), que e o sentido restaurador.
    {
        TorsoStabilizer stab(base);
        settle(stab, DOUBLE_SUPPORT);

        Joints j;
        const double pitch = 5.0 / RAD2DEG;  // 5 graus para frente
        stab.update(pitch, 0.0, DOUBLE_SUPPORT, DT, j);

        check("tombando para frente: delta > 0 [rad]", stab.getState().deltaPitch,
              base.kpPitch * pitch, 1e-9);
        check("tombando para frente: hip pitch esquerdo somado [rad]",
              j.leftHipPitch, base.kpPitch * pitch, 1e-9);
        check("tombando para frente: hip pitch direito somado [rad]",
              j.rightHipPitch, base.kpPitch * pitch, 1e-9);
    }

    // --- 2. SINAL: tombando para tras -> correcao NEGATIVA ----------------
    {
        TorsoStabilizer stab(base);
        settle(stab, DOUBLE_SUPPORT);

        Joints j;
        const double pitch = -5.0 / RAD2DEG;
        stab.update(pitch, 0.0, DOUBLE_SUPPORT, DT, j);

        check("tombando para tras: delta < 0 [rad]", stab.getState().deltaPitch,
              base.kpPitch * pitch, 1e-9);
    }

    // --- 3. Termo derivativo entra com o mesmo sinal ----------------------
    // Caindo para frente a 1 rad/s com o tronco ainda na vertical: a correcao
    // tem que antecipar, nao esperar o angulo aparecer.
    {
        TorsoStabilizer stab(base);
        settle(stab, DOUBLE_SUPPORT);

        Joints j;
        stab.update(0.0, 1.0, DOUBLE_SUPPORT, DT, j);
        check("phi=0 e phi_dot=+1 rad/s: delta = kd [rad]",
              stab.getState().deltaPitch, base.kdPitch, 1e-9);
    }

    // --- 4. Saturacao rigida ---------------------------------------------
    {
        TorsoStabilizer stab(base);
        settle(stab, DOUBLE_SUPPORT);

        Joints j;
        // 60 graus de inclinacao: kp * phi = 0.209 rad, acima do limite 0.1745.
        stab.update(60.0 / RAD2DEG, 0.0, DOUBLE_SUPPORT, DT, j);
        check("inclinacao enorme: delta saturado [rad]", stab.getState().deltaPitch,
              base.maxPitchCorrection, 1e-9);
        check("inclinacao enorme: flag de saturacao", stab.getState().saturated ? 1.0 : 0.0,
              1.0, 1e-9);

        TorsoStabilizer stab2(base);
        settle(stab2, DOUBLE_SUPPORT);
        Joints j2;
        stab2.update(-60.0 / RAD2DEG, 0.0, DOUBLE_SUPPORT, DT, j2);
        check("inclinacao enorme negativa: delta saturado [rad]",
              stab2.getState().deltaPitch, -base.maxPitchCorrection, 1e-9);
    }

    // --- 5. Apoio simples: so a perna de apoio recebe a correcao ----------
    {
        TorsoStabilizer stab(base);
        settle(stab, LEFT_SUPPORT);

        Joints j;
        const double pitch = 5.0 / RAD2DEG;
        stab.update(pitch, 0.0, LEFT_SUPPORT, DT, j);

        check("apoio esquerdo: hip esquerdo corrigido [rad]", j.leftHipPitch,
              base.kpPitch * pitch, 1e-6);
        check("apoio esquerdo: hip direito (oscilante) intocado", j.rightHipPitch,
              0.0, 1e-12);
    }
    {
        TorsoStabilizer stab(base);
        settle(stab, RIGHT_SUPPORT);

        Joints j;
        const double pitch = 5.0 / RAD2DEG;
        stab.update(pitch, 0.0, RIGHT_SUPPORT, DT, j);

        check("apoio direito: hip direito corrigido [rad]", j.rightHipPitch,
              base.kpPitch * pitch, 1e-6);
        check("apoio direito: hip esquerdo (oscilante) intocado", j.leftHipPitch,
              0.0, 1e-12);
    }

    // --- 6. So o quadril e tocado ----------------------------------------
    {
        TorsoStabilizer stab(base);
        settle(stab, DOUBLE_SUPPORT);

        Joints j;
        stab.update(10.0 / RAD2DEG, 0.5, DOUBLE_SUPPORT, DT, j);

        check("joelho esquerdo intocado", j.leftKneePitch, 0.0, 1e-12);
        check("tornozelo esquerdo intocado", j.leftAnklePitch, 0.0, 1e-12);
        check("roll do quadril esquerdo intocado (Fase 3)", j.leftHipRoll, 0.0, 1e-12);
        check("joelho direito intocado", j.rightKneePitch, 0.0, 1e-12);
        check("tornozelo direito intocado", j.rightAnklePitch, 0.0, 1e-12);
    }

    // --- 7. Transicao de apoio nao da degrau na junta que sai -------------
    // Sem a suavizacao de peso, a perna que vira oscilante perderia a correcao
    // inteira em um ciclo. Mede-se o maior salto por ciclo em cada junta.
    {
        TorsoStabilizer stab(base);
        settle(stab, DOUBLE_SUPPORT);

        const double pitch = 8.0 / RAD2DEG;  // inclinacao constante
        double previousLeft = 0.0;
        double biggestJump = 0.0;

        // Alguns ciclos em apoio duplo, depois troca para apoio direito:
        // a perna esquerda passa a oscilante.
        for (int i = 0; i < 5; ++i) {
            Joints j;
            stab.update(pitch, 0.0, DOUBLE_SUPPORT, DT, j);
            previousLeft = j.leftHipPitch;
        }
        for (int i = 0; i < 40; ++i) {
            Joints j;
            stab.update(pitch, 0.0, RIGHT_SUPPORT, DT, j);
            biggestJump = std::max(biggestJump, std::fabs(j.leftHipPitch - previousLeft));
            previousLeft = j.leftHipPitch;
        }

        const double fullCorrection = base.kpPitch * pitch;
        std::printf("   (correcao cheia = %.2f graus; blend = %.3f s)\n",
                    fullCorrection * RAD2DEG, base.supportBlendTime);
        check("troca de apoio: maior salto por ciclo [graus] < 25% da correcao",
              biggestJump * RAD2DEG, 0.1 * fullCorrection * RAD2DEG,
              0.15 * fullCorrection * RAD2DEG);
    }

    // --- 8. Apoio simples estabelecido nao atrasa a correcao --------------
    // A suavizacao age no PESO da perna, nao no valor: com o peso ja em 1, a
    // correcao aparece inteira no mesmo ciclo em que a inclinacao aparece.
    {
        TorsoStabilizer stab(base);
        settle(stab, LEFT_SUPPORT);

        Joints j;
        const double pitch = 5.0 / RAD2DEG;
        stab.update(pitch, 0.0, LEFT_SUPPORT, DT, j);
        check("apoio simples assentado: correcao sem atraso [rad]", j.leftHipPitch,
              base.kpPitch * pitch, 1e-9);
    }

    // --- 9. Desligado nao toca nas juntas ---------------------------------
    {
        TorsoStabilizerParams off = base;
        off.enabled = false;
        TorsoStabilizer stab(off);

        Joints j;
        for (int i = 0; i < 100; ++i) {
            stab.update(10.0 / RAD2DEG, 1.0, DOUBLE_SUPPORT, DT, j);
        }
        check("desligado: hip esquerdo intocado", j.leftHipPitch, 0.0, 1e-12);
        check("desligado: hip direito intocado", j.rightHipPitch, 0.0, 1e-12);
    }

    // --- 10. Convergencia: malha fechada com um pendulo invertido simples --
    // Modelo grosseiro so para checar estabilidade do sinal, nao fidelidade:
    //   phi_ddot = (g/z) * phi - (g/z) * delta
    // O segundo termo representa o efeito restaurador do deslocamento do CoM
    // provocado pela rotacao do tronco. Com o sinal correto, o erro decai; com
    // o sinal trocado, diverge.
    {
        const double g = 9.80665;
        const double z = 0.2975;
        const double w2 = g / z;

        TorsoStabilizerParams p = base;
        p.kpPitch = 2.0;   // ganho de simulacao, acima do inicial de bancada
        p.kdPitch = 0.5;

        TorsoStabilizer stab(p);
        settle(stab, DOUBLE_SUPPORT);

        double phi = 5.0 / RAD2DEG;
        double phiDot = 0.0;
        double peak = std::fabs(phi);

        for (int i = 0; i < 1000; ++i) {
            Joints j;
            const double delta = stab.update(phi, phiDot, DOUBLE_SUPPORT, DT, j).deltaPitch;
            const double phiDDot = w2 * phi - w2 * delta;
            phiDot += phiDDot * DT;
            phi += phiDot * DT;
            peak = std::max(peak, std::fabs(phi));
        }

        std::printf("   (phi inicial 5.00 graus, pico %.2f graus, final %.3f graus)\n",
                    peak * RAD2DEG, phi * RAD2DEG);
        check("malha fechada converge para a vertical [graus]", phi * RAD2DEG, 0.0, 0.5);
    }

    std::printf("\n%s (%d falha(s))\n", failures == 0 ? "TODOS OS TESTES PASSARAM"
                                                      : "HOUVE FALHAS", failures);
    return failures == 0 ? 0 : 1;
}
