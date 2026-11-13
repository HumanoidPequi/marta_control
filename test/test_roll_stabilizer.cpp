// Verificacao offline do RollStabilizer (docs/plano_fase3_roll.md par. 4.4).
//
// Compilado pelo catkin_make (alvo test_roll_stabilizer). Rodar:
//   rosrun marta_controller test_roll_stabilizer
//
// 1. Sinal: roll > 0 (tombando para a direita) gera +delta no hip roll e no
//    ankle roll da perna de apoio (RollStabilizer.h, SINAL).
// 2. A perna em balanco (peso 0) nao e tocada; o peso escala a correcao.
// 3. Saturacao por junta e flag de saturacao.
// 4. Washout: remove um vies constante e preserva a alternancia por passo
//    (0,83 Hz com periodo de passo de 0,6 s) com ganho > 0,95.
// 5. Desligado: nao toca nas juntas, mas o washout continua acompanhando.

#include <cmath>
#include <cstdio>

#include "RollStabilizer.h"

using control::stabilization::RollStabilizer;
using control::stabilization::RollStabilizerParams;
using representations::Joints;

static int failures = 0;

static void check(bool ok, const char *what, double got, double want) {
    std::printf("  [%s] %-50s %+.6f (esperado %+.6f)\n", ok ? " ok " : "FALHA", what, got, want);
    if (!ok)
        ++failures;
}

static void near(double got, double want, double tol, const char *what) {
    check(std::fabs(got - want) <= tol, what, got, want);
}

static RollStabilizerParams ganhos(double kph, double kdh, double kpa, double kda) {
    RollStabilizerParams p;
    p.enabled = true;
    p.kpHip = kph;
    p.kdHip = kdh;
    p.kpAnkle = kpa;
    p.kdAnkle = kda;
    p.washoutTau = 2.0;
    p.maxCorrection = 0.14;
    return p;
}

int main() {
    const double dt = 0.01;

    std::printf("1. Sinal\n");
    {
        RollStabilizer rs(ganhos(0.5, 0.0, 0.5, 0.0));
        Joints j;
        rs.update(0.0, 0.0, 1.0, 0.0, dt, j);          // washout comeca em 0
        Joints k;
        rs.update(0.02, 0.0, 1.0, 0.0, dt, k);         // tomba para a direita
        check(k.leftHipRoll > 0.0, "hip roll de apoio > 0 com roll > 0", k.leftHipRoll, 0.01);
        check(k.leftAnkleRoll > 0.0, "ankle roll de apoio > 0 com roll > 0", k.leftAnkleRoll, 0.01);
        Joints m;
        RollStabilizer rd(ganhos(0.0, 0.1, 0.0, 0.0));
        rd.update(0.0, 0.0, 1.0, 0.0, dt, m);
        Joints n;
        rd.update(0.0, 0.3, 1.0, 0.0, dt, n);
        near(n.leftHipRoll, 0.03, 1e-12, "termo D: kd * roll_rate");
    }

    std::printf("2. Perna em balanco e pesos\n");
    {
        RollStabilizer rs(ganhos(1.0, 0.0, 0.5, 0.0));
        Joints j;
        rs.update(0.0, 0.0, 1.0, 0.0, dt, j);
        Joints k;
        rs.update(0.05, 0.0, 1.0, 0.0, dt, k);
        near(k.rightHipRoll, 0.0, 0.0, "hip roll da perna em balanco intocado");
        near(k.rightAnkleRoll, 0.0, 0.0, "ankle roll da perna em balanco intocado");
        RollStabilizer rp(ganhos(1.0, 0.0, 0.0, 0.0));
        Joints a;
        rp.update(0.0, 0.0, 0.5, 0.5, dt, a);
        Joints b;
        const auto &s = rp.update(0.04, 0.0, 0.5, 0.5, dt, b);
        near(b.leftHipRoll, 0.5 * s.uHip, 1e-12, "apoio duplo: metade em cada perna");
        near(b.rightHipRoll, 0.5 * s.uHip, 1e-12, "apoio duplo: perna direita igual");
    }

    std::printf("3. Saturacao\n");
    {
        RollStabilizer rs(ganhos(10.0, 0.0, 10.0, 0.0));
        Joints j;
        rs.update(0.0, 0.0, 1.0, 0.0, dt, j);
        Joints k;
        const auto &s = rs.update(0.3, 0.0, 1.0, 0.0, dt, k);
        near(k.leftHipRoll, 0.14, 1e-12, "hip roll saturado em max_roll_correction");
        near(k.leftAnkleRoll, 0.14, 1e-12, "ankle roll saturado");
        check(s.saturated, "flag de saturacao", s.saturated ? 1.0 : 0.0, 1.0);
    }

    std::printf("4. Washout (tau = 2 s)\n");
    {
        RollStabilizer rs(ganhos(1.0, 0.0, 0.0, 0.0));
        double e = 0.0;
        for (int i = 0; i < 3000; ++i) {                 // 30 s de vies constante
            Joints j;
            e = rs.update(0.04, 0.0, 1.0, 0.0, dt, j).error;
        }
        near(e, 0.0, 0.04 * 0.01, "vies constante de 0,04 rad removido (< 1%)");

        // Alternancia por passo: periodo de passada 1,2 s (passo de 0,6 s).
        RollStabilizer ra(ganhos(1.0, 0.0, 0.0, 0.0));
        const double f = 1.0 / 1.2, amp = 0.05;
        double maxE = 0.0, minE = 0.0;
        for (int i = 0; i < 4000; ++i) {                 // 40 s; mede nos ultimos 10 s
            const double t = i * dt;
            Joints j;
            const double err = ra.update(0.04 + amp * std::sin(2 * M_PI * f * t), 0.0,
                                         1.0, 0.0, dt, j).error;
            if (t > 30.0) {
                maxE = std::max(maxE, err);
                minE = std::min(minE, err);
            }
        }
        const double ganho = 0.5 * (maxE - minE) / amp;
        check(ganho > 0.95, "alternancia a 0,83 Hz preservada (ganho > 0,95)", ganho, 1.0);
        near(0.5 * (maxE + minE), 0.0, 0.002, "media do erro ~0 (vies removido)");
    }

    std::printf("5. Desligado\n");
    {
        RollStabilizerParams p = ganhos(1.0, 0.1, 1.0, 0.1);
        p.enabled = false;
        RollStabilizer rs(p);
        double media = 0.0;
        for (int i = 0; i < 1000; ++i) {
            Joints j;
            const auto &s = rs.update(0.03, 0.2, 1.0, 0.0, dt, j);
            media = s.rollMean;
            if (j.leftHipRoll != 0.0 || j.leftAnkleRoll != 0.0) {
                check(false, "desligado nao toca nas juntas", j.leftHipRoll, 0.0);
                break;
            }
        }
        near(media, 0.03, 1e-9, "washout acompanha mesmo desligado");
    }

    std::printf("\n%s: %d falha(s)\n", failures ? "FALHOU" : "OK", failures);
    return failures ? 1 : 0;
}
