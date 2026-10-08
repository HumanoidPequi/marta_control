// Verificacao offline das convencoes de sinal e da resposta dos filtros do
// TorsoAttitudeEstimator. Nao depende de ROS nem de catkin: e o documento
// executavel da convencao adotada na Fase 1.
//
// Rodar a partir da raiz do pacote marta_control:
//   g++ -std=c++14 -Iinclude -I/usr/include/eigen3 \
//       test/test_torso_attitude.cpp src/TorsoAttitudeEstimator.cpp \
//       -o /tmp/test_torso_attitude && /tmp/test_torso_attitude
//
// Alem de passar/falhar, imprime duas tabelas usadas para escolher os ganhos:
// o vazamento de aceleracao do corpo para o pitch em funcao de alpha, e a
// atenuacao do passa-baixa do giroscopio em funcao da frequencia.

#include "TorsoAttitudeEstimator.h"

#include <cmath>
#include <cstdio>
#include <vector>

using control::estimation::TorsoAttitudeEstimator;
using control::estimation::TorsoAttitudeParams;

static const double G = 9.80665;
static const double DT = 0.01;
static const double RAD2DEG = 180.0 / M_PI;

int failures = 0;

void check(const char *name, double got, double expected, double tol) {
    const bool ok = std::fabs(got - expected) <= tol;
    if (!ok) ++failures;
    std::printf("%-52s got=%8.3f  esperado=%8.3f  %s\n", name, got, expected,
                ok ? "OK" : "FALHOU");
}

// Aceleracao especifica no frame do corpo para uma inclinacao (pitch, roll).
Eigen::Vector3d accelBody(double pitch, double roll) {
    return Eigen::Vector3d(-G * std::sin(pitch),
                           G * std::sin(roll),
                           G * std::cos(pitch) * std::cos(roll));
}

// Leva um vetor do frame do corpo para o frame do sensor: v_imu = R^T * v_body.
Eigen::Vector3d toImu(const TorsoAttitudeParams &p, const Eigen::Vector3d &v) {
    return p.imuToBody.transpose() * v;
}

int main() {
    TorsoAttitudeParams params = TorsoAttitudeParams::getDefaultTorsoAttitudeParams();

    std::printf("R_bodyImu =\n");
    for (int i = 0; i < 3; ++i) {
        std::printf("  [%6.2f %6.2f %6.2f]\n", params.imuToBody(i, 0),
                    params.imuToBody(i, 1), params.imuToBody(i, 2));
    }
    std::printf("\n");

    // --- 1. Em pe, na vertical -------------------------------------------
    {
        TorsoAttitudeEstimator est(params);
        Eigen::Vector3d a = toImu(params, accelBody(0.0, 0.0));
        for (int i = 0; i < 500; ++i) est.update(a, Eigen::Vector3d::Zero(), DT);
        check("vertical: pitch [deg]", est.getAttitude().pitch * RAD2DEG, 0.0, 1e-6);
        check("vertical: roll  [deg]", est.getAttitude().roll * RAD2DEG, 0.0, 1e-6);
    }

    // --- 2. Estatico, inclinado 10 graus para FRENTE -> pitch > 0 ---------
    {
        TorsoAttitudeEstimator est(params);
        Eigen::Vector3d a = toImu(params, accelBody(10.0 / RAD2DEG, 0.0));
        for (int i = 0; i < 500; ++i) est.update(a, Eigen::Vector3d::Zero(), DT);
        check("inclinado 10 deg para frente: pitch [deg]",
              est.getAttitude().pitch * RAD2DEG, 10.0, 0.05);
        check("inclinado 10 deg para frente: roll [deg]",
              est.getAttitude().roll * RAD2DEG, 0.0, 0.05);
    }

    // --- 3. Estatico, inclinado 10 graus para a DIREITA -> roll > 0 -------
    {
        TorsoAttitudeEstimator est(params);
        Eigen::Vector3d a = toImu(params, accelBody(0.0, 10.0 / RAD2DEG));
        for (int i = 0; i < 500; ++i) est.update(a, Eigen::Vector3d::Zero(), DT);
        check("inclinado 10 deg para a direita: roll [deg]",
              est.getAttitude().roll * RAD2DEG, 10.0, 0.05);
        check("inclinado 10 deg para a direita: pitch [deg]",
              est.getAttitude().pitch * RAD2DEG, 0.0, 0.05);
    }

    // --- 4. Giroscopio: tombando para frente a 1 rad/s -> pitchRate > 0 ---
    {
        TorsoAttitudeEstimator est(params);
        Eigen::Vector3d w = toImu(params, Eigen::Vector3d(0.0, 1.0, 0.0));
        Eigen::Vector3d a = toImu(params, accelBody(0.0, 0.0));
        for (int i = 0; i < 500; ++i) est.update(a, w, DT);
        check("caindo para frente: pitch_rate [rad/s]",
              est.getAttitude().pitchRate, 1.0, 0.01);
    }

    // --- 5. Giroscopio: tombando para a direita a 1 rad/s -> rollRate > 0 -
    {
        TorsoAttitudeEstimator est(params);
        Eigen::Vector3d w = toImu(params, Eigen::Vector3d(1.0, 0.0, 0.0));
        Eigen::Vector3d a = toImu(params, accelBody(0.0, 0.0));
        for (int i = 0; i < 500; ++i) est.update(a, w, DT);
        check("caindo para a direita: roll_rate [rad/s]",
              est.getAttitude().rollRate, 1.0, 0.01);
    }

    // --- 6. Quanto de aceleracao do corpo vaza para a estimativa ----------
    // 0.5 s de aceleracao horizontal sustentada de 3 m/s^2 com o tronco na
    // vertical: o acelerometro sozinho leria ~17 deg de pitch falso. O quanto
    // disso passa depende de alpha, via tau = alpha*dt/(1-alpha).
    // Isso dimensiona o alpha necessario para a marcha, onde o balanco lateral
    // do LIPM gera aceleracao sustentada a cada passo (periodo 0.4 s).
    {
        const double accelOnly = std::atan2(3.0, G) * RAD2DEG;
        std::printf("   (acelerometro puro leria %.1f deg de pitch falso)\n", accelOnly);

        const double alphas[] = {0.98, 0.99, 0.995, 0.998};
        for (double alpha : alphas) {
            TorsoAttitudeParams p = params;
            p.alpha = alpha;
            const double tau = alpha * DT / (1.0 - alpha);

            TorsoAttitudeEstimator est(p);
            Eigen::Vector3d rest = toImu(p, accelBody(0.0, 0.0));
            for (int i = 0; i < 1000; ++i) est.update(rest, Eigen::Vector3d::Zero(), DT);

            Eigen::Vector3d disturbed = toImu(p, Eigen::Vector3d(-3.0, 0.0, G));
            for (int i = 0; i < 50; ++i) est.update(disturbed, Eigen::Vector3d::Zero(), DT);

            const double leaked = est.getAttitude().pitch * RAD2DEG;
            const double theory = accelOnly * (1.0 - std::exp(-0.5 / tau));
            std::printf("   alpha=%.3f  tau=%.2f s  ->  vazou %5.2f deg "
                        "(%4.1f%% do falso; teoria %.2f deg)\n",
                        alpha, tau, leaked, 100.0 * leaked / accelOnly, theory);
            check("  vazamento bate com o 1a ordem teorico", leaked, theory, 0.3);
        }
    }

    // --- 7. Portao de confianca: impacto forte e ignorado -----------------
    {
        TorsoAttitudeEstimator est(params);
        Eigen::Vector3d rest = toImu(params, accelBody(0.0, 0.0));
        for (int i = 0; i < 500; ++i) est.update(rest, Eigen::Vector3d::Zero(), DT);

        Eigen::Vector3d impact = toImu(params, Eigen::Vector3d(-15.0, 0.0, G));
        est.update(impact, Eigen::Vector3d::Zero(), DT);
        check("impacto (norma fora da faixa): aceito? 0=nao",
              est.lastAccelAccepted() ? 1.0 : 0.0, 0.0, 1e-9);
        check("impacto: pitch permanece [deg]",
              est.getAttitude().pitch * RAD2DEG, 0.0, 1e-6);
    }

    // --- 8. Atenuacao do Butterworth de 2a ordem do giro (fc = 15 Hz) -----
    // 50 Hz e Nyquist com amostragem a 100 Hz: nao da para testar (alias para
    // DC). O que o filtro cobre e a faixa util, ate ~40 Hz.
    // Referencia: um polo so daria 100/91/59/38/33% nestas mesmas frequencias.
    {
        Eigen::Vector3d a0 = accelBody(0.0, 0.0);
        const double freqs[] = {1.0, 5.0, 15.0, 30.0, 40.0};
        for (double f : freqs) {
            TorsoAttitudeEstimator est(params);
            double peak = 0.0;
            for (int i = 0; i < 2000; ++i) {
                const double vib = 2.0 * std::sin(2.0 * M_PI * f * i * DT);
                Eigen::Vector3d w = toImu(params, Eigen::Vector3d(0.0, vib, 0.0));
                est.update(toImu(params, a0), w, DT);
                if (i > 1000) peak = std::max(peak, std::fabs(est.getAttitude().pitchRate));
            }
            std::printf("   vibracao a %4.1f Hz (amplitude 2.00 rad/s) -> pico "
                        "%.3f rad/s  (%.0f%% passa)\n", f, peak, 100.0 * peak / 2.0);
        }
        // Atraso introduzido na banda de passagem: e ele que come margem de
        // fase do ganho derivativo do estabilizador. Medido a 2 Hz, proximo da
        // dinamica do pendulo invertido (omega = sqrt(g/z) ~ 0.91 Hz).
        {
            TorsoAttitudeEstimator est(params);
            const double f = 2.0;
            double bestLag = 0.0;
            double bestErr = 1e18;
            std::vector<double> in, out;
            for (int i = 0; i < 2000; ++i) {
                const double vib = 2.0 * std::sin(2.0 * M_PI * f * i * DT);
                Eigen::Vector3d w = toImu(params, Eigen::Vector3d(0.0, vib, 0.0));
                est.update(toImu(params, a0), w, DT);
                if (i > 1000) {
                    in.push_back(vib);
                    out.push_back(est.getAttitude().pitchRate);
                }
            }
            for (int lag = 0; lag < 25; ++lag) {
                double err = 0.0;
                for (size_t i = lag; i < in.size(); ++i) {
                    const double d = out[i] - in[i - lag];
                    err += d * d;
                }
                err /= (in.size() - lag);
                if (err < bestErr) {
                    bestErr = err;
                    bestLag = lag;
                }
            }
            std::printf("   atraso na banda de passagem (2 Hz): %.0f amostras "
                        "= %.1f ms\n", bestLag, bestLag * DT * 1000.0);
        }

        // Ancora: em 1 Hz o sinal util tem que passar praticamente inteiro.
        {
            TorsoAttitudeEstimator est(params);
            double peak = 0.0;
            for (int i = 0; i < 2000; ++i) {
                const double vib = 2.0 * std::sin(2.0 * M_PI * 1.0 * i * DT);
                Eigen::Vector3d w = toImu(params, Eigen::Vector3d(0.0, vib, 0.0));
                est.update(toImu(params, a0), w, DT);
                if (i > 1000) peak = std::max(peak, std::fabs(est.getAttitude().pitchRate));
            }
            check("sinal util a 1 Hz passa (>90%)", peak, 2.0, 0.2);
        }
    }

    // --- Yaw por integracao do giroscopio --------------------------------
    // Trava o SINAL (girar para a esquerda = +) atraves da montagem invertida
    // do sensor, e a correcao pelo roll: balancar em roll com o giro so em x
    // nao pode virar yaw.
    std::printf("\n-- yaw (integracao do giroscopio) --\n");
    {
        const Eigen::Vector3d a0 = toImu(params, accelBody(0.0, 0.0));
        TorsoAttitudeEstimator est(params);
        // 0,2 rad/s para a ESQUERDA (em torno de +z do corpo) por 5 s.
        for (int i = 0; i < 500; ++i) {
            est.update(a0, toImu(params, Eigen::Vector3d(0.0, 0.0, 0.2)), DT);
        }
        check("giro a esquerda 0,2 rad/s por 5 s -> yaw [rad]", est.getAttitude().yaw,
              1.0, 0.01);
        check("taxa de yaw [rad/s]", est.getAttitude().yawRate, 0.2, 1e-6);

        TorsoAttitudeEstimator est2(params);
        // Balanco em roll (+-10 graus, 1,25 Hz) sem giro em z: yaw ~ 0.
        for (int i = 0; i < 1000; ++i) {
            const double tt = i * DT;
            const double roll = 0.17 * std::sin(2.0 * M_PI * 1.25 * tt);
            const double rollRate = 0.17 * 2.0 * M_PI * 1.25 * std::cos(2.0 * M_PI * 1.25 * tt);
            est2.update(toImu(params, accelBody(0.0, roll)),
                        toImu(params, Eigen::Vector3d(rollRate, 0.0, 0.0)), DT);
        }
        check("balanco em roll nao vira yaw [rad]", est2.getAttitude().yaw, 0.0, 0.01);
    }

    std::printf("\n%s (%d falha(s))\n", failures == 0 ? "TODOS OS TESTES PASSARAM"
                                                      : "HOUVE FALHAS", failures);
    return failures == 0 ? 0 : 1;
}
