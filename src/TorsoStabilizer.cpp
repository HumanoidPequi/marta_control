/*
 * TorsoStabilizer.cpp
 *
 * Ver TorsoStabilizer.h para a derivacao do sinal da correcao e para a politica
 * de distribuicao entre as pernas.
 */

#include "TorsoStabilizer.h"

#include <algorithm>
#include <cmath>

namespace control {

namespace stabilization {

TorsoStabilizerParams::TorsoStabilizerParams() :
        enabled(true),
        kpPitch(0.0),
        kdPitch(0.0),
        pitchRef(0.0),
        maxPitchCorrection(0.1745),  // 10 graus
        supportBlendTime(0.05) {
}

TorsoStabilizerParams TorsoStabilizerParams::getDefaultTorsoStabilizerParams() {
    TorsoStabilizerParams params;

    // Ganhos iniciais conservadores, para o teste estatico sob perturbacao.
    //
    // Ordem de grandeza: o pendulo invertido equivalente tem
    // omega = sqrt(g / zCom) = sqrt(9.80665 / 0.2975) ~ 5.7 rad/s (~0.91 Hz).
    // Com kp = 0.2, uma inclinacao de 5 graus (0.087 rad) gera 0.017 rad
    // (~1 grau) de correcao no quadril. A razao kd/kp ~ 1/omega ~ 0.17 s
    // colocaria o zero do PD sobre a dinamica do pendulo; comeca-se abaixo
    // disso e sobe-se kd ate o balanco parar de oscilar.
    params.kpPitch = 0.2;
    params.kdPitch = 0.03;
    params.pitchRef = 0.0;
    params.maxPitchCorrection = 0.1745;
    params.supportBlendTime = 0.05;
    params.enabled = true;

    return params;
}

TorsoStabilizer::TorsoStabilizer(const TorsoStabilizerParams &params) :
        params_(params), leftWeight_(0.0), rightWeight_(0.0) {
}

void TorsoStabilizer::reset() {
    state_ = TorsoStabilizerState();
    leftWeight_ = 0.0;
    rightWeight_ = 0.0;
}

double TorsoStabilizer::computePitchCorrection(double pitch, double pitchRate) const {
    // delta = kp * (phi - phi_ref) + kd * phi_dot
    // Positivo gira o tronco para tras; ver a derivacao no cabecalho.
    double delta = params_.kpPitch * (pitch - params_.pitchRef)
            + params_.kdPitch * pitchRate;

    const double limit = std::fabs(params_.maxPitchCorrection);
    return std::max(-limit, std::min(limit, delta));
}

void TorsoStabilizer::blendWeight(double &weight, double target, double dt) const {
    if (params_.supportBlendTime <= 0.0 || dt <= 0.0) {
        weight = target;
        return;
    }

    const double beta = std::min(1.0, dt / params_.supportBlendTime);
    weight += beta * (target - weight);

    // Encosta no alvo para nao ficar assintoticamente perto para sempre.
    if (std::fabs(target - weight) < 1e-4) {
        weight = target;
    }
}

const TorsoStabilizerState &TorsoStabilizer::update(double pitch, double pitchRate,
                                                    walking::SupportState support,
                                                    double dt,
                                                    representations::Joints &joints) {
    if (!params_.enabled) {
        // Desligado: nao toca nas juntas. Os pesos decaem para que, se for
        // religado, a correcao volte pela transicao suave e nao em degrau.
        blendWeight(leftWeight_, 0.0, dt);
        blendWeight(rightWeight_, 0.0, dt);

        state_ = TorsoStabilizerState();
        state_.leftWeight = leftWeight_;
        state_.rightWeight = rightWeight_;
        return state_;
    }

    const double delta = computePitchCorrection(pitch, pitchRate);

    // A perna de apoio recebe a correcao; a oscilante nao, para nao mexer na
    // colocacao do passo planejada pelo gerador de marcha.
    double leftTarget = 0.0;
    double rightTarget = 0.0;
    switch (support) {
        case walking::LEFT_SUPPORT:
            leftTarget = 1.0;
            break;
        case walking::RIGHT_SUPPORT:
            rightTarget = 1.0;
            break;
        case walking::DOUBLE_SUPPORT:
        default:
            leftTarget = 1.0;
            rightTarget = 1.0;
            break;
    }

    blendWeight(leftWeight_, leftTarget, dt);
    blendWeight(rightWeight_, rightTarget, dt);

    const double leftDelta = leftWeight_ * delta;
    const double rightDelta = rightWeight_ * delta;

    joints.leftHipPitch += leftDelta;
    joints.rightHipPitch += rightDelta;

    state_.deltaPitch = delta;
    state_.leftHipDelta = leftDelta;
    state_.rightHipDelta = rightDelta;
    state_.leftWeight = leftWeight_;
    state_.rightWeight = rightWeight_;

    const double raw = params_.kpPitch * (pitch - params_.pitchRef)
            + params_.kdPitch * pitchRate;
    state_.saturated = std::fabs(raw) > std::fabs(params_.maxPitchCorrection);

    return state_;
}

} /* namespace stabilization */

} /* namespace control */
