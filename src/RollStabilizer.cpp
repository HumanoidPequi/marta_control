/*
 * RollStabilizer.cpp
 *
 * Ver RollStabilizer.h para a lei de controle, o washout e a convencao de sinal.
 */

#include "RollStabilizer.h"

#include <algorithm>
#include <cmath>

namespace control {

namespace stabilization {

RollStabilizerParams::RollStabilizerParams() :
        enabled(false), kpHip(0.0), kdHip(0.0), kpAnkle(0.0), kdAnkle(0.0),
        washoutTau(2.0), maxCorrection(0.14) {
}

RollStabilizer::RollStabilizer(const RollStabilizerParams &params) :
        params_(params), initialized_(false) {
}

const RollStabilizerState &RollStabilizer::update(double roll, double rollRate,
        double wLeft, double wRight, double dt, representations::Joints &joints) {
    // Washout: passa-baixa de primeira ordem; phi - phi_m e o passa-alta.
    if (!initialized_) {
        state_.rollMean = roll;
        initialized_ = true;
    } else if (params_.washoutTau > 0.0 && dt > 0.0) {
        const double a = std::min(1.0, dt / params_.washoutTau);
        state_.rollMean += a * (roll - state_.rollMean);
    }
    state_.error = roll - state_.rollMean;
    state_.leftWeight = wLeft;
    state_.rightWeight = wRight;

    if (!params_.enabled) {
        state_.uHip = 0.0;
        state_.uAnkle = 0.0;
        state_.saturated = false;
        return state_;
    }

    const double lim = std::fabs(params_.maxCorrection);
    const double uh = params_.kpHip * state_.error + params_.kdHip * rollRate;
    const double ua = params_.kpAnkle * state_.error + params_.kdAnkle * rollRate;
    state_.uHip = std::max(-lim, std::min(lim, uh));
    state_.uAnkle = std::max(-lim, std::min(lim, ua));
    state_.saturated = std::fabs(uh) > lim || std::fabs(ua) > lim;

    joints.leftHipRoll += wLeft * state_.uHip;
    joints.leftAnkleRoll += wLeft * state_.uAnkle;
    joints.rightHipRoll += wRight * state_.uHip;
    joints.rightAnkleRoll += wRight * state_.uAnkle;
    return state_;
}

void RollStabilizer::reset() {
    initialized_ = false;
    state_ = RollStabilizerState();
}

} /* namespace stabilization */

} /* namespace control */
