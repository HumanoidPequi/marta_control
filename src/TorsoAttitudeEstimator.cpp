/*
 * TorsoAttitudeEstimator.cpp
 *
 * Ver TorsoAttitudeEstimator.h para a descricao do filtro e das convencoes de
 * sinal.
 */

#include "TorsoAttitudeEstimator.h"

#include <cmath>

namespace control {

namespace estimation {

// ─── Butterworth de 2a ordem ────────────────────────────────────────────────

Butterworth2::Butterworth2() :
        b0_(1.0), b1_(0.0), b2_(0.0), a1_(0.0), a2_(0.0),
        x1_(0.0), x2_(0.0), y1_(0.0), y2_(0.0),
        designCutoffHz_(0.0), designDt_(0.0),
        designed_(false), primed_(false) {
}

bool Butterworth2::needsDesign(double cutoffHz, double dt) const {
    if (!designed_) {
        return true;
    }
    if (cutoffHz != designCutoffHz_) {
        return true;
    }
    // Redesenha so quando dt sai mais de 5% do valor de projeto: o jitter normal
    // do laco nao justifica recalcular coeficientes a cada amostra.
    return std::fabs(dt - designDt_) > 0.05 * designDt_;
}

void Butterworth2::design(double cutoffHz, double dt) {
    designCutoffHz_ = cutoffHz;
    designDt_ = dt;
    designed_ = true;

    // Nao adianta pedir corte acima de Nyquist: o que passou de fs/2 ja chegou
    // aliasado na aquisicao e nenhum filtro digital recupera.
    const double nyquist = 0.5 / dt;
    double fc = cutoffHz;
    if (fc > 0.45 / dt) {
        fc = 0.45 / dt;
    }
    (void) nyquist;

    // Transformada bilinear com pre-distorcao: K = tan(pi * fc * dt).
    const double K = std::tan(M_PI * fc * dt);
    const double K2 = K * K;
    const double norm = 1.0 / (1.0 + M_SQRT2 * K + K2);

    b0_ = K2 * norm;
    b1_ = 2.0 * b0_;
    b2_ = b0_;
    a1_ = 2.0 * (K2 - 1.0) * norm;
    a2_ = (1.0 - M_SQRT2 * K + K2) * norm;
}

double Butterworth2::filter(double x) {
    if (!primed_) {
        // Parte do regime permanente para a entrada atual, para nao arrastar um
        // transitorio de partida para dentro da malha de controle.
        x1_ = x2_ = x;
        y1_ = y2_ = x;
        primed_ = true;
        return x;
    }

    const double y = b0_ * x + b1_ * x1_ + b2_ * x2_ - a1_ * y1_ - a2_ * y2_;

    x2_ = x1_;
    x1_ = x;
    y2_ = y1_;
    y1_ = y;

    return y;
}

void Butterworth2::reset() {
    x1_ = x2_ = y1_ = y2_ = 0.0;
    primed_ = false;
}

// ─── Estimador ──────────────────────────────────────────────────────────────

TorsoAttitudeParams::TorsoAttitudeParams() :
        alpha(0.998),
        gyroCutoffHz(15.0),
        accelTrustBand(2.0),
        gravity(9.80665),
        imuToBody(Eigen::Matrix3d::Identity()) {
}

TorsoAttitudeParams TorsoAttitudeParams::getDefaultTorsoAttitudeParams() {
    TorsoAttitudeParams params;
    // Montagem declarada no URDF: sensor com rpy = (pi, 0, pi) em relacao a
    // base_link. Rz(pi) * Rx(pi) = Ry(pi) = diag(-1, 1, -1), ou seja, o eixo x
    // e o eixo z do sensor apontam ao contrario dos do corpo.
    params.imuToBody = TorsoAttitudeEstimator::rotationFromRPY(0.0, M_PI, 0.0);
    return params;
}

TorsoAttitudeEstimator::TorsoAttitudeEstimator(const TorsoAttitudeParams &params) :
        params_(params),
        gyroFiltered_(Eigen::Vector3d::Zero()),
        initialized_(false),
        lastAccelAccepted_(false) {
}

void TorsoAttitudeEstimator::reset() {
    attitude_ = TorsoAttitude();
    gyroFiltered_.setZero();
    for (int i = 0; i < 3; ++i) {
        gyroFilter_[i].reset();
    }
    initialized_ = false;
    lastAccelAccepted_ = false;
}

Eigen::Matrix3d TorsoAttitudeEstimator::rotationFromRPY(double roll, double pitch,
                                                        double yaw) {
    return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

void TorsoAttitudeEstimator::anglesFromAccel(const Eigen::Vector3d &accelBody,
                                             double &pitch, double &roll) const {
    // Em repouso o acelerometro le +g na direcao "para cima" do corpo.
    // Inclinando para frente de phi:  a_x = -g * sin(phi)  ->  pitch > 0.
    // Inclinando para a direita de phi: a_y = +g * sin(phi) ->  roll  > 0
    // (y aponta para a esquerda, entao tombar para a direita joga a projecao
    // da gravidade no sentido +y).
    const double ax = accelBody.x();
    const double ay = accelBody.y();
    const double az = accelBody.z();

    pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
    roll = std::atan2(ay, std::sqrt(ax * ax + az * az));
}

void TorsoAttitudeEstimator::update(const Eigen::Vector3d &accelImu,
                                    const Eigen::Vector3d &gyroImu, double dt) {
    if (!(dt > 0.0)) {
        return;
    }

    // ---- 1. Frame do sensor -> frame do corpo -----------------------------
    const Eigen::Vector3d accelBody = params_.imuToBody * accelImu;
    const Eigen::Vector3d gyroBody = params_.imuToBody * gyroImu;

    // ---- 2. Passa-baixa Butterworth de 2a ordem no giroscopio -------------
    // Corte <= 0 significa "sem filtragem".
    if (params_.gyroCutoffHz > 0.0) {
        for (int i = 0; i < 3; ++i) {
            if (gyroFilter_[i].needsDesign(params_.gyroCutoffHz, dt)) {
                gyroFilter_[i].design(params_.gyroCutoffHz, dt);
            }
            gyroFiltered_[i] = gyroFilter_[i].filter(gyroBody[i]);
        }
    } else {
        gyroFiltered_ = gyroBody;
    }

    // Rotacao em torno de +y = pitch para frente; em torno de +x = roll para a
    // direita. As taxas vem direto do giroscopio, nunca de derivada numerica do
    // angulo estimado.
    const double pitchRate = gyroFiltered_.y();
    const double rollRate = gyroFiltered_.x();

    // ---- 4. Portao de confianca no acelerometro ---------------------------
    // Fora da faixa a leitura contem aceleracao do corpo (impacto do pe, empurrao)
    // e nao aponta mais para cima: nesse passo integra-se apenas o giroscopio.
    const double accelNorm = accelBody.norm();
    const bool accelUsable = (params_.accelTrustBand <= 0.0)
            || (std::fabs(accelNorm - params_.gravity) <= params_.accelTrustBand);

    double pitchAccel = 0.0;
    double rollAccel = 0.0;
    if (accelNorm > 1e-6) {
        anglesFromAccel(accelBody, pitchAccel, rollAccel);
    }

    // ---- 3. Filtro complementar -------------------------------------------
    if (!initialized_) {
        // Primeira amostra util: parte direto do acelerometro para nao arrastar
        // o transitorio do filtro. Se ela nao for confiavel, espera a proxima.
        if (!accelUsable || accelNorm <= 1e-6) {
            lastAccelAccepted_ = false;
            return;
        }
        attitude_.pitch = pitchAccel;
        attitude_.roll = rollAccel;
        initialized_ = true;
    } else {
        const double pitchGyro = attitude_.pitch + pitchRate * dt;
        const double rollGyro = attitude_.roll + rollRate * dt;

        if (accelUsable && accelNorm > 1e-6) {
            attitude_.pitch = params_.alpha * pitchGyro
                    + (1.0 - params_.alpha) * pitchAccel;
            attitude_.roll = params_.alpha * rollGyro
                    + (1.0 - params_.alpha) * rollAccel;
        } else {
            attitude_.pitch = pitchGyro;
            attitude_.roll = rollGyro;
        }
    }

    attitude_.pitchRate = pitchRate;
    attitude_.rollRate = rollRate;

    // ---- 5. Yaw por integracao do giroscopio ------------------------------
    // Taxa de yaw no frame do MUNDO a partir do giro no frame do corpo, na
    // sequencia ZYX:  psi_dot = (sin(roll)*w_y + cos(roll)*w_z) / cos(pitch).
    // Com o tronco quase vertical e quase so w_z, mas na marcha o roll chega a
    // ~10 graus e a correcao evita que o balanco vire giro falso.
    const double cp = std::cos(attitude_.pitch);
    if (std::fabs(cp) > 0.1) {
        attitude_.yawRate = (std::sin(attitude_.roll) * gyroFiltered_.y()
                             + std::cos(attitude_.roll) * gyroFiltered_.z()) / cp;
        attitude_.yaw += attitude_.yawRate * dt;
    }
    lastAccelAccepted_ = accelUsable;
}

} /* namespace estimation */

} /* namespace control */
