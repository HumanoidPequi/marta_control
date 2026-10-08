/*
 * TorsoAttitudeEstimator.h
 *
 * Estimador de atitude do tronco (pitch e roll) a partir de uma IMU de 6 eixos.
 *
 * Substitui a integracao morta de 9 estados que existia em imu.cpp (predicao de
 * Kalman sem etapa de correcao, cujo estado nunca era consumido).
 *
 * Estrutura, na ordem em que os dados sao processados:
 *
 *   1. Rotacao estatica de montagem: as leituras chegam no frame do sensor e
 *      sao levadas ao frame do corpo por  v_body = R_bodyImu * v_imu.
 *      No Gazebo o sensor esta declarado com rpy = (pi, 0, pi) em relacao a
 *      base_link, o que equivale a R_y(pi) = diag(-1, 1, -1). No robo real a
 *      montagem pode ser outra, por isso a rotacao e parametro.
 *
 *   2. Passa-baixa Butterworth de 2a ordem no giroscopio, para atenuar a
 *      vibracao dos servos antes que o sinal chegue ao ganho derivativo do
 *      estabilizador (Fase 2). Essa e a unica fonte de phi_dot: nao se deriva o
 *      angulo numericamente. Dois polos dao 40 dB/decada na banda de rejeicao
 *      contra 20 dB/decada de um polo so, ao custo de pouco atraso a mais na
 *      banda de passagem (atraso de grupo em DC: sqrt(2)/(2*pi*fc) contra
 *      1/(2*pi*fc)).
 *
 *   3. Filtro complementar por eixo:
 *          phi_k = alpha * (phi_{k-1} + omega * dt) + (1 - alpha) * phi_acc
 *      O giroscopio fornece a banda alta (sem deriva no curto prazo) e o
 *      acelerometro fornece a referencia de gravidade na banda baixa (sem
 *      deriva no longo prazo).
 *
 *   4. Portao de confianca no acelerometro: durante impactos de pe no solo a
 *      norma da aceleracao especifica se afasta de g e o vetor medido deixa de
 *      apontar para cima. Nesses instantes a correcao pelo acelerometro e
 *      suspensa e o filtro integra apenas o giroscopio.
 *
 * Convencao de sinais (frame do corpo: x para frente, y para a esquerda,
 * z para cima), fixada conforme definido para as Fases 2 e 3:
 *
 *   pitch > 0  ->  tronco tombando para FRENTE   (rotacao em torno de +y)
 *   roll  > 0  ->  tronco tombando para a DIREITA (rotacao em torno de +x)
 *
 * Com o robo em repouso e na vertical o acelerometro le (0, 0, +g) no frame do
 * corpo. Inclinando para frente de um angulo phi:  a_x = -g * sin(phi), donde
 *   phi_pitch = atan2(-a_x, sqrt(a_y^2 + a_z^2))
 *   phi_roll  = atan2(+a_y, sqrt(a_x^2 + a_z^2))
 * e, coerentemente,  phi_dot_pitch = omega_y,  phi_dot_roll = omega_x.
 *
 * A classe nao depende de ROS: pode ser exercitada offline com dados de rosbag.
 */

#ifndef TORSO_ATTITUDE_ESTIMATOR_H_
#define TORSO_ATTITUDE_ESTIMATOR_H_

#include <Eigen/Dense>

namespace control {

namespace estimation {

/**
 * Passa-baixa Butterworth de 2a ordem (biquad), obtido por transformada
 * bilinear com pre-distorcao da frequencia de corte.
 *
 * Os coeficientes dependem do passo de amostragem, que aqui varia um pouco de
 * ciclo para ciclo: design() e refeito quando dt muda mais que 5%, preservando
 * o estado interno.
 */
class Butterworth2 {
public:
    Butterworth2();

    /**
     * Recalcula os coeficientes para o corte e o passo dados.
     * O corte e limitado a 45% da taxa de amostragem (o filtro nao pode
     * corrigir o que ja foi aliasado na aquisicao).
     */
    void design(double cutoffHz, double dt);

    /** Processa uma amostra. A primeira leva o filtro ao regime permanente. */
    double filter(double x);

    void reset();

    /** Verdadeiro se design() ja foi chamado para este dt e corte. */
    bool needsDesign(double cutoffHz, double dt) const;

private:
    double b0_, b1_, b2_, a1_, a2_;
    double x1_, x2_, y1_, y2_;
    double designCutoffHz_, designDt_;
    bool designed_;
    bool primed_;
};

/** Atitude estimada do tronco. Angulos em rad, taxas em rad/s. */
struct TorsoAttitude {
    double pitch;      ///< positivo = tombando para frente
    double pitchRate;  ///< d(pitch)/dt, medido pelo giroscopio (filtrado)
    double roll;       ///< positivo = tombando para a direita
    double rollRate;   ///< d(roll)/dt, medido pelo giroscopio (filtrado)
    /**
     * Yaw por integracao pura do giroscopio, positivo = girando para a
     * ESQUERDA (em torno de +z, para cima). Sem magnetometro nao ha referencia
     * absoluta: comeca em zero na inicializacao e deriva com o bias do giro
     * (bias * t). So a VARIACAO em janelas curtas e significativa - e e assim
     * que a malha de rumo do controller a usa.
     */
    double yaw;
    double yawRate;    ///< d(yaw)/dt no frame do mundo, do giroscopio (filtrado)

    TorsoAttitude() : pitch(0.0), pitchRate(0.0), roll(0.0), rollRate(0.0),
                      yaw(0.0), yawRate(0.0) {}
};

struct TorsoAttitudeParams {
    /**
     * Peso do ramo giroscopico do filtro complementar, em (0, 1).
     * A constante de tempo do crossover e  tau = alpha * dt / (1 - alpha):
     * com alpha = 0.998 a 100 Hz, tau ~ 5.0 s. Aumentar alpha confia mais no
     * giro (mais imune a aceleracao do corpo, mais sujeito a deriva).
     *
     * tau precisa ser bem maior que o periodo do passo (0.4 s) para que o
     * balanco do LIPM nao entre como inclinacao falsa. Em contrapartida, um
     * bias residual do giroscopio produz erro de regime  bias * tau: com
     * tau = 5 s, 0.01 rad/s de bias viram ~2.9 graus de offset. Em simulacao o
     * giro nao tem bias; no robo real, calibrar o bias em repouso.
     */
    double alpha;

    /** Frequencia de corte do passa-baixa Butterworth do giroscopio [Hz]. */
    double gyroCutoffHz;

    /**
     * Largura da faixa de confianca do acelerometro [m/s^2]. A correcao pelo
     * acelerometro so e aplicada quando  | ||a|| - g | <= accelTrustBand.
     * Valor <= 0 desativa o portao (aceita sempre).
     */
    double accelTrustBand;

    /** Aceleracao da gravidade usada como referencia [m/s^2]. */
    double gravity;

    /**
     * Rotacao do frame do sensor para o frame do corpo:  v_body = R * v_imu.
     * Default: R_y(pi), que corresponde a montagem declarada no URDF.
     */
    Eigen::Matrix3d imuToBody;

    TorsoAttitudeParams();

    static TorsoAttitudeParams getDefaultTorsoAttitudeParams();
};

class TorsoAttitudeEstimator {
public:
    explicit TorsoAttitudeEstimator(const TorsoAttitudeParams &params);

    /**
     * Processa uma amostra da IMU.
     *
     * @param accelImu Aceleracao especifica no frame do sensor [m/s^2].
     * @param gyroImu  Velocidade angular no frame do sensor [rad/s].
     * @param dt       Intervalo desde a amostra anterior [s]. Deve ser > 0.
     */
    void update(const Eigen::Vector3d &accelImu, const Eigen::Vector3d &gyroImu,
                double dt);

    /** Ultima atitude estimada. Zerada enquanto isInitialized() for falso. */
    const TorsoAttitude &getAttitude() const { return attitude_; }

    /**
     * Verdadeiro apos a primeira amostra util, quando os angulos ja foram
     * inicializados diretamente pelo acelerometro (sem transitorio do filtro).
     */
    bool isInitialized() const { return initialized_; }

    /**
     * Verdadeiro se a ultima amostra foi aceita pelo portao de confianca do
     * acelerometro. Util para diagnostico: se ficar falso por muito tempo
     * durante a marcha, o portao esta apertado demais e o pitch vai derivar.
     */
    bool lastAccelAccepted() const { return lastAccelAccepted_; }

    void setParams(const TorsoAttitudeParams &params) { params_ = params; }

    const TorsoAttitudeParams &getParams() const { return params_; }

    /** Descarta o estado do filtro. A proxima amostra reinicializa os angulos. */
    void reset();

    /** Matriz de rotacao a partir de angulos roll-pitch-yaw (Rz * Ry * Rx). */
    static Eigen::Matrix3d rotationFromRPY(double roll, double pitch, double yaw);

private:
    TorsoAttitudeParams params_;
    TorsoAttitude attitude_;

    Butterworth2 gyroFilter_[3];    ///< um biquad por eixo do giroscopio
    Eigen::Vector3d gyroFiltered_;  ///< giroscopio no frame do corpo, pos passa-baixa
    bool initialized_;
    bool lastAccelAccepted_;

    /** Angulos de inclinacao medidos so pelo acelerometro, no frame do corpo. */
    void anglesFromAccel(const Eigen::Vector3d &accelBody, double &pitch,
                         double &roll) const;
};

} /* namespace estimation */

} /* namespace control */

#endif /* TORSO_ATTITUDE_ESTIMATOR_H_ */
