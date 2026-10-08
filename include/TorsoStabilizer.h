/*
 * TorsoStabilizer.h
 *
 * Fase 2: estabilizador de pitch do tronco (quadril sagital).
 *
 * Adaptacao do controle de atitude do corpo de Raibert ("Legged Robots That
 * Balance", cap. 2 e 3) para um robo com servos de POSICAO. Raibert injeta
 * torque no tronco pela perna de apoio:
 *
 *     tau_P = -k_p (phi_P - phi_P_d) - k_v * d(phi_P)/dt
 *
 * Sem controle de torque, o equivalente e um desvio angular corretivo somado a
 * saida da cinematica inversa, como Kajita descreve em 4.5.1 ao separar o
 * gerador de marcha (malha aberta) das malhas rapidas de realimentacao:
 *
 *     theta_junta = theta_IK + delta_theta
 *     delta_theta = k_p (phi_P - phi_P_ref) + k_d * d(phi_P)/dt
 *
 * SINAL (a parte que mais custa errar)
 * ------------------------------------
 * No URDF, l_hip_pitch_link e r_hip_pitch_link tem eixo +y, e o comando enviado
 * ao servo e o proprio angulo da junta. Um angulo positivo gira a COXA de
 * +delta em torno de +y em relacao ao torso.
 *
 * Na perna de apoio a coxa esta presa ao solo pelo pe e pelo joelho, entao quem
 * gira e o torso, de -delta em torno de +y. Como girar em torno de +y inclina o
 * tronco para FRENTE (convencao fixada na Fase 1, ver TorsoAttitudeEstimator.h),
 * somar delta > 0 ao hip pitch gira o tronco para TRAS.
 *
 * Logo, com phi_P > 0 significando "tombando para frente", a correcao
 * restauradora e delta_theta > 0, e os ganhos k_p e k_d entram POSITIVOS:
 *
 *     delta_theta = k_p (phi_P - phi_P_ref) + k_d * phi_P_dot     [rad]
 *     hipPitch_cmd = hipPitch_IK + delta_theta
 *
 * Note que so o quadril muda: joelho e tornozelo ficam como a IK calculou, de
 * modo que a postura da perna e o contato do pe com o solo nao se alteram - o
 * tronco gira em torno do quadril, deslocando o CoM. E a estrategia de quadril,
 * nao a de tornozelo.
 *
 * DISTRIBUICAO ENTRE AS PERNAS
 * ----------------------------
 * A correcao vai para a perna de APOIO. Aplicar na perna oscilante mudaria a
 * colocacao do passo, que e justamente o que o gerador de marcha planejou.
 * Em apoio duplo (e com o robo parado) vai para as duas.
 *
 * A troca de apoio muda o alvo de uma perna para a outra. Para nao dar um degrau
 * no comando da junta que sai de servico, cada perna tem um peso proprio que
 * caminha suavemente entre 0 e 1 com constante de tempo supportBlendTime. Em
 * apoio simples estabelecido o peso ja esta em 1, entao a correcao efetiva NAO
 * sofre atraso: o que e suavizado e a ativacao da perna, nao o valor da correcao.
 */

#ifndef TORSO_STABILIZER_H_
#define TORSO_STABILIZER_H_

#include "Joints.h"
#include "SupportState.h"

namespace control {

namespace stabilization {

struct TorsoStabilizerParams {
    /** Chave geral. Desligado, o estabilizador nao toca nas juntas. */
    bool enabled;

    /** Ganho proporcional [rad de correcao por rad de inclinacao]. */
    double kpPitch;

    /** Ganho derivativo [rad de correcao por rad/s de velocidade angular]. */
    double kdPitch;

    /** Referencia de pitch [rad]. Zero = tronco na vertical. */
    double pitchRef;

    /**
     * Saturacao rigida do delta_theta [rad]. Limite mecanico de seguranca:
     * nenhuma condicao de falha do estimador pode mandar a junta para o batente.
     */
    double maxPitchCorrection;

    /** Constante de tempo da transicao de peso entre as pernas [s]. */
    double supportBlendTime;

    TorsoStabilizerParams();

    static TorsoStabilizerParams getDefaultTorsoStabilizerParams();
};

/** Ultimo resultado do estabilizador, para log, rosbag e rqt_plot. */
struct TorsoStabilizerState {
    double deltaPitch;       ///< correcao calculada, ja saturada [rad]
    double leftHipDelta;     ///< o que foi somado ao hip pitch esquerdo [rad]
    double rightHipDelta;    ///< o que foi somado ao hip pitch direito [rad]
    double leftWeight;       ///< peso de ativacao da perna esquerda, [0, 1]
    double rightWeight;      ///< peso de ativacao da perna direita, [0, 1]
    bool saturated;          ///< a saturacao esta ativa neste ciclo

    TorsoStabilizerState() : deltaPitch(0.0), leftHipDelta(0.0), rightHipDelta(0.0),
            leftWeight(0.0), rightWeight(0.0), saturated(false) {}
};

class TorsoStabilizer {
public:
    explicit TorsoStabilizer(const TorsoStabilizerParams &params);

    /**
     * Calcula a correcao de pitch e a soma aos hip pitch das pernas de apoio.
     *
     * @param pitch     Inclinacao do tronco [rad], positivo tombando para frente.
     * @param pitchRate Velocidade angular de pitch [rad/s], do giroscopio filtrado.
     * @param support   Estado de apoio no instante atual.
     * @param dt        Passo do laco [s], usado so na transicao de pesos.
     * @param joints    Alvos vindos da IK; os hip pitch sao modificados no lugar.
     */
    const TorsoStabilizerState &update(double pitch, double pitchRate,
                                       walking::SupportState support, double dt,
                                       representations::Joints &joints);

    /** Correcao PD saturada, sem aplicar nas juntas. */
    double computePitchCorrection(double pitch, double pitchRate) const;

    /** Zera os pesos das pernas e o ultimo estado (uso apos queda/levantar). */
    void reset();

    void setParams(const TorsoStabilizerParams &params) { params_ = params; }

    const TorsoStabilizerParams &getParams() const { return params_; }

    const TorsoStabilizerState &getState() const { return state_; }

private:
    TorsoStabilizerParams params_;
    TorsoStabilizerState state_;

    double leftWeight_;
    double rightWeight_;

    /** Move o peso em direcao ao alvo com constante de tempo supportBlendTime. */
    void blendWeight(double &weight, double target, double dt) const;
};

} /* namespace stabilization */

} /* namespace control */

#endif /* TORSO_STABILIZER_H_ */
