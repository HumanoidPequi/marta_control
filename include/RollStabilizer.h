/*
 * RollStabilizer.h
 *
 * Fase 3: estabilizador de roll do tronco (docs/plano_fase3_roll.md par. 3 e 4.3).
 *
 * Controle de atitude do corpo pela perna de apoio, como Raibert ("Legged
 * Robots That Balance", cap. 3: pitch e roll como malhas independentes), na
 * forma de Maximo (doutorado, ITA, 2017, par. 4.4.2, eq. 4.34-4.35): um P+D de
 * roll aplicado ao hip roll e ao ankle roll, com ganhos separados. Como no
 * pitch (TorsoStabilizer.h), o servo e de POSICAO, entao a correcao e um desvio
 * angular somado a saida da IK:
 *
 *     phi_m  <- phi_m + (dt / tau_w) (phi - phi_m)        washout
 *     u_h    = kp_h (phi - phi_m) + kd_h phi_dot
 *     u_a    = kp_a (phi - phi_m) + kd_a phi_dot
 *     hipRoll_X   += w_X sat(u_h)        X = perna esquerda ou direita,
 *     ankleRoll_X += w_X sat(u_a)        w_X = peso de carga do gerador
 *
 * WASHOUT
 * -------
 * A estimativa de roll tem vies de ~+2,3 graus contra a verdade (achado N do
 * README). Um P sobre o roll cru comandaria uma inclinacao permanente. O P age
 * sobre phi - phi_m, com phi_m um passa-baixa lento: o vies (e qualquer
 * inclinacao lenta) sai, e a alternancia a cada passo (~0,83 Hz com periodo de
 * passo de 0,6 s) passa com ganho > 0,99 para tau_w = 2 s.
 *
 * SINAL
 * -----
 * hip_roll e ank_roll tem eixo +x nas duas pernas (marta_joints.xacro); roll > 0
 * e o tronco tombando para a DIREITA (rotacao +x). Na perna de apoio o pe esta
 * preso ao chao: +delta no hip roll gira a coxa +delta em relacao ao tronco,
 * entao quem gira e o tronco, de -delta (para a esquerda); +delta no ankle roll
 * gira a perna inteira (e o tronco) de -delta. Logo delta = +K phi restaura nas
 * duas juntas, com ganhos POSITIVOS - a mesma estrutura do pitch.
 *
 * PESOS
 * -----
 * w_X vem de OmnidirectionalWalkZMP::getLoadWeights (somam 1; 1 na perna de
 * apoio em apoio simples, ~0 na de balanco, 0,5 parado). A perna em balanco
 * nao e corrigida, entao a colocacao do pe planejada pelo gerador nao muda.
 */

#ifndef ROLL_STABILIZER_H_
#define ROLL_STABILIZER_H_

#include "Joints.h"

namespace control {

namespace stabilization {

struct RollStabilizerParams {
    bool enabled;
    double kpHip;        ///< [rad de correcao por rad de roll]
    double kdHip;        ///< [rad por rad/s]
    double kpAnkle;
    double kdAnkle;
    double washoutTau;   ///< constante de tempo do washout [s]
    double maxCorrection;  ///< saturacao por junta [rad]

    RollStabilizerParams();
};

struct RollStabilizerState {
    double rollMean;     ///< phi_m, a saida do washout [rad]
    double error;        ///< phi - phi_m [rad]
    double uHip;         ///< correcao do hip roll, saturada, antes do peso [rad]
    double uAnkle;       ///< correcao do ankle roll, saturada, antes do peso [rad]
    double leftWeight;
    double rightWeight;
    bool saturated;

    RollStabilizerState() : rollMean(0.0), error(0.0), uHip(0.0), uAnkle(0.0),
            leftWeight(0.0), rightWeight(0.0), saturated(false) {}
};

class RollStabilizer {
public:
    explicit RollStabilizer(const RollStabilizerParams &params = RollStabilizerParams());

    /**
     * Atualiza o washout e, se ligado, soma as correcoes ao hip roll e ao ankle
     * roll de cada perna, ponderadas pelo peso de carga.
     *
     * O washout roda mesmo desligado, para que ligar o estabilizador no meio da
     * marcha nao produza um degrau.
     *
     * @param roll      roll do tronco [rad], + tombando para a direita
     * @param rollRate  velocidade de roll [rad/s]
     * @param wLeft     peso de carga da perna esquerda, [0, 1]
     * @param wRight    peso de carga da perna direita, [0, 1]
     * @param dt        passo do laco [s]
     * @param joints    alvos da IK; hip roll e ankle roll sao modificados no lugar
     */
    const RollStabilizerState &update(double roll, double rollRate, double wLeft,
                                      double wRight, double dt,
                                      representations::Joints &joints);

    /** Reinicia o washout (apos queda/levantar): o proximo roll vira a media. */
    void reset();

    void setParams(const RollStabilizerParams &params) { params_ = params; }

    const RollStabilizerParams &getParams() const { return params_; }

    const RollStabilizerState &getState() const { return state_; }

private:
    RollStabilizerParams params_;
    RollStabilizerState state_;
    bool initialized_;
};

} /* namespace stabilization */

} /* namespace control */

#endif /* ROLL_STABILIZER_H_ */
