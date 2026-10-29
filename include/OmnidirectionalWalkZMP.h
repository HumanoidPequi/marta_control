/*
 * OmnidirectionalWalkZMP.h
 *
 *  Created on: Sep 14, 2014
 *      Author: mmaximo
 */

#ifndef OMNIDIRECTIONALWALKZMP_H_
#define OMNIDIRECTIONALWALKZMP_H_

#include "Vector3.h"
#include "Pose2D.h"
#include "Joints.h"
#include "InverseKinematics.h"
#include "SupportState.h"


#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>

namespace control {

namespace walking {

using namespace representations;

enum StoppingState {
    NONE,
    STOP_REQUESTED,
    ADJUSTING_1ST,
    ADJUSTING_2ND,
    STOPPED
};

struct OmnidirectionalWalkZMPParams {
    double period;
    double zCom;
    double zStep;
    double doubleSupportRatio;
    double ySeparation;
    double armsCompensationFactor;
    double comSwingFactor;
    double hipPitchOffset;  // [rad], somado ao hip pitch dos dois lados depois da IK

    OmnidirectionalWalkZMPParams();
    static OmnidirectionalWalkZMPParams getDefaultOmnidirecionalWalkZMPParams();
};

class OmnidirectionalWalkZMP {
public:
    
    std::string j_filename = "J_trajectory.csv";

    bool leftFootIsSwing;
    itandroids_lib::math::Vector3<double> torso;

    itandroids_lib::math::Vector3<double> leftFoot_;
    itandroids_lib::math::Vector3<double> rightFoot_;
    double zLeft_;
    double zRight_;

    OmnidirectionalWalkZMP(OmnidirectionalWalkZMPParams walkParams,
                           RobotPhysicalParameters physicalParams,
                           InverseKinematics *inverseKinematics);

    virtual ~OmnidirectionalWalkZMP();

    void restart();

    /**
     * Pe que balanca no PRIMEIRO passo depois de restart(). Default false: o
     * direito balanca primeiro (comportamento historico). true espelha a
     * partida inteira em y. Existe para o teste do espelho da investigacao do
     * desvio de direcao: se a curva trocar de lado com o pe de partida, ela e
     * transiente de iniciacao. Vale a partir do proximo restart().
     */
    void setStartWithLeftSwing(bool startLeft) { startLeftSwing = startLeft; }

    /**
     * Partida suave: ao longo dos primeiros N passos depois de restart(), um
     * fator que sobe de 0 a 1 (smoothstep, derivada nula nas pontas) escala o
     * balanco lateral do CoM em torno do ponto medio entre os pes, a altura do
     * passo e a velocidade comandada. Sem isso o gerador parte do repouso
     * exigindo ja a velocidade lateral do ciclo periodico, e esse tranco
     * assimetrico curva o robo para o lado do pe de partida. N <= 0 desliga
     * (comportamento historico).
     */
    void setStartRampSteps(int steps) { startRampSteps = steps; }

    void appendToCSV(const std::string& filename, const std::string& data);

    void update(double elapsedTime, Joints &joints, float &Xscale);

    void setDesiredVelocity(itandroids_lib::math::Pose2D velocity);

    void requestStop();

    /**
     * Para JA, sem os dois passos de ajuste de requestStop(): o gerador fica em
     * STOPPED na postura de apoio duplo do inicio do passo corrente. Usado logo
     * apos o Agacha(), quando o robo ainda nao andou e os passos de ajuste so
     * arrastariam os pes.
     */
    void stopNow();

    bool hasStopped();

    itandroids_lib::math::Vector3<double> getOdometry();

    /** Fase normalizada do passo, em [0, 1). */
    double getPhase() const;

    /**
     * Verdadeiro nas duas janelas de apoio duplo do ciclo (inicio e fim), e
     * tambem quando a caminhada esta parada (robo em pe sobre os dois pes).
     */
    bool isDoubleSupport() const;

    /** Estado de apoio no instante atual do ciclo. */
    SupportState getSupportState() const;

    /**
     * Fracao do peso em cada perna (somam 1), interpolada nos apoios duplos
     * como em Maximo (doutorado, eq. 4.23-4.24): a perna de apoio do passo vai
     * de 0,5 a 1 no apoio duplo inicial, fica em 1 no apoio simples e volta a
     * 0,5 no apoio duplo final; continuo na troca de passo. Parado: 0,5 cada.
     */
    void getLoadWeights(double &left, double &right) const;

    /** Parametros do LIPM e da marcha (zCom, period, doubleSupportRatio, ...). */
    const OmnidirectionalWalkZMPParams &getParams() const { return walkParams; }

    /**
     * Constante de tempo do LIPM, sqrt(zCom * comSwingFactor / g).
     * E o Tc da eq. (41) de Kajita 2001; a telemetria precisa dele para calcular
     * energia orbital e ponto de captura com o MESMO valor que o gerador usou.
     */
    double getTimeConstant() const { return tZMP; }

    /**
     * Pegadas planejadas do passo corrente, em coordenadas ABSOLUTAS do frame do
     * passo (diferente de leftFoot_/rightFoot_, que sao relativos ao torso).
     */
    const itandroids_lib::math::Vector3<double> &getSwingFootBegin() const {
        return swingFootBegin;
    }
    const itandroids_lib::math::Vector3<double> &getSwingFootEnd() const {
        return swingFootEnd;
    }

private:
    const double GRAVITY;

    StoppingState stoppingState;

    OmnidirectionalWalkZMPParams walkParams;
    InverseKinematics *inverseKinematics;
    RobotPhysicalParameters physicalParams;

    double tZMP;

    itandroids_lib::math::Vector3<double> swingFootBegin;
    itandroids_lib::math::Vector3<double> swingFootEnd;
    itandroids_lib::math::Vector3<double> torsoBegin;
    itandroids_lib::math::Vector3<double> torsoEnd;

    double walkTime;
    bool startLeftSwing = false;   ///< ver setStartWithLeftSwing()
    int startRampSteps = 0;        ///< ver setStartRampSteps()
    int stepsSinceRestart = 0;     ///< passos completos desde restart()

    /** Fator da partida suave no passo corrente, fase t em [0, 1]. */
    double startRamp(double t) const;

    itandroids_lib::math::Vector3<double> velocity;

    itandroids_lib::math::Vector3<double> previousTorso;
    itandroids_lib::math::Vector3<double> odometry;

    void switchSwingFoot();

    void solveZMP(double zs, double z1, double z2, double x1, double x2, double &aP,
                  double &aN);

    double getCoM(double t, double aP, double aN, double zs, double z1, double z2);

    void computeNewStep();

    itandroids_lib::math::Vector3<double> poseGlobal(itandroids_lib::math::Vector3<double> poseRelative,
                                         itandroids_lib::math::Vector3<double> pose);

    itandroids_lib::math::Vector3<double> poseRelative(itandroids_lib::math::Vector3<double> poseGlobal,
                                           itandroids_lib::math::Vector3<double> pose);

    itandroids_lib::math::Vector3<double> getDesiredTorsoDisplacement(itandroids_lib::math::Vector3<double> torsoBegin);

    itandroids_lib::math::Vector3<double>
    limitTorso(itandroids_lib::math::Vector3<double> beginTorso, itandroids_lib::math::Vector3<double> torsoDisplacement);

    double computeXFrac(double t);

    double computeYFrac(double t);

    double computeZFrac(double t);

    double computeThetaFrac(double t);
};

} /* namespace walking*/

} /* namespace control */

#endif /* OMNIDIRECTIONALWALKZMP_H_ */
