/*
 * OmnidirectionalWalkZMP.cpp
 *
 *  Created on: Sep 14, 2014
 *      Author: mmaximo
 */

#include "OmnidirectionalWalkZMP.h"

#include <stdio.h>
#include <math.h>
#include <cfloat>
#include <Vector3.h>
#include <fstream>

#include <MathUtils.h>

namespace control {

namespace walking {

using itandroids_lib::math::MathUtils;

OmnidirectionalWalkZMPParams::OmnidirectionalWalkZMPParams() :
        period(0.0), zCom(0.0), zStep(0.0), doubleSupportRatio(0.0), ySeparation(
        0.0), armsCompensationFactor(0.0), comSwingFactor(0.0) {
    
}

OmnidirectionalWalkZMPParams OmnidirectionalWalkZMPParams::getDefaultOmnidirecionalWalkZMPParams() {
    OmnidirectionalWalkZMPParams params;
    params.period = 0.4;
    params.doubleSupportRatio = 0.5;          // Proporção de tempo em que ambos os pés estão no chão
    params.ySeparation = 0.08;                // Separação lateral entre os pés.
    params.zCom = 0.297509;                   // Altura do centro de massa (CoM - Center of Mass)
    params.zStep = 0.06;                      // Altura do movimento oscilatório do pé durante o passo
    params.armsCompensationFactor = 1.0;      // Fator de compensação do movimento dos braços para equilibrar o movimento do torso
    params.comSwingFactor = 1.0;              // Fator que ajusta a amplitude do movimento do CoM
    return params;
}

OmnidirectionalWalkZMP::OmnidirectionalWalkZMP(
        OmnidirectionalWalkZMPParams walkParams,
        RobotPhysicalParameters physicalParams,
        InverseKinematics *inverseKinematics) :
        GRAVITY(9.80665), walkParams(walkParams), physicalParams(
        physicalParams), inverseKinematics(inverseKinematics), walkTime(
        0.0) {
    tZMP = sqrt(walkParams.zCom * walkParams.comSwingFactor / GRAVITY);
    restart();

    //appendToCSV(j_filename, "q1,q2,q3,q4");
}

OmnidirectionalWalkZMP::~OmnidirectionalWalkZMP() {
}

// TODO: make the walking works with Pose2D instead of Vector3
void OmnidirectionalWalkZMP::setDesiredVelocity(itandroids_lib::math::Pose2D velocity) {
    this->velocity.x = velocity.translation.x;
    this->velocity.y = velocity.translation.y;
    this->velocity.z = velocity.rotation;
}

void OmnidirectionalWalkZMP::restart() {
    walkTime = 0.0;
    stepsSinceRestart = 0;
    // Partida espelhavel: com o direito balancando primeiro (default), o pe de
    // apoio esquerdo esta na origem e o torso e o pe direito ficam em -y; com
    // o esquerdo primeiro, tudo vai para +y.
    leftFootIsSwing = startLeftSwing;
    const double side = startLeftSwing ? 1.0 : -1.0;
    torsoBegin.x = 0.0;
    torsoBegin.y = side * walkParams.ySeparation;
    // .z e o YAW do torso (Vector3 = x, y, theta), nao a altura. Ja foi
    // 0.297509 (o valor de zCom colado no campo errado), o que comandava um
    // giro de ~17 graus nas duas hip_yaw no primeiro passo de cada marcha.
    // Travado por test/test_walk_symmetry.cpp.
    torsoBegin.z = 0.0;
    torsoEnd.x = 0.0;
    torsoEnd.y = side * walkParams.ySeparation;
    torsoEnd.z = 0.0;
    swingFootBegin.x = 0.0;
    swingFootBegin.y = torsoBegin.y + side * walkParams.ySeparation;
    swingFootBegin.z = 0.0;
    swingFootEnd.x = 0.0;
    swingFootEnd.y = torsoEnd.y + side * walkParams.ySeparation;
    swingFootEnd.z = 0.0;
    odometry.x = 0.0;
    odometry.y = 0.0;
    odometry.z = 0.0;
    previousTorso = torsoBegin;

    stoppingState = NONE;
}

/**
 * @brief Atualiza o estado da caminhada com base no tempo decorrido, computando a nova posição do torso,
 *        pés, braços e odometria do robô para gerar uma nova pose de caminhada.
 *
 * @param elapsedTime Tempo decorrido desde a última atualização (em segundos).
 * @param joints Referência para a estrutura de juntas que será atualizada com os novos ângulos alvo.
 * @param Xscale_ Fator de escala adicional passado para o sistema de cinemática inversa para controlar o blanço no eixo x.
 */
void OmnidirectionalWalkZMP::update(double elapsedTime, Joints &joints, float &Xscale_) {
    // Incrementa o tempo total de caminhada
    walkTime += elapsedTime;

    // Se o robô está parado, zera o tempo
    if (stoppingState == STOPPED)
        walkTime = 0.0;

    // Se o tempo de caminhada ultrapassou o período de um passo, inicia novo passo
    if (walkTime >= walkParams.period) {
        walkTime -= walkParams.period;
        computeNewStep();
    }

    // Calcula a fase normalizada do passo [0.0, 1.0]
    double t = walkTime / walkParams.period;

    // Coeficientes da solução do modelo LIPM para eixo X e Y
    double aPx;
    double aNx;
    double aPy;
    double aNy;

    // Resolve o modelo LIPM para o deslocamento do torso em X e Y
    solveZMP(0.0, torsoBegin.x, torsoEnd.x, torsoBegin.x, torsoEnd.x, aPx,
             aNx);
    solveZMP(0.0, torsoBegin.y, torsoEnd.y, torsoBegin.y, torsoEnd.y, aPy,
             aNy);

    // Obtém a posição atual do centro de massa (CoM) do torso usando os coeficientes
    double comX = getCoM(t, aPx, aNx, 0.0, torsoBegin.x, torsoEnd.x);
    double comY = getCoM(t, aPy, aNy, 0.0, torsoBegin.y, torsoEnd.y);
    
    // Frações de interpolação para deslocamento linear e angular
    double xFrac = computeXFrac(t);
    double yFrac = computeYFrac(t);
    double zFrac = computeZFrac(t);
    double thetaFrac = computeThetaFrac(t);

    // Partida suave: aproxima o CoM lateral do ponto medio entre os pes (no
    // frame do pe de apoio, que esta na origem, o ponto medio e metade do y do
    // pe oscilante) e reduz a altura do passo, na proporcao do fator.
    const double ramp = startRamp(t);
    if (ramp < 1.0) {
        const double ySwingNow = swingFootBegin.y
                                 + (swingFootEnd.y - swingFootBegin.y) * yFrac;
        const double yMid = 0.5 * ySwingNow;
        comY = yMid + ramp * (comY - yMid);
    }

    // Calcula a rotação do torso ao longo do passo
    double thetaTorso = torsoBegin.z + (torsoEnd.z - torsoBegin.z) * thetaFrac;

    // Atualiza a pose absoluta do torso
    torso.x = comX;
    torso.y = comY;
    torso.z = thetaTorso;

    // Calcula a odometria incremental (pose relativa do torso em relação ao frame anterior)
    odometry = poseRelative(torso, previousTorso);

    previousTorso = torso;

    //std::cout << "torso: " << torso.x << " " << torso.y << " " << torso.z << "\n";
    //itandroids_lib::math::Vector3<double> torso(comX, comY, thetaTorso);

    // Interpola a posição e orientação do pé oscilante no frame global
    double xSwingGlobal = swingFootBegin.x
                          + (swingFootEnd.x - swingFootBegin.x) * xFrac;
    double ySwingGlobal = swingFootBegin.y
                          + (swingFootEnd.y - swingFootBegin.y) * yFrac;
    double thetaSwingGlobal = swingFootBegin.z
                              + (swingFootEnd.z - swingFootBegin.z) * thetaFrac;

    itandroids_lib::math::Vector3<double> swingFoot(xSwingGlobal, ySwingGlobal,
                                                    thetaSwingGlobal);

    // Transforma o pé oscilante para o frame do torso (posição relativa ao torso)
    swingFoot = poseRelative(swingFoot, torso);

    // O pé de suporte é considerado na origem relativa ao torso
    itandroids_lib::math::Vector3<double> supportFoot = poseRelative(itandroids_lib::math::Vector3<double>(), torso);

    // Altura do CoM dos pés
    double zSupport = walkParams.zCom;
    double zSwing = walkParams.zCom - zFrac * walkParams.zStep * ramp;

    // Determina qual pé está oscilando e configura as posições de acordo
    itandroids_lib::math::Vector3<double> leftFoot;
    itandroids_lib::math::Vector3<double> rightFoot;
    double zLeft;
    double zRight;
    if (leftFootIsSwing) {
        leftFoot = swingFoot;
        rightFoot = supportFoot;
        zLeft = zSwing;
        zRight = zSupport;
    } else {
        leftFoot = supportFoot;
        rightFoot = swingFoot;
        zLeft = zSupport;
        zRight = zSwing;
    }

    // Compensação dos braços para contrabalancear o movimento dos pés
    double xLeftArm = rightFoot.x * walkParams.armsCompensationFactor;
    double xRightArm = leftFoot.x * walkParams.armsCompensationFactor;

    // Define posições fixas para rotação lateral dos ombros
    joints.leftShoulderPitch = - asin(xLeftArm / physicalParams.armLength);
    joints.rightShoulderPitch = - asin(xRightArm / physicalParams.armLength);
    joints.leftShoulderRoll = MathUtils::degreesToRadians(-50.0);
    joints.rightShoulderRoll = MathUtils::degreesToRadians(50.0);


	// Calcula a cinemática inversa dos pés
    inverseKinematics->computeJointsTargets(leftFoot.x, leftFoot.y, zLeft,
                                            leftFoot.z, rightFoot.x, rightFoot.y, zRight, rightFoot.z, joints, Xscale_);

    // Ajuste fino na postura para evitar sobrecarga nos quadris
    joints.leftHipPitch -= 5.0 * M_PI / 180.0;
    joints.rightHipPitch -= 5.0 * M_PI / 180.0;

    // Armazena estado atual dos pés
    leftFoot_ = leftFoot;
    rightFoot_ = rightFoot;
    zLeft_ = zLeft;
    zRight_ = zRight;
}

void OmnidirectionalWalkZMP::appendToCSV(const std::string& filename, const std::string& data) {
    std::ofstream file;
    file.open(filename, std::ios::app); // Abre em modo append (adiciona linhas)
    if (file.is_open()) {
        file << data << "\n";
        file.close();
    } else {
        std::cerr << "Erro ao abrir arquivo: " << filename << std::endl;
    }
}

void OmnidirectionalWalkZMP::computeNewStep() {
    if (stoppingState == STOP_REQUESTED)
        stoppingState = ADJUSTING_1ST;
    else if (stoppingState == ADJUSTING_1ST)
        stoppingState = ADJUSTING_2ND;
    else if (stoppingState == ADJUSTING_2ND)
        stoppingState = STOPPED;

    //printf("stoppingState: %d", stoppingState);

    if (stoppingState == ADJUSTING_1ST || stoppingState == ADJUSTING_2ND) {
        velocity.x = 0.0;
        velocity.y = 0.0;
        velocity.z = 0.0;
    }

    // Partida suave: o passo que comeca agora usa a velocidade escalada pelo
    // fator no meio dele. A escala vale so dentro deste calculo; a velocidade
    // comandada e restaurada no fim, para nao acumular entre passos.
    ++stepsSinceRestart;
    const itandroids_lib::math::Vector3<double> velocityCommanded = velocity;
    {
        const double r = startRamp(0.5);
        velocity.x *= r;
        velocity.y *= r;
        velocity.z *= r;
    }

    switchSwingFoot();

    torsoBegin = poseRelative(torsoEnd, swingFootEnd);
    itandroids_lib::math::Vector3<double> supportFoot(0.0, 0.0, 0.0);
    swingFootBegin = poseRelative(supportFoot, swingFootEnd);
    previousTorso = poseRelative(previousTorso, swingFootEnd); // for odometry
    itandroids_lib::math::Vector3<double> torsoDisplacement = getDesiredTorsoDisplacement(
            torsoBegin);
    torsoEnd = limitTorso(torsoBegin, torsoDisplacement);

    itandroids_lib::math::Vector3<double> torsoTwoSteps = torsoEnd
                                                          + getDesiredTorsoDisplacement(torsoEnd);

    bool oY = ((torsoDisplacement.y >= 0 && leftFootIsSwing)
               || (torsoDisplacement.y < 0 && !leftFootIsSwing));
    bool oPsi = ((torsoDisplacement.z >= 0 && leftFootIsSwing)
                 || (torsoDisplacement.z < 0 && !leftFootIsSwing));

    itandroids_lib::math::Vector3<double> p;

    if (oPsi) {
        p = torsoTwoSteps;
    } else {
        p = torsoEnd;
    }

    swingFootEnd.z = 0.0;

    itandroids_lib::math::Vector3<double> torsoEndRelativeP = poseRelative(torsoEnd, p);
    itandroids_lib::math::Vector3<double> torsoTwoStepsRelativeP = poseRelative(
            torsoTwoSteps, p);

    double swingFootDisplacementY;
    if (leftFootIsSwing) {
        swingFootDisplacementY = walkParams.ySeparation;
    } else {
        swingFootDisplacementY = -walkParams.ySeparation;
    }

    if (oY) {
        swingFootEnd.y = torsoTwoStepsRelativeP.y + swingFootDisplacementY;
    } else {
        swingFootEnd.y = torsoEndRelativeP.y + swingFootDisplacementY;
    }

    double dx;
    double a = 2.0 * (velocity.x / velocity.z)
               * sin(velocity.z * walkParams.period / 2.0);
    double b = 2.0 * (velocity.y / velocity.z)
               * sin(velocity.z * walkParams.period / 2.0);
    double cossine = cos(velocity.z * walkParams.period / 2.0);
    double sine = sin(velocity.z * walkParams.period / 2.0);

    if (oPsi) {
        dx = a * cossine + b * sine;
    } else {
        dx = a * cossine - b * sine;
    }

    if (std::fabs(velocity.z) < DBL_EPSILON) {
        dx = walkParams.period * velocity.x;
    }

    swingFootEnd.x = torsoEndRelativeP.x + dx / 2.0;

    swingFootEnd = poseGlobal(swingFootEnd, p);

    velocity = velocityCommanded;
}

double OmnidirectionalWalkZMP::startRamp(double t) const {
    if (startRampSteps <= 0)
        return 1.0;
    double x = (stepsSinceRestart + t) / startRampSteps;
    x = std::max(0.0, std::min(1.0, x));
    return x * x * (3.0 - 2.0 * x);   // smoothstep
}

itandroids_lib::math::Vector3<double> OmnidirectionalWalkZMP::limitTorso(
        itandroids_lib::math::Vector3<double> beginTorso,
        itandroids_lib::math::Vector3<double> torsoDisplacement) {
    itandroids_lib::math::Vector3<double> torsoEnd = beginTorso + torsoDisplacement;

    if (leftFootIsSwing) {
        torsoEnd.y = std::max(walkParams.ySeparation, torsoEnd.y);
        torsoEnd.z = std::max(0.0, torsoEnd.z);
    } else {
        torsoEnd.y = std::min(-walkParams.ySeparation, torsoEnd.y);
        torsoEnd.z = std::min(0.0, torsoEnd.z);
    }

    return torsoEnd;
}

void OmnidirectionalWalkZMP::switchSwingFoot() {
    leftFootIsSwing = !leftFootIsSwing;
}

/// Calcula o deslocamento desejado do torso do robô durante um ciclo de caminhada.
///
/// Esta função considera o vetor de velocidade (`velocity`) do torso para calcular
/// a nova posição relativa ao início do passo. Quando não há rotação (theta ~= 0),
/// a posição final é obtida por multiplicação direta. Quando há rotação, é aplicada
/// uma interpolação circular no plano XY baseada em seno/cosseno.
///
/// @param torsoBegin Pose inicial do torso (x, y, theta) no início do ciclo.
/// @return Vetor 3D (x, y, theta) representando o deslocamento desejado do torso após um ciclo de caminhada.
itandroids_lib::math::Vector3<double> OmnidirectionalWalkZMP::getDesiredTorsoDisplacement(
        itandroids_lib::math::Vector3<double> torsoBegin) {
    itandroids_lib::math::Vector3<double> displacement;
    if (std::fabs(velocity.z) < DBL_EPSILON) {
        displacement = velocity * walkParams.period;
    } else {
        double a = 2.0 * (velocity.x / velocity.z)
                   * sin(velocity.z * walkParams.period / 2.0);
        double b = 2.0 * (velocity.y / velocity.z)
                   * sin(velocity.z * walkParams.period / 2.0);
        double cossine = cos(torsoBegin.z + velocity.z * walkParams.period / 2.0);
        double sine = sin(torsoBegin.z + velocity.z * walkParams.period / 2.0);
        displacement.x = a * cossine - b * sine;
        displacement.y = a * sine + b * cossine;
        displacement.z = walkParams.period * velocity.z;
    }
    return displacement;
}

/// Resolve os coeficientes `aP` e `aN` da equação diferencial da trajetória do centro de massa (CoM)
/// baseada no modelo Linear Inverted Pendulum Model (LIPM) em ZMP.
///
/// Esta função resolve os coeficientes de uma equação da forma:
/// `x(t) = aP * exp(t/T) + aN * exp(-t/T)`
/// onde `T = tZMP` (constante de tempo) e `x(t)` é a posição do CoM.
///
/// @param zs Posição do ponto de apoio central (ZMP) durante a fase de suporte único.
/// @param z1 ZMP no início.
/// @param z2 ZMP no fim.
/// @param x1 Posição inicial do CoM.
/// @param x2 Posição final desejada do CoM.
/// @param aP (saída) Coeficiente da exponencial crescente (exp(+t/T)).
/// @param aN (saída) Coeficiente da exponencial decrescente (exp(-t/T)).
void OmnidirectionalWalkZMP::solveZMP(double zs, double z1, double z2, double x1,
                                      double x2, double &aP, double &aN) {
    double T1 = walkParams.period * walkParams.doubleSupportRatio / 2.0;
    double T2 = walkParams.period - T1;

    double m1 = (zs - z1) / T1;
    double m2 = -(zs - z2) / (walkParams.period - T2);

    double c1 = x1 - z1 + tZMP * m1 * sinh(-T1 / tZMP);
    double c2 = x2 - z2 + tZMP * m2 * sinh((walkParams.period - T2) / tZMP);
    double expTStep = exp(walkParams.period / tZMP);
    aP = (c2 - c1 / expTStep) / (expTStep - 1 / expTStep);
    aN = (c1 * expTStep - c2) / (expTStep - 1 / expTStep);
}

/// Calcula a posição do centro de massa (CoM) no tempo `t` durante um ciclo de caminhada.
///
/// Utiliza a solução da equação diferencial baseada no modelo LIPM e aplica correções
/// de aceleração durante as fases de suporte duplo para garantir continuidade dinâmica.
///
/// @param t Tempo normalizado no intervalo [0, 1] ao longo do ciclo de caminhada.
/// @param aP Coeficiente da exponencial crescente da solução do LIPM.
/// @param aN Coeficiente da exponencial decrescente da solução do LIPM.
/// @param zs Posição do ZMP central (meio do passo).
/// @param z1 ZMP no início do passo (pé anterior).
/// @param z2 ZMP no final do passo (pé seguinte).
/// @return Posição do centro de massa (CoM) no instante `t`.
double OmnidirectionalWalkZMP::getCoM(double t, double aP, double aN, double zs,
                                      double z1, double z2) {
    double t1 = walkParams.doubleSupportRatio / 2.0;
    double t2 = 1.0 - t1;

    double expT = exp(walkParams.period * t / tZMP);
    double com = aP * expT + aN / expT;

    double m1 = (zs - z1) / (t1 * walkParams.period);
    double m2 = -(zs - z2) / (walkParams.period - (t2 * walkParams.period));

    if (t < t1) {
        com += z1 + m1 * walkParams.period * t// - t1)
               - tZMP * m1 * sinh(walkParams.period * (t - t1) / tZMP);
    } else if (t < t2) {
        com += zs;
    } else {
        com += zs + m2 * walkParams.period * (t - t2)
                - tZMP * m2 * sinh(walkParams.period * (t - t2) / tZMP);
    }

    return com;
}

/// Converte uma pose relativa em relação a uma pose base para coordenadas globais.
///
/// Esta função aplica uma transformação de pose (x, y, theta) baseada em rotação
/// e translação do referencial `pose`, útil para concatenar transformações em trajetória.
///
/// @param poseRelative Pose (x, y, theta) no referencial local (relativo à `pose`).
/// @param pose Pose base em coordenadas globais.
/// @return Pose resultante em coordenadas globais após a transformação.
itandroids_lib::math::Vector3<double> OmnidirectionalWalkZMP::poseGlobal(
        itandroids_lib::math::Vector3<double> poseRelative,
        itandroids_lib::math::Vector3<double> pose) {
    double ca = cos(pose.z);
    double sa = sin(pose.z);
    double x = pose.x + ca * poseRelative.x - sa * poseRelative.y;
    double y = pose.y + sa * poseRelative.x + ca * poseRelative.y;
    double z = pose.z + poseRelative.z;

    return itandroids_lib::math::Vector3<double>(x, y, MathUtils::normalizeAngle(z));
}

/// Retorna a pose `poseGlobal` relativa à pose `pose` em coordenadas locais.
///
/// @param poseGlobal Pose no referencial global (x, y, theta).
/// @param pose Pose de referência para o referencial local (x, y, theta).
/// @return Vetor 3D representando `poseGlobal` no referencial relativo a `pose`.
itandroids_lib::math::Vector3<double> OmnidirectionalWalkZMP::poseRelative(
        itandroids_lib::math::Vector3<double> poseGlobal, itandroids_lib::math::Vector3<double> pose) {
    double ca = cos(pose.z);
    double sa = sin(pose.z);
    double px = poseGlobal.x - pose.x;
    double py = poseGlobal.y - pose.y;
    double pa = poseGlobal.z - pose.z;
    return itandroids_lib::math::Vector3<double>(ca * px + sa * py, -sa * px + ca * py,
                                                 MathUtils::normalizeAngle(pa));
}

/// Calcula a fração da trajetória no eixo X para um instante de tempo `t`
/// usando interpolação suave em um ciclo de caminhada.
///
/// @param t Tempo normalizado no intervalo [0, 1] representando o progresso do passo.
/// @return Fração da interpolação para o eixo X no instante `t`.
double OmnidirectionalWalkZMP::computeXFrac(double t) {
    double t1 = walkParams.doubleSupportRatio / 2.0;
    double t2 = 1.0 - t1;

    if (t < t1)
        return 0.0;
    if (t < t2) {
        return 0.5f * (1.0 - cos(M_PI * (t - t1) / (t2 - t1)));
    }
    return 1.0;
}

/// Calcula a fração da trajetória no eixo Y para um instante de tempo `t`
/// usando interpolação suave em um ciclo de caminhada.
///
/// @param t Tempo normalizado no intervalo [0, 1] representando o progresso do passo.
/// @return Fração da interpolação para o eixo Y no instante `t`.
double OmnidirectionalWalkZMP::computeYFrac(double t) {
    double t1 = walkParams.doubleSupportRatio / 2.0;
    double t2 = 1.0 - t1;

    if (t < t1)
        return 0.0;
    if (t < t2) {
        return 0.5f * (1.0 - cos(M_PI * (t - t1) / (t2 - t1)));
    }
    return 1.0;
}

/// Calcula a fração da elevação no eixo Z do pé durante o passo em função de `t`.
/// A elevação é máxima no meio do passo e nula nas extremidades.
///
/// @param t Tempo normalizado no intervalo [0, 1] representando o progresso do passo.
/// @return Fração da elevação do pé (eixo Z) no instante `t`.
double OmnidirectionalWalkZMP::computeZFrac(double t) {
    double t1 = walkParams.doubleSupportRatio / 2.0;
    double t2 = 1.0 - t1;

    if (t > t1 && t < t2)
        return 0.5f * (1.0 - cos(2.0 * M_PI * (t - t1) / (t2 - t1)));
    return 0.0;
}

/// Calcula a fração da rotação (theta) em torno do eixo Z para o passo no tempo `t`.
/// Utiliza interpolação suave durante a fase de suporte simples.
///
/// @param t Tempo normalizado no intervalo [0, 1] representando o progresso do passo.
/// @return Fração da rotação theta no instante `t`.
double OmnidirectionalWalkZMP::computeThetaFrac(double t) {
    double t1 = walkParams.doubleSupportRatio / 2.0;
    double t2 = 1.0 - t1;

    if (t < t1)
        return 0.0;
    if (t < t2) {
        return 0.5f * (1.0 - cos(M_PI * (t - t1) / (t2 - t1)));
    }
    return 1.0;
}

itandroids_lib::math::Vector3<double> OmnidirectionalWalkZMP::getOdometry() {
    return odometry;
}

double OmnidirectionalWalkZMP::getPhase() const {
    if (walkParams.period <= 0.0)
        return 0.0;
    return walkTime / walkParams.period;
}

bool OmnidirectionalWalkZMP::isDoubleSupport() const {
    // Parado: os dois pes estao no chao o tempo todo.
    if (stoppingState == STOPPED)
        return true;

    // Mesmas fronteiras usadas por computeXFrac/YFrac/ZFrac: fora de
    // [t1, t2] o pe oscilante esta no chao (zFrac = 0).
    const double t = getPhase();
    const double t1 = walkParams.doubleSupportRatio / 2.0;
    const double t2 = 1.0 - t1;

    return (t < t1) || (t > t2);
}

SupportState OmnidirectionalWalkZMP::getSupportState() const {
    if (isDoubleSupport())
        return DOUBLE_SUPPORT;

    return leftFootIsSwing ? RIGHT_SUPPORT : LEFT_SUPPORT;
}

void OmnidirectionalWalkZMP::requestStop() {
    if (stoppingState == NONE)
        stoppingState = STOP_REQUESTED;
}

void OmnidirectionalWalkZMP::stopNow() {
    stoppingState = STOPPED;
    walkTime = 0.0;
}

bool OmnidirectionalWalkZMP::hasStopped() {
    return (stoppingState == STOPPED);
}

} /* namespace walking */

} /* namespace control */

