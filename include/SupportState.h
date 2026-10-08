/*
 * SupportState.h
 *
 * Qual pe sustenta o robo no instante atual do ciclo de caminhada.
 *
 * O gerador de marcha nunca teve uma maquina de estados explicita: o apoio
 * estava implicito no par (leftFootIsSwing, walkTime / period). Este enum e os
 * getters de OmnidirectionalWalkZMP apenas expoem essa informacao, sem alterar
 * a geracao de trajetoria, para que o estabilizador saiba em qual perna injetar
 * a correcao.
 *
 * Fica em um cabecalho proprio para que o estabilizador nao precise incluir o
 * gerador de marcha inteiro.
 */

#ifndef SUPPORT_STATE_H_
#define SUPPORT_STATE_H_

namespace control {

namespace walking {

enum SupportState {
    DOUBLE_SUPPORT,
    LEFT_SUPPORT,   ///< pe esquerdo no chao, direito no ar
    RIGHT_SUPPORT   ///< pe direito no chao, esquerdo no ar
};

} /* namespace walking */

} /* namespace control */

#endif /* SUPPORT_STATE_H_ */
