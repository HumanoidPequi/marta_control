/*
 * Interpolator.h
 *
 * Created on: Oct 16, 2015
 * Author: Lucas Reis
 */

#ifndef SOURCE_MATH_INTERPOLATOR_H_
#define SOURCE_MATH_INTERPOLATOR_H_

namespace itandroids_lib {
namespace math {

/**
 * Classe base abstrata para interpoladores.
 */
class Interpolator {
public:
    /**
     * Destrutor virtual para permitir herança adequada.
     */
    virtual ~Interpolator() {}

    /**
     * Método de interpolação. Deve ser implementado pelas classes derivadas.
     * @param fraction Um valor entre 0 e 1 que representa a fração do intervalo.
     * @return O valor interpolado.
     */
    virtual double interpolate(double fraction) = 0;
};

} // namespace math
} // namespace itandroids_lib

#endif /* SOURCE_MATH_INTERPOLATOR_H_ */
