#ifndef __Vector2_h__
#define __Vector2_h__

#include <cmath>

namespace itandroids_lib {
namespace math {

template <typename T>
class Vector2 {
public:
    T x; ///< Componente X do vetor
    T y; ///< Componente Y do vetor

    /** Construtor padrão */
    Vector2() : x(0), y(0) {}

    /** Construtor com valores */
    Vector2(T x, T y) : x(x), y(y) {}

    /** Construtor de cópia */
    Vector2(const Vector2& other) : x(other.x), y(other.y) {}

    /** Operador de atribuição */
    Vector2& operator=(const Vector2& other) {
        if (this != &other) {
            x = other.x;
            y = other.y;
        }
        return *this;
    }

    /** Soma de dois vetores */
    Vector2 operator+(const Vector2& other) const {
        return Vector2(x + other.x, y + other.y);
    }

    /** Subtração de dois vetores */
    Vector2 operator-(const Vector2& other) const {
        return Vector2(x - other.x, y - other.y);
    }

    /** Multiplicação por escalar */
    Vector2 operator*(T scalar) const {
        return Vector2(x * scalar, y * scalar);
    }

    /** Divisão por escalar */
    Vector2 operator/(T scalar) const {
        return Vector2(x / scalar, y / scalar);
    }

    /** Produto escalar (dot product) */
    T dot(const Vector2& other) const {
        return x * other.x + y * other.y;
    }

    /** Comprimento (magnitude) do vetor */
    T abs() const {
        return std::sqrt(x * x + y * y);
    }

    /** Normalização do vetor */
    Vector2 normalize() const {
        T magnitude = abs();
        return magnitude == 0 ? Vector2(0, 0) : (*this / magnitude);
    }

    /** Rotação do vetor por um ângulo (em radianos) */
    Vector2 rotate(T angle) const {
        T cosA = std::cos(angle);
        T sinA = std::sin(angle);
        return Vector2(x * cosA - y * sinA, x * sinA + y * cosA);
    }

    /** Comparação de igualdade */
    bool operator==(const Vector2& other) const {
        return x == other.x && y == other.y;
    }

    /** Comparação de desigualdade */
    bool operator!=(const Vector2& other) const {
        return !(*this == other);
    }

    /** Operador composto de soma */
    Vector2& operator+=(const Vector2& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    /** Operador composto de subtração */
    Vector2& operator-=(const Vector2& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    /** Operador composto de multiplicação por escalar */
    Vector2& operator*=(T scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    /** Operador composto de divisão por escalar */
    Vector2& operator/=(T scalar) {
        x /= scalar;
        y /= scalar;
        return *this;
    }
};

} // namespace math
} // namespace itandroids_lib

#endif // __Vector2_h__
