#pragma once
#include "NumberType.hpp"
#include <cassert>

inline constexpr size_t power_of_two(size_t exponent) {
	if (exponent == 0) return 1;
	else return 2 * power_of_two(exponent - 1);
}

template<class FloatType> static inline std::pair<FloatType, FloatType> solveQuadradic(FloatType a, FloatType b, FloatType c) {
    assert(std::numeric_limits<FloatType>::has_infinity && std::numeric_limits<FloatType>::has_quiet_NaN);
    if (a == 0.0) return std::pair<FloatType, FloatType>{-std::numeric_limits<FloatType>::infinity(), std::numeric_limits<FloatType>::infinity()};
    FloatType b_sqrd = b * b;
    FloatType temp_coeff_1 = b_sqrd - 4.0 * a * c;
    if (temp_coeff_1 < 0.0) return std::pair<FloatType, FloatType>{std::numeric_limits<FloatType>::quiet_NaN(), std::numeric_limits<FloatType>::quiet_NaN()};
    FloatType temp_coeff_2 = sqrt(temp_coeff_1);
    FloatType fraction = 1.0 / (2.0 * a);
    FloatType positive = (-b + temp_coeff_2) * fraction;
    FloatType negative = (-b - temp_coeff_2) * fraction;
    return std::pair<FloatType, FloatType>{positive, negative};
}