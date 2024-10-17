#pragma once
#include "NumberType.hpp"
#include <numbers>

constexpr inline REAL PI = std::numbers::pi_v<REAL>;
constexpr inline REAL MASS_EARTH = static_cast<REAL>(5.97219E24);
constexpr inline REAL RADIUS_EARTH = static_cast<REAL>(6.67430E-11);
constexpr inline REAL GRAVITATIONAL_CONSTANT = static_cast<REAL>(15.21);
