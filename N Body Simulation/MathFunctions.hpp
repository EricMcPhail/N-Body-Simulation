#pragma once
#include "VectorSpace.hpp"
#include <utility>
#include <vector>
#include <cassert>



template<class FloatType>
std::pair<FloatType, FloatType> solveQuadradic(FloatType a, FloatType b, FloatType c) {
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



// Helper: Build a Givens rotation in the (i,j) plane for an N-dimensional space.
template <int N>
Eigen::Matrix<double, N, N> givensRotation(int i, int j, double angle) {
    Eigen::Matrix<double, N, N> G = Eigen::Matrix<double, N, N>::Identity();
    double c = std::cos(angle);
    double s = std::sin(angle);
    G(i, i) = c;
    G(i, j) = s;
    G(j, i) = -s;
    G(j, j) = c;
    return G;
}




// Extract rotation angles in lexicographic order without needing to reverse the array.
// The rotation matrix is assumed to be constructed as:
//   R = G(0,1,θ₀₁) * G(0,2,θ₀₂) * ... * G(N-2, N-1, θ₍ₙ₋₂,ₙ₋₁₎)
// The extraction must "peel off" the rotations in reverse order. 
// Here we preallocate a vector and insert each extracted angle in the correct position.
template <int N>
std::vector<double> extractRotationAnglesLex(const Eigen::Matrix<double, N, N>& R_input) {
    // Work on a copy since we'll modify it.
    Eigen::Matrix<double, N, N> R = R_input;
    const int numAngles = (N * (N - 1)) / 2;
    std::vector<double> angles(numAngles, 0.0);

    // Use an index that starts at the end so that the first extracted angle
    // is stored at index numAngles-1, and so on.
    int index = numAngles - 1;

    // Loop over axis pairs in reverse order (peeling off rotations from the right).
    for (int i = N - 2; i >= 0; i--) {
        for (int j = N - 1; j > i; j--) {
            // Extract the angle in the (i, j) plane.
            double angle = std::atan2(R(j, i), R(i, i));
            // Store in the precomputed index to achieve lexicographic order.
            angles[index] = angle;
            index--;

            // Build the corresponding Givens rotation and remove it from R.
            Eigen::Matrix<double, N, N> G = givensRotation<N>(i, j, angle);
            R = R * G.transpose();
        }
    }
    return angles;
}