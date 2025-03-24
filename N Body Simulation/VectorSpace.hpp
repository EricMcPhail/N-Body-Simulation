#pragma once
#include <Eigen/Dense>

#define VECTOR_SPACE_NUMBER_OF_DIMENSIONS 3
// The field that the scalars and elements of the vectors are in
#define VECTOR_SPACE_FIELD double
#define N_CHOOSE_TWO(N) = (N*(N-1))/2
// The dimension of the Lie algebra of the special orthogonal group SO(n)
// i.e. number of ways an n dimensional object can rotate
#define SPECIAL_ORTHOGONAL_GROUP_DIMENSION N_CHOOSE_TWO(VECTOR_SPACE_NUMBER_OF_DIMENSIONS)

using VectorND = Eigen::Vector<VECTOR_SPACE_FIELD, VECTOR_SPACE_NUMBER_OF_DIMENSIONS>;
using VectorNPlusOneD = Eigen::Vector<VECTOR_SPACE_FIELD, VECTOR_SPACE_NUMBER_OF_DIMENSIONS + 1>;
using MatrixND = Eigen::Matrix<VECTOR_SPACE_FIELD, VECTOR_SPACE_NUMBER_OF_DIMENSIONS, VECTOR_SPACE_NUMBER_OF_DIMENSIONS>;
using MatrixNPlusOneD = Eigen::Matrix<VECTOR_SPACE_FIELD, VECTOR_SPACE_NUMBER_OF_DIMENSIONS + 1, VECTOR_SPACE_NUMBER_OF_DIMENSIONS + 1>;
using Matrix = Eigen::MatrixX<VECTOR_SPACE_FIELD>;
