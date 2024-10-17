#pragma once
#define DEFAULT_NUMBER_OF_DIMENSIONS 3
#define DEFAULT_NUMBER_TYPE double
#define HEADER_TO_USE 1

#if HEADER_TO_USE == 1
#include <glm/fwd.hpp>
#define DEFAULT_GLM_PRECISION_LEVEL glm::packed_highp
using Vector = glm::vec<DEFAULT_NUMBER_OF_DIMENSIONS, DEFAULT_NUMBER_TYPE, DEFAULT_GLM_PRECISION_LEVEL>;

Vector getZeroVector();
Vector getOneVector();

#elif HEADER_TO_USE == 2
#include <Eigen/Dense>
using Vector = Eigen::Vector<DEFAULT_NUMBER_TYPE, DEFAULT_NUMBER_OF_DIMENSIONS>;



#else

#include <vector>


class Vector {
	std::vector<DEFAULT_FLOAT_TYPE> numbers;
	


};


#endif