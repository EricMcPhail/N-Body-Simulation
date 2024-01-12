#include "Vector.hpp"

#if HEADER_TO_USE == 1
#include <glm/glm.hpp>
Vector getZeroVector() {
	Vector ret_val;
	for (size_t i = 0; i < DEFAULT_NUMBER_OF_DIMENSIONS; i++) {
		ret_val[i] = (DEFAULT_NUMBER_TYPE)(0);
	}
	return ret_val;
}

Vector getOneVector() {
	Vector ret_val;
	for (size_t i = 0; i < DEFAULT_NUMBER_OF_DIMENSIONS; i++) {
		ret_val[i] = (DEFAULT_NUMBER_TYPE)(1);
	}
	return ret_val;
}
#endif