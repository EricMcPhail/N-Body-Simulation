#pragma once
#include "NumberType.hpp"
#include <Eigen/Dense>
#define VECTOR_SPACE_NUMBER_OF_DIMENSIONS 3
#define VECTOR_SPACE_FIELD REAL // The field that the scalars and elements of the vectors are in
using VectorND = Eigen::Vector<VECTOR_SPACE_FIELD, VECTOR_SPACE_NUMBER_OF_DIMENSIONS>;
using VectorNPlusOneD = Eigen::Vector<VECTOR_SPACE_FIELD, VECTOR_SPACE_NUMBER_OF_DIMENSIONS + 1ull>;
using MatrixND = Eigen::Matrix<VECTOR_SPACE_FIELD, VECTOR_SPACE_NUMBER_OF_DIMENSIONS, VECTOR_SPACE_NUMBER_OF_DIMENSIONS>;
using MatrixNPlusOneD = Eigen::Matrix<VECTOR_SPACE_FIELD, VECTOR_SPACE_NUMBER_OF_DIMENSIONS + 1ull, VECTOR_SPACE_NUMBER_OF_DIMENSIONS + 1ull>;
using Matrix = Eigen::MatrixX<VECTOR_SPACE_FIELD>;

#if 0
class VectorSpace {
private:

protected:
    VectorSpace() : ones(VectorND::Ones())
	{}
    static VectorSpace* vectorspace_;
public:
	const VectorND ones;

    /**
     * Singletons should not be cloneable.
     */
    VectorSpace(VectorSpace& other) = delete;
    /**
     * Singletons should not be assignable.
     */
    void operator=(const VectorSpace&) = delete;

    /**
    * This is the static method that controls the access to the singleton
    * instance. On the first run, it creates a singleton object and places it
    * into the static field. On subsequent runs, it returns the client existing
    * object stored in the static field.
    */
    static VectorSpace* GetInstance();
};

VectorSpace* VectorSpace::vectorspace_ = nullptr;



VectorSpace* VectorSpace::GetInstance() {
    /**
     * This is a safer way to create an instance. instance = new Singleton is
     * dangeruous in case two instance threads wants to access at the same time
     */
    if (vectorspace_ == nullptr) {
        vectorspace_ = new VectorSpace();
    }
    return vectorspace_;
}
#endif




