#pragma once
#include "VectorSpace.hpp"
#define N_CHOOSE_TWO(N) N*(N - 1) / 2
#define NUMBER_OF_ROTATIONAL_AXES N_CHOOSE_TWO(VECTOR_SPACE_NUMBER_OF_DIMENSIONS)

struct RigidBody {
	VectorND position;
	VectorND velocity;
	VectorND acceleration;

	REAL mass;
	REAL inv_mass;
	REAL coefficient_of_restitution;






	VectorND center_of_mass;
	MatrixND moment_of_inertia_tensor;
	MatrixND inv_moment_of_inertia_tensor;
	MatrixND orientation_matrix;
	MatrixND angular_velocity_matrix;
	MatrixND angular_acceleration_matrix;


	

	Eigen::Vector<VECTOR_SPACE_FIELD, NUMBER_OF_ROTATIONAL_AXES> get_angular_position();
	Eigen::Vector<VECTOR_SPACE_FIELD, NUMBER_OF_ROTATIONAL_AXES> get_angular_velocity();
	Eigen::Vector<VECTOR_SPACE_FIELD, NUMBER_OF_ROTATIONAL_AXES> get_angular_momentum();
	Eigen::Vector<VECTOR_SPACE_FIELD, NUMBER_OF_ROTATIONAL_AXES> get_angular_acceleration();



	VectorND getVelocityAtPoint(const VectorND& point) {
		return angular_velocity_matrix * (point - center_of_mass);
	}
};



MatrixND getAngularVelocityTensor(const std::array<VECTOR_SPACE_FIELD, VECTOR_SPACE_NUMBER_OF_DIMENSIONS>& vals) {
	MatrixND ret_val = MatrixND::Zero();
	size_t counter = 0;
	for (size_t i = 0; i < VECTOR_SPACE_NUMBER_OF_DIMENSIONS; i++) {
		for (size_t j = i + 1; j < VECTOR_SPACE_NUMBER_OF_DIMENSIONS; j++) {
			ret_val(i, j) = -vals[counter];
			ret_val(j, i) = vals[counter];
			counter++;
		}
	}
	return ret_val;
}
/*
* collision_normal is the direction of the force to be applied on B
*/
void resolveCollision(RigidBody A, RigidBody B, VectorND collision_point, VectorND collision_normal) {

	REAL coefficient_of_restitution = std::min(A.coefficient_of_restitution, B.coefficient_of_restitution); // can be cached
	REAL sum_of_inv_masses = A.inv_mass + B.inv_mass; // can be cached
	REAL temp_numerator_coeff_1 = -(1 + coefficient_of_restitution); // can be cached

	VectorND n = collision_normal;
	REAL impulse_magnitude;
	VectorND rAP = collision_point - A.center_of_mass;
	VectorND rBP = collision_point - B.center_of_mass;
	VectorND vAP = A.angular_velocity_matrix * rAP;
	VectorND vBP = B.angular_velocity_matrix * rBP;
	VectorND vAB = vAP - vBP;


	REAL numerator = temp_numerator_coeff_1 * vAB.dot(n);
	REAL denom1 = n.dot(n) * sum_of_inv_masses;
	VectorND denom21 = (A.inv_moment_of_inertia_tensor * (rAP.cross(n))).cross(rAP);
	VectorND denom22 = (B.inv_moment_of_inertia_tensor * (rBP.cross(n))).cross(rBP);
	REAL denom = denom1 + (denom21 + denom22).dot(n);


	VectorND impulse;
	A.velocity += A.inv_mass * impulse;
	A.angular_momentum += (Position - Configuration.CMPosition).cross(impulse);
	// apply impulse to primary quantities
	Configuration.CMVelocity += Body.OneOverMass * Impulse;
	Configuration.AngularMomentum += CrossProduct(R, Impulse);

	// compute affected auxiliary quantities
	Configuration.AngularVelocity = Configuration.InverseWorldInertiaTensor * Configuration.AngularMomentum;
}

// point point
// point edge
// edge edge
// edge face
// face face
// face point

