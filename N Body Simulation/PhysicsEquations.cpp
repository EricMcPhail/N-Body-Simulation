#include "PhysicsEquations.hpp"
#include "Constants.hpp"
#include "Particle.hpp"
#include <limits> // for std::infinity
#include "MathFunctions.hpp"

std::pair<double, double> solveQuadradic(double a, double b, double c) {
    return solveQuadradic<double>(a, b, c);
}

VectorND getForceGravity(const VectorND& position_A, const double mass_A, const VectorND& position_B, const double mass_B) {
    VectorND relative_position_BA = position_B - position_A;
    double distance_between_A_and_B = relative_position_BA.norm();
    VectorND direction = relative_position_BA / distance_between_A_and_B;
    return ((GRAVITATIONAL_CONSTANT * mass_A * mass_B) / (distance_between_A_and_B * distance_between_A_and_B)) * direction;
}

VectorND getAccelerationGravity(const VectorND& posA, const double massA, const VectorND& posB, const double massB) {
    return getForceGravity(posA, massA, posB, massB) / massA;
}

double getKineticEnergy(const double mass, const VectorND& velocity) {
    double velocity_squared = velocity.dot( velocity);
    return 0.5 * mass * velocity_squared;
}

double getPotentialEnergy(const VectorND& position_A, const double mass_A, const VectorND& position_B, const double mass_B) {
    VectorND relative_position_AB = position_A - position_B;
    return -1.0 * ((GRAVITATIONAL_CONSTANT * mass_A * mass_B) / relative_position_AB.norm());
}

VectorND getForceGravityOnParticleAFromB(const Particle& A, const Particle& B) {
    if (!A.does_emit_gravitational_field || !B.does_emit_gravitational_field || !A.is_affected_by_gravity) {
        return VectorND::Zero();
    }
    return getForceGravity(A.position, A.mass, B.position, B.mass);
}

double getKineticEnergy(const Particle& P) {
    return getKineticEnergy(P.mass, P.velocity);
}

double getPotentialEnergy(const Particle& A, const Particle& B) {
    if (!A.does_emit_gravitational_field || !B.does_emit_gravitational_field || !A.is_affected_by_gravity || !B.is_affected_by_gravity) {
        return 0.0;
    }
    return getPotentialEnergy(A.position, A.mass, B.position, B.mass);
}

void doCollision(Particle& A, Particle& B) {
    A.collision_count++;
    B.collision_count++;

    double coefficient_of_restitution = 1.0;
    VectorND relative_velocity_AB = A.velocity - B.velocity;
    if (A.mass == std::numeric_limits<double>::infinity()) {
        if (B.mass == std::numeric_limits<double>::infinity()) {
            // This is not really mathematically correct but I am doing it for fun
            A.velocity = (A.velocity + B.velocity - coefficient_of_restitution * relative_velocity_AB) / 2.0;
            B.velocity = (A.velocity + B.velocity + coefficient_of_restitution * relative_velocity_AB) / 2.0;
            return;
        }
        B.velocity = A.velocity + relative_velocity_AB * coefficient_of_restitution;
        return;
    }
    if (B.mass == std::numeric_limits<double>::infinity()) {
        A.velocity = B.velocity - relative_velocity_AB * coefficient_of_restitution;
        return;
    }

    double mass_sum = A.mass + B.mass;
    double inverse_mass_sum = 1.0 / mass_sum;
    VectorND momentum_A = A.mass * A.velocity;
    VectorND momentum_B = B.mass * B.velocity;
    VectorND momentum_sum = momentum_A + momentum_B;

    A.velocity = (momentum_sum - B.mass * coefficient_of_restitution * relative_velocity_AB) * inverse_mass_sum;
    B.velocity = (momentum_sum + A.mass * coefficient_of_restitution * relative_velocity_AB) * inverse_mass_sum;
}




double timeTillCollisionAccurate(const Particle& A, const Particle& B) {
    VectorND relative_position_AB = A.position - B.position;
    VectorND relative_velocity_AB = A.velocity - B.velocity;
    VectorND relative_acceleration_AB = A.acceleration - B.acceleration;
    double radii_sum = A.radius + B.radius;
    double radii_sum_sqrd = radii_sum * radii_sum;
    double distance_between_centers_sqrd = relative_position_AB.dot(relative_position_AB);
    if (distance_between_centers_sqrd <= radii_sum_sqrd) return 0.0;

    double a = 0.25 * relative_acceleration_AB.dot(relative_acceleration_AB);
    double b = relative_velocity_AB.dot(relative_acceleration_AB);
    double c = relative_position_AB.dot(relative_acceleration_AB) + relative_velocity_AB.dot(relative_velocity_AB);
    double d = 2.0 * relative_position_AB.dot(relative_velocity_AB);
    double e = distance_between_centers_sqrd - radii_sum_sqrd;

    // TODO SOLVE THIS QUARTIC FUNCTION
    return -1.0;
}

double timeTillCollision(const Particle& A, const Particle& B) {
    VectorND relative_position_AB = A.position - B.position;
    double radii_sum = A.radius + B.radius;
    double radii_sum_sqrd = radii_sum * radii_sum;
    double distance_between_centers_sqrd = relative_position_AB.dot(relative_position_AB);
    if (distance_between_centers_sqrd <= radii_sum_sqrd) return 0.0; // already touching or intercecting 

    VectorND relative_velocity_AB = A.velocity - B.velocity;
    double a = relative_velocity_AB.dot(relative_velocity_AB);
    double b = 2.0 * relative_position_AB.dot(relative_velocity_AB);
    double c = distance_between_centers_sqrd - radii_sum_sqrd;
    std::pair<double, double> sol = solveQuadradic(a, b, c);
    // If value was complex, no solution so my convention here is to return -inf
    if (std::isnan(sol.first)) return -std::numeric_limits<double>::infinity();

    // Solution to quadradic formula was not complex
    // Return the number with the smallest absolute value, prioritizing positive numbers
    // meaning if there is a positive number, and a negative number, we return the positive number
    // otherwise return the negative number closest to zero
    if (sol.first < sol.second) {
        if (sol.first >= 0.0) return sol.first;
        else if (sol.second >= 0.0) return sol.second;
        else return sol.second;
    }
    else {
        if (sol.second >= 0.0) return sol.second;
        else if (sol.first >= 0.0) return sol.first;
        else return sol.first;
    }
}