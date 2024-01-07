#include "PhysicsEquations.hpp"
#include "Constants.hpp"
#include "Particle.hpp"
#include <limits> // for std::infinity

std::pair<double, double> solveQuadradic(double a, double b, double c) {
    if (a == 0.0) return std::pair<double, double>{-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    double b_sqrd = b * b;
    double temp_coeff_1 = b_sqrd - 4.0 * a * c;
    if (temp_coeff_1 < 0.0) return std::pair<double, double>{std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
    double temp_coeff_2 = sqrt(temp_coeff_1);
    double fraction = 1.0 / (2.0 * a);
    double positive = (-b + temp_coeff_2) * fraction;
    double negative = (-b - temp_coeff_2) * fraction;
    return std::pair<double, double>{positive, negative};
}

vec3 getForceGravity(const vec3& position_A, const double mass_A, const vec3& position_B, const double mass_B) {
    vec3 relative_position_BA = position_B - position_A;
    double distance_between_A_and_B = glm::length(relative_position_BA);
    vec3 direction = relative_position_BA / distance_between_A_and_B;
    return ((GRAVITATIONAL_CONSTANT * mass_A * mass_B) / (distance_between_A_and_B * distance_between_A_and_B)) * direction;
}

vec3 getAccelerationGravity(const vec3& posA, const double massA, const vec3& posB, const double massB) {
    return getForceGravity(posA, massA, posB, massB) / massA;
}

double getKineticEnergy(const double mass, const vec3& velocity) {
    double velocity_squared = glm::dot(velocity, velocity);
    return 0.5 * mass * velocity_squared;
}

double getPotentialEnergy(const vec3& position_A, const double mass_A, const vec3& position_B, const double mass_B) {
    vec3 relative_position_AB = position_A - position_B;
    return -1.0 * ((GRAVITATIONAL_CONSTANT * mass_A * mass_B) / glm::length(relative_position_AB));
}

vec3 getForceGravityOnParticleAFromB(const Particle& A, const Particle& B) {
    if (!A.does_emit_gravitational_field || !B.does_emit_gravitational_field || !A.is_affected_by_gravity) {
        return vec3{ 0.0, 0.0, 0.0 };
    }
    return getForceGravity(A.pos, A.mass, B.pos, B.mass);
}

double getKineticEnergy(const Particle& P) {
    return getKineticEnergy(P.mass, P.vel);
}

double getPotentialEnergy(const Particle& A, const Particle& B) {
    if (!A.does_emit_gravitational_field || !B.does_emit_gravitational_field || !A.is_affected_by_gravity || !B.is_affected_by_gravity) {
        return 0.0;
    }
    return getPotentialEnergy(A.pos, A.mass, B.pos, B.mass);
}

double timeTillCollisionAccurate(const Particle& A, const Particle& B) {
    vec3 relative_position_AB = A.pos - B.pos;
    vec3 relative_velocity_AB = A.vel - B.vel;
    vec3 relative_acceleration_AB = A.acc - B.acc;
    double radii_sum = A.radius + B.radius;
    double radii_sum_sqrd = radii_sum * radii_sum;
    double distance_between_centers_sqrd = glm::dot(relative_position_AB, relative_position_AB);
    if (distance_between_centers_sqrd <= radii_sum_sqrd) return 0.0;

    double a = 0.25 * glm::dot(relative_acceleration_AB, relative_acceleration_AB);
    double b = glm::dot(relative_velocity_AB, relative_acceleration_AB);
    double c = glm::dot(relative_position_AB, relative_acceleration_AB) + glm::dot(relative_velocity_AB, relative_velocity_AB);
    double d = 2.0 * glm::dot(relative_position_AB, relative_velocity_AB);
    double e = distance_between_centers_sqrd - radii_sum_sqrd;

    // TODO SOLVE THIS QUARTIC FUNCTION
    return -1.0;
}

double timeTillCollision(const Particle& A, const Particle& B) {
    vec3 relative_position_AB = A.pos - B.pos;
    double radii_sum = A.radius + B.radius;
    double radii_sum_sqrd = radii_sum * radii_sum;
    double distance_between_centers_sqrd = glm::dot(relative_position_AB, relative_position_AB);
    if (distance_between_centers_sqrd <= radii_sum_sqrd) return 0.0; // already touching or intercecting 

    vec3 relative_velocity_AB = A.vel - B.vel;
    double a = glm::dot(relative_velocity_AB, relative_velocity_AB);
    double b = 2.0 * glm::dot(relative_position_AB, relative_velocity_AB);
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

void doCollision(Particle& A, Particle& B) {
    A.collision_count++;
    B.collision_count++;

    double coefficient_of_restitution = 1.0;
    vec3 relative_velocity_AB = A.vel - B.vel;
    if (A.mass == std::numeric_limits<double>::infinity()) {
        if (B.mass == std::numeric_limits<double>::infinity()) {
            // This is not really mathematically correct but I am doing it for fun
            A.vel = (A.vel + B.vel - coefficient_of_restitution * relative_velocity_AB) / 2.0;
            B.vel = (A.vel + B.vel + coefficient_of_restitution * relative_velocity_AB) / 2.0;
            return;
        }
        B.vel = A.vel + relative_velocity_AB * coefficient_of_restitution;
        return;
    }
    if (B.mass == std::numeric_limits<double>::infinity()) {
        A.vel = B.vel - relative_velocity_AB * coefficient_of_restitution;
        return;
    }

    double mass_sum = A.mass + B.mass;
    double inverse_mass_sum = 1.0 / mass_sum;
    vec3 momentum_A = A.mass * A.vel;
    vec3 momentum_B = B.mass * B.vel;
    vec3 momentum_sum = momentum_A + momentum_B;

    

    A.vel = (momentum_sum - B.mass * coefficient_of_restitution * relative_velocity_AB) * inverse_mass_sum;
    B.vel = (momentum_sum + A.mass * coefficient_of_restitution * relative_velocity_AB) * inverse_mass_sum;
}
