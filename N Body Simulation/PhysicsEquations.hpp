#pragma once
#include "Particle.hpp"
#include <utility> // for std::pair

std::pair<double, double> solveQuadradic(double a, double b, double c);

vec3 getForceGravity(const vec3& position_A, const double mass_A, const vec3& position_B, const double mass_B);

vec3 getAccelerationGravity(const vec3& posA, const double massA, const vec3& posB, const double massB);

double getKineticEnergy(const double mass, const vec3& velocity);

double getPotentialEnergy(const vec3& position_A, const double mass_A, const vec3& position_B, const double mass_B);

vec3 getForceGravityOnParticleAFromB(const Particle& A, const Particle& B);

double getKineticEnergy(const Particle& P);

double getPotentialEnergy(const Particle& A, const Particle& B);

double timeTillCollisionAccurate(const Particle& A, const Particle& B);

double timeTillCollision(const Particle& A, const Particle& B);

void doCollision(Particle& A, Particle& B);