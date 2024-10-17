#pragma once
#include "VectorSpace.hpp"
#include <utility> // for std::pair

class Particle;

std::pair<double, double> solveQuadradic(double a, double b, double c);

VectorND getForceGravity(const VectorND& position_A, const double mass_A, const VectorND& position_B, const double mass_B);

VectorND getAccelerationGravity(const VectorND& posA, const double massA, const VectorND& posB, const double massB);

double getKineticEnergy(const double mass, const VectorND& velocity);

double getPotentialEnergy(const VectorND& position_A, const double mass_A, const VectorND& position_B, const double mass_B);

VectorND getForceGravityOnParticleAFromB(const Particle& A, const Particle& B);

double getKineticEnergy(const Particle& P);

double getPotentialEnergy(const Particle& A, const Particle& B);

double timeTillCollisionAccurate(const Particle& A, const Particle& B);

double timeTillCollision(const Particle& A, const Particle& B);

void doCollision(Particle& A, Particle& B);

