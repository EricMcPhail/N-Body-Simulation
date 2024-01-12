#pragma once
#include <utility> // for std::pair
#include "Vector.hpp"

class Particle;

std::pair<double, double> solveQuadradic(double a, double b, double c);

Vector getForceGravity(const Vector& position_A, const double mass_A, const Vector& position_B, const double mass_B);

Vector getAccelerationGravity(const Vector& posA, const double massA, const Vector& posB, const double massB);

double getKineticEnergy(const double mass, const Vector& velocity);

double getPotentialEnergy(const Vector& position_A, const double mass_A, const Vector& position_B, const double mass_B);

Vector getForceGravityOnParticleAFromB(const Particle& A, const Particle& B);

double getKineticEnergy(const Particle& P);

double getPotentialEnergy(const Particle& A, const Particle& B);

double timeTillCollisionAccurate(const Particle& A, const Particle& B);

double timeTillCollision(const Particle& A, const Particle& B);

void doCollision(Particle& A, Particle& B);
