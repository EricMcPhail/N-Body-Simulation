#pragma once
#include "VectorSpace.hpp" // For vector lol
#include <vector> // for std::vector
#include <utility> // for std::pair
#include <glm/glm.hpp> // For rendering


class Model;
class Shader;
class Particle;

class ParticleManager {
    VectorND getFutureNetForce(size_t particle_index, const double future_time, const std::vector<VectorND>& future_positions, const std::vector<VectorND>& future_velocities) const;

    VectorND getFutureNetAcceleration(size_t particle_index, const double future_time, const std::vector<VectorND>& future_positions, const std::vector<VectorND>& future_velocities) const;

    void systemOfEquations(const double t, const std::vector<std::pair<VectorND, VectorND>>& z, std::vector<std::pair<VectorND, VectorND>>& dzdt) const;

    void updateVerlet(double dt);

    void updateSystemVerlet(double dt);

    void updateSystemRK4(double dt);

    void updateSystemRK(double dt, unsigned int number_of_steps = 4);

    void updateSystemRK45(double dt, size_t number_of_steps = 10, double tolerance = 1e-12);

    std::vector<Particle> particles;
    double time = 0.0;
public:
    friend class Simulation;

    double getTotalKineticEnergy() const;

    double getTotalPotentialEnergy() const;

    double getTotalEnergy() const;

    VectorND getNetForceOnParticle(const size_t particle_index) const;

    VectorND getNetAccelerationOnParticle(const size_t particle_index) const;

    void add(double mass = 1.0, VectorND pos = VectorND::Zero(), VectorND vel = VectorND::Zero(), VectorND acc = VectorND::Zero());

    void update(double dt, size_t integration_method = 0);

    void draw(Model& particle_model, Shader& shader, const glm::vec3& scale = { 1.0f, 1.0f, 1.0f });

    void updateDataInGPU(Model& particle_model, const glm::vec3& scale = { 1.0f, 1.0f, 1.0f });

    void updateAndResolveCollisions(double dt, const size_t integration_method = 1, const bool avoid_displacement = true);
};
