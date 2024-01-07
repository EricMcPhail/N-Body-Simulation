#pragma once
#include <vector>

extern class Model;
extern class Shader;
extern class Particle;

#include <glm/glm.hpp>

using vec3 = glm::highp_dvec3;

class ParticleManager {
    vec3 getFutureNetForce(size_t particle_index, const double future_time, const std::vector<vec3>& future_positions, const std::vector<vec3>& future_velocities) const;
    
    vec3 getFutureNetAcceleration(size_t particle_index, const double future_time, const std::vector<vec3>& future_positions, const std::vector<vec3>& future_velocities) const;

    void systemOfEquations(const double t, const std::vector<std::pair<vec3, vec3>>& z, std::vector<std::pair<vec3, vec3>>& dzdt) const;

    void updateVerlet(double dt);

    void updateSystemVerlet(double dt);

    void updateSystemRK4(double dt);

    void updateSystemRK(double dt, unsigned int number_of_steps = 4);

    // Runge-Kutta-Fehlberg (RK45) adaptive integration method for a system of equations
    void updateSystemRK45(double dt, size_t number_of_steps = 10, double tolerance = 1e-12);


    std::vector<Particle> particles;
    double time = 0.0;
public: 
    friend class Simulation;

    double getTotalKineticEnergy() const;

    double getTotalPotentialEnergy() const;

    double getTotalEnergy() const;

    vec3 getNetForceOnParticle(const size_t particle_index) const;

    vec3 getNetAccelerationOnParticle(const size_t particle_index) const;

    void add(double mass = 1.0, vec3 pos = vec3{ 0.0, 0.0, 0.0 }, vec3 vel = vec3{ 0.0, 0.0, 0.0 }, vec3 acc = vec3{ 0.0, 0.0, 0.0 });

    void print();

    void update(double dt, size_t integration_method = 0);

    void draw(Model& particle_model, Shader& shader);

    void updateDataInGPU(Model& particle_model);


    void updateAndResolveCollisions(double dt, const size_t integration_method = 1, const bool avoid_displacement = true);



};




