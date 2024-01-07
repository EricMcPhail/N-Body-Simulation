#pragma once
#include <glm/glm.hpp>
#include <iostream>

using vec3 = glm::highp_dvec3;

struct Particle {
public:
    vec3 pos;
    vec3 vel;
    vec3 acc;
    vec3 angle;
    vec3 angle_vel;
    vec3 angle_acc;

    double mass = 1.0;
    double restitution = 1.0;
    double radius = 1.0;
    double drag = 0.1;
    double mu = 0.2;
    double time = 0.0;

    bool has_infinite_mass = false;
    bool is_affected_by_gravity = true;
    bool does_emit_gravitational_field = true;
    bool is_collisions_enabled = true;

    size_t collision_count = 0;



    Particle();
};

std::ostream& operator<<(std::ostream& out, const Particle& A);