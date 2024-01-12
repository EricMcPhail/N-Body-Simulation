#pragma once
#include <glm/glm.hpp>
#include "Vector.hpp"

class Particle {
public:
    Vector position;
    Vector velocity;
    Vector acceleration;

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
