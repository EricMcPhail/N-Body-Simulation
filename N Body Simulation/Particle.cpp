#include "Particle.hpp"

Particle::Particle() : pos{ vec3{0.0, 0.0, 0.0} },
vel{ vec3{0.0, 0.0, 0.0} },
acc{ vec3{0.0, 0.0, 0.0} },
angle{ vec3{0.0, 0.0, 0.0} },
angle_vel{ vec3{0.0, 0.0, 0.0} },
angle_acc{ vec3{0.0, 0.0, 0.0} },
mass{ 1.0 },
restitution{ 1.0 },
radius{ 1.0 },
drag{ 0.1 }
{}

std::ostream& operator<<(std::ostream& out, const Particle& A) {
    out << "Position = (" << A.pos.x << ", " << A.pos.y << ", " << A.pos.z << ")\n";
    out << "Velocity = (" << A.vel.x << ", " << A.vel.y << ", " << A.vel.z << ")\n";
    out << "Acceleration = (" << A.acc.x << ", " << A.acc.y << ", " << A.acc.z << ")\n";
    out << "time = " << A.time << '\n';
    return out;
}