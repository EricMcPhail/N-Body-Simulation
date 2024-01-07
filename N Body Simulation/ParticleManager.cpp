#include "ParticleManager.hpp"
#include <utility> // for std::pair
#include <vector> // for std::vector
#include <glm/gtc/matrix_transform.hpp> // for glm::translate
#include <glm/glm.hpp> // also needed for vec stuff i think

#include "Model.hpp"
#include "PhysicsEquations.hpp"

vec3 ParticleManager::getFutureNetForce(size_t particle_index, const double future_time, const std::vector<vec3>& future_positions, const std::vector<vec3>& future_velocities) const {
    vec3 netForceOnParticle = vec3(0.0, 0.0, 0.0);
    // Calculate force on self

    // Calculate force by other particles
    for (size_t i = 0; i < particles.size(); i++) {
        if (i == particle_index) continue;
        if (!particles[i].does_emit_gravitational_field) continue;
        netForceOnParticle += getForceGravity(future_positions[particle_index], particles[particle_index].mass, future_positions[i], particles[i].mass);
    }
    return netForceOnParticle;
}

vec3 ParticleManager::getFutureNetAcceleration(size_t particle_index, const double future_time, const std::vector<vec3>& future_positions, const std::vector<vec3>& future_velocities) const {
    if (particles[particle_index].is_affected_by_gravity == false) return vec3{ 0.0, 0.0, 0.0 };
    return getFutureNetForce(particle_index, future_time, future_positions, future_velocities) / particles[particle_index].mass;
}

//----------------------------------------------------------------------------------------------------//
//--------------------------------------| INTEGRATION METHODS |---------------------------------------//
//----------------------------------------------------------------------------------------------------//
// z     = (y, dy/dx)          = (position, velocity)
// dz/dt = (dy/dt, d^2y, dt^2) = (velocity, acceleration)
void ParticleManager::systemOfEquations(const double t, const std::vector<std::pair<vec3, vec3>>& z, std::vector<std::pair<vec3, vec3>>& dzdt) const {
    std::vector<vec3> future_posistions(particles.size());
    std::vector<vec3> future_velocities(particles.size());

    for (size_t i = 0; i < particles.size(); i++) {
        future_posistions[i] = z[i].first;
        future_velocities[i] = z[i].second;
    }

    for (int i = 0; i < particles.size(); ++i) {
        dzdt[i].first = z[i].second;
        dzdt[i].second = getFutureNetAcceleration(i, t, future_posistions, future_velocities);
    }
}

void ParticleManager::updateVerlet(double dt) {
    for (size_t i = 0; i < particles.size(); i++) {
        vec3 new_pos = particles[i].pos + particles[i].vel * dt + particles[i].acc * (dt * dt * 0.5);
        vec3 new_acc = getNetAccelerationOnParticle(i);
        vec3 new_vel = particles[i].vel + (particles[i].acc + new_acc) * (dt * 0.5);
        particles[i].pos = new_pos;
        particles[i].vel = new_vel;
        particles[i].acc = new_acc;
    }
    time += dt;
}

void ParticleManager::updateSystemVerlet(double dt) {
    std::vector<vec3> future_posistions(particles.size());
    std::vector<vec3> future_velocities(particles.size());

    for (size_t i = 0; i < particles.size(); i++) {
        future_posistions[i] = particles[i].pos + particles[i].vel * dt + particles[i].acc * (dt * dt * 0.5);
        future_velocities[i] = particles[i].vel; // Not entirly correct but not dealing with that
    }

    for (size_t i = 0; i < particles.size(); i++) {
        vec3 new_pos = particles[i].pos + particles[i].vel * dt + particles[i].acc * (dt * dt * 0.5);
        vec3 new_acc = getFutureNetAcceleration(i, time + dt, future_posistions, future_velocities);
        vec3 new_vel = particles[i].vel + (particles[i].acc + new_acc) * (dt * 0.5);
        particles[i].pos = new_pos;
        particles[i].vel = new_vel;
        particles[i].acc = new_acc;
    }


    time += dt;
}

void ParticleManager::updateSystemRK4(double dt) {
    const size_t num_particles = particles.size();

    // Pairs of Temperary Position and Velocity vectors for each Particle
    std::vector<std::pair<vec3, vec3>> z(num_particles);

    // k1,...,k4 are arrays of pairs of velocity and acceleration vectors for each particle
    std::vector<std::pair<vec3, vec3>> k1(num_particles);
    std::vector<std::pair<vec3, vec3>> k2(num_particles);
    std::vector<std::pair<vec3, vec3>> k3(num_particles);
    std::vector<std::pair<vec3, vec3>> k4(num_particles);

    for (size_t i = 0; i < num_particles; ++i) {
        z[i] = std::pair<vec3, vec3>(particles[i].pos, particles[i].vel);
    }

    systemOfEquations(time, z, k1);

    for (int j = 0; j < num_particles; ++j) {
        z[j].first = particles[j].pos + (dt / 2.0) * k1[j].first;
        z[j].second = particles[j].vel + (dt / 2.0) * k1[j].second;
    }

    systemOfEquations(time + dt / 2.0, z, k2);

    for (int j = 0; j < num_particles; ++j) {
        z[j].first = particles[j].pos + (dt / 2.0) * k2[j].first;
        z[j].second = particles[j].vel + (dt / 2.0) * k2[j].second;
    }

    systemOfEquations(time + dt / 2.0, z, k3);

    for (int j = 0; j < num_particles; ++j) {
        z[j].first = particles[j].pos + dt * k3[j].first;
        z[j].second = particles[j].vel + dt * k3[j].second;
    }

    systemOfEquations(time + dt, z, k4);

    for (int j = 0; j < num_particles; ++j) {
        particles[j].pos += (dt / 6.0) * (k1[j].first + 2.0 * k2[j].first + 2.0 * k3[j].first + k4[j].first);
        particles[j].vel += (dt / 6.0) * (k1[j].second + 2.0 * k2[j].second + 2.0 * k3[j].second + k4[j].second);
    }
    for (int j = 0; j < num_particles; j++) {
        particles[j].acc = getNetAccelerationOnParticle(j);
    }
    time += dt;
}

void ParticleManager::updateSystemRK(double dt, unsigned int number_of_steps) {
    const size_t num_particles = particles.size();

    // Pairs of Temperary Position and Velocity vectors for each Particle
    std::vector<std::pair<vec3, vec3>> z(num_particles);

    std::vector<std::vector<double>> a{ {0.5}, {0.0, 0.5}, {0.0, 0.0, 1.0} };
    std::vector<double> b{ 1.0 / 6.0, 1.0 / 3.0, 1.0 / 3.0, 1.0 / 6.0 };
    std::vector<double> c{ 0.0, 0.5, 0.5, 1.0 };

    const int stages = c.size();

    std::vector<std::vector<std::pair<vec3, vec3>>> k_values(stages);
    for (int i = 0; i < stages; i++) {
        for (int j = 0; j < num_particles; j++) {
            k_values[i].push_back(std::pair<vec3, vec3>{vec3{ 0.0, 0.0, 0.0 }, vec3{ 0.0, 0.0, 0.0 }});
        }
    }

    double h = dt / number_of_steps;
    double t = time;
    double endTime = time + dt;

    while (t < endTime) {
        //std::cout << "t : " << t << "  et: " << endTime << "  h " << h << '\n';

        if (t + h > endTime) {
            h = endTime - t;
        }



        for (int i = 0; i < stages; i++) {
            for (int x = 0; x < num_particles; ++x) {
                z[x] = std::pair<vec3, vec3>(particles[x].pos, particles[x].vel);
            }
            for (int j = 0; j < i; ++j) {
                for (int x = 0; x < num_particles; ++x) {
                    z[x].first += h * a[i - 1][j] * k_values[j][x].first;
                    z[x].second += h * a[i - 1][j] * k_values[j][x].second;
                }
            }
            systemOfEquations(t + c[i] * h, z, k_values[i]);
        }
        std::vector<std::pair<vec3, vec3>> y(num_particles);

        for (int j = 0; j < num_particles; ++j) {
            y[j].first = particles[j].pos;
            y[j].second = particles[j].vel;
            for (int i = 0; i < stages; ++i) {
                y[j].first += h * b[i] * k_values[i][j].first;
                y[j].second += h * b[i] * k_values[i][j].second;
            }
        }


        for (size_t i = 0; i < num_particles; ++i) {
            particles[i].pos = y[i].first;
            particles[i].vel = y[i].second;
        }
        t += h;
        time += h;


    }
    for (int j = 0; j < num_particles; j++) {
        particles[j].acc = getNetAccelerationOnParticle(j);
    }
    //std::cout << "t : " << t << "  et: " << endTime << "  h " << h << '\n';

    time = t;


}

// Runge-Kutta-Fehlberg (RK45) adaptive integration method for a system of equations
void ParticleManager::updateSystemRK45(double dt, size_t number_of_steps , double tolerance ) {
    double h = dt / number_of_steps;

    double t = time;
    double endTime = t + dt;


    std::vector<double> b4{
        25.0 / 216.0,
        0.0,
        1408.0 / 2565.0,
        2197.0 / 4104.0,
        -1.0 / 5.0,
        0.0
    };



    std::vector<double> b5{
        16.0 / 135.0,
        0.0,
        6656.0 / 12825.0,
        28561.0 / 56430.0,
        -9.0 / 50.0,
        2.0 / 55.0
    };



    std::vector<std::vector<double>> a{
        {1.0 / 4.0},
        {3.0 / 32.0,	9.0 / 32.0},
        {1932.0 / 2197.0,	-7200.0 / 2197.0,	7296.0 / 2197.0},
        {439.0 / 216.0,	-8.0,	3680.0 / 513.0,	-845.0 / 4104.0},
        {-8.0 / 27.0,	2.0,	-3544.0 / 2565.0,	1859.0 / 4104.0	,-11.0 / 40.0}
    };
    std::vector<double> c{ 0.0, 1.0 / 4.0, 3.0 / 8.0, 12.0 / 13.0, 1.0, 1.0 / 2.0 };

    const int stages = c.size();
    const size_t num_particles = particles.size();

    std::vector<std::vector<std::pair<vec3, vec3>>> k_values(stages);

    for (int i = 0; i < stages; i++) {
        for (int j = 0; j < num_particles; j++) {
            k_values[i].push_back(std::pair<vec3, vec3>{vec3{ 0.0, 0.0, 0.0 }, vec3{ 0.0, 0.0, 0.0 }});
        }
    }

    std::vector<std::pair<vec3, vec3>> z(num_particles);


    while (t < endTime) {
        if (t + h > endTime) {
            h = endTime - t;
        }

        for (int i = 0; i < stages; i++) {
            for (int x = 0; x < num_particles; ++x) {
                z[x] = std::pair<vec3, vec3>(particles[x].pos, particles[x].vel);
            }
            for (int j = 0; j < i; ++j) {
                for (int x = 0; x < num_particles; ++x) {
                    z[x].first += h * a[i - 1][j] * k_values[j][x].first;
                    z[x].second += h * a[i - 1][j] * k_values[j][x].second;
                }
            }
            systemOfEquations(t + c[i] * h, z, k_values[i]);
        }

        // Calculate the 4th and 5th order solutions
        std::vector<std::pair<vec3, vec3>> y4(num_particles), y5(num_particles);

        for (int j = 0; j < num_particles; ++j) {
            y4[j].first = particles[j].pos;
            y4[j].second = particles[j].vel;
            y5[j].first = particles[j].pos;
            y5[j].second = particles[j].vel;
            for (int i = 0; i < stages; ++i) {
                y4[j].first += h * b4[i] * k_values[i][j].first;
                y4[j].second += h * b4[i] * k_values[i][j].second;
                y5[j].first += h * b5[i] * k_values[i][j].first;
                y5[j].second += h * b5[i] * k_values[i][j].second;
            }
        }

        // Calculate the error
        double error_p = 0.0;
        double error_v = 0.0;
        vec3 error_pos = vec3{ 0.0, 0.0, 0.0 };
        vec3 error_vel = vec3{ 0.0, 0.0, 0.0 };
        for (size_t i = 0; i < num_particles; ++i) {
            error_pos = y5[i].first - y4[i].first;
            error_vel = y5[i].second - y4[i].second;
            error_p += glm::length(error_pos);
            error_v += glm::length(error_vel);
        }
        double error = std::max(error_p, error_v);




        // Update the solution if the error is acceptable
        if (error < tolerance) {
            for (size_t i = 0; i < num_particles; ++i) {
                particles[i].pos = y5[i].first;
                particles[i].vel = y5[i].second;
            }
            t += h;
        }

        double scaleFactor = 0.9 * std::pow(tolerance / error, 1.0 / 5.0);
        double h_new = h * scaleFactor;
        h = h_new;




    }
    for (int j = 0; j < num_particles; j++) {
        particles[j].acc = getNetAccelerationOnParticle(j);
    }

    time = t;
}

double ParticleManager::getTotalKineticEnergy() const {
    double ret_val = 0.0;
    for (const auto& p : particles) {
        ret_val += getKineticEnergy(p);
    }
    return ret_val;
}

double ParticleManager::getTotalPotentialEnergy() const {
    double ret_val = 0.0;
    for (size_t i = 0; i < particles.size(); i++) {
        for (size_t j = i + 1; j < particles.size(); j++) {
            ret_val += getPotentialEnergy(particles[i], particles[j]);
        }
    }
    return ret_val;
}

double ParticleManager::getTotalEnergy() const {
    return getTotalKineticEnergy() + getTotalPotentialEnergy();
}

vec3 ParticleManager::getNetForceOnParticle(const size_t particle_index) const {
    vec3 ret_val = vec3{ 0.0,0.0,0.0 };
    for (size_t i = 0; i < particles.size(); i++) {
        if (i == particle_index) continue;
        ret_val += getForceGravityOnParticleAFromB(particles[particle_index], particles[i]);
    }
    return ret_val;
}

vec3 ParticleManager::getNetAccelerationOnParticle(const size_t particle_index) const {
    return getNetForceOnParticle(particle_index) / particles[particle_index].mass;
}

void ParticleManager::add(double mass, vec3 pos , vec3 vel, vec3 acc ) {
    Particle p;
    p.pos = pos;
    p.vel = vel;
    p.acc = acc;
    p.mass = mass;
    p.radius = 1.0;
    particles.push_back(p);
    //particles_temp.push_back(p);
}

void ParticleManager::print() {
    for (auto& p : particles) {
        std::cout << p << std::endl;
    }
}

void ParticleManager::update(double dt, size_t integration_method ) {
    switch (integration_method) {
    case 0:
        //red
        updateVerlet(dt);
        break;
    case 1:
        //green
        updateSystemVerlet(dt);
        break;
    case 2:
        //blue
        updateSystemRK4(dt);
        break;
    case 3:
        //yellow
        updateSystemRK(dt);
        break;
    case 4:
        //cyan
        updateSystemRK45(dt);
        break;
    default:
        break;
    };
}

void ParticleManager::draw(Model& particle_model, Shader& shader) {
    std::vector<glm::mat4> transformations(particles.size());
    for (const auto& p : particles) {
        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, glm::vec3{ static_cast<float>(p.pos.x), static_cast<float>(p.pos.y), static_cast<float>(p.pos.z) });
        model = glm::scale(model, glm::vec3{ 1.0f, 1.0f, 1.0f });
        transformations.push_back(model);
    }
    particle_model.updateAndDrawInstanced(transformations, shader);
}

void ParticleManager::updateDataInGPU(Model& particle_model) {
    std::vector<glm::mat4> transformations(particles.size());
    for (const auto& p : particles) {
        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, glm::vec3{ static_cast<float>(p.pos.x), static_cast<float>(p.pos.y), static_cast<float>(p.pos.z) });
        model = glm::scale(model, glm::vec3{ 1.0f, 1.0f, 1.0f });
        transformations.push_back(model);
    }
    particle_model.updateInstancedData(transformations);
}

/*
* First we look at all particle pairs in the system and if those two particles
* were to collide within the timestep dt, we add that pair of particles to a
* list of particles that are colliding in that frame
*
* If the list is not empty, we find the particle pair with the smallest collision time
* in other words, the partcle pair the collides first. Let A,B be those particles and let
* T1 be the time those two particles collide and let T0 be the start of the substep. Now we know
* that between T0 and T1, there are no collisions except for the collision that occurs at T1, we
* resolve the collision for particles A and B and then update all particles with the substep of T1-T0
*
* Since the positions and velocities of particles A and B have changed, we now must rechecek all particle
* pairs that contain particle A or particle B. If A and B are both convex, im pretty sure we dont need to
* recheck A and B. Anyways if there is a collision between A and some other particle, update lol
*
*
*
*/
void ParticleManager::updateAndResolveCollisions(double dt, const size_t integration_method , const bool avoid_displacement ) {
    double update_remaining_time = dt;
    while (update_remaining_time > 0.0) {
        size_t index_A;
        size_t index_B;
        bool circles_currently_overlapping = false; // the edges of both circles intersect at two points
        bool circles_currently_touching = false; // the edges of both cirlces only intercect at one point
        bool circles_future_collision_found = false; // cirlces are not currently touching/overlapping but will in the future
        bool exit_double_loop_flag = false;
        bool is_collision_resolution_needed = false;
        double smallest_collision_time = update_remaining_time;
        for (size_t i = 0; i < particles.size() && !exit_double_loop_flag; i++) {
            for (size_t j = i + 1; j < particles.size() && !exit_double_loop_flag; j++) {
                double t = timeTillCollision(particles[i], particles[j]);
                if (t == 0.0) {
#if 1
                    Particle& A = particles[i];
                    Particle& B = particles[j];
                    vec3 relative_position_AB = A.pos - B.pos;
                    double radii_sum = A.radius + B.radius;
                    double radii_sum_sqrd = radii_sum * radii_sum;
                    double distance_between_centers_sqrd = glm::dot(relative_position_AB, relative_position_AB);
                    double distance_between_centers = sqrt(distance_between_centers_sqrd);
                    vec3 relative_velocity_AB = A.vel - B.vel;
                    is_collision_resolution_needed = (bool)(glm::dot(relative_position_AB, relative_velocity_AB) < 0.0);
#endif
                    if (distance_between_centers_sqrd < radii_sum_sqrd) {
                        if (!avoid_displacement || is_collision_resolution_needed) {
                            smallest_collision_time = t;
                            index_A = i;
                            index_B = j;
                            circles_currently_overlapping = true;
                            exit_double_loop_flag = true;
                        }
                    }
                    else {
                        // In this case: distance_between_centers_sqrd == radii_sum_sqrd
                        if (is_collision_resolution_needed) {
                            // In this case, circle edges intersect at exactly one point, and thier
                            // centers are moving towards eachother (expected next time step for both
                            // circles to overlap)
                            smallest_collision_time = t;
                            index_A = i;
                            index_B = j;
                            circles_currently_touching = true;
                            exit_double_loop_flag = true;
                        }
                    }
                }
                else if (0.0 < t && t <= update_remaining_time) {
                    circles_future_collision_found = true;
                    // collision occured but we only care if its the first one
                    if (t <= smallest_collision_time) {
                        smallest_collision_time = t;
                        index_A = i;
                        index_B = j;
                    }
                }
            }
        }

        if (circles_currently_touching) {
            doCollision(particles[index_A], particles[index_B]);
        }
        else if (circles_currently_overlapping) {
            if (!avoid_displacement) {
                Particle& A = particles[index_A];
                Particle& B = particles[index_B];
                vec3 relative_position_AB = A.pos - B.pos;
                vec3 relative_velocity_AB = A.vel - B.vel;

                double radii_sum = A.radius + B.radius;
                double radii_sum_sqrd = radii_sum * radii_sum;
                double distance_between_centers_sqrd = glm::dot(relative_position_AB, relative_position_AB);
                double distance_between_centers = sqrt(distance_between_centers_sqrd);

                vec3 normal = relative_position_AB / distance_between_centers;
                A.pos += 0.5 * (radii_sum - distance_between_centers) * normal;
                B.pos -= 0.5 * (radii_sum - distance_between_centers) * normal;
            }

            if (is_collision_resolution_needed) {
                doCollision(particles[index_A], particles[index_B]);
            }
        }
        else if (circles_future_collision_found) {
            update(smallest_collision_time, integration_method);
#if 1
            Particle& A = particles[index_A];
            Particle& B = particles[index_B];
            vec3 relative_position_AB = A.pos - B.pos;

            double radii_sum = A.radius + B.radius;
            double radii_sum_sqrd = radii_sum * radii_sum;
            double distance_between_centers_sqrd = glm::dot(relative_position_AB, relative_position_AB);
            double distance_between_centers = sqrt(distance_between_centers_sqrd);

            if (distance_between_centers_sqrd <= radii_sum_sqrd) {
                vec3 relative_velocity_AB = A.vel - B.vel;
                if (!avoid_displacement && distance_between_centers_sqrd < radii_sum_sqrd) {
                    vec3 normal = relative_position_AB / distance_between_centers;
                    A.pos += 0.5 * (radii_sum - distance_between_centers) * normal;
                    B.pos -= 0.5 * (radii_sum - distance_between_centers) * normal;
                }
                assert(glm::dot(relative_position_AB, relative_velocity_AB) <= 0.0);
                doCollision(particles[index_A], particles[index_B]); // TODO MAKE THIS WORK FOR MORE THAN ONE COLLISION EVENT
            }
#endif
            update_remaining_time -= smallest_collision_time;
        }
        else {
            update(update_remaining_time, integration_method);
            update_remaining_time -= update_remaining_time; // equiv to just using break or update_remaining_time = 0.0
        }
    }
}


