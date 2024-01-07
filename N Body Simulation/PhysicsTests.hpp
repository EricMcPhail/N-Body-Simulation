#pragma once
#include "ParticleManager.hpp"
#include "Constants.hpp"

#include <iomanip>
#include <iomanip>
#include <limits>
#include <windows.h>

const size_t NUMBER_OF_SUB_STEPS = 1;

struct ThreeBodyTest {
    std::vector<ParticleManager> estimated_solutions;
    ParticleManager exact_solution;

    const vec3 center_body_position_at_time_zero = vec3{ 0.0, 0.0, 0.0 };
    const vec3 center_body_velocity = vec3{ 0.0, 0.0, 0.0 };
    const double center_body_mass = 1.0;
    const double outer_bodies_mass = 1.0;
    const double outer_bodies_distance_from_center = 2.0;

    double current_time = 0.0;
    std::vector<size_t> integration_methods{ 0,1,2,3,4 };

    double init_KE = 0.0;
    double init_PE = 0.0;
    double init_TE = 0.0;

    ThreeBodyTest() {
        init();
    }

    void init() {
        double r = outer_bodies_distance_from_center;
        double m0 = center_body_mass;
        double m1 = outer_bodies_mass;
        double omega = sqrt(GRAVITATIONAL_CONSTANT / (r * r * r) * (m0 + 0.25 * m1));
        vec3 center_body_position = center_body_position_at_time_zero + center_body_velocity * current_time;
        vec3 outer_body1_position = vec3{ -1.0 * r * cos(omega * current_time), r * sin(omega * current_time), 0.0 } + center_body_position;
        vec3 outer_body2_position = vec3{ r * cos(omega * current_time), -1.0 * r * sin(omega * current_time), 0.0 } + center_body_position;
        vec3 outer_body1_velocity = vec3{ r * omega * sin(omega * current_time), r * omega * cos(omega * current_time), 0.0 } + center_body_velocity;
        vec3 outer_body2_velocity = vec3{ -1.0 * r * omega * sin(omega * current_time), -1.0 * r * omega * cos(omega * current_time), 0.0 } + center_body_velocity;
        vec3 outer_body1_acceleration = vec3{ r * omega * omega * cos(omega * current_time), -1.0 * r * omega * omega * sin(omega * current_time), 0.0 };
        vec3 outer_body2_acceleration = vec3{ -1.0 * r * omega * omega * cos(omega * current_time), r * omega * omega * sin(omega * current_time), 0.0 };

        exact_solution.add(center_body_mass, center_body_position, center_body_velocity, vec3{ 0.0, 0.0, 0.0 });
        exact_solution.add(outer_bodies_mass, outer_body1_position, outer_body1_velocity, outer_body1_acceleration);
        exact_solution.add(outer_bodies_mass, outer_body2_position, outer_body2_velocity, outer_body2_acceleration);

        for (size_t i = 0; i < integration_methods.size(); i++) {
            estimated_solutions.push_back(ParticleManager());
            estimated_solutions[i].add(center_body_mass, center_body_position, center_body_velocity, vec3{0.0, 0.0, 0.0});
            estimated_solutions[i].add(outer_bodies_mass, outer_body1_position, outer_body1_velocity, outer_body1_acceleration);
            estimated_solutions[i].add(outer_bodies_mass, outer_body2_position, outer_body2_velocity, outer_body2_acceleration);
        }


        init_KE = exact_solution.getTotalKineticEnergy();
        init_PE = exact_solution.getTotalPotentialEnergy();
        init_TE = init_KE + init_PE;
    }

    void updateExactSolution(double dt) {
        current_time += dt;
        vec3 center_body_position = center_body_position_at_time_zero + center_body_velocity * current_time;

        double r = outer_bodies_distance_from_center;
        double m0 = center_body_mass;
        double m1 = outer_bodies_mass;
        double omega = sqrt(GRAVITATIONAL_CONSTANT / (r * r * r) * (m0 + 0.25 * m1));

        vec3 outer_body1_position = vec3{ -1.0 * r * cos(omega * current_time), r * sin(omega * current_time), 0.0 } + center_body_position;
        vec3 outer_body2_position = vec3{ r * cos(omega * current_time), -1.0 * r * sin(omega * current_time), 0.0 } + center_body_position;
        vec3 outer_body1_velocity = vec3{ r * omega * sin(omega * current_time), r * omega * cos(omega * current_time), 0.0 } + center_body_velocity;
        vec3 outer_body2_velocity = vec3{ -1.0 * r * omega * sin(omega * current_time), -1.0 * r * omega * cos(omega * current_time), 0.0 } + center_body_velocity;
        vec3 outer_body1_acceleration = vec3{ r * omega * omega * cos(omega * current_time), -1.0 * r * omega * omega * sin(omega * current_time), 0.0 };
        vec3 outer_body2_acceleration = vec3{ -1.0 * r * omega * omega * cos(omega * current_time), r * omega * omega * sin(omega * current_time), 0.0 };

        exact_solution.particles[0].pos = center_body_position;
        exact_solution.particles[0].vel = center_body_velocity;
        exact_solution.particles[0].acc = vec3{0.0, 0.0, 0.0};

        exact_solution.particles[1].pos = outer_body1_position;
        exact_solution.particles[1].vel = outer_body1_velocity;
        exact_solution.particles[1].acc = outer_body1_acceleration;

        exact_solution.particles[2].pos = outer_body2_position;
        exact_solution.particles[2].vel = outer_body2_velocity;
        exact_solution.particles[2].acc = outer_body2_acceleration;
    }

    void updateEstimatedSolutions(double dt) {
        for (size_t i = 0; i < integration_methods.size(); i++) {
            estimated_solutions[i].update(dt, integration_methods[i]);
        }
    }

    void update(double dt) {
        updateExactSolution(dt);
        if (NUMBER_OF_SUB_STEPS == 1) {
            updateEstimatedSolutions(dt);
        }
        else {
            double sub_dt = dt / NUMBER_OF_SUB_STEPS;
            for (size_t i = 0; i < NUMBER_OF_SUB_STEPS; i++) {
                updateEstimatedSolutions(sub_dt);
            }
        }
    }

    void print(bool show_exact_solution = true, bool simplified_error = false) {
        const size_t num_bodies = 3;
        //std::vector<vec3> error_pos(integration_methods.size() * num_bodies);
        //std::vector<vec3> error_vel(integration_methods.size() * num_bodies);
        //std::vector<vec3> error_acc(integration_methods.size() * num_bodies);
        std::cout << std::setprecision(std::numeric_limits<double>::digits10 + 1);
        for (size_t i = 0; i < integration_methods.size(); i++) {
            std::cout << "\nERROR FOR INTEGRATION METHOD #" << integration_methods[i] << '\n';
            double temp_KE = estimated_solutions[i].getTotalKineticEnergy();
            double temp_PE = estimated_solutions[i].getTotalPotentialEnergy();
            double temp_TE = temp_KE + temp_PE;
            double error_KE = temp_KE - init_KE;
            double error_PE = temp_PE - init_PE;
            double error_TE = temp_TE - init_TE;
            std::cout << "Error KE: "
                << std::left << std::setw(std::numeric_limits<double>::digits10 + 1 + 7)
                << error_KE << '\n';
            std::cout << "Error PE: "
                << std::left << std::setw(std::numeric_limits<double>::digits10 + 1 + 7)
                << error_PE << '\n';
            std::cout << "Error TE: "
                << std::left << std::setw(std::numeric_limits<double>::digits10 + 1 + 7)
                << error_TE << '\n';

            for (size_t j = 0; j < num_bodies; j++) {
                std::cout << "DATA FOR PARTICLE #" << j << '\n';
                vec3 error_pos = estimated_solutions[i].particles[j].pos - exact_solution.particles[j].pos;
                vec3 error_vel = estimated_solutions[i].particles[j].vel - exact_solution.particles[j].vel;
                vec3 error_acc = estimated_solutions[i].particles[j].acc - exact_solution.particles[j].acc;
                if (show_exact_solution) {
                    std::cout << std::left << std::setw(15) << "Exact Pos: "
                        << std::left << std::setw(std::numeric_limits<double>::digits10 + 1 + 7)
                        << exact_solution.particles[j].pos.x << ", " 
                        << std::left << std::setw(std::numeric_limits<double>::digits10 + 1 + 7)
                        << exact_solution.particles[j].pos.y;
                    std::cout << "\n";
                    std::cout << std::left << std::setw(15) << "Estimated Pos: "
                        << std::left << std::setw(std::numeric_limits<double>::digits10 + 1 + 7)
                        << estimated_solutions[i].particles[j].pos.x << ", " 
                        << std::left << std::setw(std::numeric_limits<double>::digits10 + 1 + 7)
                        << estimated_solutions[i].particles[j].pos.y << '\n';
                    std::cout << std::left << std::setw(15) << "Exact Vel: "
                        << std::left << std::setw(std::numeric_limits<double>::digits10 + 1 + 7)
                        << exact_solution.particles[j].vel.x << ", " 
                        << std::left << std::setw(std::numeric_limits<double>::digits10 + 1 + 7)
                        << exact_solution.particles[j].vel.y;
                    std::cout << "\n";
                    std::cout << std::left << std::setw(15) << "Estimated Vel: "
                        << std::left << std::setw(std::numeric_limits<double>::digits10 + 1 + 7)
                        << estimated_solutions[i].particles[j].vel.x << ", " 
                        << std::left << std::setw(std::numeric_limits<double>::digits10 + 1 + 7)
                        << estimated_solutions[i].particles[j].vel.y << '\n';
                    std::cout << std::left << std::setw(15) << "Exact Acc: "
                        << std::left << std::setw(std::numeric_limits<double>::digits10 + 1 + 7)
                        << exact_solution.particles[j].acc.x << ", " 
                        << std::left << std::setw(std::numeric_limits<double>::digits10 + 1 + 7)
                        << exact_solution.particles[j].acc.y;
                    std::cout << "\n";
                    std::cout << std::left << std::setw(15) << "Estimated Acc: "
                        << std::left << std::setw(std::numeric_limits<double>::digits10 + 1 + 7)
                        << estimated_solutions[i].particles[j].acc.x << ", " 
                        << std::left << std::setw(std::numeric_limits<double>::digits10 + 1 + 7)
                        << estimated_solutions[i].particles[j].acc.y << '\n';
                }
                std::cout << "Errors\n";
                if (simplified_error) {
                    std::cout << "Error Pos Magnitude: "
                        << std::left << std::setw(std::numeric_limits<double>::digits10 + 1 + 7)
                        << glm::length(error_pos) << '\n';
                    std::cout << "Error Vel Magnitude: "
                        << std::left << std::setw(std::numeric_limits<double>::digits10 + 1 + 7)
                        << glm::length(error_vel) << '\n';
                    std::cout << "Error Acc Magnitude: "
                        << std::left << std::setw(std::numeric_limits<double>::digits10 + 1 + 7)
                        << glm::length(error_acc) << '\n';
                }
                else {
                    std::cout << "Error Pos: " 
                        << std::left << std::setw(std::numeric_limits<double>::digits10 + 1 + 7)
                        << error_pos.x << ", " 
                        << std::left << std::setw(std::numeric_limits<double>::digits10 + 1 + 7)
                        << error_pos.y << '\n';
                    std::cout << "Error Vel: " 
                        << std::left << std::setw(std::numeric_limits<double>::digits10 + 1 + 7)
                        << error_vel.x << ", " 
                        << std::left << std::setw(std::numeric_limits<double>::digits10 + 1 + 7)
                        << error_vel.y << '\n';
                    std::cout << "Error Acc: " 
                        << std::left << std::setw(std::numeric_limits<double>::digits10 + 1 + 7)
                        << error_acc.x << ", " 
                        << std::left << std::setw(std::numeric_limits<double>::digits10 + 1 + 7)
                        << error_acc.y << '\n';
                }
            }
        }
    }
};