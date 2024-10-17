#pragma once
#include <vector>
#include "HyperSphereCollision.hpp"
#include "VectorSpace.hpp"




struct ConvexHull {
    VectorND verticies;
};

struct CollisionResolutionData {
    VectorND center_of_mass;
    double mass = 1.0;
    double restitution = 1.0;
    double drag = 0.1;
    double mu = 0.2;
    

    //bool has_infinite_mass = false;
    //bool is_affected_by_gravity = true;
    //bool does_emit_gravitational_field = true;
};

struct CollidableObject {
    VectorND position;
    VectorND velocity;
    VectorND acceleration;
    CollisionResolutionData* crd;

	double bounding_circle_radius;
	std::vector<ConvexHull> collision_hull;
    size_t collision_count = 0; // number of collision events for this object (used in a test to check no skipped collisions)
    bool is_collisions_enabled = true;
    double time = 0.0; // how long this object has existed

	bool broadPhaseCollisionCheck(const CollidableObject& other, double& time_till_collision);
	bool narrowPhaseCollisionCheck(const CollidableObject& other, double& time_till_collision);


    CollidableObjectPair** pairs;
};


#include <unordered_set>
#include <queue>


struct CollisionEvent {
    std::unordered_set<size_t> object_indices;
    std::vector<CollidableObject>* all_objects;
    

    //double collision_time_start;
    //double collision_time_end;
    CollisionEvent(std::vector<CollidableObject>& all, size_t i, size_t j) : all_objects{ &all }, object_indices{i,j} {}
};

struct CollidableObjectPair {
    const CollidableObject* A;
    const CollidableObject* B;
    // cached data for the pair

};

void resolveCollisions() {



}


void removeAssociatedCollisionEventsAtIndex(std::unordered_map<double, CollisionEvent>& collision_events, const std::unordered_set<size_t>& object_indices) {
    for (auto& target_index : object_indices) {
        std::erase_if(collision_events, [&](CollisionEvent& ce) {
        for (auto& obj_index : ce.object_indices) {
            if (obj_index == target_index) return true;
        }
        return false;});
    }
}


struct CollidableObjectGroup {
    size_t number_of_objects;
    std::vector<CollidableObject> collidable_objects;
    std::vector<CollidableObjectPair> collidable_object_pairs;

    std::vector<std::vector<bool>> collision_matrix;

    void updateAndResolveCollisions(double dt, const size_t integration_method, const bool avoid_displacement) {
        double update_remaining_time = dt;
        std::unordered_set<size_t> objects_to_check;
        for (size_t i = 0; i < number_of_objects; i++) objects_to_check.insert(i);
        std::unordered_map<double, CollisionEvent> collision_events;
        double smallest_collision_time_found = update_remaining_time;
        while (update_remaining_time > 0.0) {
            for (size_t i = 0; i < number_of_objects; i++) {
                for (size_t j : objects_to_check) {
                    if (i == j || !(objects_to_check.count(i) == 0 || (objects_to_check.count(i) > 0 && i < j))) continue;
                    // Now (i,j) is a unique valid pair
                    if (!collision_matrix[i][j] && !collision_matrix[j][i]) continue;
                    double collision_time_start;
                    double collision_time_end;
                    if (!broadPhaseCollisionCheck(collidable_objects[i], collidable_objects[j], collision_time_start, collision_time_end)) continue;
                    if (collision_time_start < 0.0 || update_remaining_time < collision_time_start) continue;
                    // Maybe cache broadPhaseCollisionCheck data here
                    if (!narrowPhaseCollisionCheck(collidable_objects[i], collidable_objects[j], collision_time_start, collision_time_end)) continue;
                    if (collision_time_start < smallest_collision_time_found) {
                        smallest_collision_time_found = collision_time_start;
                    }
                    if (collision_events.find(collision_time_start) != collision_events.end()) {
                        collision_events[collision_time_start].object_indices.insert(i);
                        collision_events[collision_time_start].object_indices.insert(j);
                    }
                    else {
                        collision_events[collision_time_start] = CollisionEvent(collidable_objects, i, j);
                    }
                }
            }
            // The smallest interval that is safe to advance everything is smallest_collision_time_found so we update it
            updateObjects(smallest_collision_time_found);

            if (!collision_events.empty()) {
                // Since this is the smallest collision we resolve it
                // Note that we should also check if we update using our integeration method
                // that if any of the objects inside this collision event are actually colliding
                resolveCollisions(collision_events[smallest_collision_time_found]);
                // update objects_to_check by adding the indicies of the objects in the collision event
                objects_to_check = collision_events[smallest_collision_time_found].object_indices;
                // remove all collision events that contain any of the objects found in objects_to_check
                removeAssociatedCollisionEventsAtIndex(collision_events, objects_to_check);

                if (!collision_events.empty()) {
                    auto aa = collision_events.begin();
                    smallest_collision_time_found = aa->first;
                }
                else {
                    smallest_collision_time_found = update_remaining_time;
                }
            }
            
            update_remaining_time -= smallest_collision_time_found; // could be subtracting zero
            // so its important to makesure updateObjects will result in a non zero val next loop
        }
    }

    bool broadPhaseCollisionCheck(const CollidableObject& A, const CollidableObject& B, double& start_time, double& end_time) {
        return start_time == end_time;
    }

    bool narrowPhaseCollisionCheck(const CollidableObject& A, const CollidableObject& B, double& start_time, double& end_time) {
        return broadPhaseCollisionCheck(A, B, start_time, end_time);
    }
    void addObject(CollidableObject&& obj, bool collidable = true) {
        for (size_t i = 0; i < number_of_objects; i++) {
            collision_matrix[i].push_back(collidable);
        }

        number_of_objects++;
        collision_matrix.push_back(std::vector<bool>(number_of_objects, collidable));
        collision_matrix.back().back() = false;
        collidable_objects.push_back(std::move(obj));
    }

    void removeObject(size_t index) {
        number_of_objects--;
        collision_matrix.erase(collision_matrix.begin() + index);
        for (size_t i = 0; i < number_of_objects; i++) {
            collision_matrix[i].erase(collision_matrix[i].begin() + index);
        }
        collidable_objects.erase(collidable_objects.begin() + index);
    }

};

/*
* All collidable objects are orginized into groups (bullets, players, world geometry, etc.)
* we will have a collision matrix for each group
* say groups 1 represent players, group 2 represent world geometry
* collision_matrix[1][2] = 1 represents that players can collide with world geometry
* collision_matrix[2][1] = 0 represents that world geometry does not collide with players (no collision responce needed for world geometry)
* collision_matrix[1][1] = 1 represents players can collide with players
* collision_matrix[2][2] = 0 world cannot collide with world
*/
struct CollidableGroupManager {
    // This vector contains all objects that are grouped together
    // This makes it easier to create temperary groups
    std::vector<CollidableObjectGroup> groups;
    std::vector<std::vector<bool>> collision_matrix;


    void add_group()
};






void ParticleManager::updateAndResolveCollisions(double dt, const size_t integration_method, const bool avoid_displacement) {
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
                    VectorND relative_position_AB = A.position - B.position;
                    double radii_sum = A.radius + B.radius;
                    double radii_sum_sqrd = radii_sum * radii_sum;
                    double distance_between_centers_sqrd = relative_position_AB.dot(relative_position_AB);
                    double distance_between_centers = sqrt(distance_between_centers_sqrd);
                    VectorND relative_velocity_AB = A.velocity - B.velocity;
                    is_collision_resolution_needed = (bool)(relative_position_AB.dot(relative_velocity_AB) < 0.0);
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
                VectorND relative_position_AB = A.position - B.position;
                VectorND relative_velocity_AB = A.velocity - B.velocity;

                double radii_sum = A.radius + B.radius;
                double radii_sum_sqrd = radii_sum * radii_sum;
                double distance_between_centers_sqrd = relative_position_AB.dot(relative_position_AB);
                double distance_between_centers = sqrt(distance_between_centers_sqrd);

                VectorND normal = relative_position_AB / distance_between_centers;
                A.position += 0.5 * (radii_sum - distance_between_centers) * normal;
                B.position -= 0.5 * (radii_sum - distance_between_centers) * normal;
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
            VectorND relative_position_AB = A.position - B.position;

            double radii_sum = A.radius + B.radius;
            double radii_sum_sqrd = radii_sum * radii_sum;
            double distance_between_centers_sqrd = relative_position_AB.dot(relative_position_AB);
            double distance_between_centers = sqrt(distance_between_centers_sqrd);

            if (distance_between_centers_sqrd <= radii_sum_sqrd) {
                VectorND relative_velocity_AB = A.velocity - B.velocity;
                if (!avoid_displacement && distance_between_centers_sqrd < radii_sum_sqrd) {
                    VectorND normal = relative_position_AB / distance_between_centers;
                    A.position += 0.5 * (radii_sum - distance_between_centers) * normal;
                    B.position -= 0.5 * (radii_sum - distance_between_centers) * normal;
                }
                assert(relative_position_AB.dot(relative_velocity_AB) <= 0.0);
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


