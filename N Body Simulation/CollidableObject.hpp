#pragma once

#include <vector>
#include "HyperSphereCollision.hpp"
#include "VectorSpace.hpp"

struct ConvexHull {
    std::vector<VectorND> vertices;
};

struct CollisionHull {
    std::vector<ConvexHull> convex_hulls;
};

struct CollisionHullManager {
    std::vector<CollisionHull> collision_hulls;
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

    /*
    * collision_hull could be null, meaning it is just a primative shape like a sphere
    *
    */
    CollisionHull* collision_hull = nullptr;

    size_t collision_count = 0; // number of collision events for this object (used in a test to check no skipped collisions)
    bool is_collisions_enabled = true;
    double time = 0.0; // how long this object has existed

    bool broadPhaseCollisionCheck(const CollidableObject& other, double& time_till_collision);
    bool narrowPhaseCollisionCheck(const CollidableObject& other, double& time_till_collision);

    CollidableObjectPair** pairs;
};


