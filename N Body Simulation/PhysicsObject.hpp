#pragma once
#include "Model.hpp"
#include "VectorSpace.hpp"

enum ShapeType
{
	SPHERE,
	ELLIPSOID,
	OBB,
	MESH
};


template<int N> struct PhysicsObject {

    // Vertices of the polygon stored as N-dimensional column vectors
    std::vector<Eigen::Matrix<double, N, 1>> vertices;

    // Position (center of mass)
    Eigen::Matrix<double, N, 1> position;

    // Orientation stored as an N x N rotation matrix (element of SO(N))
    Eigen::Matrix<double, N, N> orientation;

    // Constructor that takes a vector of vertices
    PhysicsObject(const std::vector<Eigen::Matrix<double, N, 1>>& verts)
        : vertices(verts)
    {
        // Compute the centroid as the average of the vertices
        position = Eigen::Matrix<double, N, 1>::Zero();
        for (const auto& v : vertices) {
            position += v;
        }
        if (!vertices.empty()) {
            position /= static_cast<double>(vertices.size());
        }

        // Initialize the orientation to the identity matrix (no rotation)
        orientation = Eigen::Matrix<double, N, N>::Identity();
    }

    // Example: apply a rotation to the polygon
    void applyRotation(const Eigen::Matrix<double, N, N>& rotationMatrix) {
        orientation = rotationMatrix * orientation;
        // Optionally, you might also rotate the vertices around the centroid
        for (auto& v : vertices) {
            v = rotationMatrix * (v - position) + position;
        }
    }
};

