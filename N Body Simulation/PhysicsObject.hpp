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



struct PhysicsObject {
    CollisionHull* collisionHull = nullptr; // if null, then it is a point mass


    // Vertices of the polygon stored as N-dimensional column vectors
    std::vector<VectorND> vertices;

    // The reference position (e.g., centroid or center of mass)
    VectorND position;

    // Orientation stored as an N x N rotation matrix (element of SO(N))
    MatrixND orientation;

    // Linear velocity (N-dimensional)
    VectorND velocity;

    // Angular velocity represented by the independent degrees of freedom
    // for a skew-symmetric matrix: a vector of length N(N-1)/2.
    // The ordering is assumed to be lexicographic (e.g., for 4D: (XY, XZ, XW, YZ, YW, ZW)).
    std::vector<VECTOR_SPACE_FIELD> angularVelocity;


    // Constructor initializes the polygon from its vertices.
    PhysicsObject(const std::vector<Eigen::Matrix<double, N, 1>>& verts)
        : vertices(verts),
        velocity(Eigen::Matrix<double, N, 1>::Zero()),
        orientation(Eigen::Matrix<double, N, N>::Identity())
    {
        // Compute the centroid as the average of the vertices.
        position = Eigen::Matrix<double, N, 1>::Zero();
        for (const auto& v : vertices) {
            position += v;
        }
        if (!vertices.empty()) {
            position /= static_cast<double>(vertices.size());
        }

        // Initialize angular velocity as zeros.
        angularVelocity.resize((N * (N - 1)) / 2, 0.0);
    }

    // Example: apply a rotation to the polygon
    void applyRotation(const Eigen::Matrix<double, N, N>& rotationMatrix) {
        orientation = rotationMatrix * orientation;
        // Optionally, you might also rotate the vertices around the centroid
        for (auto& v : vertices) {
            v = rotationMatrix * (v - position) + position;
        }
    }

    // Update function that advances the polygon's state by a time step dt.
    // Linear motion: position is updated by the velocity.
    // Angular motion: the orientation is updated using a small-angle approximation.
    void update(double dt) {
        // Update position using linear velocity.
        position += velocity * dt;

        // Build the skew-symmetric matrix (omegaHat) representing the angular velocity.
        Eigen::Matrix<double, N, N> omegaHat = Eigen::Matrix<double, N, N>::Zero();
        int idx = 0;
        for (int i = 0; i < N - 1; i++) {
            for (int j = i + 1; j < N; j++) {
                double omega = angularVelocity[idx++];
                // The skew-symmetric matrix has entries:
                //   omegaHat(i, j) = -omega   and   omegaHat(j, i) = omega.
                omegaHat(i, j) = -omega;
                omegaHat(j, i) = omega;
            }
        }

        // Update orientation using the first-order approximation:
        //   exp(omegaHat * dt) ≈ I + omegaHat * dt
        orientation = (Eigen::Matrix<double, N, N>::Identity() + omegaHat * dt) * orientation;
    }

};

