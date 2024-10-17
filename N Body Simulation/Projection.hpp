#pragma once

#include <Eigen/Dense>
#include <iostream>

// Function to project a point y onto the plane defined by points p1, p2, p3 in N-dimensional space
Eigen::VectorXd projectPointOntoPlaneND(const Eigen::MatrixXd& simplex, const Eigen::VectorXd& y) {
    // Assuming simplex is an Nx3 matrix where each column represents p1, p2, p3 in N-dimensional space
    Eigen::MatrixXd vectorsInPlane = simplex.rightCols(2) - simplex.leftCols(1).replicate(1, 2); // [p2-p1, p3-p1]

    // Find an orthonormal basis for the space spanned by [p2-p1, p3-p1]
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(vectorsInPlane);
    Eigen::MatrixXd Q = qr.householderQ();

    // The first two columns of Q form an orthonormal basis for the plane.
    // The rest of the columns form an orthonormal basis for the space orthogonal to the plane.

    // Project y onto the plane by removing its component in the orthogonal space
    Eigen::VectorXd yOrthogonalComponent = Q.rightCols(Q.cols() - 2) * Q.rightCols(Q.cols() - 2).transpose() * (y - simplex.col(0));
    Eigen::VectorXd yProjection = y - yOrthogonalComponent;

    return yProjection;
}

int test() {
    // Example with 4D space
    Eigen::MatrixXd simplex(4, 3); // 4D space, 3 points
    simplex << 0, 1, 0, // p1
        0, 0, 1, // p2
        0, 1, 1, // p3
        0, 0, 0; // p4 (extra dimension)
    Eigen::VectorXd y(4);
    y << 0.5, 0.5, 0.5, 1; // Example point in 4D space

    Eigen::VectorXd yProjection = projectPointOntoPlaneND(simplex, y);

    std::cout << "Projected point: " << yProjection.transpose() << std::endl;

    return 0;
}





#include <vector>

// Function to project a point y onto an m-simplex defined by vertices in N-dimensional space
Eigen::VectorXd projectPointOntoSimplex(const std::vector<Eigen::VectorXd>& simplexVertices, const Eigen::VectorXd& y) {
    // Ensure there are at least two vertices to define a simplex
    if (simplexVertices.size() < 2) {
        std::cerr << "Need at least two vertices to define a simplex." << std::endl;
        exit(EXIT_FAILURE);
    }

    // m is the simplex dimension
    int m = simplexVertices.size() - 1;
    int N = simplexVertices[0].size(); // Dimension of the space

    // Step 1: Compute vectors spanning the simplex
    Eigen::MatrixXd A(N, m);
    for (int i = 0; i < m; ++i) {
        A.col(i) = simplexVertices[i + 1] - simplexVertices[0];
    }

    // Step 2: Perform QR decomposition to find an orthogonal basis for the subspace spanned by the simplex
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(A);
    Eigen::MatrixXd Q = qr.householderQ();

    // Step 3: Project y onto the subspace
    Eigen::VectorXd y_proj = Q * Q.transpose() * (y - simplexVertices[0]) + simplexVertices[0];

    return y_proj;
}

int test2() {
    // Example usage
    std::vector<Eigen::VectorXd> simplexVertices;
    simplexVertices.push_back(Eigen::VectorXd::Zero(4)); // p1
    simplexVertices.push_back(Eigen::VectorXd::Unit(4, 0)); // p2
    simplexVertices.push_back(Eigen::VectorXd::Unit(4, 1)); // p3
    // Add more vertices as needed to define your simplex

    Eigen::VectorXd y(4); // Example point in 4D space
    y << 1, 2, 3, 4;

    Eigen::VectorXd y_proj = projectPointOntoSimplex(simplexVertices, y);
    std::cout << "Projected point: " << y_proj.transpose() << std::endl;

    return 0;
}
