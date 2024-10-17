#pragma once
#include "VectorSpace.hpp"
#include <cassert>
#include <iostream>
#include <Eigen/Dense>
#include <vector>




// Function to project a point onto a hyperplane using QR decomposition
Eigen::VectorXd projectOntoHyperplane(const Eigen::MatrixXd& points, const Eigen::VectorXd& point) {
    int m = points.cols() - 1;  // Number of direction vectors
    int n = points.rows();      // Dimension of the space

    // Step 1: Compute direction vectors
    Eigen::MatrixXd V(n, m);
    for (int i = 0; i < m; ++i) {
        V.col(i) = points.col(i + 1) - points.col(0);
    }

    // Step 2: Compute the QR decomposition
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(V);
    Eigen::MatrixXd Q = qr.householderQ() * Eigen::MatrixXd::Identity(n, m); // Extract the orthogonal matrix

    // Step 3: Create projection matrix
    Eigen::MatrixXd P = Q * Q.transpose();

    // Step 4: Project the point
    Eigen::VectorXd projectedPoint = P * (point - points.col(0)) + points.col(0);

    return projectedPoint;
}

void SND(std::vector<VectorND>& simplex_points, std::vector<VECTOR_SPACE_FIELD>& lambda) {
    MatrixNPlusOneD M;
    // creating a sub matrix
    VECTOR_SPACE_FIELD c[VECTOR_SPACE_NUMBER_OF_DIMENSIONS + 1];
    VECTOR_SPACE_FIELD detM = 0;
    for (size_t i = 0; i < VECTOR_SPACE_NUMBER_OF_DIMENSIONS + 1; i++) {
        MatrixND C;
        for (size_t j = 0; j < VECTOR_SPACE_NUMBER_OF_DIMENSIONS + 1; j++) {
            if (j == i) continue;
            C << simplex_points[j];
        }
        c[i] = (i % 2) ? -C.determinant() : C.determinant(); // todo: check this
        detM += c[i];
    }
    bool all_same_sign = true;
    for (size_t i = 0; i < VECTOR_SPACE_NUMBER_OF_DIMENSIONS + 1; i++) {
        all_same_sign = all_same_sign && CompareSigns(detM, c[i]);
    }
    if (all_same_sign) {
        for (size_t i = 0; i < VECTOR_SPACE_NUMBER_OF_DIMENSIONS + 1; i++) {
            lambda[i] = c[i] / detM;
        }
        // simplex_points is unmodified
    }
    else {
        VectorND d_vec = VectorND::Zero();
        for (size_t j = 0; j < simplex_points.size(); j++) {
            d_vec = d_vec + lambda[j] * simplex_points[j];
        }
        VECTOR_SPACE_FIELD d = d_vec.norm();
        // cast to a dimension lower
        for (size_t i = 1; i < VECTOR_SPACE_NUMBER_OF_DIMENSIONS + 1; i++) {
            if (CompareSigns(detM, c[i])) {
                std::vector<VectorND> temp_simplex_points = simplex_points;
                std::vector<VECTOR_SPACE_FIELD> temp_lambda = lambda;
                temp_simplex_points.erase(temp_simplex_points.begin() + i);
                temp_lambda.erase(temp_lambda.begin() + i);
                DistanceSubalgorithm(temp_simplex_points, temp_lambda);
                VectorND d_star_vec = VectorND::Zero();
                for (size_t j = 0; j < temp_simplex_points.size(); j++) {
                    d_star_vec = d_star_vec + temp_lambda[j] * temp_simplex_points[j];
                }
                VECTOR_SPACE_FIELD d_star = d_star_vec.norm();
                if (d_star < d) {
                    simplex_points = temp_simplex_points;
                    lambda = temp_lambda;
                    d = d_star;
                }
            }
        }
    }
}

size_t nCr(size_t n, size_t r) {
    return fact(n) / (fact(r) * fact(n - r));
}

// Returns factorial of n
size_t fact(size_t n) {
    if (n == 0) return 1;
    size_t res = 1;
    for (size_t i = 2; i <= n; i++) res = res * i;
    return res;
}

// Function to count the number of 1s in the binary representation of an integer
int countOnes(int n) {
    int count = 0;
    while (n) {
        count += n & 1;
        n >>= 1;
    }
    return count;
}

const std::vector<std::vector<size_t>> generateIndexCombinations(size_t n, int m) {
    int N = n;
    int limit = 1 << N;  // 2^N combinations
    std::vector<std::vector<size_t>> combinations;
    // Iterate over all possible combinations
    for (int i = 0; i < limit; ++i) {
        if (countOnes(i) == m) {  // Check if the combination has exactly m elements
            std::vector<size_t> combination;
            // Construct the combination
            for (int j = 0; j < N; ++j) {
                if (i & (1 << j)) {
                    std::cout << j << " ";
                    combination.push_back(j);
                }
            }
            combinations.push_back(combination);
        }
    }
    return combinations;
}

void SMD(std::vector<VectorND>& simplex_points, std::vector<VECTOR_SPACE_FIELD>& lambda) {
    assert(simplex_points.size() > 2); // dimension > 1
    assert(simplex_points.size() < VECTOR_SPACE_NUMBER_OF_DIMENSIONS + 1); // dimension < VECTOR_SPACE_NUMBER_OF_DIMENSIONS
    size_t target_dimension = simplex_points.size() - 1;
    size_t number_of_possible_hyperplanes = nCr(VECTOR_SPACE_NUMBER_OF_DIMENSIONS, target_dimension);


   
    VectorND p0;
    

    
    std::vector<std::vector<size_t>> index_combinations = generateIndexCombinations(VECTOR_SPACE_NUMBER_OF_DIMENSIONS, target_dimension);
    assert(index_combinations.size() == number_of_possible_hyperplanes);

    VECTOR_SPACE_FIELD mu_max = 0;
    size_t I = 0;
    size_t current_index = 0;

    for (auto& index_combination : index_combinations) {
        Eigen::MatrixX<VECTOR_SPACE_FIELD> C(target_dimension, target_dimension);
        for (size_t i = 0; i < simplex_points.size(); i++) {
            for (size_t j = 0; j < index_combination.size(); j++) {
                C(j,i) = simplex_points[i][index_combination[j]];
            }
        } 
        VECTOR_SPACE_FIELD mu = C.determinant();
        if (abs(mu) > abs(mu_max)) {
            mu_max = mu;
            I = current_index;
        }
        current_index++;
    }
    // Here we keep only 
    // TODO: FINISH THE REST OF THIS
    std::vector<VECTOR_SPACE_FIELD> c(simplex_points.size());

    for (size_t i = 0; i < simplex_points.size() - 1; i++) {
        // i is the index for the column in the matrix that we replace the point s_i with p0
        Eigen::MatrixX<VECTOR_SPACE_FIELD> M(target_dimension, target_dimension);
        Eigen::VectorX<VECTOR_SPACE_FIELD> origin_projected_on_hyperplane(target_dimension);
        size_t counter = 0;

        for (size_t j = 0; j < i; j++) {
            for (size_t k = 0; k < target_dimension; k++) {
                M(j, k) = simplex_points[j][index_combinations[I][k]];
            }
        }
        for (size_t k = 0; k < target_dimension; k++) {
            M(i, k) = p0[index_combinations[I][k]];
        }
        for (size_t j = i + 1; j < simplex_points.size(); j++) {
            for (size_t k = 0; k < target_dimension; k++) {
                M(j, k) = simplex_points[j][index_combinations[I][k]];
            }
        }

        c[i] = (i % 2) ? -M.determinant() : M.determinant();
    }

    bool is_all_same_signs = true;

    for (size_t i = 0; i < target_dimension; i++) {
        is_all_same_signs = is_all_same_signs && CompareSigns(mu_max, c[i]);
    }
    if (is_all_same_signs) {
        for (size_t i = 0; i < target_dimension; i++) {
            ;// TODO: SET LAMBDA VALUES
        }
    }
    else {

    }





}

bool inline CompareSigns(const VECTOR_SPACE_FIELD a, const VECTOR_SPACE_FIELD b) {
    return (a > 0 && b > 0) || (a < 0 && b < 0);
}

void S1D(std::vector<VectorND>& simplex_points, std::vector<VECTOR_SPACE_FIELD>& lambda) {
    VectorND s1 = simplex_points[0];
    VectorND s2 = simplex_points[1];

    VectorND t = s2 - s1;
    VectorND p0 = s2.dot(t) / t.dot(t) * t + s2;
    VECTOR_SPACE_FIELD mu_max = 0;
    size_t I = 0;
    for (size_t i = 0; i < VECTOR_SPACE_NUMBER_OF_DIMENSIONS; i++) {
        VECTOR_SPACE_FIELD mu = s1[i] - s2[i];
        if (abs(mu) > abs(mu_max)) {
            mu_max = mu;
            I = i;
        }
    }
    VECTOR_SPACE_FIELD c1 = p0[I] - s2[I];
    VECTOR_SPACE_FIELD c2 = s1[I] - p0[I];
    std::vector<VectorND> W; // W
    std::vector<VECTOR_SPACE_FIELD> barycentric_coordinates; // lambda

    if (CompareSigns(mu_max, c1) && CompareSigns(mu_max, c2)) {
        barycentric_coordinates.push_back(c1 / mu_max);
        W.push_back(s1);
        barycentric_coordinates.push_back(c2 / mu_max);
        W.push_back(s2);
    }
    else {
        barycentric_coordinates.push_back(1);
        W.push_back(s1);
    }
    simplex_points = W;
    lambda = barycentric_coordinates;
}

VectorND SupportFunction(const std::vector<VectorND>& A, const std::vector<VectorND>& B, VectorND direction) {
    return VectorND::Zero();
}



VECTOR_SPACE_FIELD gjk(const std::vector<VectorND>& A, const std::vector<VectorND>& B, VECTOR_SPACE_FIELD e) {
    // A and B are assumed to be convex sets
    size_t k = 0;
    VectorND initial_search_direction = VectorND::Zero(); // can be anything
    std::vector<VectorND> W;
    std::vector<VectorND> w;
    std::vector<VectorND> v;

    std::vector<VectorND> simplex_sofar;
    while (W.size() < VECTOR_SPACE_NUMBER_OF_DIMENSIONS + 1 && v[k].squaredNorm() <= e) {
        w.push_back(SupportFunction(A, B, -v[k]));
        if (v[k].squaredNorm() - v[k].dot(w[k]) <= e * e * v[k].squaredNorm()) continue;

        simplex_sofar.push_back(w[k]);


        k++;
    }
    return v[k].norm();
}


void DistanceSubalgorithm(std::vector<VectorND>& W, std::vector<VECTOR_SPACE_FIELD>& lambda) {
    assert(W.size());
    switch (W.size() - 1) {
        case VECTOR_SPACE_NUMBER_OF_DIMENSIONS:
            SND(W, lambda);
            break;
        case 1:
            S1D(W, lambda);
            break;
        case 0:
            assert(0);
            break;
        default:
            SMD(W, lambda);
    }
}


