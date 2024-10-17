#if 0

#include "GJK.hpp" 
#include <vector>


Eigen::VectorXd findOrthogonalVector(const std::vector<Eigen::VectorXd>& vectors, size_t number_of_dimensions) {
    assert(vectors.size() == number_of_dimensions - 2); // Assume we were given N lniearly independent vectors :)

    MatrixND matrix;
    for (size_t i = 0; i < number_of_dimensions - 1; ++i) {
        matrix.row(i) = vectors[i];
    }

    // The last row is a bit tricky; we're essentially looking for a solution
    // where we set this row such that the determinant will give us the desired orthogonal vector.
    // One approach is to solve for each component of the orthogonal vector directly,
    // but here we'll use a simpler conceptual placeholder.

    VectorND result;
    for (size_t i = 0; i < number_of_dimensions; ++i) {
        MatrixND tempMat = matrix;
        tempMat.row(number_of_dimensions - 1) = VectorND::Unit(i);
        result[i] = tempMat.determinant();
    }

    return result.normalized(); // Normalize the resulting vector
}



// Helper function to calculate the normal and distance from the origin to a face defined by 4 points
// Returns a pair: (normal vector, distance from the origin)
std::pair<VectorND, double> faceNormalAndDistance(const VectorND& a, const VectorND& b, const VectorND& c, const VectorND& d) {
    std::vector<VectorND> t{ (b - a), (c - a), (d - a) };
    VectorND normal = findOrthogonalVectorND(t).normalized(); // Placeholder for 4D cross product
    double distance = normal.dot(a); // Distance from the origin to the plane
    return { normal, distance };
}

bool Simplex4D(Simplex& points, VectorND& direction) {
    bool originInsideAllFaces = true;
    double maxDistance = -std::numeric_limits<double>::infinity();
    int faceToRemove = -1;

    // Check each face of the 4-simplex
    for (int i = 0; i < 5; ++i) {
        std::vector<VectorND> faceVertices;
        for (int j = 0; j < 5; ++j) {
            if (i != j) faceVertices.push_back(points[j]);
        }

        auto [normal, distance] = faceNormalAndDistance(faceVertices[0], faceVertices[1], faceVertices[2], faceVertices[3]);

        if (distance < 0) {
            // The origin is outside this face; simplex can be reduced
            originInsideAllFaces = false;
            if (distance > maxDistance) {
                maxDistance = distance;
                faceToRemove = i;
                direction = normal; // Update search direction towards the origin
            }
        }
    }

    if (!originInsideAllFaces) {
        // Remove the farthest face from the origin
        points.erase(simplex.begin() + faceToRemove);
        return false; // The simplex does not enclose the origin
    }

    // If the origin is inside all faces, the simplex encloses the origin
    return true;
}



// =-=-=-=-=-=-=-=-=-=-==-=-=-=-=-=-=-=-=-=-=-=-=-===-=-=-=-=-


bool Line4D(Simplex& points, VectorND& direction) {
    VectorND a = points[0];
    VectorND b = points[1];

    VectorND ab = b - a;
    VectorND ao = -a;

    if (SameDirection(ab, ao)) {
        // We project AO onto the line AB, then we subtract this projection from AO to get
        // a direction that is orthogonal to AB and points towards the origin
        direction = ao - (ao.dot(ab) / ab.dot(ab)) * ab; // AO - proj_AB(AO)
    }
    else {
        points = { a };
        direction = ao;
    }

    return false;
}

bool Triangle4D(Simplex& points, VectorND& direction) {
    VectorND a = points[0];
    VectorND b = points[1];
    VectorND c = points[2];

    VectorND ab = b - a;
    VectorND ac = c - a;
    VectorND ao = -a;

    VectorND abc = ab.cross(ac);


    (ab.cross(ac)).cross(ac);


    ab.cross(ab.cross(ac))


    if (SameDirection(abc.cross(ac), ao)) {
        if (SameDirection(ac, ao)) {
            points = { a, c };





            direction = ao - (ao.dot(ac) / ac.dot(ac)) * ac; // AO - proj_AC(AO)
        }

        else {
            return Line(points = { a, b }, direction);
        }
    }

    else {
        if (SameDirection(ab.cross(abc), ao)) {
            return Line(points = { a, b }, direction);
        }

        else {
            if (SameDirection(abc, ao)) {
                direction = abc;
            }

            else {
                points = { a, c, b };
                direction = -abc;
            }
        }
    }

    return false;
}


bool CompareSigns(double a, double b) {
    return ((a > 0 && b > 0) || (a < 0 && b < 0));
}

void S3D(Simplex& s) {
    VectorND s1 = s[0];
    VectorND s2 = s[2];
    VectorND s3 = s[3];
    VectorND s4 = s[4];
    std::vector<VECTOR_SPACE_NUMBER_TYPE> C4;
    std::vector<VECTOR_SPACE_NUMBER_TYPE> M4;
    



    Eigen::Matrix3d M41;
    Eigen::Matrix3d M42;
    Eigen::Matrix3d M43;
    Eigen::Matrix3d M44;

    Eigen::Matrix<VECTOR_SPACE_NUMBER_TYPE, 4, 4> M;
    M.col(0) = Eigen::Vector<VECTOR_SPACE_NUMBER_TYPE, 4>(s[0], 1);
    M.col(1) = Eigen::Vector<VECTOR_SPACE_NUMBER_TYPE, 4>(s[1], 1);
    M.col(2) = Eigen::Vector<VECTOR_SPACE_NUMBER_TYPE, 4>(s[2], 1);
    M.col(3) = Eigen::Vector<VECTOR_SPACE_NUMBER_TYPE, 4>(s[3], 1);

    M41.col(0) = s[1];
    M41.col(1) = s[2];
    M41.col(2) = s[3];

    M42.col(0) = s[0];
    M42.col(1) = s[2];
    M42.col(2) = s[3];

    M43.col(0) = s[0];
    M43.col(1) = s[1];
    M43.col(2) = s[3];

    M44.col(0) = s[0];
    M44.col(1) = s[1];
    M44.col(2) = s[2];

    M4.push_back(M41.determinant());
    M4.push_back(M42.determinant());
    M4.push_back(M43.determinant());
    M4.push_back(M44.determinant());

    VECTOR_SPACE_NUMBER_TYPE detM = 0;

    for (int j = 1; j <= 4; j++) {
        VECTOR_SPACE_NUMBER_TYPE temp;
        temp = pow((-1), j + 4) * M4[j - 1];
        C4.push_back(temp);
        detM += C4[j - 1];
    }

    bool ans = true;
    for (int i = 0; i < 4; i++) ans = ans && CompareSigns(detM, C4[i]);
    std::vector<double> lambda;
    std::vector<VectorND> W;
    if (ans) {
        for (int j = 0; j < 4; j++) lambda.push_back(C4[j] / detM);
        for (int j = 0; j < 4; j++) W.push_back(s[j]);
        return;
    }


    for (int j = 2; j <= 4; j++) {
        if (CompareSigns(detM, -C4[j - 1])) {

        }

    }

    
}

VectorND grad_f(VectorND y) {

}

double f(VectorND y) {

}
double g(VectorND y) {

}



double get_t_at_i(std::vector<double> y, size_t i) {
    size_t n = y.size();
    double sum = 0;
    for (int j = i; j < n; j++) sum += y[j] - 1;
    return sum / (n - i);
}

#include <algorithm>    // std::sort

VectorND getProjection(const std::vector<double> y, size_t n) {
    std::vector<double> sorted_y;
    for (const auto& e : y) sorted_y.push_back(e);
    std::sort(sorted_y.begin(), sorted_y.end());
    size_t n = y.size();
    size_t i = n - 1;
    double t_hat = get_t_at_i(sorted_y, 0); // set to t0
    for (size_t i = n - 1; i > 0; i--) {
        double ti = get_t_at_i(sorted_y, i);
        if (ti >= sorted_y[i]) {
            t_hat = ti;
            break;
        }
    }
    VectorND x = VectorND::Zero();
    for (size_t i = 0; i < n; i++) {
        if (y[i] > t_hat) x[i] = y[i] - t_hat;
        else x[i] = 0;
    }
    return x;
}   



Eigen::VectorXd projectOntoSimplex(const Simplex& simplex, const Eigen::VectorXd& point) {
    // Number of dimensions
    int N = point.size();

    // Build the matrix for the linear system: [simplexVertices; ones] and [point; 1]
    Eigen::MatrixXd A(N + 1, N + 1);

    Eigen::MatrixXd simplexVertices(simplex.size(), point.size()); //TODO COMPLETE
    for (size_t i = 0; i < simplex.size(); i++) {
        simplexVertices.row(i) = simplex[i];
    }   
    
    A << simplexVertices.transpose(), Eigen::VectorXd::Ones(N + 1);
    Eigen::VectorXd b(N + 1);
    b << point, 1;

    // Solve for the barycentric coordinates
    Eigen::VectorXd lambda = A.colPivHouseholderQr().solve(b);

    // Project the point onto the simplex using the barycentric coordinates
    Eigen::VectorXd projectedPoint = simplexVertices.transpose() * lambda.head(N);

    return projectedPoint;
}

void cullSimplexPoints(Simplex& s, const VectorND& target) {
    Eigen::MatrixXd simplexVertices(simplex.size(), point.size()); //TODO COMPLETE
    for (size_t i = 0; i < simplex.size(); i++) {
        simplexVertices.row(i) = simplex[i];
    }

    A << simplexVertices.transpose(), Eigen::VectorXd::Ones(N + 1);
}


double NesterovAcceleratedGJK(VectorND x0 /*Starting Point*/) {
    double epsilon = 0.00001;
    VectorND d_last = x0;
    VectorND s_last = x0;
    VectorND x = x0;
    size_t k = 0;
    bool flag = false;
    VectorND d;
    Simplex w;

    while (true) {
        double delta = (k + 1) / (k + 3);
        VectorND y = delta * x + (1 - delta) * s_last;
        if (!flag) {
            d = delta * d_last + (1 - delta) * grad_f(y);
        }
        else {
            d = x;
        }
        VectorND s = Support(d);

        
        if (g(x) <= epsilon) {
            if (d == x) return f(x);
            s = Support(grad_f(x));
            flag = true;
        }

        w.push_front(s);
        x = projectOntoSimplex(w, VectorND::Zero());
        if (x == VectorND::Zero()) return 0.0;
        cullSimplexPoints(w, x);
        s_last = s;
        d_last = d;
        ++k;
    }
}

double getDistance(const std::pair<Simplex, VectorND>& coord) {
    VectorND sum = VectorND::Zero();
    for (size_t i = 0; i < coord.first.size(); i++) {
        sum += coord.second[i] * coord.first[i];
    }
    return sqrt(sum.dot(sum));
}

template<size_t N> std::pair<Simplex, VectorND> SD(Simplex& s) {
   
    double smallest_distance = 99999999.9;

    std::vector<VECTOR_SPACE_NUMBER_TYPE> CNp1;
    std::vector<VECTOR_SPACE_NUMBER_TYPE> MNp1;
    Eigen::Matrix<VECTOR_SPACE_NUMBER_TYPE, N + 1, N + 1> M;


    Eigen::Matrix<VECTOR_SPACE_NUMBER_TYPE, 4, 4> M;
    Eigen::MatrixXd simplexVertices(simplex.size(), point.size());
    for (size_t i = 0; i < simplex.size(); i++) {
        simplexVertices.row(i) = simplex[i];
    }
    M << simplexVertices.transpose(), Eigen::VectorXd::Ones(N + 1);

    std::vector<Eigen::Matrix<VECTOR_SPACE_NUMBER_TYPE, N, N>> M_NPlusOne(N);

    for (size_t i = 0; i < N; i++) {
        M_NPlusOne.push_back(Eigen::Matrix<VECTOR_SPACE_NUMBER_TYPE, N, N>::Zero());
        bool skipped_flag = false;
        for (size_t j = 0; j < N; j++) {
            if (j == i) {
                skipped_flag = true;
                continue;
            }
            M_NPlusOne[i].col(j - skipped_flag) = s[j];
        }
    }

    std::vector<double> M4;

    for (size_t i = 0; i < N; i++) {
        M4.push_back(M_NPlusOne[i].determinant());
    }

    VECTOR_SPACE_NUMBER_TYPE detM = 0;

    for (int j = 1; j <= 4; j++) {
        VECTOR_SPACE_NUMBER_TYPE temp;
        temp = pow((-1), j + 4) * M4[j - 1];
        C4.push_back(temp);
        detM += C4[j - 1];
    }

    bool ans = true;
    for (int i = 0; i < 4; i++) ans = ans && CompareSigns(detM, C4[i]);
    std::vector<double> lambda;
    std::vector<VectorND> W;
    if (ans) {
        for (int j = 0; j < 4; j++) lambda.push_back(C4[j] / detM);
        for (int j = 0; j < 4; j++) W.push_back(s[j]);
        return;
    }


    for (int j = 2; j <= 4; j++) {
        if (CompareSigns(detM, -C4[j - 1])) {
            std::pair<Simplex, VectorND> temp = SD<N - 1>(s.without(j));
            double distance = getDistance(temp);
            if (distance < smallest_distance) {
                // Change the simplex

                s = s.without(j);


                smallest_distance = distance;
            }
        }

    }


}


std::pair<Simplex, Eigen::VectorXd> SignedVolumes(const Simplex& s) {
    size_t r = s.size() - 1;
    switch (r)
    {
    case 3:

        break;

    case 2:

        break;

    case 1:

        break;

    case 0:
        Eigen::Vector<double, 1> lambda = Eigen::Vector<double, 1>::Ones();
        return std::pair<Simplex, Eigen::VectorXd>{s, lambda};
    default:
        throw(1);
        break;
    }

}


Eigen::VectorXd projectPointOntoHyperplane(const Eigen::VectorXd& point, const Eigen::VectorXd& normal) {
    double distance = point.dot(normal) / normal.norm();
    Eigen::VectorXd projection = point - distance * normal.normalized();
    return projection;
}


double detM(Simplex s, size_t i, size_t j) {

}

void SD_N_to_R(Simplex& s) {
    // Given that we are working in N dimensions
    // A full simplex is N + 1 verticies
    // Here our simplex s has R + 1 verticies
    // We need to project onto a hyperplane R dimensions
    // We assume that vectors in s are of dimensionality N

    size_t R = 2;
    VectorND projection_of_origin_to_hyperplane;
    double mu_max = 0;

    size_t J = -1;
    // Calculate max{|M_{1,4}|, M_{2,4}|,M_{3,4}|} and corresponding coordinate J index to discard
    for (size_t i = 0; i < 3; i++) {
        double mu = detM(s, i + 1, 4);
        if (abs(mu) > abs(mu_max)) {
            mu_max = mu;
            J = i;
        }
    }
}

double getSubDeterminate(Simplex s, int k, int l) {

}
Vector projectOntoAffineHull(Simplex s, Vector point) {

}

size_t n;

double subDeterminateWithOriginReplacement();

bool all_same_sign(const std::vector<double>& c, double mu) { return true; }



#include "MinHeap.hpp"


// Function to calculate the k-measure of an m-simplex in N-dimensional space
double simplexKMeasure(const std::vector<VectorXd>& points, int k) {
    int m = points.size() - 1; // m-simplex
    int N = points[0].size();  // Dimensionality of space

    // Check if the input is valid
    if (k < 1 || k > N || m < 1 || m > N || k > m) {
        std::cerr << "Invalid parameters for k, m, or N." << std::endl;
        return -1.0;
    }

    // Construct vectors from the first point to the others
    std::vector<VectorXd> vectors(m, VectorXd(N));
    for (int i = 0; i < m; ++i) {
        vectors[i] = points[i + 1] - points[0];
    }

    // Build the Gram matrix
    MatrixXd G = MatrixXd::Zero(k, k);
    for (int i = 0; i < k; ++i) {
        for (int j = 0; j < k; ++j) {
            G(i, j) = vectors[i].dot(vectors[j]);
        }
    }

    // Calculate the k-measure
    double kMeasure = std::pow(std::abs(G.determinant()), 0.5) / std::tgamma(k + 1);
    return G.determinant();
}


void SMD(Simplex& s, std::vector<REAL>& barycentric_coordinates, size_t m) {

    // Suppose we are given a m dimensional simplex
    Vector p = projectOntoAffineHull(s, Vector::Zero());
    double mu_max = 0.0;
    int k = 2;
    int l = 3;
    int J = -1;
    
    double mu_min = 0.0;
    // We wish to keep only m verticies

    MinHeap heap = MinHeap(m);
    for (size_t i = 0; i < VECTOR_SPACE_NUMBER_OF_DIMENSIONS; i++) {
        double mu = getSubDeterminate(s, k, l);
        if (mu > mu_min) {
            heap.extractMin();
            heap.insertKey(mu, i);
            mu_min = heap.getMin();
        }
        k = l;
        l = i;
    }
    // we keep only the coordinates in the heap



    // Suppose we discarded Jth coordinate -----------------------------------------

    k = 2;
    l = 3;
    std::vector<double> Cj;

    // Test simplicites
    // We skip the first one bc the last few gives us enough information
    for (int j = 1; j < s.size(); j++) {
        // We will replace the jth simplex vertex with the origin
        int tmp = 1 - 2 * ((j-1) % 2); // either 1 or -1
        double Mj = subDeterminateWithOriginReplacement();
        Cj.push_back(tmp*Mj);
    }

    if (all_same_sign(Cj, mu_max)) {
        // CALCULATE POINTS HERE
        barycentric_coordinates = ? ? ? ;
        s = s;
    }
    else {
        // start at 1, we skip an index for reasons
        for (int j = 1; j < s.size(); j++) {
            if (CompareSigns(mu_max, Cj[j - 1])) {
                SMD(s, barycentric_coordinates, m - 1);

            }

        }
    }
}



#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>



#endif