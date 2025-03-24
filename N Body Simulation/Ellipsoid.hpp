#if 0
#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <functional>
#include <vector>
#include <cmath>
#include <limits>

#include "Model.hpp"

using namespace Eigen;

//------------------------------------------------------------------------------
// Data structure for an ellipsoid in canonical form.
// The ellipsoid is defined by the quadratic form: Xᵀ A X = 0,
// where A is a 4×4 symmetric matrix in homogeneous coordinates.
// For a canonical ellipsoid (x/a)² + (y/b)² + (z/c)² = 1, A is given by:
//     diag(1/a², 1/b², 1/c², -1).
//------------------------------------------------------------------------------
struct Ellipsoid {
    Matrix4d A;
    Vector3d pos;

    Model* m;
    Ellipsoid(double a, double b, double c) {
        m = new Model(a, b, c);
        
    }
};

//------------------------------------------------------------------------------
// Given an ellipsoid with quadratic form A and a 4×4 motion matrix M,
// the transformed ellipsoid is represented by Aₜ = (M⁻¹)ᵀ * A * M⁻¹.
//------------------------------------------------------------------------------
Matrix4d transformEllipsoid(const Matrix4d& A, const Matrix4d& M) {
    Matrix4d Minv = M.inverse();
    return Minv.transpose() * A * Minv;
}

//------------------------------------------------------------------------------
// In our simple example the motion is given as a translation.
// This function computes a translation matrix M(t) that moves a point
// from position "start" to "end" as a function of time t ∈ [0,1].
//------------------------------------------------------------------------------
Matrix4d computeTranslationMatrix(double t, const Vector3d& start, const Vector3d& end) {
    Vector3d pos = start + t * (end - start);
    Matrix4d M = Matrix4d::Identity();
    M(0, 3) = pos(0);
    M(1, 3) = pos(1);
    M(2, 3) = pos(2);
    return M;
}

//------------------------------------------------------------------------------
// The CCD function F(u,t) is defined (after reparameterization) as:
//    F(u,t) = det( u·A₁(t) - (u-1)·A₂(t) )
// where A₁(t) and A₂(t) are the transformed quadratic forms of the two
// ellipsoids at time t. (Recall that with the substitution
// λ = u/(u-1), one “clears” the denominator by multiplying f(λ,t) by (u-1)⁴.)
//------------------------------------------------------------------------------
double evaluateF(double u, double t, const Ellipsoid& E1, const Ellipsoid& E2,
    std::function<Matrix4d(double)> M1,
    std::function<Matrix4d(double)> M2) {
    Matrix4d A1_t = transformEllipsoid(E1.A, M1(t));
    Matrix4d A2_t = transformEllipsoid(E2.A, M2(t));
    Matrix4d M = u * A1_t - (u - 1) * A2_t;
    return M.determinant();
}

//------------------------------------------------------------------------------
// For a given t, we sample F(u,t) over u ∈ [0,1] (here using 101 samples)
// and return the maximum value. This is used to determine the state of
// the collision (separated, touching, or overlapping).
//------------------------------------------------------------------------------
double getMaxF(double t, const Ellipsoid& E1, const Ellipsoid& E2,
    std::function<Matrix4d(double)> M1,
    std::function<Matrix4d(double)> M2) {
    int numUSamples = 101;
    double maxVal = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < numUSamples; i++) {
        double u = i / static_cast<double>(numUSamples - 1);
        double val = evaluateF(u, t, E1, E2, M1, M2);
        if (val > maxVal)
            maxVal = val;
    }
    return maxVal;
}

//------------------------------------------------------------------------------
// The state function for the collision detection.
// It returns:
//   +1 if the maximum F(u,t) > tol  (ellipsoids are separated),
//    0 if |max F(u,t)| < tol       (ellipsoids are touching),
//   -1 if max F(u,t) < -tol       (ellipsoids are overlapping).
//------------------------------------------------------------------------------
int getState(double t, const Ellipsoid& E1, const Ellipsoid& E2,
    std::function<Matrix4d(double)> M1,
    std::function<Matrix4d(double)> M2, double tol = 1e-6) {
    double maxVal = getMaxF(t, E1, E2, M1, M2);
    if (maxVal > tol)
        return +1;
    else if (std::abs(maxVal) < tol)
        return 0;
    else
        return -1;
}

//------------------------------------------------------------------------------
// Bézier Shoot:
// Given a candidate time interval [t1, t2] and the bivariate CCD function F(u,t),
// this routine fixes u at û (the value that maximizes F(u,t1)) and then performs
// a Bézier–clipping–like procedure (by sampling at Bernstein nodes) to narrow down
// the interval in t where F(û,t) crosses zero (i.e. where the ellipsoids come into contact).
//------------------------------------------------------------------------------
double bezierShoot(double t1, double t2,
    const Ellipsoid& E1, const Ellipsoid& E2,
    std::function<Matrix4d(double)> M1,
    std::function<Matrix4d(double)> M2,
    double tol = 1e-6, int maxIter = 20) {
    // Wrap the CCD function F(u,t) into a lambda.
    auto F = [&](double u, double t) -> double {
        return evaluateF(u, t, E1, E2, M1, M2);
        };
    // Step 1: Find û ∈ [0,1] that maximizes F(u, t1).
    int numUSamples = 101;
    double u_hat = 0.0;
    double maxF = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < numUSamples; i++) {
        double u = i / static_cast<double>(numUSamples - 1);
        double val = F(u, t1);
        if (val > maxF) {
            maxF = val;
            u_hat = u;
        }
    }
    if (maxF <= 0.0) {
        // No separation in the interval; return t1.
        return t1;
    }
    // Step 2: With û fixed, define f(t) = F(û,t).
    auto f = [u_hat, &F](double t) -> double {
        return F(u_hat, t);
        };
    double f1 = f(t1);
    double f2 = f(t2);
    if (f1 <= 0.0)
        return t1;  // already non-positive at the left endpoint
    if (f2 > 0.0)
        return t2;  // no sign change in the interval
    // Step 3: Perform a Bézier clipping–like procedure.
    int n = 4;  // degree = 4 => 5 control points
    double left = t1, right = t2;
    for (int iter = 0; iter < maxIter; iter++) {
        std::vector<double> controlValues(n + 1);
        // Sample at Bernstein nodes.
        for (int i = 0; i <= n; i++) {
            double ti = left + (i / static_cast<double>(n)) * (right - left);
            controlValues[i] = f(ti);
        }
        // Look for the first index where the value becomes non-positive.
        int idx = -1;
        for (int i = 1; i <= n; i++) {
            if (controlValues[i] <= 0.0) {
                idx = i;
                break;
            }
        }
        if (idx == -1) {
            // No sign change in this sample: return right.
            return right;
        }
        double newRight = left + (idx / static_cast<double>(n)) * (right - left);
        if ((right - left) < tol)
            return newRight;
        right = newRight;
    }
    return right;
}

//--------------------------|ALGORITHM 1|--------------------------------
// Recursive function to find the first contact time in [t1, t2].
// It assumes that at t1 the ellipsoids are separate (state +1)
// and at t2 they are either touching or overlapping (state 0 or -1).
// It uses the Bézier shoot to extract a candidate contact time and then
// recursively refines the interval.
//------------------------------------------------------------------------------
double findContactTime(double t1, double t2, const Ellipsoid& E1, const Ellipsoid& E2,
    std::function<Matrix4d(double)> M1,
    std::function<Matrix4d(double)> M2, double tol = 1e-6) {


    int stateZero = getState(t1, E1, E2, M1, M2, tol);
    int stateOne = getState(t2, E1, E2, M1, M2, tol);


    if (stateZero == 0) return t1;
    if (stateZero == stateOne) return t2;  // no state change; return the upper bound

    double t_contact = bezierShoot(t1, t2, E1, E2, M1, M2, tol, 20);
    int sc = getState(t_contact, E1, E2, M1, M2, tol);
    if (sc == 0 || (t2 - t1) < tol)
        return t_contact;
    double mid = (t1 + t_contact) / 2.0;
    if (getState(mid, E1, E2, M1, M2, tol) == +1)
        return findContactTime(mid, t_contact, E1, E2, M1, M2, tol);
    else
        return findContactTime(t1, mid, E1, E2, M1, M2, tol);
}

int test() {
    // Define two ellipsoids in canonical form.
    // For E1: (x/1)² + (y/2)² + (z/1.5)² = 1.
    // For E2: (x/1.2)² + (y/1.8)² + (z/1)² = 1.
    Ellipsoid E1, E2;
    double a1 = 1.0, b1 = 2.0, c1 = 1.5;
    double a2 = 1.2, b2 = 1.8, c2 = 1.0;

    E1.A = Matrix4d::Zero();
    E1.A(0, 0) = 1.0 / (a1 * a1);
    E1.A(1, 1) = 1.0 / (b1 * b1);
    E1.A(2, 2) = 1.0 / (c1 * c1);
    E1.A(3, 3) = -1.0;

    E2.A = Matrix4d::Zero();
    E2.A(0, 0) = 1.0 / (a2 * a2);
    E2.A(1, 1) = 1.0 / (b2 * b2);
    E2.A(2, 2) = 1.0 / (c2 * c2);
    E2.A(3, 3) = -1.0;

    // Define motion paths.
    // E1 remains static.
    Vector3d start1(0, 0, 0), end1(0, 0, 0);
    // E2 moves from left to right.
    Vector3d start2(-5, 0, 0), end2(5, 0, 0);

    // Construct motion matrices (here, using translation only).
    auto M1 = [=](double t) -> Matrix4d {
        return computeTranslationMatrix(t, start1, end1);
        };
    auto M2 = [=](double t) -> Matrix4d {
        return computeTranslationMatrix(t, start2, end2);
        };

    // Check the collision state at the endpoints.
    int state0 = getState(0.0, E1, E2, M1, M2);
    int state1 = getState(1.0, E1, E2, M1, M2);
    std::cout << "State at t = 0: " << state0 << " (expected +1: separated)" << std::endl;
    std::cout << "State at t = 1: " << state1 << " (expected 0 or -1: contacting/overlapping)" << std::endl;

    // If the ellipsoids are separated at t=0 and in contact/overlap at t=1,
    // find the first contact time.
    if (state0 == +1 && state1 != +1) {
        double contactTime = findContactTime(0.0, 1.0, E1, E2, M1, M2, 1e-6);
        std::cout << "First contact occurs at t = " << contactTime << std::endl;
        double maxF_contact = getMaxF(contactTime, E1, E2, M1, M2);
        std::cout << "Max F(u,t) at contact time = " << maxF_contact << std::endl;
    }
    else {
        std::cout << "No collision (or already colliding) in the interval [0,1]." << std::endl;
    }

    return 0;
}



#endif