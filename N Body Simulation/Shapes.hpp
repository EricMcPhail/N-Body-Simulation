#pragma once
#include <Eigen/Dense>
#include <vector>
#include <limits>
#include <iostream>

// Define the space dimensionality as a compile-time constant.
constexpr int N = 3;  // Change this to your desired dimension.

//
// AABB: Axis-Aligned Bounding Box
//
template<typename Scalar = double, int Dim = N>
struct AABB {
    Eigen::Matrix<Scalar, Dim, 1> min;
    Eigen::Matrix<Scalar, Dim, 1> max;

    AABB() {
        min = Eigen::Matrix<Scalar, Dim, 1>::Constant(std::numeric_limits<Scalar>::max());
        max = Eigen::Matrix<Scalar, Dim, 1>::Constant(std::numeric_limits<Scalar>::lowest());
    }

    // Expand the AABB to include a given point.
    void expand(const Eigen::Matrix<Scalar, Dim, 1>& point) {
        min = min.cwiseMin(point);
        max = max.cwiseMax(point);
    }

    // Simple intersection test with another AABB.
    bool intersects(const AABB<Scalar, Dim>& other) const {
        for (int i = 0; i < Dim; ++i)
            if (max[i] < other.min[i] || min[i] > other.max[i])
                return false;
        return true;
    }

    // Print data about the AABB.
    void print() const {
        std::cout << "AABB:\n";
        std::cout << "  Min: " << min.transpose() << "\n";
        std::cout << "  Max: " << max.transpose() << "\n";
    }
};

//
// OBB: Oriented Bounding Box
//
template<typename Scalar = double, int Dim = N>
struct OBB {
    Eigen::Matrix<Scalar, Dim, 1> center;
    Eigen::Matrix<Scalar, Dim, 1> halfExtents;
    Eigen::Matrix<Scalar, Dim, Dim> orientation; // Columns are the box's local axes.

    // Print data about the OBB.
    void print() const {
        std::cout << "OBB:\n";
        std::cout << "  Center: " << center.transpose() << "\n";
        std::cout << "  Half Extents: " << halfExtents.transpose() << "\n";
        std::cout << "  Orientation:\n" << orientation << "\n";
    }
};

//
// K-DOP: K-Discrete Oriented Polytope
//
template<typename Scalar = double, int Dim = N>
struct KDOP {
    // A set of normalized directions (usually 2N <= k).
    std::vector<Eigen::Matrix<Scalar, Dim, 1>> directions;
    std::vector<Scalar> minProj;
    std::vector<Scalar> maxProj;

    KDOP(const std::vector<Eigen::Matrix<Scalar, Dim, 1>>& dirs)
        : directions(dirs),
        minProj(dirs.size(), std::numeric_limits<Scalar>::max()),
        maxProj(dirs.size(), std::numeric_limits<Scalar>::lowest())
    { }

    // Expand the KDOP to include a point by projecting onto each direction.
    void expand(const Eigen::Matrix<Scalar, Dim, 1>& point) {
        for (size_t i = 0; i < directions.size(); ++i) {
            Scalar proj = directions[i].dot(point);
            if (proj < minProj[i]) minProj[i] = proj;
            if (proj > maxProj[i]) maxProj[i] = proj;
        }
    }

    // Print data about the KDOP.
    void print() const {
        std::cout << "KDOP:\n";
        for (size_t i = 0; i < directions.size(); ++i) {
            std::cout << "  Direction " << i << ": " << directions[i].transpose() << "\n";
            std::cout << "  Min Projection: " << minProj[i]
                << ", Max Projection: " << maxProj[i] << "\n";
        }
    }
};

//
// Capsule: defined by two endpoints and a radius.
//
template<typename Scalar = double, int Dim = N>
struct Capsule {
    Eigen::Matrix<Scalar, Dim, 1> pointA;
    Eigen::Matrix<Scalar, Dim, 1> pointB;
    Scalar radius;

    // Print data about the Capsule.
    void print() const {
        std::cout << "Capsule:\n";
        std::cout << "  Point A: " << pointA.transpose() << "\n";
        std::cout << "  Point B: " << pointB.transpose() << "\n";
        std::cout << "  Radius: " << radius << "\n";
    }
};

//
// S-Shell: Spherical Shell (an annulus in N-dimensions)
// (Defined by a center and inner/outer radii.)
//
template<typename Scalar = double, int Dim = N>
struct SShell {
    Eigen::Matrix<Scalar, Dim, 1> center;
    Scalar innerRadius;
    Scalar outerRadius;

    // Print data about the S-Shell.
    void print() const {
        std::cout << "S-Shell:\n";
        std::cout << "  Center: " << center.transpose() << "\n";
        std::cout << "  Inner Radius: " << innerRadius << "\n";
        std::cout << "  Outer Radius: " << outerRadius << "\n";
    }
};

//
// Sphere
//
template<typename Scalar = double, int Dim = N>
struct Sphere {
    Eigen::Matrix<Scalar, Dim, 1> center;
    Scalar radius;

    // Print data about the Sphere.
    void print() const {
        std::cout << "Sphere:\n";
        std::cout << "  Center: " << center.transpose() << "\n";
        std::cout << "  Radius: " << radius << "\n";
    }
};

//
// Ellipsoid: represented by a center, radii along principal axes, and an orientation matrix.
//
template<typename Scalar = double, int Dim = N>
struct Ellipsoid {
    Eigen::Matrix<Scalar, Dim, 1> center;
    Eigen::Matrix<Scalar, Dim, 1> radii; // Semi-axes lengths.
    Eigen::Matrix<Scalar, Dim, Dim> orientation; // Rotation aligning the ellipsoid.

    // Print data about the Ellipsoid.
    void print() const {
        std::cout << "Ellipsoid:\n";
        std::cout << "  Center: " << center.transpose() << "\n";
        std::cout << "  Radii: " << radii.transpose() << "\n";
        std::cout << "  Orientation:\n" << orientation << "\n";
    }
};

//
// K-IOS: Intersection of K Spheres
//
template<typename Scalar = double, int Dim = N>
struct KIOS {
    // A collection of spheres whose intersection defines the volume.
    std::vector<Sphere<Scalar, Dim>> spheres;

    // Print data about the K-IOS.
    void print() const {
        std::cout << "K-IOS (Intersection of " << spheres.size() << " spheres):\n";
        for (size_t i = 0; i < spheres.size(); ++i) {
            std::cout << "  Sphere " << i << ":\n";
            std::cout << "    Center: " << spheres[i].center.transpose() << "\n";
            std::cout << "    Radius: " << spheres[i].radius << "\n";
        }
    }
};

//
// SCB: Smallest Containing Ball (or similar concept)
//
template<typename Scalar = double, int Dim = N>
struct SCB {
    Eigen::Matrix<Scalar, Dim, 1> center;
    Scalar radius;

    // Print data about the SCB.
    void print() const {
        std::cout << "SCB (Smallest Containing Ball):\n";
        std::cout << "  Center: " << center.transpose() << "\n";
        std::cout << "  Radius: " << radius << "\n";
    }
};

//
// Convex Hull: a collection of vertices (and optionally faces/hyperplanes).
//
template<typename Scalar = double, int Dim = N>
struct ConvexHull {
    std::vector<Eigen::Matrix<Scalar, Dim, 1>> vertices;

    // Print data about the Convex Hull.
    void print() const {
        std::cout << "Convex Hull with " << vertices.size() << " vertices:\n";
        for (size_t i = 0; i < vertices.size(); ++i) {
            std::cout << "  Vertex " << i << ": " << vertices[i].transpose() << "\n";
        }
    }
};

//
// Cylinder: Generalized cylinder defined by a line segment (its axis) and a radius.
//
template<typename Scalar = double, int Dim = N>
struct Cylinder {
    Eigen::Matrix<Scalar, Dim, 1> pointA;
    Eigen::Matrix<Scalar, Dim, 1> pointB;
    Scalar radius;

    // Print data about the Cylinder.
    void print() const {
        std::cout << "Cylinder:\n";
        std::cout << "  Point A: " << pointA.transpose() << "\n";
        std::cout << "  Point B: " << pointB.transpose() << "\n";
        std::cout << "  Radius: " << radius << "\n";
    }
};

//
// Cone: defined by an apex, an axis direction, a half-angle (at the apex), and a height.
//
template<typename Scalar = double, int Dim = N>
struct Cone {
    Eigen::Matrix<Scalar, Dim, 1> apex;
    Eigen::Matrix<Scalar, Dim, 1> axis; // Should be normalized.
    Scalar angle; // Half-angle at the apex.
    Scalar height;

    // Print data about the Cone.
    void print() const {
        std::cout << "Cone:\n";
        std::cout << "  Apex: " << apex.transpose() << "\n";
        std::cout << "  Axis: " << axis.transpose() << "\n";
        std::cout << "  Angle (half-angle): " << angle << "\n";
        std::cout << "  Height: " << height << "\n";
    }
};


// Templated data structure for an M-dimensional plane in R^N.
template<typename Scalar = double, int M, int Dim = N>
struct Plane {
    // Ensure that the plane's dimension M is less than the ambient dimension N.
    static_assert(M < DIM, "Plane dimension M must be less than ambient space dimension N.");

    // The unique point on the plane (e.g., the point closest to the origin).
    Eigen::Matrix<Scalar, DIM, 1> point;

    // An orthonormal basis for the M-dimensional plane, stored as an N x M matrix.
    // Each column is one basis vector.
    Eigen::Matrix<Scalar, DIM, M> basis;

    // Constructor to initialize the plane.
    Plane(const Eigen::Matrix<Scalar, DIM, 1>& pt, const Eigen::Matrix<Scalar, DIM, M>& b)
        : point(pt), basis(b)
    {
        // Optionally, you might want to verify that the basis is orthonormal.
    }

    // A method to display the plane's data.
    void print() const {
        std::cout << "Plane point:\n" << point.transpose() << "\n\n";
        std::cout << "Plane basis (each column is a basis vector):\n" << basis << "\n";
    }
};