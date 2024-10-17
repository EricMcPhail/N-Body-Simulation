#pragma once
#if 0
#include "VectorSpace.hpp"


VectorND getUnitNormalVector(const std::vector<VectorND>& vectors);


struct Collider {
    virtual VectorND FindFurthestPoint(VectorND direction) const = 0;
};

struct MeshCollider : Collider {
private:
    std::vector<VectorND> vertices;

    //-----------------------------------------------------------------------------
    // Returns the index of the vertex that yields the highest value
    // dot product with d out of all the verticies. In other words, it finds 
    // the furthest vertex (from verticies) along a certain direction (VectorND direction)
    size_t indexOfFurthestPoint(VectorND direction) const {
        VECTOR_SPACE_NUMBER_TYPE maxProduct = direction.dot(vertices[0]);
        size_t index = 0;
        for (size_t i = 1; i < vertices.size(); i++) {
            VECTOR_SPACE_NUMBER_TYPE product = direction.dot(vertices[i]);
            if (product > maxProduct) {
                maxProduct = product;
                index = i;
            }
        }
        return index;
    }
public:

    VectorND FindFurthestPoint(VectorND direction) const override {
        return vertices[indexOfFurthestPoint(direction)];
    }
};

VectorND Support(const Collider& colliderA, const Collider& colliderB, VectorND direction) {
    return colliderA.FindFurthestPoint(direction) - colliderB.FindFurthestPoint(-direction);
}

struct Simplex {
private:
    std::array<VectorND, VECTOR_SPACE_NUMBER_OF_DIMENSIONS + 1u> m_points;
    unsigned int m_size;

public:
    Simplex()
        : m_size(0)
    {}

    Simplex& operator=(std::initializer_list<VectorND> list) {
        for (VectorND point : list) m_points[m_size++] = point;
        return *this;
    }

    void push_front(VectorND point) {
        for (size_t i = std::min(m_size, VECTOR_SPACE_NUMBER_OF_DIMENSIONS); i >= 1; i--) {
            m_points[i] = m_points[i - 1];
        }
        m_points[0] = point;
        m_size = std::min(m_size + 1u, VECTOR_SPACE_NUMBER_OF_DIMENSIONS + 1u);
    }

    Simplex without(size_t j) {
        Simplex ret_val;
        for (size_t i = 0; i < m_size; i++) {
            if (i == j) continue;
            ret_val.push_front(m_points[i]);
        }
        return ret_val;
    }

    VectorND& operator[](const size_t i) { return m_points[i]; }
    size_t size() const { return m_size; }

    auto begin() const { return m_points.begin(); }
    auto end() const { return m_points.end() - (VECTOR_SPACE_NUMBER_OF_DIMENSIONS + 1u - m_size); }
};

bool GJK(const Collider& colliderA, const Collider& colliderB) {
    // Simplex is an array of points, max count is 4
    Simplex points;
    VectorND direction = VectorND::Unit(0); // TODO: Find if there is an optimal initial direction
    // Get initial support point in initial direction
    VectorND support = Support(colliderA, colliderB, direction);
    points.push_front(support);
    direction = -support; // New direction is towards the origin
    while (true) {
        support = Support(colliderA, colliderB, direction);
        if (support.dot(direction) <= 0) {
            return false; // no collision
        }
        points.push_front(support);
        if (NextSimplex(points, direction)) {
            return true;
        }
    }
}

bool NextSimplex(Simplex& points, VectorND& direction) {
    switch (points.size()) {
    case 2: return Line(points, direction);
    case 3: return Triangle(points, direction);
    case 4: return Tetrahedron(points, direction);
    case 5: return Simplex4D(points, direction);
    }

    // never should be here
    return false;
}

bool SameDirection(const VectorND& direction, const VectorND& ao) {
    return direction.dot(ao) > 0;
}

bool Line(Simplex& points, VectorND& direction) {
    VectorND a = points[0];
    VectorND b = points[1];

    VectorND ab = b - a;
    VectorND ao = -a;

    if (SameDirection(ab, ao)) {
        direction = (ab.cross(ao)).cross(ab);
    }
    else {
        points = { a };
        direction = ao;
    }

    return false;
}

bool Triangle(Simplex& points, VectorND& direction) {
    VectorND a = points[0];
    VectorND b = points[1];
    VectorND c = points[2];

    VectorND ab = b - a;
    VectorND ac = c - a;
    VectorND ao = -a;

    VectorND abc = ab.cross(ac); // ideally the shortest dir to the origin

    if (SameDirection(abc.cross(ac), ao)) {
        if (SameDirection(ac, ao)) {
            points = { a, c };
            direction = (ac.cross(ao)).cross(ac);
        } else {
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
            } else {
                points = { a, c, b };
                direction = -abc;
            }
        }
    }

    return false;
}

bool Tetrahedron(Simplex& points, VectorND& direction) {
    VectorND a = points[0];
    VectorND b = points[1];
    VectorND c = points[2];
    VectorND d = points[3];

    VectorND ab = b - a;
    VectorND ac = c - a;
    VectorND ad = d - a;
    VectorND ao = -a;

    VectorND abc = ab.cross(ac);
    VectorND acd = ac.cross(ad);
    VectorND adb = ad.cross(ab);

    if (SameDirection(abc, ao)) {
        return Triangle(points = { a, b, c }, direction);
    }

    if (SameDirection(acd, ao)) {
        return Triangle(points = { a, c, d }, direction);
    }

    if (SameDirection(adb, ao)) {
        return Triangle(points = { a, d, b }, direction);
    }

    return true;
}
#endif