#include "VectorSpace.hpp"

#include <float.h>
 /*! @brief Precision of floating-point numbers.
  *
  * Default is set to 64-bit (Double). Change this to quickly play around with 16- and 32-bit. */
#ifdef USE_32BITS
#define gkFloat   float
#define gkEpsilon FLT_EPSILON
#else
#define gkFloat   double
#define gkEpsilon DBL_EPSILON
#endif

  /*! @brief Data structure for convex polytopes.
     *
     * Polytopes are three-dimensional shapes and the GJK algorithm works directly on their convex-hull. However the convex-hull is never computed explicitly, instead each GJK-iteration employs a support function that has a cost linearly dependent on the number of points defining the polytope. */
typedef struct gkPolytope_ {
    int numpoints; /*!< Number of points defining the polytope. */
    gkFloat s
        [3]; /*!< Furthest point returned by the support function and updated at each GJK-iteration. For the first iteration this value is a guess - and this guess not irrelevant. */
    gkFloat**
        coord; /*!< Coordinates of the points of the polytope. This is owned by user who manages and garbage-collects the memory for these coordinates. */
} gkPolytope;

/*! @brief Data structure for simplex.
   *
   * The simplex is updated at each GJK-iteration. For the first iteration this value is a guess - and this guess not irrelevant. */
struct gkSimplex {
    size_t nvrtx; /*!< Number of points defining the simplex. */    
    std::array<VectorND, VECTOR_SPACE_NUMBER_OF_DIMENSIONS + 1> vrtx;  /*!< Coordinates of the points of the simplex. gkFloat vrtx[4][3]; */
};



#include <stdio.h>
#include <stdlib.h>
#include "math.h"

#define mexPrintf printf

#define eps_rel22        (gkFloat) gkEpsilon * 1e4f
#define eps_tot22        (gkFloat) gkEpsilon * 1e2f

#define norm2(a)         (a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
#define dotProduct(a, b) (a[0] * b[0] + a[1] * b[1] + a[2] * b[2])

#define S3Dregion1234()                                                                                                \
  v[0] = 0;                                                                                                            \
  v[1] = 0;                                                                                                            \
  v[2] = 0;                                                                                                            \
  s->nvrtx = 4;

#define select_1ik()                                                                                                   \
  s->nvrtx = 3;                                                                                                        \
  for (t = 0; t < 3; t++)                                                                                              \
    s->vrtx[2][t] = s->vrtx[3][t];                                                                                     \
  for (t = 0; t < 3; t++)                                                                                              \
    s->vrtx[1][t] = si[t];                                                                                             \
  for (t = 0; t < 3; t++)                                                                                              \
    s->vrtx[0][t] = sk[t];

#define select_1ij()                                                                                                   \
  s->nvrtx = 3;                                                                                                        \
  for (t = 0; t < 3; t++)                                                                                              \
    s->vrtx[2][t] = s->vrtx[3][t];                                                                                     \
  for (t = 0; t < 3; t++)                                                                                              \
    s->vrtx[1][t] = si[t];                                                                                             \
  for (t = 0; t < 3; t++)                                                                                              \
    s->vrtx[0][t] = sj[t];

#define select_1jk()                                                                                                   \
  s->nvrtx = 3;                                                                                                        \
  for (t = 0; t < 3; t++)                                                                                              \
    s->vrtx[2][t] = s->vrtx[3][t];                                                                                     \
  for (t = 0; t < 3; t++)                                                                                              \
    s->vrtx[1][t] = sj[t];                                                                                             \
  for (t = 0; t < 3; t++)                                                                                              \
    s->vrtx[0][t] = sk[t];

#define select_1i()                                                                                                    \
  s->nvrtx = 2;                                                                                                        \
  for (t = 0; t < 3; t++)                                                                                              \
    s->vrtx[1][t] = s->vrtx[3][t];                                                                                     \
  for (t = 0; t < 3; t++)                                                                                              \
    s->vrtx[0][t] = si[t];

#define select_1j()                                                                                                    \
  s->nvrtx = 2;                                                                                                        \
  for (t = 0; t < 3; t++)                                                                                              \
    s->vrtx[1][t] = s->vrtx[3][t];                                                                                     \
  for (t = 0; t < 3; t++)                                                                                              \
    s->vrtx[0][t] = sj[t];

#define select_1k()                                                                                                    \
  s->nvrtx = 2;                                                                                                        \
  for (t = 0; t < 3; t++)                                                                                              \
    s->vrtx[1][t] = s->vrtx[3][t];                                                                                     \
  for (t = 0; t < 3; t++)                                                                                              \
    s->vrtx[0][t] = sk[t];

#define getvrtx(point, location)                                                                                       \
  point[0] = s->vrtx[location][0];                                                                                     \
  point[1] = s->vrtx[location][1];                                                                                     \
  point[2] = s->vrtx[location][2];

#define calculateEdgeVector(p1p2, p2)                                                                                  \
  p1p2[0] = p2[0] - s->vrtx[3][0];                                                                                     \
  p1p2[1] = p2[1] - s->vrtx[3][1];                                                                                     \
  p1p2[2] = p2[2] - s->vrtx[3][2];

#define S1Dregion1()                                                                                                   \
  v[0] = s->vrtx[1][0];                                                                                                \
  v[1] = s->vrtx[1][1];                                                                                                \
  v[2] = s->vrtx[1][2];                                                                                                \
  s->nvrtx = 1;                                                                                                        \
  s->vrtx[0][0] = s->vrtx[1][0];                                                                                       \
  s->vrtx[0][1] = s->vrtx[1][1];                                                                                       \
  s->vrtx[0][2] = s->vrtx[1][2];

#define S2Dregion1()                                                                                                   \
  v[0] = s->vrtx[2][0];                                                                                                \
  v[1] = s->vrtx[2][1];                                                                                                \
  v[2] = s->vrtx[2][2];                                                                                                \
  s->nvrtx = 1;                                                                                                        \
  s->vrtx[0][0] = s->vrtx[2][0];                                                                                       \
  s->vrtx[0][1] = s->vrtx[2][1];                                                                                       \
  s->vrtx[0][2] = s->vrtx[2][2];

#define S2Dregion12()                                                                                                  \
  s->nvrtx = 2;                                                                                                        \
  s->vrtx[0][0] = s->vrtx[2][0];                                                                                       \
  s->vrtx[0][1] = s->vrtx[2][1];                                                                                       \
  s->vrtx[0][2] = s->vrtx[2][2];

#define S2Dregion13()                                                                                                  \
  s->nvrtx = 2;                                                                                                        \
  s->vrtx[1][0] = s->vrtx[2][0];                                                                                       \
  s->vrtx[1][1] = s->vrtx[2][1];                                                                                       \
  s->vrtx[1][2] = s->vrtx[2][2];

#define S3Dregion1()                                                                                                   \
  v[0] = s1[0];                                                                                                        \
  v[1] = s1[1];                                                                                                        \
  v[2] = s1[2];                                                                                                        \
  s->nvrtx = 1;                                                                                                        \
  s->vrtx[0][0] = s1[0];                                                                                               \
  s->vrtx[0][1] = s1[1];                                                                                               \
  s->vrtx[0][2] = s1[2];


inline static gkFloat
determinanta(const gkFloat* p, const gkFloat* q, const gkFloat* r) {
    return p[0] * ((q[1] * r[2]) - (r[1] * q[2])) - p[1] * (q[0] * r[2] - r[0] * q[2])
        + p[2] * (q[0] * r[1] - r[0] * q[1]);
}

inline static void
crossProducta(const gkFloat* a, const gkFloat* b, gkFloat* c) {
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}











inline static void
support(gkPolytope* body, const gkFloat* v) {
    gkFloat s, maxs;
    gkFloat* vrt;
    int better = -1;

    maxs = dotProduct(body->s, v);

    for (int i = 0; i < body->numpoints; ++i) {
        vrt = body->coord[i];
        s = dotProduct(vrt, v);
        if (s > maxs) {
            maxs = s;
            better = i;
        }
    }

    if (better != -1) {
        body->s[0] = body->coord[better][0];
        body->s[1] = body->coord[better][1];
        body->s[2] = body->coord[better][2];
    }
}

inline static void
subalgorithm(gkSimplex* s, gkFloat* v) {
    switch (s->nvrtx) {
    case 4:
        S3D(s, v);
        break;
    case 3:
        S2D(s, v);
        break;
    case 2:
        S1D(s, v);
        break;
    default:
        mexPrintf("\nERROR:\t invalid simplex\n");
    }
}

gkFloat
compute_minimum_distance(gkPolytope bd1, gkPolytope bd2, gkSimplex* s) {
    unsigned int k = 0;                /**< Iteration counter                 */
    const int mk = 25;                 /**< Maximum number of GJK iterations  */
    const gkFloat eps_rel = eps_rel22; /**< Tolerance on relative             */
    const gkFloat eps_tot = eps_tot22; /**< Tolerance on absolute distance    */

    const gkFloat eps_rel2 = eps_rel * eps_rel;
    unsigned int i;
    gkFloat w[3];
    gkFloat v[3];
    gkFloat vminus[3];
    gkFloat norm2Wmax = 0;

    /* Initialise search direction */
    v[0] = bd1.coord[0][0] - bd2.coord[0][0];
    v[1] = bd1.coord[0][1] - bd2.coord[0][1];
    v[2] = bd1.coord[0][2] - bd2.coord[0][2];

    /* Inialise simplex */
    s->nvrtx = 1;
    for (int t = 0; t < 3; ++t) {
        s->vrtx[0][t] = v[t];
    }

    for (int t = 0; t < 3; ++t) {
        bd1.s[t] = bd1.coord[0][t];
    }

    for (int t = 0; t < 3; ++t) {
        bd2.s[t] = bd2.coord[0][t];
    }

    /* Begin GJK iteration */
    do {
        k++;

        /* Update negative search direction */
        for (int t = 0; t < 3; ++t) {
            vminus[t] = -v[t];
        }

        /* Support function */
        support(&bd1, vminus);
        support(&bd2, v);
        for (int t = 0; t < 3; ++t) {
            w[t] = bd1.s[t] - bd2.s[t];
        }

        /* Test first exit condition (new point already in simplex/can't move
         * further) */
        gkFloat exeedtol_rel = (norm2(v) - dotProduct(v, w));
        if (exeedtol_rel <= (eps_rel * norm2(v)) || exeedtol_rel < eps_tot22) {
            break;
        }

        if (norm2(v) < eps_rel2) { // it a null V
            break;
        }

        /* Add new vertex to simplex */
        i = s->nvrtx;
        for (int t = 0; t < 3; ++t) {
            s->vrtx[i][t] = w[t];
        }
        s->nvrtx++;

        /* Invoke distance sub-algorithm */
        subalgorithm(s, v);

        /* Test */
        for (int jj = 0; jj < s->nvrtx; jj++) {
            gkFloat tesnorm = norm2(s->vrtx[jj]);
            if (tesnorm > norm2Wmax) {
                norm2Wmax = tesnorm;
            }
        }

        if ((norm2(v) <= (eps_tot * eps_tot * norm2Wmax))) {
            break;
        }

    } while ((s->nvrtx != 4) && (k != mk));

    if (k == mk) {
        mexPrintf("\n * * * * * * * * * * * * MAXIMUM ITERATION NUMBER REACHED!!!  "
            " * * * * * * * * * * * * * * \n");
    }

    return sqrt(norm2(v));
}

/// WORKING ON ======================================================================
//==================================================================================


inline static void projectOnHyperPlane(std::vector<const VectorND&> hp, VectorND& v) {
    VectorND n, pq, pr;

    for (int i = 0; i < VECTOR_SPACE_NUMBER_OF_DIMENSIONS; i++) {
        pq[i] = p[i] - q[i];
    }
    for (int i = 0; i < VECTOR_SPACE_NUMBER_OF_DIMENSIONS; i++) {
        pr[i] = p[i] - r[i];
    }

    n = pq.cross(pr);
    const REAL tmp = n.dot(p) / n.dot(n);

    for (int i = 0; i < 3; i++) {
        v[i] = n[i] * tmp;
    }
}

inline static void SND(gkSimplex& s, VectorND& v) {
    std::vector<VectorND> simplex_points; // N + 1
    std::vector<VectorND> s_coord; // N
    std::vector<bool> hyperplane_tests;



}

inline static void S3D(gkSimplex& s, VectorND& v) {
    VectorND s1, s2, s3, s4, s1s2, s1s3, s1s4;
    VectorND si, sj, sk;
    int testLineThree, testLineFour, testPlaneTwo, testPlaneThree, testPlaneFour, dotTotal;
    int i, j, k, t;

    s1 = s.vrtx[3]; // getvrtx(s1, 3);
    s2 = s.vrtx[2]; // getvrtx(s2, 2);
    s3 = s.vrtx[1]; // getvrtx(s3, 1);
    s4 = s.vrtx[0]; // getvrtx(s4, 0);

    
    s1s2 = s2 - s.vrtx[3];  // calculateEdgeVector(s1s2, s2);
    s1s3 = s3 - s.vrtx[3];  // calculateEdgeVector(s1s3, s3);
    s1s4 = s4 - s.vrtx[3];  // calculateEdgeVector(s1s4, s4);

    int hff1_tests[3];
    hff1_tests[2] = hff1(s1, s2);
    hff1_tests[1] = hff1(s1, s3);
    hff1_tests[0] = hff1(s1, s4);
    testLineThree = hff1(s1, s3);
    testLineFour = hff1(s1, s4);

    dotTotal = hff1(s1, s2) + testLineThree + testLineFour;
    if (dotTotal == 0) { /* case 0.0 -------------------------------------- */
        S3DR1(s,v,s1);
        return;
    }

    const gkFloat det134 = determinant(s1s3, s1s4, s1s2);
    const int sss = (det134 <= 0);

    testPlaneTwo = hff3(s1, s3, s4) - sss;
    testPlaneTwo = testPlaneTwo * testPlaneTwo;
    testPlaneThree = hff3(s1, s4, s2) - sss;
    testPlaneThree = testPlaneThree * testPlaneThree;
    testPlaneFour = hff3(s1, s2, s3) - sss;
    testPlaneFour = testPlaneFour * testPlaneFour;

    switch (testPlaneTwo + testPlaneThree + testPlaneFour) {
    case 3:
        S3Dregion1234();
        break;

    case 2:
        // Only one facing the oring
        // 1,i,j, are the indices of the points on the triangle and remove k from
        // simplex
        s.nvrtx = 3;
        if (!testPlaneTwo) { // k = 2;   removes s2
            for (i = 0; i < 3; i++) {
                s.vrtx[2][i] = s.vrtx[3][i];
            }
        }
        else if (!testPlaneThree) { // k = 1; // removes s3
            for (i = 0; i < 3; i++) {
                s.vrtx[1][i] = s2[i];
            }
            for (i = 0; i < 3; i++) {
                s.vrtx[2][i] = s.vrtx[3][i];
            }
        }
        else if (!testPlaneFour) { // k = 0; // removes s4  and no need to reorder
            for (i = 0; i < 3; i++) {
                s.vrtx[0][i] = s3[i];
            }
            for (i = 0; i < 3; i++) {
                s.vrtx[1][i] = s2[i];
            }
            for (i = 0; i < 3; i++) {
                s.vrtx[2][i] = s->vrtx[3][i];
            }
        }
        // Call S2D
        S2D(s, v);
        break;
    case 1:
        // Two triangles face the origins:
        //    The only positive hff3 is for triangle 1,i,j, therefore k must be in
        //    the solution as it supports the the point of minimum norm.

        // 1,i,j, are the indices of the points on the triangle and remove k from
        // simplex
        s.nvrtx = 3;
        if (testPlaneTwo) {
            k = 2; // s2
            i = 1;
            j = 0;
        }
        else if (testPlaneThree) {
            k = 1; // s3
            i = 0;
            j = 2;
        }
        else {
            k = 0; // s4
            i = 2;
            j = 1;
        }

        getvrtx(si, i);
        getvrtx(sj, j);
        getvrtx(sk, k);

        if (dotTotal == 1) {
            if (hff1_tests[k]) {
                if (!hff2(s1, sk, si)) {
                    select_1ik();
                    projectOnPlane(s1, si, sk, v);
                }
                else if (!hff2(s1, sk, sj)) {
                    select_1jk();
                    projectOnPlane(s1, sj, sk, v);
                }
                else {
                    select_1k(); // select region 1i
                    projectOnLine(s1, sk, v);
                }
            }
            else if (hff1_tests[i]) {
                if (!hff2(s1, si, sk)) {
                    select_1ik();
                    projectOnPlane(s1, si, sk, v);
                }
                else {
                    select_1i(); // select region 1i
                    projectOnLine(s1, si, v);
                }
            }
            else {
                if (!hff2(s1, sj, sk)) {
                    select_1jk();
                    projectOnPlane(s1, sj, sk, v);
                }
                else {
                    select_1j(); // select region 1i
                    projectOnLine(s1, sj, v);
                }
            }
        }
        else if (dotTotal == 2) {
            // Two edges have positive hff1, meaning that for two edges the origin's
            // project fall on the segement.
            //  Certainly the edge 1,k supports the the point of minimum norm, and so
            //  hff1_1k is positive

            if (hff1_tests[i]) {
                if (!hff2(s1, sk, si)) {
                    if (!hff2(s1, si, sk)) {
                        select_1ik(); // select region 1ik
                        projectOnPlane(s1, si, sk, v);
                    }
                    else {
                        select_1k(); // select region 1k
                        projectOnLine(s1, sk, v);
                    }
                }
                else {
                    if (!hff2(s1, sk, sj)) {
                        select_1jk(); // select region 1jk
                        projectOnPlane(s1, sj, sk, v);
                    }
                    else {
                        select_1k(); // select region 1k
                        projectOnLine(s1, sk, v);
                    }
                }
            }
            else if (hff1_tests[j]) { //  there is no other choice
                if (!hff2(s1, sk, sj)) {
                    if (!hff2(s1, sj, sk)) {
                        select_1jk(); // select region 1jk
                        projectOnPlane(s1, sj, sk, v);
                    }
                    else {
                        select_1j(); // select region 1j
                        projectOnLine(s1, sj, v);
                    }
                }
                else {
                    if (!hff2(s1, sk, si)) {
                        select_1ik(); // select region 1ik
                        projectOnPlane(s1, si, sk, v);
                    }
                    else {
                        select_1k(); // select region 1k
                        projectOnLine(s1, sk, v);
                    }
                }
            }
            else {
                throw(9); // ERROR;
            }

        }
        else if (dotTotal == 3) {
            // MM : ALL THIS HYPHOTESIS IS FALSE
            // sk is s.t. hff3 for sk < 0. So, sk must support the origin because
            // there are 2 triangles facing the origin.

            int hff2_ik = hff2(s1, si, sk);
            int hff2_jk = hff2(s1, sj, sk);
            int hff2_ki = hff2(s1, sk, si);
            int hff2_kj = hff2(s1, sk, sj);

            if (hff2_ki == 0 && hff2_kj == 0) {
                printf("\n\n UNEXPECTED VALUES!!! \n\n");
            }
            if (hff2_ki == 1 && hff2_kj == 1) {
                select_1k();
                projectOnLine(s1, sk, v);
            }
            else if (hff2_ki) {
                // discard i
                if (hff2_jk) {
                    // discard k
                    select_1j();
                    projectOnLine(s1, sj, v);
                }
                else {
                    select_1jk();
                    projectOnPlane(s1, sk, sj, v);
                }
            }
            else {
                // discard j
                if (hff2_ik) {
                    // discard k
                    select_1i();
                    projectOnLine(s1, si, v);
                }
                else {
                    select_1ik();
                    projectOnPlane(s1, sk, si, v);
                }
            }
        }
        break;

    case 0:
        // The origin is outside all 3 triangles
        if (dotTotal == 1) {
            // Here si is set such that hff(s1,si) > 0
            if (testLineThree) {
                k = 2;
                i = 1; // s3
                j = 0;
            }
            else if (testLineFour) {
                k = 1; // s3
                i = 0;
                j = 2;
            }
            else {
                k = 0;
                i = 2; // s2
                j = 1;
            }
            getvrtx(si, i);
            getvrtx(sj, j);
            getvrtx(sk, k);

            if (!hff2(s1, si, sj)) {
                select_1ij();
                projectOnPlane(s1, si, sj, v);
            }
            else if (!hff2(s1, si, sk)) {
                select_1ik();
                projectOnPlane(s1, si, sk, v);
            }
            else {
                select_1i();
                projectOnLine(s1, si, v);
            }
        }
        else if (dotTotal == 2) {
            // Here si is set such that hff(s1,si) < 0
            s->nvrtx = 3;
            if (!testLineThree) {
                k = 2;
                i = 1; // s3
                j = 0;
            }
            else if (!testLineFour) {
                k = 1;
                i = 0; // s4
                j = 2;
            }
            else {
                k = 0;
                i = 2; // s2
                j = 1;
            }
            getvrtx(si, i);
            getvrtx(sj, j);
            getvrtx(sk, k);

            if (!hff2(s1, sj, sk)) {
                if (!hff2(s1, sk, sj)) {
                    select_1jk(); // select region 1jk
                    projectOnPlane(s1, sj, sk, v);
                }
                else if (!hff2(s1, sk, si)) {
                    select_1ik();
                    projectOnPlane(s1, sk, si, v);
                }
                else {
                    select_1k();
                    projectOnLine(s1, sk, v);
                }
            }
            else if (!hff2(s1, sj, si)) {
                select_1ij();
                projectOnPlane(s1, si, sj, v);
            }
            else {
                select_1j();
                projectOnLine(s1, sj, v);
            }
        }
        break;
    default:
        printf("\nERROR:\tunhandled");
    }
}



/// FIXXED ============================================================================
// ====================================================================================
// ====================================================================================


inline static void projectOnLine(const VectorND& p, const VectorND& q, VectorND& v) {
    VectorND pq;
    pq[0] = p[0] - q[0];
    pq[1] = p[1] - q[1];
    pq[2] = p[2] - q[2];

    const REAL tmp = p.dot(pq) / pq.dot(pq);

    for (int i = 0; i < VECTOR_SPACE_NUMBER_OF_DIMENSIONS; i++) {
        v[i] = p[i] - pq[i] * tmp;
    }
}

inline static void projectOnPlane(const VectorND& p, const VectorND& q, const VectorND& r, VectorND& v) {
    VectorND n, pq, pr;

    for (int i = 0; i < VECTOR_SPACE_NUMBER_OF_DIMENSIONS; i++) {
        pq[i] = p[i] - q[i];
    }
    for (int i = 0; i < VECTOR_SPACE_NUMBER_OF_DIMENSIONS; i++) {
        pr[i] = p[i] - r[i];
    }

    n = pq.cross(pr);
    const REAL tmp = n.dot(p) / n.dot(n);

    for (int i = 0; i < 3; i++) {
        v[i] = n[i] * tmp;
    }
}

void S1DR1(gkSimplex& s, VectorND& v) {
    v = s.vrtx[1];                                                                  
    s.nvrtx = 1;
    s.vrtx[0] = s.vrtx[1];
}

inline static void S1D(gkSimplex& s, VectorND& v) {
    const VectorND& s1p = s.vrtx[1];
    const VectorND& s2p = s.vrtx[0];

    if (hff1(s1p, s2p)) {
        projectOnLine(s1p, s2p, v); // Update v, no need to update s
        return;                     // Return V{1,2}
    }
    else {
        // Update v and s
        S1DR1(s, v); // S1Dregion1();
        return;       // Return V{1}
    }
}

void S2DR1(gkSimplex& s, VectorND& v) {
    v = s.vrtx[2];
    s.nvrtx = 1;
    s.vrtx[0][0] = s.vrtx[2][0];
}

void S2DR12(gkSimplex& s, VectorND& v) {
    s.nvrtx = 2;
    s.vrtx[0] = s.vrtx[2];
}

void S2DR13(gkSimplex& s, VectorND& v) {
    s.nvrtx = 2;
    s.vrtx[1] = s.vrtx[2];
}

void S3DR1(gkSimplex& s, VectorND& v, VectorND& s1) {
    v = s1;
    s.nvrtx = 1;
    //fuck
    s.vrtx[0][0] = s1[0];
    s.vrtx[0][1] = s1[1];
}

inline static void S2D(gkSimplex& s, VectorND& v) {
    const VectorND& s1p = s.vrtx[2];
    const VectorND& s2p = s.vrtx[1];
    const VectorND& s3p = s.vrtx[0];
    const int hff1f_s12 = hff1(s1p, s2p);
    const int hff1f_s13 = hff1(s1p, s3p);

    if (hff1(s1p, s2p)) {
        const int hff2f_23 = !hff2(s1p, s2p, s3p);
        if (hff2f_23) {
            if (hff1f_s13) {
                const int hff2f_32 = !hff2(s1p, s3p, s2p);
                if (hff2f_32) {
                    projectOnPlane(s1p, s2p, s3p, v); // Update s, no need to update c
                    return;                           // Return V{1,2,3}
                }
                else {
                    projectOnLine(s1p, s3p, v); // Update v
                    S2DR13(s,v);              // Update s
                    return;                     // Return V{1,3}
                }
            }
            else {
                projectOnPlane(s1p, s2p, s3p, v); // Update s, no need to update c
                return;                           // Return V{1,2,3}
            }
        }
        else {
            projectOnLine(s1p, s2p, v); // Update v
            S2DR12(s,v);              // Update s
            return;                     // Return V{1,2}
        }
    }
    else if (hff1(s1p, s3p)) {
        const int hff2f_32 = !hff2(s1p, s3p, s2p);
        if (hff2f_32) {
            projectOnPlane(s1p, s2p, s3p, v); // Update s, no need to update v
            return;                           // Return V{1,2,3}
        }
        else {
            projectOnLine(s1p, s3p, v); // Update v
            S2DR13(s,v);              // Update s
            return;                     // Return V{1,3}
        }
    }
    else {
        S2DR1(s,v); // Update s and v
        return;       // Return V{1}
    }
}

inline static bool hff1(const VectorND& p, const VectorND& q) {
    return p.dot(p - q) > 0;
}


inline static bool hff2(const VectorND& p, const VectorND& q, const VectorND& r) {
    VectorND ntmp;
    VectorND n;
    VectorND pq;
    VectorND pr;

    for (int i = 0; i < VECTOR_SPACE_NUMBER_OF_DIMENSIONS; i++) {
        pq[i] = q[i] - p[i];
    }
    for (int i = 0; i < VECTOR_SPACE_NUMBER_OF_DIMENSIONS; i++) {
        pr[i] = r[i] - p[i];
    }
    ntmp = pq.cross(pr);
    n = pq.cross(ntemp);

    return p.dot(n) < 0; // Discard r if true
}


inline static bool hff3(const VectorND& p, const VectorND& q, const VectorND& r) {
    VectorND n;
    VectorND pq;
    VectorND pr;

    for (int i = 0; i < VECTOR_SPACE_NUMBER_OF_DIMENSIONS; i++) {
        pq[i] = q[i] - p[i];
    }
    for (int i = 0; i < VECTOR_SPACE_NUMBER_OF_DIMENSIONS; i++) {
        pr[i] = r[i] - p[i];
    }

    n = pq.cross(pr);
    return p.dot(n) <= 0; // discard s if true
}

