

#include "MathUtils.h"
#include "UT/UT_Vector4.h"
// #include "UT/UT_Matrix4.h"

/**
 * NOTE: Quaternions in Houdini are stored x - y - z - w
 * https://www.sidefx.com/docs/houdini/vex/lang.html#data-types:~:text=Quaternions%20in%20VEX%20are%20laid%20out%20in%20x/y/z/w%20order%2C%20not%20w/x/y/z.
 */
#define SCALAR_LAST

using namespace PBD;

UT_Vector4R MathUtils::quatProd(UT_Vector4R p, UT_Vector4R q) 
{
#ifdef SCALAR_LAST
    // Scalar-last version
    return {
        p.w() * q.x() + q.w() * p.x() + (p.y() * q.z() - p.z() * q.y()),
        p.w() * q.y() + q.w() * p.y() + (p.z() * q.x() - p.x() * q.z()),
        p.w() * q.z() + q.w() * p.z() + (p.x() * q.y() - p.y() * q.x()),
        p.w() * q.w() - (p.x() * q.x() + p.y() * q.y() + p.z() * q.z())
    };
#else
    // Scalar-first version
    return {
        p[0] * q[0] - (p[1] * q[1] + p[2] * q[2] + p[3] * q[3]),
        p[0] * q[1] + q[0] * p[1] + (p[2] * q[3] - p[3] * q[2]),
        p[0] * q[2] + q[0] * p[2] + (p[3] * q[1] - p[1] * q[3]),
        p[0] * q[3] + q[0] * p[3] + (p[1] * q[2] - p[2] * q[1])
    }
#endif
}

UT_Vector3R MathUtils::quatImagPart(UT_Vector4R q)
{
#ifdef SCALAR_LAST
     // Scalar-last version
    return {q.x(), q.y(), q.z()};
#else
    // Scalar-first version
    return {q[1], q[2], q[3]};
#endif   
}

UT_Vector4R MathUtils::quatConjugate(UT_Vector4R q) 
{
#ifdef SCALAR_LAST
    // Scalar-last version
    return {-q.x(), -q.y(), -q.z(), q.w()};
#else
    // Scalar-first verison
    return {q[0], -q[1], -q[2], -q[3]};
#endif
}

// UT_Matrix4R MathUtils::quatMatrix(UT_Vector4R q)
// {
// #ifdef SCALAR_LAST
//     return {
//         q.w(), -q.z(), q.y(), q.x(),
//         q.z(), q.w(), -q.x(), q.y(),
//         -q.y(), q.x(), q.w(), q.z(),
//         -q.x(), -q.y(), -q.z(), q.w()
//     };
// #else
//     return {
//         q[0], -q[1], -q[2], -q[3],
//         q[1], q[0], -q[3], q[2],
//         q[2], q[3], q[0], -q[1],
//         q[3], -q[2], q[1], q[0]
//     };
// #endif
// }

// UT_Matrix4R MathUtils::quatMatrixHat(UT_Vector4R q)
// {
// #ifdef SCALAR_LAST
//     return {
//         q.w(), q.z(), -q.y(), q.x(),
//         -q.z(), q.w(), q.x(), q.y(),
//         q.y(), -q.x(), q.w(), q.z(),
//         -q.x(), -q.y(), -q.z(), q.w()
//     };
// #else
//     return {
//         q[0], -q[1], -q[2], -q[3],
//         q[1], q[0], q[3], -q[2],
//         q[2], -q[3], q[0], q[1],
//         q[3], q[2], -q[1], q[0]
//     };
// #endif
// }

UT_Vector4R MathUtils::quatEmbed(UT_Vector3R v)
{
#ifdef SCALAR_LAST
    return {v.x(), v.y(), v.z(), 0};
#else
    return {0., v[0], v[1], v[2]};
#endif
}

// UT_Matrix3R MathUtils::outerProd(UT_Vector3R v1, UT_Vector3R v2)
// {
//     return {
//         v1[0] * v2[0],    v1[0] * v2[1],    v1[0] * v2[2],
//         v1[1] * v2[0],    v1[1] * v2[1],    v1[1] * v2[2],
//         v1[2] * v2[0],    v1[2] * v2[1],    v1[2] * v2[2]
//     };
// }

// UT_Matrix3R MathUtils::skewSym(UT_Vector3R v)
// {
//     return {
//         0, -v[2], v[1],
//         v[2], 0, -v[0],
//         -v[1], v[0], 0
//     };
// }

// UT_Matrix3R MathUtils::rotation(UT_Vector3R q)
// {
//     return 2. * outerProd(q, q) + (q[0] * q[0] - dot(q, q)) * UT_Matrix3R(1.) + 2. * q[0] * skewSym(q);
// }

UT_Vector3R MathUtils::darbouxVector(UT_Vector4R q1, UT_Vector4R q2, float length)
{
    return /* (2. / length) * */ quatImagPart(quatProd(quatConjugate(q1), q2));
}

UT_Vector4R MathUtils::darbouxQuat(UT_Vector4R q1, UT_Vector4R q2, float length)
{
    return /* (2. / length) * */ quatProd(quatConjugate(q1), q2);
}

 UT_Vector4R MathUtils::basisQuat(int idx)
 {
#ifdef SCALAR_LAST
    switch(idx) {
        case (0): {
            return {1, 0, 0, 0};
        }
        case (1): {
            return {0, 1, 0, 0};
        }
        case (2): {
            return {0, 0, 1, 0};
        }
    }
#else
    switch(idx) {
        case (0): {
            return {0, 1, 0, 0};
        }
        case (1): {
            return {0, 0, 1, 0};
        }
        case (2): {
            return {0, 0, 0, 1};
        }
    }
#endif
    return {0, 0, 0, 0};
 }

UT_Vector3R MathUtils::strain(UT_Vector3R x1, UT_Vector3R x2, 
                              UT_Vector4R q, float length) 
{
    return (1. / length) * (x2 - x1) - quatImagPart(quatProd(quatProd(q, basisQuat(2)), quatConjugate(q)));
}

UT_Vector3R MathUtils::deltaDarboux(UT_Vector4R q1, UT_Vector4R q2, 
                                    UT_Vector4R restQ1, UT_Vector4R restQ2, 
                                    float length)
{
    return (2. / length) * quatImagPart(quatProd(quatConjugate(q1), q2) - quatProd(quatConjugate(restQ1), restQ2));
}
