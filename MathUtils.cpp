

#include "MathUtils.h"
#include "UT/UT_Vector4.h"
#include "UT/UT_Matrix4.h"

using namespace PBD;

UT_Vector4R MathUtils::quatProd(UT_Vector4R p, UT_Vector4R q) 
{
    UT_Vector4R product;

    // p0q0 - P^TQ
    product[0] = p[0] * q[0] - (p[1] * q[1] + p[2] * q[2] + p[3] * q[3]);
    
    // p0Q + q0P + P x Q
    product[1] = p[0] * q[1] + q[0] * p[1] + (p[2] * q[3] - p[3] * q[2]);
    product[2] = p[0] * q[2] + q[0] * p[2] + (p[3] * q[1] - p[1] * q[3]);
    product[3] = p[0] * q[3] + q[0] * p[3] + (p[1] * q[2] - p[2] * q[1]);

    return product;
}

UT_Vector3R MathUtils::quatImagPart(UT_Vector4R q)
{
    return {q[1], q[2], q[3]};
}

UT_Vector4R MathUtils::quatConjugate(UT_Vector4R q) 
{
    return {q[0], -q[1], -q[2], -q[3]};
}

UT_Matrix4R MathUtils::quatMatrix(UT_Vector4R q)
{
    return {
        q[0], -q[1], -q[2], -q[3],
        q[1], q[1], -q[3], q[2],
        q[2], q[3], q[2], -q[1],
        q[3], -q[2], q[1], q[3]
    };
}

UT_Matrix4R MathUtils::quatMatrixHat(UT_Vector4R q)
{
    return {
        q[0], -q[1], -q[2], -q[3],
        q[1], q[1], q[3], -q[2],
        q[2], -q[3], q[2], q[1],
        q[3], q[2], -q[1], q[3]
    };
}

UT_Vector4R MathUtils::quatEmbed(UT_Vector3R v)
{
    return {0., v[0], v[1], v[2]};
}

UT_Matrix3R MathUtils::outerProd(UT_Vector3R v1, UT_Vector3R v2)
{
    return {
        v1[0] * v2[0],    v1[0] * v2[1],    v1[0] * v2[2],
        v1[1] * v2[0],    v1[1] * v2[1],    v1[1] * v2[2],
        v1[2] * v2[0],    v1[2] * v2[1],    v1[2] * v2[2]
    };
}

UT_Matrix3R MathUtils::skewSym(UT_Vector3R v)
{
    return {
        0, -v[2], v[1],
        v[2], 0, -v[0],
        -v[1], v[0], 0
    };
}

UT_Matrix3R MathUtils::rotation(UT_Vector3R q)
{
    return 2. * outerProd(q, q) + (q[0] * q[0] - dot(q, q)) * UT_Matrix3R(1.) + 2. * q[0] * skewSym(q);
}

UT_Vector3R MathUtils::darbouxVector(UT_Vector4R q1, UT_Vector4R q2, float length)
{
    return (2. / length) * quatImagPart(quatProd(quatConjugate(q1), q2));
}

 UT_Vector4R MathUtils::basisQuat(int idx)
 {
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
