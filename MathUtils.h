
#ifndef __PBD_MathUtils_h__
#define __PBD_MathUtils_h__

#include <UT/UT_VectorTypes.h>

namespace PBD {
class MathUtils {
public:
    static UT_Vector3R quatImagPart(UT_Vector4R q);
    static UT_Vector4R quatConjugate(UT_Vector4R q);
    // static UT_Matrix4R quatMatrix(UT_Vector4R q);
    // static UT_Matrix4R quatMatrixHat(UT_Vector4R q);

    static UT_Vector4R quatEmbed(UT_Vector3R v);

    static UT_Vector4R quatProd(UT_Vector4R p, UT_Vector4R q);

    // static UT_Matrix3R outerProd(UT_Vector3R v1, UT_Vector3R v2);
    // static UT_Matrix3R skewSym(UT_Vector3R v);

    // static UT_Matrix3R rotation(UT_Vector3R q);
    
    static UT_Vector4R basisQuat(int idx);

    static UT_Vector3R darbouxVector(UT_Vector4R q1, UT_Vector4R q2, float length);
    static UT_Vector4R darbouxQuat(UT_Vector4R q1, UT_Vector4R q2, float length);
    static UT_Vector3R deltaDarboux(UT_Vector4R q1, UT_Vector4R q2, 
                                    UT_Vector4R restQ1, UT_Vector4R restQ2, 
                                    float length);
    static UT_Vector3R strain(UT_Vector3R x1, UT_Vector3R x2, UT_Vector4R q, float length);
};
} // end namespace PBD

#endif