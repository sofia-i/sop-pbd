#ifndef __Stretch_Shear_Constraint_h__
#define __Stretch_Shear_Constraint_h__

#include "Constraint.h"
#include "MathUtils.h"

#include <UT/UT_Matrix3.h>
#include <UT/UT_FloatArray.h>

using qm = HDK_PBD::MathUtils;

namespace HDK_PBD {

class StretchShearConstraint {
public:
    static bool solve(const UT_Vector3F& pos1, const UT_Vector3F& pos2,
                      const UT_Vector4F& q, 
                      float w1, float w2, float wq, float length,
                      StiffnessMode stiffMode,
                      float stiffness, const UT_Vector3F& compliance, int nIterations,
                      UT_Vector3F& correction1, UT_Vector3F& correction2,
                      UT_Vector4F& correctionq)
    {
        UT_Vector4F e3 = qm::quatEmbed({0., 0., 1.});
        UT_Vector3F d3 = qm::quatImagPart(qm::quatProd(qm::quatProd(q, e3), qm::quatConjugate(q)));

        UT_Vector3F c = (1. / length) * (pos2 - pos1) - d3;

        switch (stiffMode) {
            case NONE: {
                float s = (length) / (w1 + w2 + 4. * wq * length * length);
                correction1 = w1 * s * c;
                correction2 = -w2 * s * c;
                correctionq = - 2. * wq * length * s * qm::quatProd(qm::quatEmbed(c), qm::quatProd(q, qm::quatConjugate(e3)));
                break;
            }
            case STIFFNESS: {
                float s = (length) / (w1 + w2 + 4. * wq * length * length);
                // apply stiffness
                s *= 1. - pow(1. - stiffness, (1. / nIterations));
                correction1 = w1 * s * c;
                correction2 = -w2 * s * c;
                correctionq = 2. * wq * length * s * qm::quatProd(qm::quatEmbed(c), qm::quatProd(q, qm::quatConjugate(e3)));
                break;
            }
            case COMPLIANCE: {
                float lengthSq = length * length;
                UT_Matrix3F s = {
                    lengthSq / (w1 + w2 + 4 * wq * lengthSq + lengthSq * compliance[0]), 0., 0.,
                    0., lengthSq / (w1 + w2 + 4 * wq * lengthSq + lengthSq * compliance[1]), 0.,
                    0., 0., lengthSq / (w1 + w2 + 4 * wq * lengthSq + lengthSq * compliance[2])
                };
                correction1 = (w1 / length) * c * s;
                correction2 = (-w2 / length) * c * s;
                // correctionq = (2. * wq) * qm::quatImagPart(qm::quatProd(qm::quatEmbed(c), qm::quatProd(q, qm::quatConjugate(e3)))) * s;
                correctionq = (-2. * wq) * qm::quatProd(qm::quatEmbed(c * s), qm::quatProd(q, qm::quatConjugate(e3)));
                break;
            }
            default:
                return false;
        }

        return true;
    }

    static bool solve(const UT_Vector3F& pos1, const UT_Vector3F& pos2,
                      const UT_Vector4F& q, 
                      float w1, float w2, float wq, float length,
                      StiffnessMode stiffMode,
                      float stiffness, const UT_FloatArray& compliance, int nIterations,
                      UT_Vector3F& correction1, UT_Vector3F& correction2,
                      UT_Vector4F& correctionq)
    {
        UT_Vector3F complianceVec;
        if (stiffMode == COMPLIANCE) {
            complianceVec = UT_Vector3F{compliance[0], compliance[1], compliance[2]};
        }
        return solve(pos1, pos2, q, w1, w2, wq, length, stiffMode, stiffness,
            complianceVec, nIterations, correction1, correction2, correctionq);
    }
};
}

#endif  // __Stretch_Shear_Constraint_h__