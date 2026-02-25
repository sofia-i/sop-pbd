#ifndef __Bend_Twist_Constraint_h__
#define __Bend_Twist_Constraint_h__

#include "Constraint.h"
#include "MathUtils.h"

#include <UT/UT_Matrix3.h>
#include <UT/UT_FloatArray.h>

using qm = HDK_PBD::MathUtils;

namespace HDK_PBD {

class BendTwistConstraint {
public:
    static bool solve(const UT_Vector4F& q, const UT_Vector4F& u,
                    float wq, float wu, float length, 
                    const UT_Vector4F& restDarboux,
                    const StiffnessMode& stiffMode,
                    float stiffness, int nIterations,
                    const UT_Vector3F& compliance,
                    UT_Vector4F& correctionq, UT_Vector4F& correctionu)
    {
        UT_Vector4 darboux = qm::darbouxQuat(q, u, length);

        UT_Vector4 darbouxDiff = darboux - restDarboux;
        UT_Vector4 darbouxSum = darboux + restDarboux;

        UT_Vector4 darbouxTerm;

        if (darbouxDiff.length() > darbouxSum.length()) {
            darbouxTerm = darbouxSum;
        }
        else {
            darbouxTerm = darbouxDiff;
        }

        // discrete darboux vector does not have vanishing scalar part, so get rid of it
        // ref: PositionBasedElasticRods.cpp::72, https://github.com/InteractiveComputerGraphics/PositionBasedDynamics/blob/master/PositionBasedDynamics/PositionBasedElasticRods.cpp
        darbouxTerm.w() = 0;

        switch (stiffMode) {
            case NONE: {
                float s = 1./ (wq + wu);
                correctionq = s * qm::quatProd(u, darbouxTerm);
                correctionu = s * qm::quatProd(q, darbouxTerm);
                break;
            } 
            case STIFFNESS: {
                float s = 1./ (wq + wu);
                // apply stiffness
                s *= 1. - pow(1. - stiffness, (1. / nIterations));
                // UT_Vector4 darbouxTerm_quat = qm::quatEmbed(darbouxTerm);
                correctionq = wq * s * qm::quatProd(u, darbouxTerm);
                correctionu = -wu * s * qm::quatProd(q, darbouxTerm);
                break;
            }
            case COMPLIANCE: {
                UT_Matrix3F s = {
                    1 / (wq + wu + compliance[0]), 0., 0.,
                    0., 1 / (wq + wu + compliance[1]), 0.,
                    0., 0., 1 / (wq + wu + compliance[2])
                };
                UT_Vector3 sDarbouxTerm = qm::quatImagPart(darbouxTerm) * s;
                UT_Vector4 sDarbouxTerm_quat = qm::quatEmbed(sDarbouxTerm);
                correctionq = wq * qm::quatProd(u, sDarbouxTerm_quat);
                correctionu = -wu * qm::quatProd(q, sDarbouxTerm_quat);
                break;
            }
            default:
                return false;
        }
        return true;
    }

    static bool solve(const UT_Vector4F& q, const UT_Vector4F& u,
                    float wq, float wu, float length, 
                    const UT_Vector4F& restDarboux,
                    const StiffnessMode& stiffMode,
                    float stiffness, int nIterations,
                    const UT_FloatArray& compliance,
                    UT_Vector4F& correctionq, UT_Vector4F& correctionu)
    {
        UT_Vector3F complianceVec;
        if (!compliance.isEmpty()) {
            complianceVec = UT_Vector3F{compliance[0], compliance[1], compliance[2]};
        }
        return solve(q, u, wq, wu, length, restDarboux, stiffMode, stiffness, nIterations,
            complianceVec, correctionq, correctionu);
    }

};

}


#endif  // __Bend_Twist_Constraint_h__