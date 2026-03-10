#ifndef __Distance_Constraint_h__
#define __Distance_Constraint_h__

#include "Constraint.h"
#include <UT/UT_FloatArray.h>

namespace HDK_PBD {

class DistanceConstraint {
public:
    constexpr static float eps = 1.E-7;

    static bool solve(const UT_Vector3F& pos1, const UT_Vector3F& pos2,
                    float distance, float w1, float w2,
                    const StiffnessMode& stiffMode,
                    float stiffness, float compliance,
                    int nIterations,
                    UT_Vector3F& correction1, UT_Vector3F& correction2) 
    {
        UT_Vector3F diff = pos1 - pos2;
        float newDist = diff.length();
        diff.normalize();

        float distDiff = newDist - distance;
        if (distDiff < eps) {
            // If distance is within precision of the value, continue.
            correction1 = UT_Vector3F{0., 0., 0.};
            correction2 = UT_Vector3F{0., 0., 0.};
            return true;
        }

        float s;
        switch (stiffMode) {
            case NONE: {
                s = (distDiff / (w1 + w2));
                break;
            }
            case STIFFNESS: {
                s = (distDiff / (w1 + w2));
                // apply stiffness
                s *= 1. - pow(1. - stiffness, (1. / nIterations));
                break;
            }
            case COMPLIANCE: {
                s = distDiff / (w1 + w2 + compliance);
                break;
            }
            default:
                return false;
        }

        correction1 = -diff * w1 * s;
        correction2 = diff * w2 * s;

        return true;
    }

    static bool solve(const UT_Vector3F& pos1, const UT_Vector3F& pos2,
                    float distance, float w1, float w2,
                    const StiffnessMode& stiffMode,
                    float stiffness, const UT_FloatArray& compliance,
                    int nIterations,
                    UT_Vector3F& correction1, UT_Vector3F& correction2) 
    {
        float complianceVal;
        if (stiffMode == HDK_PBD::COMPLIANCE) {
            complianceVal = compliance[0];
        }
        return solve(pos1, pos2, distance, w1, w2, stiffMode, stiffness,
            complianceVal, nIterations, correction1, correction2);
    }

};

}  // end namespace HDK_PBD


#endif  // __Distance_Constraint_h__