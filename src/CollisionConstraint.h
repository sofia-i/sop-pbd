#ifndef __Collision_Constraint_h__
#define __Collision_Constraint_h__

#include "Constraint.h"
#include <UT/UT_FloatArray.h>

namespace HDK_PBD {

class CollisionConstraint {
public:
    static bool solve(const UT_Vector3F& hitP, const UT_Vector3F& hitN,
                    const UT_Vector3F& pos, float w, StiffnessMode stiffMode,
                    float stiffness, float compliance, int nIterations,
                    UT_Vector3F& correction)
    {
        correction = UT_Vector3F{0., 0., 0.};
        // Check if constraint is satisfied
        if (dot(pos - hitP, hitN) >= 0) {
            // NOTE: break doesn't work, but I think continue should?
            // (see https://www.sidefx.com/docs/hdk/_g_a___g_b_macros_8h.html#:~:text=%23define%20GA_FOR_ALL_PTOFF)
            return true;
        }

        float s;
        switch (stiffMode) {
            case NONE: {
                s = dot(pos - hitP, hitN) / (w);
                break;
            }
            case STIFFNESS: {
                s = dot(pos - hitP, hitN) / (w);
                // apply stiffness
                s *= 1. - pow(1. - stiffness, (1. / nIterations));
                return false;
            }
            case COMPLIANCE: {
                s = dot(pos - hitP, hitN) / (w + compliance);
                break;
            }
            default: {
                return false;
            }
        }

        // UT_Vector3 correction = -hit_n * dot(hit_n, propp - hit_p) / dot(hit_n, hit_n);
        correction = -s * w * hitN;

        return true;
    }

    static bool solve(const UT_Vector3F& hitP, const UT_Vector3F& hitN,
                    const UT_Vector3F& pos, float w, StiffnessMode stiffMode,
                    float stiffness, const UT_FloatArray& compliance, int nIterations,
                    UT_Vector3F& correction)
    {
        float complianceVal = 0.;
        if (stiffMode == COMPLIANCE) {
            complianceVal = compliance[0];
        }
        return solve(hitP, hitN, pos, w, stiffMode, stiffness, complianceVal, nIterations, correction);
    }
};

}


#endif  // __Collision_Constraint_h__