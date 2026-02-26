#ifndef __Attachment_Constraint_h__
#define __Attachment_Constraint_h__

#include "Constraint.h"

#include <UT/UT_FloatArray.h>

namespace HDK_PBD {

class AttachmentConstraint {
public:
    static bool solve(const UT_Vector3F& pos, const UT_Vector3F& attachLocation,
                const StiffnessMode& stiffMode, const UT_Vector3F& compliance,
                float stiffness, int nIterations,
                UT_Vector3F& correction) 
    {
        correction = attachLocation - pos;

        switch (stiffMode) {
            case STIFFNESS: {
                // multiply by k' (based on PBD)
                correction *= 1. - pow(1. - stiffness, (1. / nIterations));
                break;
            }
            case COMPLIANCE: {
                correction[0] *= 1. / (1. + compliance[0]);
                correction[1] *= 1. / (1. + compliance[1]);
                correction[2] *= 1. / (1. + compliance[2]);
                break;
            }
            case NONE:
            default:
                break;
        }

        return true;
    }

    static bool solve(const UT_Vector3F& pos, const UT_Vector3F& attachLocation,
                const StiffnessMode& stiffMode, const UT_FloatArray& compliance,
                float stiffness, int nIterations,
                UT_Vector3F& correction) 
    {
        UT_Vector3F complianceVec;
        if (!compliance.isEmpty()) {
            complianceVec = UT_Vector3F{compliance[0], compliance[1], compliance[2]};
        }
        return solve(pos, attachLocation, stiffMode, complianceVec, 
            stiffness, nIterations, correction);
    }

};

} // end namespace HDK_PBD


#endif  // __Attachment_Constraint_h__