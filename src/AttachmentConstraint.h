#ifndef __Attachment_Constraint_h__
#define __Attachment_Constraint_h__

#include "Constraint.h"

#include <UT/UT_FloatArray.h>

namespace HDK_PBD {

class AttachmentConstraint {
public:
    static bool solve(const UT_Vector3F& pos, const UT_Vector3F& attachLocation,
                const UT_Vector3F& compliance, bool doXpbd,
                UT_Vector3F& correction,
                float stiffness, int nIterations) 
    {
        correction = attachLocation - pos;

        if (doXpbd) {
            correction[0] *= 1. / (1. + compliance[0]);
            correction[1] *= 1. / (1. + compliance[1]);
            correction[2] *= 1. / (1. + compliance[2]);
        }
        else {
            // multiply by k' (based on PBD)
            correction *= 1. - pow(1. - stiffness, (1. / nIterations));
        }
        return true;
    }

    static bool solve(const UT_Vector3F& pos, const UT_Vector3F& attachLocation,
                const UT_FloatArray& compliance, bool doXpbd,
                UT_Vector3F& correction,
                float stiffness, int nIterations) 
    {
        UT_Vector3F complianceVec;
        if (!compliance.isEmpty()) {
            complianceVec = UT_Vector3F{compliance[0], compliance[1], compliance[2]};
        }
        return solve(pos, attachLocation, complianceVec, 
            doXpbd, correction, stiffness, nIterations);
    }

};

} // end namespace HDK_PBD


#endif  // __Attachment_Constraint_h__