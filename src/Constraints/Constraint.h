#ifndef __Constraint_h__
#define __Constraint_h__

#include <UT/UT_VectorTypes.h>

namespace HDK_PBD {

enum StiffnessMode {
    NONE,
    STIFFNESS,
    COMPLIANCE
};

StiffnessMode getStiffnessMode(bool doIgnoreStiffness, bool doXpbd);

void complianceApplyTimestep(UT_FloatArray& compliance, float timestep);
void complianceApplyTimestep(UT_Vector3F& compliance, float timestep);
float complianceApplyTimestep(float compliance, float timestep);

class Constraint {


};
} // end namespace HDK_PBD


#endif  // __Constraint_h__