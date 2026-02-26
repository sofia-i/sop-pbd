#ifndef __Constraint_h__
#define __Constraint_h__

#include <UT/UT_Vector3.h>

namespace HDK_PBD {

enum StiffnessMode {
    NONE,
    STIFFNESS,
    COMPLIANCE
};

StiffnessMode getStiffnessMode(bool doIgnoreStiffness, bool doXpbd);


class Constraint {


};
} // end namespace HDK_PBD


#endif  // __Constraint_h__