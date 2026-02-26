#include "Constraint.h"

using namespace HDK_PBD;

StiffnessMode 
HDK_PBD::getStiffnessMode(bool doIgnoreStiffness, bool doXpbd) {
    if (doIgnoreStiffness)
        return NONE;
    else if (doXpbd)
        return COMPLIANCE;
    else
        return STIFFNESS;
}