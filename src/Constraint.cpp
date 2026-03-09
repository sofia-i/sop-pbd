#include "Constraint.h"

using namespace HDK_PBD;

#include <UT/UT_FloatArray.h>
#include <UT/UT_Vector3.h>

StiffnessMode 
HDK_PBD::getStiffnessMode(bool doIgnoreStiffness, bool doXpbd) 
{
    if (doIgnoreStiffness)
        return NONE;
    else if (doXpbd)
        return COMPLIANCE;
    else
        return STIFFNESS;
}

void 
HDK_PBD::complianceApplyTimestep(UT_FloatArray& compliance, float timestep) 
{
    double inv_time_squared = 1. / (timestep * timestep);
    for (int i = 0; i < compliance.size(); ++i)
        compliance[i] *= inv_time_squared;
}

void 
HDK_PBD::complianceApplyTimestep(UT_Vector3F& compliance, float timestep)
{
    double inv_time_squared = 1. / (timestep * timestep);
    for (int i = 0; i < 3; ++i)
        compliance[i] *= inv_time_squared;
}

float 
HDK_PBD::complianceApplyTimestep(float compliance, float timestep)
{
    double inv_time_squared = 1. / (timestep * timestep);
    return compliance * inv_time_squared;
}