#ifndef __Integration_h__
#define __Integration_h__

#include <UT/UT_Vector3.h>
#include "MathUtils.h"

namespace HDK_PBD {
using qm = MathUtils;

inline UT_Vector3F integrateVelocity(const UT_Vector3F& vel, const UT_Vector3F& exForce, float mass, float timestep)
{
    UT_Vector3F acceleration = exForce / mass;
    return vel + acceleration * timestep;
}

inline UT_Vector3F integratePosition(const UT_Vector3F& pos, const UT_Vector3F& vel, float timestep)
{
    return pos + vel * timestep;
}

inline UT_Vector3F integrateAngularVelocity(const UT_Vector3F& angVel, UT_Vector3F& torque, float oriInvMass, float timestep)
{
    return angVel + timestep * oriInvMass * (torque - cross(angVel, (1. / oriInvMass) * angVel));
}

inline UT_Vector4F integrateOrientation(const UT_Vector4F& orient, UT_Vector3F& angVel, float timestep)
{
    UT_Vector4F newOrientation = orient + UT_Vector4F(qm::quatProd(orient, qm::quatEmbed(angVel)) * 0.5 * timestep);
    newOrientation.normalize();
    return newOrientation;
}

inline UT_Vector3F getAngularVelocityUpdate(const UT_Vector4F& orientation, const UT_Vector4F& proposedOrientation,
                                           float timestep) 
{
    UT_Vector4F temp = (2. / timestep) * qm::quatProd(qm::quatConjugate(orientation), proposedOrientation);
    return {temp.x(), temp.y(), temp.z()};
}

inline UT_Vector3F getVelocityUpdate(const UT_Vector3F& pos, const UT_Vector3F& newPos, float timestep)
{
    return (1. / timestep) * (newPos - pos);
}

}

#endif  // __Integration_h__