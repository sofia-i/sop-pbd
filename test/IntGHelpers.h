#ifndef __Int_G_Helpers_h__
#define __Int_G_Helpers_h__

#include <UT/UT_Vector3.h>
#include <UT/UT_Vector4.h>

#include "../extern/intg-pbd/Common/Common.h"
#include "../extern/intg-pbd/PositionBasedElasticRods.h"
#include "../extern/intg-pbd/TimeIntegration.h"

#include "Constraints/Constraint.h"

namespace PBD_TEST {

inline UT_Vector3F getVecUT(const Vector3r& v) {
    return UT_Vector3F{v.x(), v.y(), v.z()};
}

inline Vector3r getVecr(const UT_Vector3F& v) {
    return Vector3r{v.x(), v.y(), v.z()};
}

inline UT_Vector4F getQuatUT(const Quaternionr& q) {
    return UT_Vector4F{q.x(), q.y(), q.z(), q.w()};
}

inline Quaternionr getQuatr(const UT_Vector4F& q) {
    return Quaternionr{q.w(), q.x(), q.y(), q.z()};
}

bool get_intg_ss_correction(const UT_Vector3F& pos1, const UT_Vector3F& pos2,
                      const UT_Vector4F& q, 
                      float w1, float w2, float wq, float length,
                      HDK_PBD::StiffnessMode stiffMode,
                      float stiffness, const UT_Vector3F& compliance, int nIterations,
                      UT_Vector3F& correction1, UT_Vector3F& correction2,
                      UT_Vector4F& correctionq)
{
    Vector3r p0 = getVecr(pos1); 
    Real invMass0 = w1;
    Vector3r p1 = getVecr(pos2);
    Real invMass1 = w2;
    Quaternionr q0 = getQuatr(q);
    Real invMassq0 = wq;
    Vector3r stretchingAndShearingKs;
    Real restLength = length;
    switch (stiffMode) {
        case HDK_PBD::COMPLIANCE: {
            stretchingAndShearingKs = {compliance[0], compliance[1], compliance[2]};
            break;
        }
        case HDK_PBD::STIFFNESS: {
            stretchingAndShearingKs = {stiffness, stiffness, stiffness};
            break;
        }
        case HDK_PBD::NONE:
        default: {
            stretchingAndShearingKs = {1., 1., 1.};
        }
    }

    Vector3r corr0, corr1;
    Quaternionr corrq0;

    bool result = solve_StretchShearConstraint(
        p0, invMass0, p1, invMass1,
        q0, invMassq0,
        stretchingAndShearingKs,
        restLength,
        corr0, corr1, corrq0
    );
    
    correction1 = getVecUT(corr0);  // UT_Vector3F{corr0.x(), corr0.y(), corr0.z()};
    correction2 = getVecUT(corr1);  // UT_Vector3F{corr1.x(), corr1.y(), corr1.z()};
    correctionq = getQuatUT(corrq0);  // UT_Vector4F{corrq0.x(), corrq0.y(), corrq0.z(), corrq0.w()};

    return result;
}

bool get_intg_bt_correction(const UT_Vector4F& q, const UT_Vector4F& u,
                    float wq, float wu, float length, 
                    const UT_Vector4F& restDarboux,
                    const HDK_PBD::StiffnessMode& stiffMode,
                    float stiffness, int nIterations,
                    const UT_Vector3F& compliance,
                    UT_Vector4F& correctionq, UT_Vector4F& correctionu)
{
    Quaternionr q0 = getQuatr(q);
    Quaternionr q1 = getQuatr(u);
    Real invMassq0 = wq;
    Real invMassq1 = wu;
    Quaternionr restDarbouxVector = getQuatr(restDarboux);
    Vector3r bendingAndTwistingKs;
    switch (stiffMode) {
        case HDK_PBD::COMPLIANCE: {
            bendingAndTwistingKs = {compliance[0], compliance[1], compliance[2]};
            break;
        }
        case HDK_PBD::STIFFNESS: {
            bendingAndTwistingKs = {stiffness, stiffness, stiffness};
            break;
        }
        case HDK_PBD::NONE:
        default: {
            bendingAndTwistingKs = {1., 1., 1.};
        };
    }

    Quaternionr corrq0, corrq1;

    bool result = solve_BendTwistConstraint(
        q0, invMassq0,
        q1, invMassq1,
        bendingAndTwistingKs,
        restDarbouxVector,
        corrq0, corrq1);

    correctionq = getQuatUT(corrq0);
    correctionu = getQuatUT(corrq1);

    return result;
}

void get_intg_ang_vel_ori_integ(const UT_Vector3F& angVel, const UT_Vector3F& torque, const UT_Vector4F& orientation,
    float oriInvMass, float timestep, UT_Vector3F& newAngVel, UT_Vector4F& newOrientation)
{
    // void TimeIntegration::semiImplicitEulerRotation(
	// const Real h,
	// const Real mass,
	// const Matrix3r& invertiaW,
	// const Matrix3r &invInertiaW,
	// Quaternionr &rotation,
	// Vector3r &angularVelocity,	
	// const Vector3r &torque)
    Real h = timestep;
    Real mass = 1.;
    Real oriMass = 1. / oriInvMass;
    Matrix3r inertiaW {
        {oriMass, 0., 0.},
        {0., oriMass, 0.},
        {0., 0., oriMass}
    };
    Matrix3r invInertiaW {
        {oriInvMass, 0., 0.},
        {0., oriInvMass, 0.},
        {0., 0., oriInvMass}
    };
    Quaternionr rotation = getQuatr(orientation);
    Vector3r angVelr = getVecr(angVel);
    Vector3r torquer = getVecr(torque);

    PBD::TimeIntegration::semiImplicitEulerRotation(
        h, mass, inertiaW, invInertiaW,
        rotation, angVelr, torquer
    );

    newAngVel = getVecUT(angVelr);
    newOrientation = getQuatUT(rotation);
}


UT_Vector3F get_intg_ang_vel_update(const UT_Vector4F& orientation, const UT_Vector4F& proposedOrientation, float timestep)
{
    Real h = timestep;
    Real mass = 1.;
    Quaternionr rotation = getQuatr(proposedOrientation);
    Quaternionr oldRotation = getQuatr(orientation);

    Vector3r angularVelocity;
    PBD::TimeIntegration::angularVelocityUpdateFirstOrder(h, mass, rotation, oldRotation, angularVelocity);
    
    return getVecUT(angularVelocity);
}

}  // end namespace PBD_TEST

#endif  // __Int_G_Helpers_h__
