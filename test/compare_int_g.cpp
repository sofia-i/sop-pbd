#include "../extern/catch.hpp"

#include "IntGHelpers.h"
#include "TestConstants.h"

#include "Constraints/StretchShearConstraint.h"
#include "Constraints/BendTwistConstraint.h"
#include "Utility/Integration.h"

using namespace PBD_TEST;

TEST_CASE("Compute stretch-shear projection (stiffness)", "[ss-proj]") {
    // Test variables
    UT_Vector3F pos1 = {0., 0.2, 0.04};
    UT_Vector3F pos2 = {0., 0., 0.};
    UT_Vector4F q = QUAT_A;
    float w1 = 1.;
    float w2 = 1.;
    float wq = 1.;
    float length = 0.2;
    HDK_PBD::StiffnessMode stiffMode = HDK_PBD::STIFFNESS;
    float stiffness = 1.;
    UT_Vector3F compliance;
    int nIterations = 1;

    UT_Vector3F correction1, correction2;
    UT_Vector4F correctionq;

    bool myResult = HDK_PBD::StretchShearConstraint::solve(
        pos1, pos2, q, 
        w1, w2, wq, length, 
        stiffMode, stiffness, compliance, nIterations, 
        correction1, correction2, correctionq);

    UT_Vector3F pbd_correction1, pbd_correction2;
    UT_Vector4F pbd_correctionq;
    bool intG_result = get_intg_ss_correction(
        pos1, pos2, q, w1, w2, wq, length, 
        stiffMode, stiffness, compliance, nIterations,
        pbd_correction1, pbd_correction2, pbd_correctionq
    );

    REQUIRE( myResult == true );
    REQUIRE( intG_result == true );
    
    {
        // CAPTURE(correction1);
        // CAPTURE(pbd_correction1);
        INFO( "correction1 = \t" << correction1 );
        INFO( "pbd_correction1 = \t" << pbd_correction1 );
        REQUIRE( correction1.isEqual(pbd_correction1, PBD_TEST::eps) );
    }
    {
        // CAPTURE(correction2);
        // CAPTURE(pbd_correction2);
        INFO( "correction2 = \t" << correction2 );
        INFO( "pbd_correction2 = \t" << pbd_correction2 );
        REQUIRE( correction2.isEqual(pbd_correction2, PBD_TEST::eps) );
    }    
    {
        // CAPTURE(correctionq);
        // CAPTURE(pbd_correctionq);
        INFO( "correctionq = \t" << correctionq );
        INFO( "pbd_correctionq = \t" << pbd_correctionq );
        REQUIRE( correctionq.isEqual(pbd_correctionq, PBD_TEST::eps) );
    }
}

TEST_CASE("Compute product with conjugate", "[q-prod-conj]") {
    UT_Vector4F e3 = {0., 0., 1., 0.};
    UT_Vector4F q = QUAT_B;

    UT_Vector4F qe3bar = qm::quatProd(q, qm::quatConjugate(e3));

    // IG pbd
    Quaternionr qr_e3 = getQuatr(e3);
    Quaternionr qr_q = getQuatr(q);
    Quaternionr q_e_3_bar(qr_q.z(), -qr_q.y(), qr_q.x(), -qr_q.w());


    {
        INFO( "qe3bar math = \t" << qe3bar );
        INFO( "qe3bar ig = \t" << q_e_3_bar );
        REQUIRE(qe3bar.isEqual(getQuatUT(q_e_3_bar), PBD_TEST::eps));
    }

}

TEST_CASE("Compute bend-twist projection (stiffness)", "[bt-proj]") {
    UT_Vector4F q = QUAT_A;
    UT_Vector4F u = QUAT_B;
    float wq = 1.;
    float wu = 1.;
    float length = 0.2;
    UT_Vector4F restDarboux{UNIT_VEC_A.x(), UNIT_VEC_A.y(), UNIT_VEC_A.z(), 0.};
    HDK_PBD::StiffnessMode stiffMode = HDK_PBD::STIFFNESS;
    float stiffness = 1.;
    int nIterations = 1.;
    UT_Vector3F compliance = {1., 1., 1.};


    UT_Vector4F correctionq, correctionu;
    bool myResult = HDK_PBD::BendTwistConstraint::solve(
        q, u,
        wq, wu, length, 
        restDarboux,
        stiffMode,
        stiffness, nIterations,
        compliance,
        correctionq, correctionu);

    UT_Vector4F pbd_correctionq, pbd_correctionu;
    bool intG_result = get_intg_bt_correction(
        q, u, wq, wu, length, restDarboux,
        stiffMode, stiffness, nIterations, compliance, 
        pbd_correctionq, pbd_correctionu
    );


    REQUIRE( myResult == true );
    REQUIRE( intG_result == true );
    
    {
        INFO( "correctionq = \t" << correctionq );
        INFO( "pbd_correctionq = \t" << pbd_correctionq );
        REQUIRE( correctionq.isEqual(pbd_correctionq, PBD_TEST::eps) );
    }
    {
        INFO( "correctionu = \t" << correctionu );
        INFO( "pbd_correctionu = \t" << pbd_correctionu );
        REQUIRE( correctionu.isEqual(pbd_correctionu, PBD_TEST::eps) );
    }
    
}

TEST_CASE("Integrate angular velocity and orientation", "[ang-ori-integrate]")
{
    UT_Vector3F angVel = VEC_A;
    UT_Vector3F torque = {0., 0., 0.};
    UT_Vector4F orient = QUAT_A;
    float oriInvMass = 1.;
    float timestep = 0.01;

    UT_Vector3F newAngVel = HDK_PBD::integrateAngularVelocity(angVel, torque, oriInvMass, timestep);
    UT_Vector4F newOrientation = HDK_PBD::integrateOrientation(orient, newAngVel, timestep);

    UT_Vector3F pbd_newAngVel;
    UT_Vector4F pbd_newOrientation;
    get_intg_ang_vel_ori_integ(angVel, torque, orient, oriInvMass, timestep,
        pbd_newAngVel, pbd_newOrientation);

    {
        INFO( "angVel = \t" << newAngVel );
        INFO( "pbd_angVel = \t" << pbd_newAngVel );
        REQUIRE( newAngVel.isEqual(pbd_newAngVel, PBD_TEST::eps) );
    }

    {
        INFO(  "ori = \t" << newOrientation );
        INFO( "pbd_ori = \t" << pbd_newOrientation );
        REQUIRE( newOrientation.isEqual(pbd_newOrientation, PBD_TEST::eps) );
    }
}

TEST_CASE("Update angular velocity", "[ang-vel-update]") {
    UT_Vector4F orientation = QUAT_B;
    UT_Vector4F proposedOrientation = QUAT_C;
    float timestep = 0.01;

    UT_Vector3F angVel = HDK_PBD::getAngularVelocityUpdate(orientation, proposedOrientation, timestep);
    UT_Vector3F pbd_angVel = get_intg_ang_vel_update(orientation, proposedOrientation, timestep);

    {
        INFO( "angVel = \t" << angVel );
        INFO( "pbd_angVel = \t" << pbd_angVel );
        REQUIRE( angVel.isEqual(pbd_angVel, PBD_TEST::eps) );
    }
}