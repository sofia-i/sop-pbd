#define CATCH_CONFIG_MAIN
#include "../extern/catch.hpp"

#include "../src/StretchShearConstraint.h"
#include "../src/BendTwistConstraint.h"
#include "../Integration.h"

#include "TestHelpers.h"

namespace PBD_TEST {

const float eps = 1.E-6;

const UT_Vector4F QUAT_A = {0.557293, 0.557293, -0.435229, 0.435229};
const UT_Vector4F QUAT_B = {0.94373, -0.127704, 0.144805, 0.268509};
const UT_Vector4F QUAT_C = {0.938315, -0.104257, 0.104257, 0.312772};

const UT_Vector3F UNIT_VEC_A = {0.199007, 0.398015, 0.895533};

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


// TEST_CASE("Update angular velocity", "[ang-vel-update]") {
//     UT_Vector3F angVel;
//     UT_Vector3F torque = {0., 0., 0.};
//     float oriInvMass = 1.;
//     float timestep = 0.01;

//     UT_Vector3F newAngVel = 
    

// }

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

}