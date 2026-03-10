#include "../extern/catch.hpp"

#include <vector>
#include <iostream>

#include "TestConstants.h"
#include "Constraints/Constraint.h"
#include "Constraints/AttachmentConstraint.h"
#include "Constraints/DistanceConstraint.h"

using namespace PBD_TEST;

TEST_CASE("Attachment XPBD", "[attach-xpbd]")
{
    UT_Vector3F pos = {0., 0., 0.};
    UT_Vector3F attachLocation = {0.05, 0., 0.};

    float stiffness = 1.;
    UT_Vector3F compliance = {1., 1., 1.};
    int nIterations = 1.;

    UT_Vector3F pbdCorrection;
    HDK_PBD::AttachmentConstraint::solve(pos, attachLocation, 
        HDK_PBD::STIFFNESS, compliance, stiffness, nIterations, pbdCorrection);


    float framestep = 1.;
    std::vector<float> substeps = {1, 10, 20, 100, 200};
    std::vector<UT_Vector3F> xpbdCorrections;

    for (size_t i = 0; i < substeps.size(); ++i) {
        float ss = substeps[i];
        float timestep = framestep / ss;

        UT_Vector3F timedCompliance = compliance;
        HDK_PBD::complianceApplyTimestep(timedCompliance, timestep);

        UT_Vector3F testPos = pos;

        UT_Vector3F xpbdCorrection;
        UT_Vector3F totalCorrection = {0., 0., 0.};
        for (int j = 0; j < int(ss); ++j) {
            HDK_PBD::AttachmentConstraint::solve(testPos, attachLocation, 
                HDK_PBD::COMPLIANCE, timedCompliance, stiffness, nIterations, xpbdCorrection);

            testPos += xpbdCorrection;
            totalCorrection += xpbdCorrection;
        }

        xpbdCorrections.push_back(totalCorrection);
    }

    std::cout << "ATTACHMENT " << std::endl;
    for (size_t i = 0; i < xpbdCorrections.size(); ++i) {
        float ss = substeps[i];
        float timestep = framestep / ss;
        std::cout << "Correction for " << ss << " substeps (dt=" << timestep << ")" << std::endl << "\t" << xpbdCorrections[i] << std::endl;
    }
    std::cout << std::endl;


}

TEST_CASE("Bend-Twist XPBD", "[bt-xpbd]")
{

}

TEST_CASE("Collision XPBD", "[coll-xpbd]")
{
    // UT_Vector3F hitP;
    // UT_Vector3F hitN;
    // UT_Vector3F pos;

    // float stiffness = 1.;
    // float compliance = 1.;

    // int nIterations = 1;

}

TEST_CASE("Distance XPBD", "[dist-xpbd]")
{
    UT_Vector3F pos1 = {0., 0., 0.};
    UT_Vector3F pos2 = {0.3, 0., 0.};
    float distance = 0.2;
    float w1 = 0;
    float w2 = 1;
    float stiffness = 1.;
    float compliance = 1.;
    int nIterations = 1.;

    // float framestep = 1.;
    float framestep = 1;
    std::vector<float> substeps = {1, 2, 10, 15, 20, 100, 200};
    std::vector<UT_Vector3F> xpbdCorrections;

   
    for (size_t i = 0; i < substeps.size(); ++i) {
        UT_Vector3F testPos = pos2;
        UT_Vector3F correction1, correction2;
        UT_Vector3F totalCorrection = {0., 0., 0.};

        float ss = substeps[i];
        std::cout << ss << " Substeps" << std::endl;

        float timedCompliance = HDK_PBD::complianceApplyTimestep(compliance, framestep);

        for (int j = 0; j < int(ss); ++j) {
            HDK_PBD::DistanceConstraint::solve(pos1, testPos, distance, w1, w2, 
                HDK_PBD::COMPLIANCE, stiffness, timedCompliance, nIterations, correction1, correction2);

            testPos += correction2;
            totalCorrection += correction2;

            std::cout << "\t" << "Iter " << j << ": " << correction2 << std::endl;
        }

        xpbdCorrections.push_back(totalCorrection);
       std::cout << "\t" << "Total: " << totalCorrection << std::endl << std::endl;
    }

    std::cout << "DISTANCE" << std::endl;
    for (size_t i = 0; i < xpbdCorrections.size(); ++i) {
        float ss = substeps[i];
        std::cout << "Correction for " << ss << " substeps" << std::endl << "\t" << xpbdCorrections[i] << std::endl;
    }
    std::cout << std::endl;
}

TEST_CASE("Stretch-Shear XPBD", "[ss-xpbd]")
{

}

