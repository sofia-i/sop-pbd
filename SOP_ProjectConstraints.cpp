/*
 * Copyright (c) 2025
 *	Side Effects Software Inc.  All rights reserved.
 *
 * Redistribution and use of Houdini Development Kit samples in source and
 * binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. The name of Side Effects Software may not be used to endorse or
 *    promote products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY SIDE EFFECTS SOFTWARE `AS IS' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO EVENT SHALL SIDE EFFECTS SOFTWARE BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *----------------------------------------------------------------------------
 * The PBD SOP.  
 */

#include "SOP_ProjectConstraints.h"
#include "MathUtils.h"
#include "src/Constraint.h"
#include "src/AttachmentConstraint.h"
#include "src/BendTwistConstraint.h"
#include "src/CollisionConstraint.h"
#include "src/DistanceConstraint.h"
#include "src/StretchShearConstraint.h"

#include <GU/GU_Detail.h>
#include <OP/OP_AutoLockInputs.h>
#include <OP/OP_Operator.h>
#include <OP/OP_SaveFlags.h>
#include <PRM/PRM_Include.h>
#include <PRM/PRM_TemplateBuilder.h>

#include <UT/UT_Interrupt.h>

using namespace UT::Literal;
using namespace HDK_PBD;

using qm = MathUtils;


OP_Operator*
SOP_ProjectConstraints::getOperator()
{
    return new OP_Operator(
        "hdk_project_constraints",
        "Project Constraints",
        SOP_ProjectConstraints::myConstructor,
        SOP_ProjectConstraints::buildTemplates(),
        2,
        3,
        0
    );
}

// void
// newSopOperator(OP_OperatorTable *table)
// {
//     table->addOperator(HDK_PBD::SOP_ProjectConstraints::getOperator());
//     // table->addOperator(HDK_PBD::SOP_CreateCollisionConstraints::getOperator());
// }

const UT_StringHolder SOP_ProjectConstraintsVerb::theSOPTypeName("hdk_projectconstraints"_sh);
const SOP_NodeVerb::Register<SOP_ProjectConstraintsVerb> SOP_ProjectConstraintsVerb::theVerb;

const UT_StringHolder HDK_PBD::attachment_type = "attachment";
const UT_StringHolder HDK_PBD::dist_type = "dist";
const UT_StringHolder HDK_PBD::coll_type = "collision";
const UT_StringHolder HDK_PBD::rod_ss_type = "rod_ss";
const UT_StringHolder HDK_PBD::rod_bt_type = "rod_bt";

static const char *theDsFile = R"THEDSFILE(
{
    name    parameters
    parm {
        name    "iters"     // Internal parameter name
        cppname "Iterations"
        label   "Iterations" // Descriptive paramter name
        type    integer
        default { "1" }
        range   { 1! 100 }
    }
    parm {
        name    "doColl"
        cppname "DoCollision"
        label   "Do Collisions"
        type    toggle
        default { "1" }
    }
    parm {
        name    "doAttach"
        cppname "DoAttachment"
        label   "Do Attachments"
        type    toggle
        default { "1" }
    }
    parm {
        name    "doDist"
        cppname "DoDistance"
        label   "Do Distance"
        type    toggle
        default { "1" }
    }
    parm {
        name    "doStretchStrain"
        cppname "DoStretchStrain"
        label   "Do Stretch and Strain"
        type    toggle
        default { "1" }
    }
    parm {
        name    "doBendTwist"
        cppname "DoBendTwist"
        label   "Do Bend and Twist"
        type    toggle
        default { "1" }
    }
    parm {
        name    "iterType"
        cppname "IterationType"
        label   "Iteration Type"
        type    ordinal
        default { "0" }
        menu    {
            "gauss"      "Gauss-Seidel"
            "jacobi"     "Jacobi"
        }
    }
    parm {
        name    "timestep"
        cppname "Timestep"
        label   "Timestep"
        type    float
        default { "1/($FPS * chs(\"../../substep\"))" }
    }
    parm {
        name    "distPrecision"
        cppname "DistPrecision"
        label   "Distance Precision"
        type    float
        default { "0.00001" }
    }
    parm {
        name    "xpbd_flag"
        cppname "doXpbd"
        label   "Do XPBD"
        type    toggle
        default { "1" }
    }
    parm {
        name    "ignore_stiffness_flag"
        cppname "DoIgnoreStiffness"
        label   "Ignore Stiffness"
        type    toggle
        default { "0" }
    }
    parm {
        name    "debug_flag"
        cppname "Debug"
        label   "Debug"
        type    toggle
        default { "0" }
    }
    groupcollapsible {
        name        "prop_name_folder"
        label       "Attribute Names"
        grouptag    { "group_type" "collapsible" }
        parmtag     { "group_default" "1" }

        groupsimple {
            name        "constr_prop_name_folder"
            label       "Constraint Attributes"
            grouptag    { "group_type" "simple" }
            parmtag     { "group_default" "1" }

            parm {
                name    "type_attr"
                cppname "TypeAttributeName"
                label   "Type"
                type    string
                default { "type" }
            }
            parm {
                name    "compliance_attr"
                cppname "ComplianceAttributeName"
                label   "Compliance"
                type    string
                default { "compliance" }
            }
            parm {
                name    "stiffness_attr"
                cppname "StiffnessAttributeName"
                label   "Stiffness"
                type    string
                default { "stiffness" }
            }
            parm {
                name    "dimension_attr"
                cppname "DimensionAttributeName"
                label   "Dimension"
                type    string
                default { "dim" }
            }
            parm {
                name    "target_attr"
                cppname "TargetAttributeName"
                label   "Target"
                type    string
                default { "target" }
            }
            parm {
                name    "target2_attr"
                cppname "Target2AttributeName"
                label   "Target 2"
                type    string
                default { "target2" }
            }
            parm {
                name    "hitp_attr"
                cppname "HitPAttributeName"
                label   "Hit P"
                type    string
                default { "P" }
            }
            parm {
                name    "hitn_attr"
                cppname "HitNAttributeName"
                label   "Hit N"
                type    string
                default { "N" }
            }
            parm {
                name    "dist_attr"
                cppname "DistanceAttributeName"
                label   "Distance"
                type    string
                default { "dist" }
            }
        }
        groupsimple {
            name        "simgeo_prop_name_folder"
            label       "Sim Geo Attributes"
            grouptag    { "group_type" "simple" }
            parmtag     { "group_default" "1" }

            parm {
                name    "invMass_attr"
                cppname "InvMassAttributeName"
                label   "Inv Mass"
                type    string
                default { "invMass" }
            }
            parm {
                name    "oriInvMass_attr"
                cppname "OrientationInvMassAttributeName"
                label   "Orientation Inv Mass"
                type    string
                default { "oriInvMass" }
            }
            parm {
                name    "length_attr"
                cppname "LengthAttributeName"
                label   "Length"
                type    string
                default { "length" }
            }
            parm {
                name    "rest_darboux"
                cppname "RestDarbouxAttributeName"
                label   "Rest Darboux Vector"
                type    string
                default { "rest_darboux" }
            }
            parm {
                name    "propp_attr"
                cppname "ProposedPositionAttributeName"
                label   "Proposed Position"
                type    string
                default { "propp" }
            }
            parm {
                name    "propo_attr"
                cppname "ProposedOrientationAttributeName"
                label   "Proposed Orientation"
                type    string
                default { "propo" }
            }
            parm {
                name    "orientation_attr"
                cppname "OrientationAttributeName"
                label   "Orientation"
                type    string
                default { "orient" }
            }
            parm {
                name    "ang_vel_attr"
                cppname "AngularVelocityAttributeName"
                label   "Angular Vel"
                type    string
                default { "w" }
            }
            parm {
                name    "inertia_mat_attr"
                cppname "InertiaMatrixAttributeName"
                label   "Inertia Mat"
                type    string
                default { "I" }
            }
            parm {
                name    "collided_attr"
                cppname "HasCollidedAttributeName"
                label   "Has Collided"
                type    string
                default { "collided" }
            }
            parm {
                name    "cN_attr"
                cppname "CollisionNormalAttributeName"
                label   "Collision Normal"
                type    string
                default { "cN" }
            }
        }
    }
}
)THEDSFILE";

const SOP_NodeVerb *
SOP_ProjectConstraints::cookVerb() const 
{
    return SOP_ProjectConstraintsVerb::theVerb.get();
}

PRM_Template *
SOP_ProjectConstraints::buildTemplates()
{
    static PRM_TemplateBuilder templ("SOP_ProjectConstraints.C"_sh, theDsFile);
    return templ.templates();
}


OP_Node *
SOP_ProjectConstraints::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_ProjectConstraints(net, name, op);
}

SOP_ProjectConstraints::SOP_ProjectConstraints(OP_Network *net, const char *name, OP_Operator *op)
    : SOP_Node(net, name, op)
{
    // all verb SOPs must manage data IDs
    mySopFlags.setManagesDataIDs(true);
}

SOP_ProjectConstraints::~SOP_ProjectConstraints() {}

OP_ERROR
SOP_ProjectConstraints::cookMySop(OP_Context &context)
{
    return cookMyselfAsVerb(context);
}

void
SOP_ProjectConstraintsVerb::cook(const CookParms &cookparms) const 
{
    auto &&sopparms = cookparms.parms<SOP_ProjectConstraintsParms>();

    // Allow for interruption during cooking
    UT_AutoInterrupt progress("Projecting constraints");

    float t = cookparms.getCookTime();
    std::cerr << "frame " << t << std::endl;

    // TODO: lock inputs? Unclear if OP_AutoLockInputs is applicable here

    // Output detail
    GEO_Detail *output_geo = cookparms.gdh().gdpNC();
    UT_ASSERT(output_geo);

    // Input detail
    const GEO_Detail *const simgeo_input = cookparms.inputGeo(0);
    if (!simgeo_input) {
        std::cerr << "invalid sim geo detail pointer" << std::endl;
        return;
    }

    // Copy input geometry into output
    output_geo->replaceWith(*simgeo_input);

    // Collect other inputs
    const GU_Detail *constraints = cookparms.inputGeo(1);
    const GU_Detail *collision = cookparms.inputGeo(2);

    // If constraints are empty, then return without doing anything
    if (!constraints || constraints->getNumPoints() == 0) {
        cookparms.sopAddMessage(SOP_MESSAGE, "Constraints empty");
        return;
    }

    // Get attribute names from parameters
    UT_StringHolder type_attr        = sopparms.getTypeAttributeName();
    UT_StringHolder target_attr      = sopparms.getTargetAttributeName();
    UT_StringHolder target2_attr     = sopparms.getTarget2AttributeName();
    UT_StringHolder hitP_attr        = sopparms.getHitPAttributeName();
    UT_StringHolder hitN_attr        = sopparms.getHitNAttributeName();
    UT_StringHolder dist_attr        = sopparms.getDistanceAttributeName();
    UT_StringHolder invMass_attr     = sopparms.getInvMassAttributeName();
    UT_StringHolder collided_attr    = sopparms.getHasCollidedAttributeName();
    UT_StringHolder cN_attr          = sopparms.getCollisionNormalAttributeName();
    UT_StringHolder propp_attr       = sopparms.getProposedPositionAttributeName();
    UT_StringHolder compliance_attr  = sopparms.getComplianceAttributeName();
    UT_StringHolder stiffness_attr   = sopparms.getStiffnessAttributeName();
    UT_StringHolder dimension_attr   = sopparms.getDimensionAttributeName();
    UT_StringHolder orient_attr      = sopparms.getOrientationAttributeName();
    UT_StringHolder ang_vel_attr     = sopparms.getAngularVelocityAttributeName();
    UT_StringHolder inert_mat_attr   = sopparms.getInertiaMatrixAttributeName();
    UT_StringHolder length_attr      = sopparms.getLengthAttributeName();
    UT_StringHolder rest_darboux_attr= sopparms.getRestDarbouxAttributeName();
    UT_StringHolder ori_invMass_attr = sopparms.getOrientationInvMassAttributeName();
    UT_StringHolder propo_attr       = sopparms.getProposedOrientationAttributeName();

    SOP_ProjectConstraintsEnums::IterationType iterType = sopparms.getIterationType();
    bool doXpbd = sopparms.getDoXpbd();
    bool doIgnoreStiffness = sopparms.getDoIgnoreStiffness();

    // -- Validate constraint property handles -- //
    GA_ROHandleS type(constraints, GA_ATTRIB_POINT, type_attr);
    if(!type.isValid()) {
        std::cerr << "SOP_ProjectConstraints::cookMySop: Invalid type handle" << std::endl;
        char buffer[100];
        snprintf(buffer, 100, "Constraints missing type property with name'%s'.", type_attr.c_str());
        cookparms.sopAddError(SOP_MESSAGE, buffer);
        return;
    }

    GA_ROHandleIA targetHandle(constraints, GA_ATTRIB_POINT, target_attr);
    if(targetHandle.isInvalid()) {
        std::cerr << "SOP_ProjectConstraints::cookMySop: Invalid target handle" << std::endl;
        char buffer[100];
        snprintf(buffer, 100, "Constraints missing target property named '%s'.", target_attr.c_str());
        cookparms.sopAddError(SOP_MESSAGE, buffer);
        return;
    }

    GA_ROHandleI dimHandle(constraints, GA_ATTRIB_POINT, dimension_attr);
    if (dimHandle.isInvalid()) {
        std::cerr << "SOP_ProjectConstraints::cookMySop: Invalid dist handle" << std::endl;
        char buffer[100];
        snprintf(buffer, 100, "Constraints missing dim property named '%s'.", dimension_attr.c_str());
        cookparms.sopAddError(SOP_MESSAGE, buffer);
        return;
    }

    // Compilance is per-constraint per-component
    GA_ROHandleFA complianceHandle(constraints, GA_ATTRIB_POINT, compliance_attr);
    if (!doIgnoreStiffness && doXpbd && complianceHandle.isInvalid()) {
        std::cerr << "SOP_ProjectConstraints::cookMySop: Invalid compliance handle" << std::endl;
        char buffer[100];
        snprintf(buffer, 100, "Constraints missing compliance property named '%s'.", compliance_attr.c_str());
        cookparms.sopAddError(SOP_MESSAGE, buffer);
        return;
    }

    GA_ROHandleF stiffnessHandle(constraints, GA_ATTRIB_POINT, stiffness_attr);
    if (!doIgnoreStiffness && !doXpbd && stiffnessHandle.isInvalid()) {
        std::cerr << "SOP_ProjectConstraints::cookMySop: Invalid stiffness handle" << std::endl;
        char buffer[100];
        snprintf(buffer, 100, "Constraints missing stiffness property named '%s'.", stiffness_attr.c_str());
        cookparms.sopAddError(SOP_MESSAGE, buffer);
        return;
    }

    // -- Validate sim geo property handles -- //
    GA_RWHandleV3 proppHandle(output_geo, GA_ATTRIB_POINT, propp_attr);
    if(proppHandle.isInvalid()) {
        std::cerr << "SOP_ProjectConstraints::cookMySop: Invalid propp handle" << std::endl;
        char buffer[100];
        snprintf(buffer, 100, "Sim geo missing proposed position property named '%s'.", propp_attr.c_str());
        cookparms.sopAddError(SOP_MESSAGE, buffer);
        return;
    }

    // FIXME: temporary default propo to 0
    GA_RWHandleV4 propoHandle(output_geo, GA_ATTRIB_POINT, propo_attr);
    if (propoHandle.isInvalid()) {
        auto propoAttr = output_geo->addFloatTuple(GA_ATTRIB_POINT, propo_attr, 4);
        propoHandle = GA_RWHandleV4(propoAttr);
        
        if (propoHandle.isInvalid()) {
            char buffer[100];
            snprintf(buffer, 100, "Failed to create missing proposed orientation property called '%s'.", propo_attr.c_str());
            cookparms.sopAddError(SOP_MESSAGE, buffer);
            return;
        }

        cookparms.sopAddMessage(SOP_MESSAGE, "Missing proposed orientation value. Defaulting to 0.");
        GA_Offset out_ptoff;
        GA_FOR_ALL_PTOFF(output_geo, out_ptoff)
        {
            propoHandle.set(out_ptoff, {0., 0., 0., 0.});
        }
        // std::cerr << "SOP_ProjectConstraints::cookMySop: Invalid propo handle" << std::endl;
        // char buffer[100];
        // snprintf(buffer, 100, "Sim geo missing proposed orientation property named '%s'.", propo_attr.c_str());
        // cookparms.sopAddError(SOP_MESSAGE, buffer);
        // return;
    }

    GA_RWHandleV4 orientHandle(output_geo, GA_ATTRIB_POINT, orient_attr);
    if (orientHandle.isInvalid()) {
        addInvalidHandleWarning(cookparms, "orientation", "Sim geo", orient_attr.c_str());
    }

    GA_RWHandleV3 angVelHandle(output_geo, GA_ATTRIB_POINT, ang_vel_attr);
    if (angVelHandle.isInvalid()) {
        addInvalidHandleWarning(cookparms, "angular velocity", "Sim geo", ang_vel_attr.c_str());
    }

    // GA_RWHandleM3 inertiaHandle(output_geo, GA_ATTRIB_POINT, inert_mat_attr);
    // if (inertiaHandle.isInvalid()) {
    //     addInvalidHandleWarning(cookparms, "inertia matrix", "Sim geo", inert_mat_attr.c_str());
    // }

    // -- Bump sim geo handles that we will change -- // 
    proppHandle.bumpDataId();
    propoHandle.bumpDataId();

    GA_RWHandleI hasCollidedHandle(output_geo, GA_ATTRIB_POINT, collided_attr);
    GA_RWHandleV3 collisionNormalHandle(output_geo, GA_ATTRIB_POINT, cN_attr);
    if (hasCollidedHandle.isValid()) {
        hasCollidedHandle.bumpDataId();
    }
    if (collisionNormalHandle.isValid()) {
        collisionNormalHandle.bumpDataId();
    }

    // Constraint projection data
    std::map<GA_Offset, UT_Vector3> ppositions;
    std::map<GA_Offset, UT_Vector3> old_ppositions;
    std::map<GA_Offset, UT_Vector3> corrections;
    std::map<GA_Offset, int> nCorrections;

    std::map<GA_Offset, UT_Vector4> porientations;
    std::map<GA_Offset, UT_Vector4> old_porientations;
    std::map<GA_Offset, UT_Vector4> oCorrections;
    std::map<GA_Offset, int> nOCorrections;

    // Fill in the proposed positions
    {
        GA_Offset ptoff;
        GA_FOR_ALL_PTOFF(simgeo_input, ptoff)
        {
            ppositions[ptoff] = proppHandle.get(ptoff);
            old_ppositions[ptoff] = proppHandle.get(ptoff);
            corrections[ptoff] = UT_Vector3{0., 0., 0.};
            nCorrections[ptoff] = 0.;

            porientations[ptoff] = propoHandle.get(ptoff);
            old_porientations[ptoff] = propoHandle.get(ptoff);
            oCorrections[ptoff] = UT_Vector4{0., 0., 0., 0.};
            nOCorrections[ptoff] = 0.;
        }
    }
    
    GA_ROHandleD invMassHandle(simgeo_input, GA_ATTRIB_POINT, invMass_attr);
    GA_ROHandleV3 hitPHandle(constraints, GA_ATTRIB_POINT, hitP_attr);
    GA_ROHandleV3 hitNHandle(constraints, GA_ATTRIB_POINT, hitN_attr);
    GA_ROHandleD distHandle(constraints, GA_ATTRIB_POINT, dist_attr);
    GA_ROHandleD lengthHandle(simgeo_input, GA_ATTRIB_POINT, length_attr);
    GA_ROHandleV3 restDarbouxHandle(simgeo_input, GA_ATTRIB_POINT, rest_darboux_attr);
    GA_ROHandleD oriInvMassHandle(simgeo_input, GA_ATTRIB_POINT, ori_invMass_attr);

    bool doAttachment = sopparms.getDoAttachment();
    bool doDist = sopparms.getDoDistance();
    bool doColl = sopparms.getDoCollision();
    bool doStretchStrain = sopparms.getDoStretchStrain();
    bool doBendTwist = sopparms.getDoBendTwist();
    bool debug = sopparms.getDebug();

    double timestep = sopparms.getTimestep();
    double inv_time_squared = 1. / (timestep * timestep);
    
    int nIterations = sopparms.getIterations();
    if (nIterations > 1 && !doIgnoreStiffness && doXpbd) {
        cookparms.sopAddError(SOP_MESSAGE, "XPBD updates for more than one projection iteration not implemented.");
        return;
    }

    // CONSTRAINT PROJECTION
    // Iterate over each constraint
    for (int i = 0; i < nIterations; ++i) 
    {
        GA_Offset constraint_ptoff;
        GA_FOR_ALL_PTOFF(constraints, constraint_ptoff)
        {
            if (progress.wasInterrupted()) {
                cookparms.sopAddError(SOP_MESSAGE, "User Interrupted");
                return;
            }
            const char *type_value = type.get(constraint_ptoff);

            int constraint_idx = constraints->pointIndex(constraint_ptoff);

            UT_Int32Array targets; 
            targetHandle.get(constraint_ptoff, targets);

            UT_FloatArray compliance;
            float stiffness;
            if (!doIgnoreStiffness) {
                if (doXpbd) {
                    complianceHandle.get(constraint_ptoff, compliance);
                    for (int i = 0; i < compliance.size(); ++i) {
                        // Incorporate timestep (XPBD!!)
                        compliance[i] *= inv_time_squared;
                    }
                }
                else {
                    stiffness = stiffnessHandle.get(constraint_ptoff);
                }
            }

            int nComponents = dimHandle.get(constraint_ptoff);

            // Attachment Constraint
            if (doAttachment && strcmp(type_value, attachment_type) == 0)
            {
                int nTargets = 1;
                if(!validateConstraint(targets, compliance, nTargets, doIgnoreStiffness, doXpbd, nComponents, constraint_idx, cookparms) || 
                        !validateAttachmentConstraint(cookparms))
                    continue;

                int target = targets[0];
                int targetPtoff = output_geo->pointOffset(target);

                // Calculate constraint correction
                UT_Vector3 location = constraints->getPos3(constraint_ptoff);
                UT_Vector3 propp = ppositions[targetPtoff];

                UT_Vector3F correction;
                AttachmentConstraint::solve(propp, location, compliance, doXpbd, correction, stiffness, nIterations);

                // Apply constraint correction
                switch (iterType) {
                    case (SOP_ProjectConstraintsEnums::IterationType::GAUSS): {
                        ppositions[targetPtoff] += correction;
                        break;
                    }
                    case (SOP_ProjectConstraintsEnums::IterationType::JACOBI): {
                        corrections[targetPtoff] += correction;
                        nCorrections[targetPtoff] += 1;
                    }
                }
            }
            // Distance Constraint
            else if (doDist && strcmp(type_value, dist_type) == 0)
            {
                if (!validateConstraint(targets, compliance, 2, doIgnoreStiffness, doXpbd, nComponents, constraint_idx, cookparms) ||
                    !validateDistanceConstraint(distHandle, invMassHandle, dist_attr.c_str(), invMass_attr.c_str(), cookparms))
                    continue;
                
                int target = targets[0];
                int target2 = targets[1];
                int targetPtoff = output_geo->pointOffset(target);
                int target2Ptoff = output_geo->pointOffset(target2);

                // Get point information
                float dist = distHandle.get(constraint_ptoff);
                UT_Vector3 p1 = ppositions[targetPtoff];
                UT_Vector3 p2 = ppositions[target2Ptoff];
                float w1 = invMassHandle(targetPtoff);
                float w2 = invMassHandle(target2Ptoff);

                UT_Vector3F correction1;
                UT_Vector3F correction2;
                StiffnessMode stiffMode = getStiffnessMode(doIgnoreStiffness, doXpbd);
                
                DistanceConstraint::solve(p1, p2, dist, w1, w2, stiffMode, stiffness, compliance, nIterations, correction1, correction2);

                // Apply constraint corrections
                switch (iterType) {
                    case (SOP_ProjectConstraintsEnums::IterationType::GAUSS): {
                        ppositions[targetPtoff] += correction1;
                        ppositions[target2Ptoff] += correction2;
                        break;
                    }
                    case (SOP_ProjectConstraintsEnums::IterationType::JACOBI): {
                        corrections[targetPtoff] += correction1;
                        corrections[target2Ptoff] += correction2;
                        nCorrections[targetPtoff] += 1;
                        nCorrections[target2Ptoff] += 1;
                    }
                }
            }
            // Collision Constraint
            else if (doColl && strcmp(type_value, coll_type) == 0)
            {
                if (!validateConstraint(targets, compliance, 1, doIgnoreStiffness, doXpbd, nComponents, constraint_idx, cookparms) |
                    !validateCollisionConstraint(
                        hitPHandle, hitNHandle, hasCollidedHandle, collisionNormalHandle, invMassHandle, 
                        hitP_attr.c_str(), hitN_attr.c_str(), collided_attr.c_str(), cN_attr.c_str(), invMass_attr.c_str(), 
                        cookparms))
                    continue;

                // Get target
                int target = targets[0];
                int targetPtoff = output_geo->pointOffset(target);

                // Get target information
                UT_Vector3 hit_p = hitPHandle.get(constraint_ptoff);
                UT_Vector3 hit_n = hitNHandle.get(constraint_ptoff);
                UT_Vector3 propp = ppositions[targetPtoff];
                float w = invMassHandle(targetPtoff);

                UT_Vector3F correction;
                StiffnessMode stiffMode = getStiffnessMode(doIgnoreStiffness, doXpbd);
                CollisionConstraint::solve(hit_p, hit_n, propp, w, stiffMode, stiffness, compliance, nIterations, correction);

                hasCollidedHandle.set(targetPtoff, 1);
                collisionNormalHandle.set(targetPtoff, hit_n);

                // Apply constraint correction
                switch (iterType) {
                    case (SOP_ProjectConstraintsEnums::IterationType::GAUSS): {
                        ppositions[targetPtoff] += correction;
                        break;
                    }
                    case (SOP_ProjectConstraintsEnums::IterationType::JACOBI): {
                        corrections[targetPtoff] += correction;
                        nCorrections[targetPtoff] += 1;
                    }
                }
            }
            // Stretch - shear constraint
            else if (doStretchStrain && strcmp(type_value, rod_ss_type) == 0)
            {
                if (!validateConstraint(targets, compliance, 2, doIgnoreStiffness, doXpbd, nComponents, constraint_idx, cookparms) ||
                    !validateStretchShearConstraint(invMassHandle, oriInvMassHandle, lengthHandle, 
                        invMass_attr.c_str(), ori_invMass_attr.c_str(), length_attr.c_str(), cookparms))
                    continue;

                // Get targets
                int target = targets[0];
                int target2 = targets[1];
                int targetPtoff = output_geo->pointOffset(target);
                int target2Ptoff = output_geo->pointOffset(target2);

                // Get target properties
                UT_Vector3 p1 = ppositions[targetPtoff];
                UT_Vector3 p2 = ppositions[target2Ptoff];
                UT_Vector4 q = porientations[targetPtoff];
                float w1 = invMassHandle.get(targetPtoff);
                float w2 = invMassHandle.get(target2Ptoff);
                float wq = oriInvMassHandle.get(targetPtoff);
                float length = lengthHandle.get(targetPtoff);

                // TODO: decide epsilon
                if ((w1 + w2 + wq) < 1.E-5) {
                    // All are pinned, no updates
                    continue;
                }

                UT_Vector3F correction1, correction2;
                UT_Vector4F correctionq;
                StiffnessMode stiffMode = getStiffnessMode(doIgnoreStiffness, doXpbd);
                StretchShearConstraint::solve(p1, p2, q, w1, w2, wq, length, stiffMode, stiffness, compliance, nIterations, correction1, correction2, correctionq);

                switch (iterType) {
                    case (SOP_ProjectConstraintsEnums::IterationType::GAUSS): {
                        ppositions[targetPtoff] += correction1;
                        ppositions[target2Ptoff] += correction2;
                        UT_Vector4 newQ = porientations[targetPtoff] + correctionq;
                        newQ.normalize();
                        porientations[targetPtoff] = newQ;
                        break;
                    }
                    case (SOP_ProjectConstraintsEnums::IterationType::JACOBI): {
                        corrections[targetPtoff] += correction1;
                        corrections[target2Ptoff] += correction2;
                        oCorrections[targetPtoff] += correctionq;
                        nCorrections[targetPtoff] += 1;
                        nCorrections[target2Ptoff] += 1;
                        nOCorrections[targetPtoff] += 1;
                        break;
                    }
                }
            }
            // Bend - Twist constraint
            else if (doBendTwist && strcmp(type_value, rod_bt_type) == 0)
            {
                if (!validateConstraint(targets, compliance, 2, doIgnoreStiffness, doXpbd, nComponents, constraint_idx, cookparms) ||
                    !validateBendTwistConstraint(oriInvMassHandle, lengthHandle, orientHandle, restDarbouxHandle,
                        ori_invMass_attr.c_str(), orient_attr.c_str(), length_attr.c_str(), rest_darboux_attr.c_str(),
                        cookparms))
                    continue;

                // Get targets
                int target1 = targets[0];
                int target2 = targets[1];
                int target1Ptoff = output_geo->pointOffset(target1);
                int target2Ptoff = output_geo->pointOffset(target2);

                // Get target information
                UT_Vector4 q = porientations[target1Ptoff];
                UT_Vector4 u = porientations[target2Ptoff];
                float wq = oriInvMassHandle.get(target1Ptoff);
                float wu = oriInvMassHandle.get(target2Ptoff);
                float length = lengthHandle.get(target1Ptoff);
                UT_Vector4 rest_darboux = qm::quatEmbed(restDarbouxHandle.get(target1Ptoff));

                // TODO: decide epsilon
                if ((wq + wu) < 1.E-5) {
                    // All are pinned, no updates
                    continue;
                }


                UT_Vector4 correctionq, correctionu;
                StiffnessMode stiffMode = getStiffnessMode(doIgnoreStiffness, doXpbd);
                BendTwistConstraint::solve(q, u, wq, wu, length, rest_darboux, stiffMode, stiffness, nIterations, compliance, correctionq, correctionu);

                switch (iterType) {
                    case (SOP_ProjectConstraintsEnums::IterationType::GAUSS): {
                        UT_Vector4 newQ = porientations[target1Ptoff] + correctionq;
                        UT_Vector4 newU = porientations[target2Ptoff] + correctionu;
                        newQ.normalize();
                        newU.normalize();
                        porientations[target1Ptoff] = newQ;
                        porientations[target2Ptoff] = newU;
                        break;
                    }
                    case (SOP_ProjectConstraintsEnums::IterationType::JACOBI): {
                        oCorrections[target1Ptoff] += correctionq;
                        oCorrections[target2Ptoff] += correctionu;
                        nOCorrections[target1Ptoff] += 1;
                        nOCorrections[target2Ptoff] += 1;
                        break;
                    }
                }
            }
        }

        // Aggregate corrections for Jacobi iteration
        if (iterType == SOP_ProjectConstraintsEnums::IterationType::JACOBI) {
            // Find the average correction for each point
            GA_Offset ptoff;
            GA_FOR_ALL_PTOFF(output_geo, ptoff) 
            {
                UT_Vector3 newPpos = ppositions[ptoff] + (1. / float(nCorrections[ptoff])) * corrections[ptoff];
                proppHandle.set(ptoff, newPpos);

                UT_Vector4 newOri = porientations[ptoff] + (1. / float(nOCorrections[ptoff])) * oCorrections[ptoff];
                newOri.normalize();
                propoHandle.set(ptoff, newOri);
            }
        }
    }

    // Apply position corrections
    {
        GA_Offset ptoff;
        GA_FOR_ALL_PTOFF(output_geo, ptoff)
        {
            proppHandle.set(ptoff, ppositions[ptoff]);
            propoHandle.set(ptoff, porientations[ptoff]);
        }
    }

    return;
}

bool 
SOP_ProjectConstraintsVerb::validateConstraint(const UT_Int32Array& targets, const UT_FloatArray& compliance, int nTargets,
        bool doIgnoreStiffness, bool doXpbd, int nComponents, int constraint_idx, const CookParms &cookparms) const
{
    if (targets.size() < nTargets) {
        char buffer[100];
        snprintf(buffer, 100, "Constraint %i: failure (expected %i target, got %lli)", constraint_idx, nTargets, targets.size());
        cookparms.sopAddWarning(SOP_MESSAGE, buffer);
        return false;
    }
    else if (!doIgnoreStiffness && doXpbd && compliance.size() < nComponents) {
        char buffer[100];
        snprintf(buffer, 100, "Constraint %i: failure (expected %i compliance value, got %lli)", constraint_idx, nComponents, compliance.size());
        cookparms.sopAddWarning(SOP_MESSAGE, buffer);
        return false;
    }
    return true;
}

bool 
SOP_ProjectConstraintsVerb::validateAttachmentConstraint(const CookParms &cookparms) const
{
    return true;
}

bool 
SOP_ProjectConstraintsVerb::validateDistanceConstraint(
    const GA_ROHandleD& distHandle, const GA_ROHandleD& invMassHandle, 
    const std::string& dist_attr, const std::string& invMass_attr,
    const CookParms &cookparms) const
{
    if (distHandle.isInvalid()) {
        addInvalidHandleWarning(cookparms, "dist", "constraint", dist_attr, true);
        return false;
    }
    else if (invMassHandle.isInvalid()) {
        addInvalidHandleWarning(cookparms, "inv mass", "Sim geo", invMass_attr, true);
        return false;
    }
    return true;
}

bool
SOP_ProjectConstraintsVerb::validateCollisionConstraint(
        const GA_ROHandleV3& hitPHandle, const GA_ROHandleV3& hitNHandle, const GA_RWHandleI& hasCollidedHandle,
        const GA_ROHandleV3& collisionNormalHandle, const GA_ROHandleD& invMassHandle,
        const std::string& hitP_attr, const std::string& hitN_attr, const std::string& collided_attr,
        const std::string& cN_attr, const std::string& invMass_attr,
        const CookParms &cookparms) const
{
    if (hitPHandle.isInvalid()) {
        addInvalidHandleWarning(cookparms, "hitP", "constraint", hitP_attr, true);
        return false;
    }
    else if (hitNHandle.isInvalid()) {
        addInvalidHandleWarning(cookparms, "hitN", "constraint", hitN_attr, true);
        return false;
    }
    else if (hasCollidedHandle.isInvalid()) {
        addInvalidHandleWarning(cookparms, "has collided", "Sim geo", collided_attr, true);
        return false;
    }
    else if (collisionNormalHandle.isInvalid()) {
        addInvalidHandleWarning(cookparms, "collision normal", "Sim geo", cN_attr, true);
        return false;
    }
    else if (invMassHandle.isInvalid()) {
        addInvalidHandleWarning(cookparms, "inv mass", "Sim geo", invMass_attr, true);
        return false;
    }
    return true;
}

bool 
SOP_ProjectConstraintsVerb::validateStretchShearConstraint(
    const GA_ROHandleD& invMassHandle, const GA_ROHandleD& oriInvMassHandle, const GA_ROHandleD& lengthHandle,
    const std::string& invMass_attr, const std::string& ori_invMass_attr, const std::string& length_attr,
    const CookParms &cookparms) const
{
    if (invMassHandle.isInvalid()) {
        addInvalidHandleWarning(cookparms, "inv mass", "Sim geo", invMass_attr.c_str(), true);
        return false;
    }
    else if (oriInvMassHandle.isInvalid()) {
        addInvalidHandleWarning(cookparms, "orientation inv mass", "Sim geo", ori_invMass_attr.c_str(), true);
        return false;
    }
    else if (lengthHandle.isInvalid()) {
        addInvalidHandleWarning(cookparms, "length", "Sim geo", length_attr.c_str(), true);
        return false;
    }
    return true;
}

bool
SOP_ProjectConstraintsVerb::validateBendTwistConstraint(
        const GA_ROHandleD& oriInvMassHandle, const GA_ROHandleD& lengthHandle, const GA_RWHandleV4& orientHandle,
        const GA_ROHandleV3& restDarbouxHandle,
        const std::string& ori_invMass_attr,  const std::string& orient_attr,  const std::string& length_attr,  const std::string& rest_darboux_attr,
        const CookParms &cookparms) const
{
    if (oriInvMassHandle.isInvalid()) {
        addInvalidHandleWarning(cookparms, "orientation inv mass", "sim geo", ori_invMass_attr.c_str(), true);
        return false;
    }
    else if (orientHandle.isInvalid()) {
        addInvalidHandleWarning(cookparms, "orientation", "sim geo", orient_attr.c_str(), true);
        return false;
    }
    else if (lengthHandle.isInvalid()) {
        addInvalidHandleWarning(cookparms, "length", "sim geo", length_attr.c_str(), true);
        return false;
    }
    else if (restDarbouxHandle.isInvalid()) {
        addInvalidHandleWarning(cookparms, "rest darboux", "sim geo", rest_darboux_attr.c_str(), true);
        return false;
    }
    return true;
}

void 
SOP_ProjectConstraintsVerb::addInvalidHandleWarning(const CookParms &cookparms, 
                                                    std::string handleName,
                                                    std::string geoName, 
                                                    std::string propName,
                                                    bool failure) const
{
    std::cerr << "invalid " << handleName << " handle" << std::endl;
    std::string message = geoName;
    if (failure) {
        message += " failure. ";
    }
    message += " missing " + handleName + " handle";
    if (!propName.empty()) {
        message += " (property named \"" + propName +  "\")";
    }
    const char* c_message = message.c_str();
    cookparms.sopAddWarning(SOP_MESSAGE, c_message);
}

const char *
SOP_ProjectConstraints::inputLabel(unsigned idx) const
{
    switch(idx) {
        case 0:
            return "Sim Geo";
        case 1:
            return "Constraints";
        case 2:
            return "Colliders";
        default:
            return "Invalid Source";
    }
}
