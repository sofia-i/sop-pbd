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

#include <GU/GU_Detail.h>
#include <OP/OP_AutoLockInputs.h>
#include <OP/OP_Operator.h>
#include <OP/OP_SaveFlags.h>
#include <PRM/PRM_Include.h>
#include <PRM/PRM_TemplateBuilder.h>

#include <UT/UT_Interrupt.h>

using namespace UT::Literal;
using namespace HDK_PBD;

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
        name    "xpbd_flag"
        cppname "doXpbd"
        label   "Do XPBD"
        type    toggle
        default { "1" }
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
            label       "Constraint Attribute Names"
            grouptag    { "group_type" "simple" }
            parmtag     { "group_default" "1" }

            parm {
                name    "type_attr"
                cppname "TypeAttributeName"
                label   "Type Attribute Name"
                type    string
                default { "type" }
            }
            parm {
                name    "compliance_attr"
                cppname "ComplianceAttributeName"
                label   "Compliance Attribute Name"
                type    string
                default { "compliance" }
            }
            parm {
                name    "target_attr"
                cppname "TargetAttributeName"
                label   "Target Attribute Name"
                type    string
                default { "target" }
            }
            parm {
                name    "target2_attr"
                cppname "Target2AttributeName"
                label   "Target 2 Attribute Name"
                type    string
                default { "target2" }
            }
            parm {
                name    "hitp_attr"
                cppname "HitPAttributeName"
                label   "Hit P Attribute Name"
                type    string
                default { "P" }
            }
            parm {
                name    "hitn_attr"
                cppname "HitNAttributeName"
                label   "Hit N Attribute Name"
                type    string
                default { "N" }
            }
            parm {
                name    "dist_attr"
                cppname "DistanceAttributeName"
                label   "Distance Attribute Name"
                type    string
                default { "dist" }
            }
        }
        groupsimple {
            name        "simgeo_prop_name_folder"
            label       "Sim Geo Attribute Names"
            grouptag    { "group_type" "simple" }
            parmtag     { "group_default" "1" }

            parm {
                name    "invMass_attr"
                cppname "InvMassAttributeName"
                label   "Inv Mass Attribute Name"
                type    string
                default { "invMass" }
            }
            parm {
                name    "oriInvMass_attr"
                cppname "OrientationInvMassAttributeName"
                label   "Orientation Inv Mass Attribute Name"
                type    string
                default { "oriInvMass" }
            }
            parm {
                name    "length_attr"
                cppname "LengthAttributeName"
                label   "Length Attribute Name"
                type    string
                default { "length" }
            }
            parm {
                name    "rest_darboux"
                cppname "RestDarbouxAttributeName"
                label   "Rest Darboux Vector Attribute Name"
                type    string
                default { "rest_darboux" }
            }
            parm {
                name    "propp_attr"
                cppname "ProposedPositionAttributeName"
                label   "Proposed Position Attribute Name"
                type    string
                default { "propp" }
            }
            parm {
                name    "propo_attr"
                cppname "ProposedOrientationAttributeName"
                label   "Proposed Orientation Attribute Name"
                type    string
                default { "propo" }
            }
            parm {
                name    "orientation_attr"
                cppname "OrientationAttributeName"
                label   "Orientation Attribute Name"
                type    string
                default { "orient" }
            }
            parm {
                name    "ang_vel_attr"
                cppname "AngularVelocityAttributeName"
                label   "Angular Vel Attribute Name"
                type    string
                default { "w" }
            }
            parm {
                name    "inertia_mat_attr"
                cppname "InertiaMatrixAttributeName"
                label   "Inertia Mat Attribute Name"
                type    string
                default { "I" }
            }
            parm {
                name    "collided_attr"
                cppname "HasCollidedAttributeName"
                label   "Has Collided Attribute Name"
                type    string
                default { "collided" }
            }
            parm {
                name    "cN_attr"
                cppname "CollisionNormalAttributeName"
                label   "Collision Normal Attribute Name"
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
    // UT_ASSERT(simgeo_input);

    // Copy input geometry into output
    output_geo->replaceWith(*simgeo_input);

    // Collect other inputs
    const GU_Detail *constraints = cookparms.inputGeo(1);
    const GU_Detail *collision = cookparms.inputGeo(2);

    // UT_ASSERT(constraints);
    // UT_ASSERT(collision);

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
    UT_StringHolder orient_attr      = sopparms.getOrientationAttributeName();
    UT_StringHolder ang_vel_attr     = sopparms.getAngularVelocityAttributeName();
    UT_StringHolder inert_mat_attr   = sopparms.getInertiaMatrixAttributeName();
    UT_StringHolder length_attr      = sopparms.getLengthAttributeName();
    UT_StringHolder rest_darboux_attr= sopparms.getRestDarbouxAttributeName();
    UT_StringHolder ori_invMass_attr = sopparms.getOrientationInvMassAttributeName();
    UT_StringHolder propo_attr       = sopparms.getProposedOrientationAttributeName();

    SOP_ProjectConstraintsEnums::IterationType iterType = sopparms.getIterationType();

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

    // Use default compliance k=1 (equiv to pbd)
    std::map<GA_Offset, float> compliance;
    GA_ROHandleF complianceProp(constraints, GA_ATTRIB_POINT, compliance_attr);
    if (complianceProp.isValid()) {
        GA_Offset constraint_ptoff;
        GA_FOR_ALL_PTOFF(constraints, constraint_ptoff)
        {
            compliance[constraint_ptoff] = complianceProp.get(constraint_ptoff);
        }
    }
    else {
        cookparms.sopAddMessage(SOP_MESSAGE, "Missing compliance attribute. Defaulting to 1.");
        GA_Offset constraint_ptoff;
        GA_FOR_ALL_PTOFF(constraints, constraint_ptoff)
        {
            compliance[constraint_ptoff] = 1.;
        }
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
        GA_FOR_ALL_PTOFF(constraints, out_ptoff)
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

    // CONSTRAINT PROJECTION
    // Iterate over each constraint
    int nIterations = sopparms.getIterations();
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

            double alpha;
            alpha = compliance[constraint_ptoff] * inv_time_squared;
            // std::cerr << "alpha for constraint " << constraint_ptoff << ": " << alpha << std::endl;

            int constraint_idx = constraints->pointIndex(constraint_ptoff);

            UT_Int32Array targets; 
            targetHandle.get(constraint_ptoff, targets);

            // Attachment Constraint
            if (doAttachment && strcmp(type_value, attachment_type) == 0)
            {
                // int target = targetHandle.get(constraint_ptoff);
                if (targets.size() < 1) {
                    char buffer[100];
                    snprintf(buffer, 100, "Constraint %i: failure (expected 1 target, got %lli)", constraint_idx, targets.size());
                    cookparms.sopAddWarning(SOP_MESSAGE, buffer);
                }
                else {
                    int target = targets[0];
                    int targetPtoff = output_geo->pointOffset(target);

                    // Calculate constraint correction
                    UT_Vector3 location = constraints->getPos3(constraint_ptoff);
                    UT_Vector3 propp = ppositions[targetPtoff];

                    UT_Vector3 correction = location - propp;

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
            }
            // Distance Constraint
            else if (doDist && strcmp(type_value, dist_type) == 0)
            {
                if (targets.size() < 2) {
                    char message[100];
                    snprintf(message, 100, "Unable to process dist constraint. Targets: expected: %i, got %lli.", 2, targets.size());
                    cookparms.sopAddWarning(SOP_MESSAGE, message);
                }
                else if (distHandle.isInvalid()) {
                    addInvalidHandleWarning(cookparms, "dist", "constraint", dist_attr.c_str(), true);
                }
                else if (invMassHandle.isInvalid()) {
                    addInvalidHandleWarning(cookparms, "inv mass", "Sim geo", invMass_attr.c_str(), true);
                }
                else {
                    int target = targets[0];
                    int target2 = targets[1];

                    int targetPtoff = output_geo->pointOffset(target);
                    int target2Ptoff = output_geo->pointOffset(target2);

                    // Calculate constraint corrections
                    float dist = distHandle.get(constraint_ptoff);

                    UT_Vector3 p1 = ppositions[targetPtoff];
                    UT_Vector3 p2 = ppositions[target2Ptoff];

                    UT_Vector3 diff = p1 - p2;
                    diff.normalize();

                    float w1 = invMassHandle(targetPtoff);
                    float w2 = invMassHandle(target2Ptoff);

                    float distDiff = (p1 - p2).length() - dist;
                    float s;
                    if (sopparms.getDoXpbd()) {
                        s = distDiff / (w1 + w2 + alpha);
                    }
                    else {
                        s = distDiff / (w1 + w2);
                    }

                    UT_Vector3 correction1 = -diff * w1 * s;
                    UT_Vector3 correction2 = diff * w2 * s;

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
            }
            // Collision Constraint
            else if (doColl && strcmp(type_value, coll_type) == 0)
            {
                if (targets.size() < 1) {
                    char buffer[100];
                    snprintf(buffer, 100, "Constraint %i: expected 1 target, got %lli.", constraint_idx, targets.size());
                    cookparms.sopAddWarning(SOP_MESSAGE, buffer);
                }
                else if (hitPHandle.isInvalid()) {
                    addInvalidHandleWarning(cookparms, "hitP", "constraint", hitP_attr.c_str(), true);
                }
                else if (hitNHandle.isInvalid()) {
                    addInvalidHandleWarning(cookparms, "hitN", "constraint", hitN_attr.c_str(), true);
                }
                else if (hasCollidedHandle.isInvalid()) {
                    addInvalidHandleWarning(cookparms, "has collided", "Sim geo", collided_attr.c_str(), true);
                }
                else if (collisionNormalHandle.isInvalid()) {
                    addInvalidHandleWarning(cookparms, "collision normal", "Sim geo", cN_attr.c_str(), true);
                }
                else if (invMassHandle.isInvalid()) {
                    addInvalidHandleWarning(cookparms, "inv mass", "Sim geo", invMass_attr.c_str(), true);
                }
                else {
                    int target = targets[0];
                    int targetPtoff = output_geo->pointOffset(target);

                    // Calculate constraint correction
                    UT_Vector3 hit_p = hitPHandle.get(constraint_ptoff);
                    UT_Vector3 hit_n = hitNHandle.get(constraint_ptoff);

                    UT_Vector3 propp = ppositions[targetPtoff];
                    float w = invMassHandle(targetPtoff);

                    float s;
                    if (sopparms.getDoXpbd()) {
                        s = dot(propp - hit_p, hit_n) / (w * dot(hit_n, hit_n) + alpha);
                    }
                    else {
                        s = dot(propp - hit_p, hit_n) / (w * dot(hit_n, hit_n));
                    }

                    // UT_Vector3 correction = -hit_n * dot(hit_n, propp - hit_p) / dot(hit_n, hit_n);
                    UT_Vector3 correction = -s * w * hit_n;

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
            }
            else if (doStretchStrain && strcmp(type_value, rod_ss_type) == 0)
            {
                if (invMassHandle.isInvalid()) {
                    addInvalidHandleWarning(cookparms, "inv mass", "Sim geo", invMass_attr.c_str(), true);
                }
                else if (oriInvMassHandle.isInvalid()) {
                    addInvalidHandleWarning(cookparms, "orientation inv mass", "Sim geo", ori_invMass_attr.c_str(), true);
                }
                else if (lengthHandle.isInvalid()) {
                    addInvalidHandleWarning(cookparms, "length", "Sim geo", length_attr.c_str(), true);
                }
                else {
                    int target = targets[0];
                    int target2 = targets[1];

                    int targetPtoff = output_geo->pointOffset(target);
                    int target2Ptoff = output_geo->pointOffset(target2);

                    UT_Vector3 p1 = ppositions[targetPtoff];
                    UT_Vector3 p2 = ppositions[target2Ptoff];

                    UT_Vector4 q = porientations[targetPtoff];
                    
                    float w1 = invMassHandle.get(target);
                    float w2 = invMassHandle.get(target2);
                    float wq = oriInvMassHandle(target);

                    float length = lengthHandle.get(targetPtoff);

                    float s;
                    if (sopparms.getDoXpbd()) {
                        s = (length) / (w1 + w2 + 4. * wq * length * length + alpha);
                    }
                    else {
                        s = (length) / (w1 + w2 + 4. * wq * length * length);
                    }

                    UT_Vector4 e3 = {0., 0., 0., 1.};
                    UT_Vector3 d3 = PBD::MathUtils::quatImagPart(PBD::MathUtils::quatProd(PBD::MathUtils::quatProd(q, e3), PBD::MathUtils::quatConjugate(q)));

                    // UT_Vector3 correction = -hit_n * dot(hit_n, propp - hit_p) / dot(hit_n, hit_n);
                    UT_Vector3 correction1 = w1 * s * ((1. / length) * (p2 - p1) - d3);
                    UT_Vector3 correction2 = -w2 * s * ((1. / length) * (p2 - p1) - d3);
                    UT_Vector4 correctionq = 2. * wq * length * s * PBD::MathUtils::quatProd(PBD::MathUtils::quatEmbed((1. / length) * (p2 - p1) - d3), PBD::MathUtils::quatProd(q, PBD::MathUtils::quatConjugate(e3)));

                    switch (iterType) {
                        case (SOP_ProjectConstraintsEnums::IterationType::GAUSS): {
                            ppositions[targetPtoff] += correction1;
                            ppositions[target2Ptoff] += correction2;
                            porientations[targetPtoff] += correctionq;
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
            }
            else if (doBendTwist && strcmp(type_value, rod_bt_type) == 0)
            {
                if (oriInvMassHandle.isInvalid()) {
                    addInvalidHandleWarning(cookparms, "orientation inv mass", "sim geo", ori_invMass_attr.c_str(), true);
                }
                else if (orientHandle.isInvalid()) {
                    addInvalidHandleWarning(cookparms, "orientation", "sim geo", orient_attr.c_str(), true);
                }
                else if (lengthHandle.isInvalid()) {
                    addInvalidHandleWarning(cookparms, "length", "sim geo", length_attr.c_str(), true);
                }
                else if (restDarbouxHandle.isInvalid()) {
                    addInvalidHandleWarning(cookparms, "rest darboux", "sim geo", rest_darboux_attr.c_str(), true);
                }
                else {
                    int target1 = targets[0];
                    int target2 = targets[1];

                    int target1Ptoff = output_geo->pointOffset(target1);
                    int target2Ptoff = output_geo->pointOffset(target2);

                    UT_Vector4 q = orientHandle.get(target1Ptoff);
                    UT_Vector4 u = orientHandle.get(target2Ptoff);

                    float w1 = oriInvMassHandle.get(target1Ptoff);
                    float w2 = oriInvMassHandle.get(target2Ptoff);

                    float length = lengthHandle.get(target1Ptoff);

                    UT_Vector3 darboux = PBD::MathUtils::darbouxVector(q, u, length);
                    UT_Vector3 rest_darboux = restDarbouxHandle.get(target1Ptoff);

                    float s;
                    if (sopparms.getDoXpbd()) {
                        s = 1. / (w1 + w2 + alpha);
                    }
                    else {
                        s = 1./ (w1 + w2);
                    }

                    float t;

                    UT_Vector3 darbouxDiff = darboux - t * rest_darboux;
                    UT_Vector4 darbouxDiffQuat = PBD::MathUtils::quatEmbed(darbouxDiff);

                    UT_Vector4 qCorrection = s * PBD::MathUtils::quatProd(u, darbouxDiffQuat);
                    UT_Vector4 uCorrection = -s * PBD::MathUtils::quatProd(q, darbouxDiffQuat);

                    switch (iterType) {
                        case (SOP_ProjectConstraintsEnums::IterationType::GAUSS): {
                            porientations[target1Ptoff] += qCorrection;
                            porientations[target2Ptoff] += uCorrection;
                            break;
                        }
                        case (SOP_ProjectConstraintsEnums::IterationType::JACOBI): {
                            oCorrections[target1Ptoff] += qCorrection;
                            oCorrections[target2Ptoff] += uCorrection;
                            nOCorrections[target1Ptoff] += 1;
                            nOCorrections[target2Ptoff] += 1;
                            break;
                        }
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
