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
#include "SOP_ProjectConstraints.proto.h"
#include "SOP_ProjectConstraints.h"

#include <GU/GU_Detail.h>
#include <OP/OP_AutoLockInputs.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <OP/OP_SaveFlags.h>
#include <PRM/PRM_Include.h>
#include <PRM/PRM_TemplateBuilder.h>

#include <UT/UT_Interrupt.h>

using namespace UT::Literal;
using namespace HDK_PBD;

void
newSopOperator(OP_OperatorTable *table)
{
    table->addOperator(new OP_Operator(
        "hdk_project_constraints",
        "Project Constraints",
        SOP_ProjectConstraints::myConstructor,
        SOP_ProjectConstraints::buildTemplates(),
        2,
        3,
        0));
}

const UT_StringHolder HDK_PBD::attachment_type = "attachment";
const UT_StringHolder HDK_PBD::dist_type = "dist";
const UT_StringHolder HDK_PBD::coll_type = "collision";

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
                default { "hitp" }
            }
            parm {
                name    "hitn_attr"
                cppname "HitNAttributeName"
                label   "Hit N Attribute Name"
                type    string
                default { "hitn" }
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
                name    "propp_attr"
                cppname "ProposedPositionAttributeName"
                label   "Proposed Position Attribute Name"
                type    string
                default { "propp" }
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
    UT_ASSERT(simgeo_input);

    // Copy input geometry into output
    output_geo->replaceWith(*simgeo_input);

    const GU_Detail *constraints = cookparms.inputGeo(1);
    const GU_Detail *collision = cookparms.inputGeo(2);

    UT_ASSERT(constraints);
    UT_ASSERT(collision);

    // If constraints are empty, then return without doing anything
    if (constraints->getNumPoints() == 0) {
        cookparms.sopAddWarning(SOP_MESSAGE, "Constraints empty");
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

    SOP_ProjectConstraintsEnums::IterationType iterType = sopparms.getIterationType();

    // Validate inputs - look for required properties for constraints
    GA_ROHandleS type(constraints, GA_ATTRIB_POINT, type_attr);
    if(!type.isValid()) {
        std::cerr << "SOP_ProjectConstraints::cookMySop: Invalid type handle" << std::endl;
        char buffer[100];
        snprintf(buffer, 100, "Constraints missing type property with name'%s'.", type_attr.c_str());
        cookparms.sopAddError(SOP_MESSAGE, buffer);
        return;
    }

    // if compliance not included, default to pbd with k=1
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
        cookparms.sopAddWarning(SOP_MESSAGE, "Missing compliance attribute. Defaulting to 1.");
        GA_Offset constraint_ptoff;
        GA_FOR_ALL_PTOFF(constraints, constraint_ptoff)
        {
            compliance[constraint_ptoff] = 1.;
        }
    }

    GA_RWHandleV3 proppHandle(output_geo, GA_ATTRIB_POINT, propp_attr);
    if(proppHandle.isInvalid()) {
        std::cerr << "SOP_ProjectConstraints::cookMySop: Invalid propp handle" << std::endl;
        char buffer[100];
        snprintf(buffer, 100, "Sim geo missing proposed position property named '%s'.", propp_attr.c_str());
        cookparms.sopAddError(SOP_MESSAGE, buffer);
        return;
    }
    proppHandle.bumpDataId();

    GA_RWHandleI hasCollidedHandle(output_geo, GA_ATTRIB_POINT, collided_attr);
    GA_RWHandleV3 collisionNormalHandle(output_geo, GA_ATTRIB_POINT, cN_attr);
    if (hasCollidedHandle.isValid()) {
        hasCollidedHandle.bumpDataId();
    }
    if (collisionNormalHandle.isValid()) {
        collisionNormalHandle.bumpDataId();
    }

    std::map<GA_Offset, UT_Vector3> ppositions;
    std::map<GA_Offset, UT_Vector3> old_ppositions;
    std::map<GA_Offset, UT_Vector3> corrections;
    std::map<GA_Offset, int> nCorrections;

    // Fill in the proposed positions
    {
        GA_Offset ptoff;
        GA_FOR_ALL_PTOFF(simgeo_input, ptoff)
        {
            ppositions[ptoff] = proppHandle.get(ptoff);
            old_ppositions[ptoff] = proppHandle.get(ptoff);
            corrections[ptoff] = UT_Vector3{0., 0., 0.};
            nCorrections[ptoff] = 0.;
        }
    }
    
    GA_ROHandleD invMassHandle(simgeo_input, GA_ATTRIB_POINT, invMass_attr);
    GA_ROHandleI targetHandle(constraints, GA_ATTRIB_POINT, target_attr);
    GA_ROHandleI target2Handle(constraints, GA_ATTRIB_POINT, target2_attr);
    GA_ROHandleV3 hitPHandle(constraints, GA_ATTRIB_POINT, hitP_attr);
    GA_ROHandleV3 hitNHandle(constraints, GA_ATTRIB_POINT, hitN_attr);
    GA_ROHandleD distHandle(constraints, GA_ATTRIB_POINT, dist_attr);

    bool doAttachment = sopparms.getDoAttachment();
    bool doDist = sopparms.getDoDistance();
    bool doColl = sopparms.getDoCollision();

    double timestep = sopparms.getTimestep();
    double inv_time_squared = 1. / (timestep * timestep);

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
            std::cerr << "alpha for constraint " << constraint_ptoff << ": " << alpha << std::endl;

            int constraint_idx = constraints->pointIndex(constraint_ptoff);

            // Attachment Constraint
            if (doAttachment && strcmp(type_value, attachment_type) == 0)
            {
                int target = targetHandle.get(constraint_ptoff);
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
            // Distance Constraint
            else if (doDist && strcmp(type_value, dist_type) == 0)
            {
                if (target2Handle.isInvalid()) {
                    std::cerr << "invalid target 2 handle" << std::endl;
                    const char* message = "Unable to process constraint with invalid target2 handle";
                    cookparms.sopAddWarning(SOP_MESSAGE, message);
                }
                else if (distHandle.isInvalid()) {
                    std::cerr << "invalid dist handle" << std::endl;
                    const char* message = "Unable to process constraint with invalid dist handle";
                    cookparms.sopAddWarning(SOP_MESSAGE, message);
                }
                else if (invMassHandle.isInvalid()) {
                    std::cerr << "invalid inv mass handle" << std::endl;
                    const char* message = "Unable to process constraint with invalid inv mass handle";
                    cookparms.sopAddWarning(SOP_MESSAGE, message);
                }
                else {
                    int target = targetHandle.get(constraint_ptoff);
                    int target2 = target2Handle.get(constraint_ptoff);

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
                if (hitPHandle.isInvalid()) {
                    std::cerr << "invalid hitP handle" << std::endl;
                    const char* message = "Unable to process constraint with invalid hitP handle";
                    cookparms.sopAddWarning(SOP_MESSAGE, message);
                }
                else if (hitNHandle.isInvalid()) {
                    std::cerr << "invalid hitN handle" << std::endl;
                    const char* message = "Unable to process constraint with invalid hitN handle";
                    cookparms.sopAddWarning(SOP_MESSAGE, message);
                }
                else if (hasCollidedHandle.isInvalid() || collisionNormalHandle.isInvalid()) {
                    std::cerr << "invalid collision normal handle" << std::endl;
                    const char* message = "Unable to process constraint with invalid collision normal handle";
                    cookparms.sopAddWarning(SOP_MESSAGE, message);
                }
                else if (invMassHandle.isInvalid()) {
                    std::cerr << "invalid inv mass handle" << std::endl;
                    const char* message = "Unable to process constraint with invalid inv mass handle";
                    cookparms.sopAddWarning(SOP_MESSAGE, message);
                }
                else {
                    int target = targetHandle.get(constraint_ptoff);
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
        }

        // Aggregate corrections for Jacobi iteration
        if (iterType == SOP_ProjectConstraintsEnums::IterationType::JACOBI) {
            // Find the average correction for each point
            GA_Offset ptoff;
            GA_FOR_ALL_PTOFF(output_geo, ptoff) 
            {
                UT_Vector3 newPpos = ppositions[ptoff] + (1. / float(nCorrections[ptoff])) * corrections[ptoff];
                proppHandle.set(ptoff, newPpos);
            }
        }
    }

    // Apply position corrections
    {
        GA_Offset ptoff;
        GA_FOR_ALL_PTOFF(output_geo, ptoff)
        {
            proppHandle.set(ptoff, ppositions[ptoff]);
        }
    }

    return;
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
