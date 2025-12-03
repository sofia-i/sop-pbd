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

#include <UT/UT_DSOVersion.h>
#include <UT/UT_StringStream.h>
#include <UT/UT_StringHolder.h>
#include <UT/UT_Interrupt.h>

using namespace UT::Literal;
using namespace HDK_PBD;
// using namespace HDK_Sample;

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

const UT_StringHolder HDK_PBD::parm_iters = "iters";
const UT_StringHolder HDK_PBD::parm_doColl = "doColl";
const UT_StringHolder HDK_PBD::parm_doAttach = "doAttach";
const UT_StringHolder HDK_PBD::parm_doDist = "doDist";
const UT_StringHolder HDK_PBD::parm_itertype = "iterType";
const UT_StringHolder HDK_PBD::parm_typeattr = "type_attr";
const UT_StringHolder HDK_PBD::parm_targetattr = "target_attr";
const UT_StringHolder HDK_PBD::parm_target2attr = "target2_attr";
const UT_StringHolder HDK_PBD::parm_hitpattr = "hitp_attr";
const UT_StringHolder HDK_PBD::parm_hitnattr = "hitn_attr";
const UT_StringHolder HDK_PBD::parm_distattr = "dist_attr";
const UT_StringHolder HDK_PBD::parm_invmassattr = "invMass_attr";

const UT_StringHolder HDK_PBD::attachment_type = "attachment";
const UT_StringHolder HDK_PBD::dist_type = "dist";
const UT_StringHolder HDK_PBD::coll_type = "collision";
const UT_StringHolder HDK_PBD::geo_propp = "propp";

static const char *theDsFile = R"THEDSFILE(
{
    name    parameters
    parm {
        name    "iters"     // Internal parameter name
        label   "Iterations" // Descriptive paramter name
        type    integer
        default { "1" }
        range   { 1! 100 }
    }
    parm {
        name    "doColl"
        label   "Do Collisions"
        type    toggle
        default { "1" }
    }
    parm {
        name    "doAttach"
        label   "Do Attachments"
        type    toggle
        default { "1" }
    }
    parm {
        name    "doDist"
        label   "Do Distance"
        type    toggle
        default { "1" }
    }
    parm {
        name    "iterType"
        label   "Iteration Type"
        type    ordinal
        default { "0" }
        menu    {
            "gauss"      "Gauss-Seidel"
            "jacobi"     "Jacobi"
        }
    }
    groupcollapsible {
        name        "prop_name_folder"
        label       "Attribute Names"
        grouptag    { "group_type" "collapsible" }
        parmtag     { "group_default" "1" }

        parm {
            name    "type_attr"
            cppname "TypeAttributeName"
            label   "Type Attribute Name"
            type    string
            default { "type" }
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
        parm {
            name    "invMass_attr"
            cppname "InvMassAttributeName"
            label   "Inv Mass Attribute Name"
            type    string
            default { "invMass" }
        }
    }
}
)THEDSFILE";

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
}

SOP_ProjectConstraints::~SOP_ProjectConstraints() {}

OP_ERROR
SOP_ProjectConstraints::cookMySop(OP_Context &context)
{
    // Allow for interruption during cooking
    UT_AutoInterrupt progress("Projecting constraints");

    float t = context.getTime();
    std::cerr << "frame " << t << std::endl;

    // We must lock our inputs before we try to access their geometry.
    // OP_AutoLockInputs will automatically unlock our inputs when we return.
    // NOTE: Don't call unlockInputs yourself when using this!
    OP_AutoLockInputs inputs(this);
    if (inputs.lock(context) >= UT_ERROR_ABORT)
        return error();

    const GU_Detail *constraints = inputGeo(1, context);
    const GU_Detail *collision = inputGeo(2, context);

    // Duplicate incoming geometry
    duplicateSource(0, context);

    // Get attribute names from parameters
    UT_String typeattr;
    UT_String targetattr, target2attr;
    UT_String hitpattr;
    UT_String hitnattr;
    UT_String distattr;
    UT_String invmassattr;

    SOP_ProjectConstraintsEnums::Itertype itertypeattr;

    evalString(typeattr, parm_typeattr, 0, context.getTime());
    evalString(targetattr, parm_targetattr, 0, context.getTime());
    evalString(target2attr, parm_target2attr, 0, context.getTime());
    evalString(hitpattr, parm_hitpattr, 0, context.getTime());
    evalString(hitnattr, parm_hitnattr, 0, context.getTime());
    evalString(distattr, parm_distattr, 0, context.getTime());
    evalString(invmassattr, parm_invmassattr, 0, context.getTime());

    // Validate inputs - look for required properties for constraints
    GA_ROHandleS type(constraints, GA_ATTRIB_POINT, typeattr);
    if(!type.isValid()) {
        std::cerr << "SOP_ProjectConstraints::cookMySop: Invalid type handle" << std::endl;
        addError(SOP_MESSAGE, strcat(strcat("Constraints missing type property with name'", + typeattr.c_str()), "'."));
        return error();
    }

    GA_RWHandleV3 proppHandle(gdp, GA_ATTRIB_POINT, geo_propp);
    if(proppHandle.isInvalid()) {
        std::cerr << "SOP_ProjectConstraints::cookMySop: Invalid propp handle" << std::endl;
        addError(SOP_MESSAGE, "Sim geo missing 'propp' property");
        return error();
    }

    std::map<GA_Offset, UT_Vector3> ppositions;
    std::map<GA_Offset, UT_Vector3> old_ppositions;

    // Fill in the proposed positions
    {
        GA_Offset ptoff;
        GA_FOR_ALL_PTOFF(gdp, ptoff)
        {
            ppositions[ptoff] = proppHandle.get(ptoff);
            old_ppositions[ptoff] = proppHandle.get(ptoff);
        }
    }
    
    GA_ROHandleD invMassHandle(gdp, GA_ATTRIB_POINT, invmassattr);
    GA_ROHandleI targetHandle(constraints, GA_ATTRIB_POINT, targetattr);
    GA_ROHandleI target2Handle(constraints, GA_ATTRIB_POINT, target2attr);
    GA_ROHandleV3 hitPHandle(constraints, GA_ATTRIB_POINT, hitpattr);
    GA_ROHandleV3 hitNHandle(constraints, GA_ATTRIB_POINT, hitnattr);
    GA_ROHandleD distHandle(constraints, GA_ATTRIB_POINT, distattr);

    bool doAttachment = evalInt(parm_doAttach, 0, context.getTime());
    bool doDist = evalInt(parm_doDist, 0, context.getTime());
    bool doColl = evalInt(parm_doColl, 0, context.getTime());

    // Iterate over each constraint
    int nIterations = evalInt(parm_iters, 0, context.getTime());
    for (int i = 0; i < nIterations; ++i) 
    {
        GA_Offset constraint_ptoff;
        GA_FOR_ALL_PTOFF(constraints, constraint_ptoff)
        {
            if (progress.wasInterrupted()) {
                addError(SOP_MESSAGE, "User Interrupted");
                return error();
            }
            const char *type_value = type.get(constraint_ptoff);
            // printf("type: %s\n", type_value);

            if (doAttachment && strcmp(type_value, attachment_type) == 0)  // attachment constraint
            {
                // std::cerr << "checkpoint 6a1" << std::endl;
                if (!targetHandle.isValid()) {
                    std::cerr << "SOP_ProjectConstraints::cookMySop: Invalid target handle" << std::endl;
                }
                else {
                    int target = targetHandle.get(constraint_ptoff);
                    int targetPtoff = gdp->pointOffset(target);
                    // std::cerr << "checkpoint 6a2" << std::endl;
                    UT_Vector3 location = constraints->getPos3(constraint_ptoff);
                    UT_Vector3 propp = ppositions[targetPtoff];

                    UT_Vector3 correction = location - propp;

                    ppositions[targetPtoff] += correction;
                }
            }
            else if (doDist && strcmp(type_value, dist_type) == 0)  // distance constraint
            {
                // std::cerr << "checkpoint 6b1" << std::endl;
                if (target2Handle.isInvalid()) {
                    std::cerr << "invalid target 2 handle" << std::endl;
                    const char* message = "Unable to process constraint with invalid target2 handle";
                    addWarning(SOP_MESSAGE, message);
                }
                else if (distHandle.isInvalid()) {
                    std::cerr << "invalid dist handle" << std::endl;
                    const char* message = "Unable to process constraint with invalid dist handle";
                    addWarning(SOP_MESSAGE, message);
                }
                else if (invMassHandle.isInvalid()) {
                    std::cerr << "invalid inv mass handle" << std::endl;
                    const char* message = "Unable to process constraint with invalid inv mass handle";
                    addWarning(SOP_MESSAGE, message);
                }
                else {
                    int target = targetHandle.get(constraint_ptoff);
                    int target2 = target2Handle.get(constraint_ptoff);

                    int targetPtoff = gdp->pointOffset(target);
                    int target2Ptoff = gdp->pointOffset(target2);

                    float dist = distHandle.get(constraint_ptoff);

                    UT_Vector3 p1 = ppositions[targetPtoff];
                    UT_Vector3 p2 = ppositions[target2Ptoff];

                    UT_Vector3 diff = p1 - p2;
                    diff.normalize();

                    float w1 = invMassHandle(targetPtoff);
                    float w2 = invMassHandle(target2Ptoff);

                    float distDiff = (p1 - p2).length() - dist;
                    float s = distDiff / (w1 + w2);

                    UT_Vector3 correction1 = -diff * w1 * s;
                    UT_Vector3 correction2 = diff * w2 * s;

                    ppositions[targetPtoff] += correction1;
                    ppositions[target2Ptoff] += correction2;
                }
            }
            else if (doColl && strcmp(type_value, coll_type) == 0)  // collision constraint
            {
                // find the point it applies to
                // std::cerr << "checkpoint 6c1" << std::endl;
                int target = targetHandle.get(constraint_ptoff);
                int targetPtoff = gdp->pointOffset(target);
                if (hitPHandle.isInvalid()) {
                    std::cerr << "invalid hitP handle" << std::endl;
                    const char* message = "Unable to process constraint with invalid hitP handle";
                    addWarning(SOP_MESSAGE, message);
                }
                else if (hitNHandle.isInvalid()) {
                    std::cerr << "invalid hitN handle" << std::endl;
                    const char* message = "Unable to process constraint with invalid hitN handle";
                    addWarning(SOP_MESSAGE, message);
                }
                else {
                    UT_Vector3 hit_p = hitPHandle.get(constraint_ptoff);
                    UT_Vector3 hit_n = hitNHandle.get(constraint_ptoff);

                    UT_Vector3 propp = ppositions[targetPtoff];
                    UT_Vector3 correction = -hit_n * dot(hit_n, propp - hit_p) / dot(hit_n, hit_n);

                    ppositions[targetPtoff] += correction;
                }
            }
        }
    }
    

    // std::cerr << "checkpoint 7" << std::endl;
    // Apply position corrections
    {
        GA_Offset ptoff;
        GA_FOR_ALL_PTOFF(gdp, ptoff)
        {
            UT_Vector3 corr = ppositions[ptoff] - old_ppositions[ptoff];
            // std::cerr << "adjust point with offset " << ptoff << " by " << corr << std::endl;
            proppHandle.set(ptoff, ppositions[ptoff]);
            // gdp->setPos3(ptoff, ppositions[ptoff]);
        }
    }

    return error();
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
            return "Source Geo";
    }
}
