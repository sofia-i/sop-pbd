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
 * The Create constraints SOPs.  
 */


#include "SOP_CreateCollisionConstraints.h"

#include <iostream>
#include <GU/GU_RayIntersect.h>
#include <PRM/PRM_TemplateBuilder.h>
#include <OP/OP_Operator.h>
#include <OP/OP_AutoLockInputs.h>

using namespace HDK_PBD;
using namespace UT::Literal;

// void
// newSopOperator(OP_OperatorTable *table)
// {
//     table->addOperator();
// }

OP_Operator*
SOP_CreateCollisionConstraints::getOperator()
{
    return new OP_Operator(
        "create_coll_constraints",
        "Create Collision Constraints",
        SOP_CreateCollisionConstraints::myConstructor,
        SOP_CreateCollisionConstraints::buildTemplates(),
        2,
        2,
        0
    );
}

static const char *theDsFile = R"THEDSFILE(
{
    name    parameters
    parm {
        name    "coll_type"
        cppname "CollisionType"
        label   "Collision Type Attribute Name"
        type    string
        default { "collision" }
    }
    parm {
        name    "pos_attr"
        cppname "PositionAttributeName"
        label   "Position Attribute Name"
        type    string
        default { "P" }
    }
    parm {
        name    "proposed_pos_attr"
        cppname "ProposedPositionAttributeName"
        label   "Proposed Position Attribute Name"
        type    string
        default { "propp" }
    }
    parm {
        name    "prev_pos_attr"
        cppname "PreviousPositionAttributeName"
        label   "Previous Position Attribute Name"
        type    string
        default { "prevP" }
    }
}
)THEDSFILE";

PRM_Template *
SOP_CreateCollisionConstraints::buildTemplates()
{
    static PRM_TemplateBuilder templ("SOP_CreateCollisionConstraints.C"_sh, theDsFile);
    return templ.templates();
}

OP_Node *
SOP_CreateCollisionConstraints::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_CreateCollisionConstraints(net, name, op);
}

SOP_CreateCollisionConstraints::SOP_CreateCollisionConstraints(OP_Network *net, const char *name, OP_Operator *op)
    : SOP_Node(net, name, op)
{
    // TODO: uncomment if using verb
    // mySopFlags.setManagesDataIDs(true);
}

SOP_CreateCollisionConstraints::~SOP_CreateCollisionConstraints() {}

OP_ERROR
SOP_CreateCollisionConstraints::cookMySop(OP_Context &context)
{
    OP_AutoLockInputs inputs(this);
    if (inputs.lock(context) >= UT_ERROR_ABORT)
        return error();

    const GU_Detail* sim_geo = inputGeo(0);
    const GU_Detail* coll_geo = inputGeo(1);

    GU_Detail* out_geo = gdp;
    gdp->clear();

    // UT_ASSERT(sim_geo);
    // UT_ASSERT(coll_geo);
    // UT_ASSERT(out_geo);

    bool checkProposed = true;
    bool checkLast = true;

    float epsilon = 1.0e-5;
    float sceneSize = 3.;

    // Get (read) attribute handles
    UT_ASSERT(sim_geo);
    UT_ASSERT(coll_geo);
    GA_ROHandleV3 simPosHandle(sim_geo, GA_ATTRIB_POINT, "P");
    GA_ROHandleV3 simPposHandle(sim_geo, GA_ATTRIB_POINT, "propp");
    GA_ROHandleV3 simPrevPHandle(sim_geo, GA_ATTRIB_POINT, "prevP");

    if (simPosHandle.isInvalid()) {
        addError(SOP_ATTRIBUTE_INVALID, "Invalid position handle on sim geo");
        return error();
    }
    if (simPposHandle.isInvalid()) {
        addError(SOP_ATTRIBUTE_INVALID, "Invalid proposed position handle on sim geo");
        return error();
    }
    if (simPrevPHandle.isInvalid()) {
        addError(SOP_ATTRIBUTE_INVALID, "Invalid previous position handle on sim geo");
        return error();
    }

    // Create point attributes
    auto typeAttr = out_geo->addStringTuple(GA_ATTRIB_POINT, "type", 1);
    auto targetAttr = out_geo->addIntArray(GA_ATTRIB_POINT, "target", 1);
    auto normalAttr = out_geo->addFloatTuple(GA_ATTRIB_POINT, "N", 3);
    auto sourceAttr = out_geo->addStringTuple(GA_ATTRIB_POINT, "source", 1);

    // Get (output/write) attribute handles
    typeHandle = GA_RWHandleS(typeAttr);
    sourceHandle = GA_RWHandleS(sourceAttr);
    targetHandle = GA_RWHandleIA(targetAttr);
    normalHandle = GA_RWHandleV3(normalAttr);
    posHandle = GA_RWHandleV3(out_geo, GA_ATTRIB_POINT, "P");

    UT_ASSERT(sourceHandle.isValid());
    UT_ASSERT(typeHandle.isValid());
    UT_ASSERT(targetHandle.isValid());
    UT_ASSERT(normalHandle.isValid());
    UT_ASSERT(posHandle.isValid());

    // Build the ray intersect cache
    GU_RayIntersect *coll = new GU_RayIntersect(coll_geo);

    {
        GU_RayInfo hitInfo;

        GA_Offset simPtoff;
        GA_FOR_ALL_PTOFF(sim_geo, simPtoff)
        {
            UT_Vector3 x = simPosHandle(simPtoff);
            UT_Vector3 p = simPposHandle(simPtoff);

            int foundCollision = 0;

            // check backface on ray between current position and last position
            if (checkLast) {
                UT_Vector3 prev_x = simPrevPHandle(simPtoff);

                UT_Vector3 start = x;
                UT_Vector3 dir = prev_x - x;
                double step_t = dir.length();
                std::string sourceType = "checkLast"; 

                bool hit = checkCollision(coll, start, dir, step_t, sourceType, hitInfo);
                if (hit) {
                    foundCollision = 1;
                    addCollConstraintFromHit(out_geo, simPtoff, start, dir, hitInfo, sourceType);
                }
            }
            // Check if the proposed position introduces a collision
            if (!foundCollision && checkProposed) {
                UT_Vector3 start = x;
                UT_Vector3 dir = p - x;
                double step_t = dir.length();
                std::string sourceType = "checkProposed";

                bool hit = checkCollision(coll, start, dir, step_t, sourceType, hitInfo);
                if (hit) {
                    foundCollision = 1;
                    addCollConstraintFromHit(out_geo, simPtoff, start, dir, hitInfo, sourceType);
                }
            }
        }
    }

    delete coll;

    return error();
}

void SOP_CreateCollisionConstraints::addCollConstraintFromHit(GU_Detail* out_geo, int target, 
                                                              UT_Vector3 start, UT_Vector3 dir, 
                                                              GU_RayInfo hitInfo, const std::string& source)
{
    UT_Vector3 hit_pos = start + hitInfo.myT * dir;
    UT_Vector3 hit_n = hitInfo.myNml;
    addCollConstraint(out_geo, target, hit_pos, hit_n, source);
}


void
SOP_CreateCollisionConstraints::addCollConstraint(GU_Detail* out_geo, int target, 
                                                  UT_Vector3 hit_p, UT_Vector3 hit_n, 
                                                  const std::string& source)
{
    GA_Offset ptOffset = out_geo->appendPoint();
    UT_Int32Array targets({target});
    targetHandle.set(ptOffset, targets);
    posHandle.set(ptOffset, hit_p);
    normalHandle.set(ptOffset, hit_n);
    sourceHandle.set(ptOffset, source);

    // TODO: get collision name from parameter
    typeHandle.set(ptOffset, "collision");
}

const char *
SOP_CreateCollisionConstraints::inputLabel(unsigned idx) const
{
    switch(idx) {
        case 0:
            return "Sim Geo";
        case 1:
            return "Colliders";
        default:
            return "Invalid Source";
    }
}

int
SOP_CreateCollisionConstraints::isRefInput(unsigned i) const
{
    UT_ASSERT(i >= 0);
    // First and second inputs use dotted line
    return (i == 0) || (i == 1);
}

bool SOP_CreateCollisionConstraints::checkCollision(GU_RayIntersect *coll, 
                                                    UT_Vector3 start, UT_Vector3 dir, 
                                                    double step_t, const std::string& source,
                                                    GU_RayInfo &hitInfo)
{
    dir.normalize();

    hitInfo.reset();
    hitInfo.init(1.e18f, 0.0, GU_FIND_CLOSEST, 1.e-5);

    int nHits = coll->sendRay(start, dir, hitInfo);

    if (nHits > 0 && hitInfo.myT <= step_t) {
       // std::cerr << "Found collision" << std::endl;
        return true;
    }

    return false;
}
