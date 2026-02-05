/**
 * 
 * Create Distance Constraints SOP
 */

#include "SOP_CreateDistanceConstraints.h"

#include <OP/OP_Operator.h>
#include <iostream>
#include <PRM/PRM_TemplateBuilder.h>
#include <UT/UT_Interrupt.h>

using namespace HDK_PBD;
using namespace UT::Literal;

OP_Operator*
SOP_CreateDistanceConstraints::getOperator()
{
    return new OP_Operator(
        "create_dist_constraints",
        "Create Distance Constraints",
        SOP_CreateDistanceConstraints::myConstructor,
        SOP_CreateDistanceConstraints::buildTemplates(),
        1,
        1,
        0
    );
}

const UT_StringHolder SOP_CreateDistanceConstraintsVerb::theSOPTypeName("hdk_createdistconstraints"_sh);
const SOP_NodeVerb::Register<SOP_CreateDistanceConstraintsVerb> SOP_CreateDistanceConstraintsVerb::theVerb;

static const char *theDsFile = R"THEDSFILE(
{
    name    parameters
    parm {
        name    "type_name"
        cppname "TypeName"
        label   "Type Name"
        type    string
        default { "dist" }
    }
    groupcollapsible {
        name        "prop_name_folder"
        label       "Attributes"
        grouptag    { "group_type" "collapsible" }
        parmtag     { "group_default" "1" }

        groupsimple {
            name        "in_geo_attr_folder"
            label       "Input Geo Attributes"
            grouptag    { "group_type" "simple" }
            parmtag     { "group_default" "1" }

            parm {
                name    "pos_attr_name"
                cppname "PositionAttributeName"
                label   "Position"
                type    string
                default { "P" }
            }
        }
        groupsimple {
            name        "out_geo_attr_folder"
            label       "Constraint Attributes"
            grouptag    { "group_type" "simple" }
            parmtag     { "group_default" "1" }

            parm {
                name    "type_attr_name"
                cppname "TypeAttributeName"
                label   "Type"
                type    string
                default { "type" }
            }
            parm {
                name    "target_attr_name"
                cppname "TargetAttributeName"
                label   "Target"
                type    string
                default { "target" }
            }
            parm {
                name    "dist_attr_name"
                cppname "DistAttributeName"
                label   "Distance"
                type    string
                default { "dist" }
            }
        }
    }
}
)THEDSFILE";

const SOP_NodeVerb *
SOP_CreateDistanceConstraints::cookVerb() const
{
    return SOP_CreateDistanceConstraintsVerb::theVerb.get();
}

PRM_Template*
SOP_CreateDistanceConstraints::buildTemplates()
{
    static PRM_TemplateBuilder templ("SOP_CreateDistanceConstraints.C"_sh, theDsFile);
    return templ.templates();
}

OP_Node *
SOP_CreateDistanceConstraints::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_CreateDistanceConstraints(net, name, op);
}

SOP_CreateDistanceConstraints::SOP_CreateDistanceConstraints(OP_Network *net, const char *name, OP_Operator *op)
    : SOP_Node(net, name, op)
{
    // NOTE: must manage data IDs if using verb
    mySopFlags.setManagesDataIDs(true);
}

SOP_CreateDistanceConstraints::~SOP_CreateDistanceConstraints() {}

OP_ERROR
SOP_CreateDistanceConstraints::cookMySop(OP_Context &context)
{
    return cookMyselfAsVerb(context);
}

void
SOP_CreateDistanceConstraintsVerb::cook(const CookParms &cookparms) const
{
    auto &&sopparms = cookparms.parms<SOP_CreateDistanceConstraintsParms>();

    // Allow for interruption during cooking
    UT_AutoInterrupt progress("Creating distance constraints");

    // Output detail
    GEO_Detail *output_geo = cookparms.gdh().gdpNC();
    UT_ASSERT(output_geo);
    output_geo->clear();

    // Input detail
    const GEO_Detail *const simgeo_input = cookparms.inputGeo(0);
    UT_ASSERT(simgeo_input);

    // Input parameters
    UT_StringHolder typeName = sopparms.getTypeName();
    UT_StringHolder inputPosAttrName = sopparms.getPositionAttributeName();
    UT_StringHolder typeAttrName = sopparms.getTypeAttributeName();
    UT_StringHolder targetAttrName = sopparms.getTargetAttributeName();
    UT_StringHolder distAttrName = sopparms.getDistAttributeName();

    // Input attributes
    GA_ROHandleV3 simPosHandle(simgeo_input, GA_ATTRIB_POINT, inputPosAttrName);
    if (simPosHandle.isInvalid()) {
        cookparms.sopAddError(SOP_ATTRIBUTE_INVALID, "Invalid position handle on sim geo");
        return;
    }

    // Output attributes
    auto typeAttr = output_geo->addStringTuple(GA_ATTRIB_POINT, typeAttrName, 1);
    auto targetAttr = output_geo->addIntArray(GA_ATTRIB_POINT, targetAttrName, 1);
    auto distAttr = output_geo->addFloatTuple(GA_ATTRIB_POINT, distAttrName, 1);

    GA_RWHandleS typeHandle(typeAttr);
    GA_RWHandleIA targetHandle(targetAttr);
    GA_RWHandleF distHandle(distAttr);
    // GA_RWHandleV3 posHandle(output_geo, GA_ATTRIB_POINT, "P");
    
    GA_Offset nPoints = simgeo_input->getNumPointOffsets();
    // Add a distance constraint between each adjacent point
    {
        GA_Offset simPtoff;
        GA_FOR_ALL_PTOFF(simgeo_input, simPtoff)
        {
            if ((simPtoff + 1) < nPoints) 
            {
                UT_Vector3 p1 = simPosHandle(simPtoff);
                UT_Vector3 p2 = simPosHandle(simPtoff + 1);
                float dist = (p2 - p1).length();
                UT_Int32Array targets({int(simPtoff), int(simPtoff) + 1});

                GA_Offset newPt = output_geo->appendPointOffset();
                typeHandle.set(newPt, typeName);
                targetHandle.set(newPt, targets);
                distHandle.set(newPt, dist);
            }
        }
    }

    return;
}

const char*
SOP_CreateDistanceConstraints::inputLabel(unsigned idx) const
{
    switch(idx) {
        case 0:
            return "Sim Geo";
        default:
            return "Invalid Source";
    }
}

int
SOP_CreateDistanceConstraints::isRefInput(unsigned i) const
{
    UT_ASSERT(i >= 0);
    // First input is reference
    return (i == 0);
}
