/**
 * Create Attachment Constraints SOP.  
 */

#include "SOP_CreateAttachmentConstraints.h"

#include <iostream>
#include <OP/OP_Operator.h>
#include <OP/OP_AutoLockInputs.h>
#include <PRM/PRM_TemplateBuilder.h>
#include <PRM/PRM_Template.h>
#include <UT/UT_Interrupt.h>

using namespace HDK_PBD;
using namespace UT::Literal;

OP_Operator*
SOP_CreateAttachmentConstraints::getOperator()
{
    return new OP_Operator(
        "create_attach_constraints",
        "Create Attachment Constraints",
        SOP_CreateAttachmentConstraints::myConstructor,
        SOP_CreateAttachmentConstraints::buildTemplates(),
        1,
        1,
        0
    );
}

const UT_StringHolder SOP_CreateAttachmentConstraintsVerb::theSOPTypeName("hdk_createattachmentconstraints"_sh);
const SOP_NodeVerb::Register<SOP_CreateAttachmentConstraintsVerb> SOP_CreateAttachmentConstraintsVerb::theVerb;

static const char *theDsFile = R"THEDSFILE(
{
    name    parameters
    parm {
        name "groupNames"
        cppname "GroupNames"
        label "Groups"
        type string
        default { "" }
    }
    parm {
        name    "type_name"
        cppname "TypeName"
        label   "Type Name"
        type    string
        default { "attachment" }
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
                name    "pos_attr"
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
                name    "type_attr"
                cppname "TypeAttributeName"
                label   "Type"
                type    string
                default { "type" }
            }
            parm {
                name    "target_attr"
                cppname "TargetAttributeName"
                label   "Target"
                type    string
                default { "target" }
            }
        }
    }    
}
)THEDSFILE";

const SOP_NodeVerb *
SOP_CreateAttachmentConstraints::cookVerb() const 
{
    return SOP_CreateAttachmentConstraintsVerb::theVerb.get();
}

PRM_Template *
SOP_CreateAttachmentConstraints::buildTemplates()
{
    static PRM_TemplateBuilder templ("SOP_CreateAttachmentConstraints.C"_sh, theDsFile);
    if (templ.justBuilt())
    {
        templ.setChoiceListPtr("groupNames", &SOP_Node::pointGroupMenu);
    }
    return templ.templates();
}

OP_Node *
SOP_CreateAttachmentConstraints::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_CreateAttachmentConstraints(net, name, op);
}

SOP_CreateAttachmentConstraints::SOP_CreateAttachmentConstraints(OP_Network *net, const char *name, OP_Operator *op)
    : SOP_Node(net, name, op)
{
    // NOTE: must manage data IDs if using verb
    mySopFlags.setManagesDataIDs(true);
}

SOP_CreateAttachmentConstraints::~SOP_CreateAttachmentConstraints() {}

OP_ERROR
SOP_CreateAttachmentConstraints::cookMySop(OP_Context &context)
{
    return cookMyselfAsVerb(context);
}

void
SOP_CreateAttachmentConstraintsVerb::cook(const CookParms &cookparms) const
{
    auto &&sopparms = cookparms.parms<SOP_CreateAttachmentConstraintsParms>();

    // Allow for interruption during cooking
    UT_AutoInterrupt progress("Creating attachment constraints");

    // Output detail
    GEO_Detail *output_geo = cookparms.gdh().gdpNC();
    UT_ASSERT(output_geo);
    output_geo->clear();

    // Input detail
    const GEO_Detail *const simgeo_input = cookparms.inputGeo(0);
    if (!simgeo_input) {
        std::cerr << "invalid sim geo detial pointer" << std::endl;
        return;
    }

    // Input parameters
    UT_StringHolder groupNames = sopparms.getGroupNames();

    // Find the constrained group
    // const GA_PointGroup* constrainedGroup = simgeo_input->findPointGroup(groupNames);
    GOP_Manager group_manager;
    bool success;
    const GA_PointGroup* constrainedGroup = group_manager.parsePointDetached(
        groupNames.c_str(),
        simgeo_input,
        true,
        success
    );

    // Get parameters
    UT_StringHolder typeName = sopparms.getTypeName();
    UT_StringHolder inputPosAttrName = sopparms.getPositionAttributeName();
    UT_StringHolder typeAttrName = sopparms.getTypeAttributeName();
    UT_StringHolder targetAttrName = sopparms.getTargetAttributeName();

    // Input attributes
    GA_ROHandleV3 simPosHandle(simgeo_input, GA_ATTRIB_POINT, inputPosAttrName);
    if (simPosHandle.isInvalid()) {
        cookparms.sopAddError(SOP_ATTRIBUTE_INVALID, "Invalid position handle on sim geo");
        return;
    }

    // Create output attributes
    auto typeAttr = output_geo->addStringTuple(GA_ATTRIB_POINT, typeAttrName, 1);
    auto targetAttr = output_geo->addIntArray(GA_ATTRIB_POINT, targetAttrName, 1);

    GA_RWHandleS typeHandle(typeAttr);
    GA_RWHandleIA targetHandle(targetAttr);
    GA_RWHandleV3 posHandle(output_geo, GA_ATTRIB_POINT, "P");

    {
        GA_Offset simPtoff;
        GA_FOR_ALL_GROUP_PTOFF(simgeo_input, constrainedGroup, simPtoff)
        {
            UT_Vector3 p = simPosHandle(simPtoff);
            UT_Int32Array targets({int(simPtoff)});

            GA_Offset newPt = output_geo->appendPointOffset();
            typeHandle.set(newPt, typeName);
            targetHandle.set(newPt, targets);
            posHandle.set(newPt, p);
        }
    }

    return;
}

const char*
SOP_CreateAttachmentConstraints::inputLabel(unsigned idx) const 
{
    switch(idx) {
        case 0:
            return "Sim Geo";
        default:
            return "Invalid Source";
    }
}

int
SOP_CreateAttachmentConstraints::isRefInput(unsigned i) const
{
    UT_ASSERT(i >= 0);
    // First input is reference
    return (i == 0);
}