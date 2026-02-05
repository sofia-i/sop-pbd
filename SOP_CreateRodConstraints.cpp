/**
 * 
 * Create Rod constraints SOP
 */

#include "SOP_CreateRodConstraints.h"

#include "GEO/GEO_Primitive.h"
#include <PRM/PRM_TemplateBuilder.h>
#include <UT/UT_Interrupt.h>

using namespace HDK_PBD;
using namespace UT::Literal;

const UT_StringHolder SOP_CreateRodConstraintsVerb::theSOPTypeName("hdk_createrodconstraints"_sh);
const SOP_NodeVerb::Register<SOP_CreateRodConstraintsVerb> SOP_CreateRodConstraintsVerb::theVerb;

static const char *theDsFile = R"THEDSFILE(
{
    name parameters
    parm {
        name    "doStretchShear"
        cppname "DoStretchShear"
        label   "Stretch-Shear"
        type    toggle
        default { "1" }
    }
    parm {
        name    "doBendTwist"
        cppname "DoBendTwist"
        label   "Bend-Twist"
        type    toggle
        default { "1" }
    }
    parm {
        name    "stretch_type_name"
        cppname "StretchTypeName"
        label   "Stretch-Shear Type Name"
        type    string
        default { "rod_ss" }
    }
    parm {
        name    "bend_type_name"
        cppname "BendTypeName"
        label   "Bend-Twist Type Name"
        type    string
        default { "rod_bt" }
    }
    groupcollapsible {
        name        "prop_name_folder"
        label       "Attributes"
        grouptag    { "group_type" "collapsible" }
        parmtag     { "group_default" "1" }

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

OP_Operator*
SOP_CreateRodConstraints::getOperator()
{
    return new OP_Operator(
        "create_rod_constraints",
        "Create Rod Constraints",
        SOP_CreateRodConstraints::myConstructor,
        SOP_CreateRodConstraints::buildTemplates(),
        1,
        1,
        0
    );
}

void
SOP_CreateRodConstraintsVerb::cook(const CookParms &cookparms) const
{
    auto &&sopparms = cookparms.parms<SOP_CreateRodConstraintsParms>();

    // Allow for interruption during cooking
    UT_AutoInterrupt progress("Creating rod constraints");

    // Output detail
    GEO_Detail *output_geo = cookparms.gdh().gdpNC();
    output_geo->clear();
    UT_ASSERT(output_geo);

    // Input detail
    const GEO_Detail *const simgeo_input = cookparms.inputGeo(0);
    UT_ASSERT(simgeo_input);

    // Node parameters
    bool doStretchShear = sopparms.getDoStretchShear();
    bool doBendTwist = sopparms.getDoBendTwist();

    UT_StringHolder stretchShearTypeName = sopparms.getStretchTypeName();
    UT_StringHolder bendTwistTypeName = sopparms.getBendTypeName();
    UT_StringHolder typeAttrName = sopparms.getTypeAttributeName();
    UT_StringHolder targetAttrName = sopparms.getTargetAttributeName();

    // Input attributes
    GA_Offset nPoints = simgeo_input->getNumPointOffsets();
    
    // Output attributes
    auto typeAttr = output_geo->addStringTuple(GA_ATTRIB_POINT, typeAttrName, 1);
    auto targetAttr = output_geo->addIntArray(GA_ATTRIB_POINT, targetAttrName, 1);

    GA_RWHandleS typeHandle(typeAttr);
    GA_RWHandleIA targetHandle(targetAttr);

    // Stretch - Shear
    if (doStretchShear) {
        GA_Offset simPtoff;
        GA_FOR_ALL_PTOFF(simgeo_input, simPtoff)
        {
            if ((simPtoff + 1) < nPoints)
            {
                // FIXME: ptoff or index?
                int targetId = simgeo_input->pointIndex(simPtoff);
                UT_Int32Array targets({targetId, targetId + 1});

                GA_Offset newPt = output_geo->appendPointOffset();
                typeHandle.set(newPt, stretchShearTypeName);
                targetHandle.set(newPt, targets);
            }
        }
    }

    // Sort the bend & twist constraints - bidirectional interleaving
    if (doBendTwist) {
        const GEO_Primitive *prim = nullptr;
        GA_Range forwardRange, backwardRange;
        GA_Range::ordered ordered;
        GA_FOR_ALL_PRIMITIVES(simgeo_input, prim)
        {
            GA_Size nPoints = prim->getPointRange().getEntries();
            GA_Index start = 0;
            GA_Index end = nPoints - 1;
            // Construct forward range to ignore last two points (bend & twist on interior points)
            forwardRange = GA_Range(simgeo_input->getPointMap(), GA_Offset(0), end);
            backwardRange = GA_Range(forwardRange, ordered, true);

            GA_Iterator fIter(forwardRange);
            GA_Iterator bIter(backwardRange);

            // make sure interleaving offset will work
            if ((nPoints % 2) != 0) {
                bIter.advance();
            }

            // Bilateral interleaving ordering
            while(!(fIter.atEnd() && bIter.atEnd())) {
                // Forward pass
                if (!fIter.atEnd()) {
                    int target1, target2;
                    target1 = fIter.getIndex();
                    fIter.advance();
                    if (!fIter.atEnd()) {
                        target2 = fIter.getIndex();
                        UT_Int32Array targets({target1, target2});

                        GA_Offset newPt = output_geo->appendPointOffset();
                        typeHandle.set(newPt, bendTwistTypeName);
                        targetHandle.set(newPt, targets);

                        // Hop forward (second advance)
                        fIter.advance();
                    }
                }

                // Backward pass
                if (!bIter.atEnd()) {
                    int target1, target2;
                    target2 = bIter.getIndex();
                    bIter.advance();
                    if (!bIter.atEnd()) {
                        target1 = bIter.getIndex();
                        UT_Int32Array targets({target1, target2});

                        GA_Offset newPt = output_geo->appendPointOffset();
                        typeHandle.set(newPt, bendTwistTypeName);
                        targetHandle.set(newPt, targets);

                        // Hop backward (second advance)
                        bIter.advance();
                    }
                }   
            }
        }
    }

    return;
}

const SOP_NodeVerb *
SOP_CreateRodConstraints::cookVerb() const 
{
    return SOP_CreateRodConstraintsVerb::theVerb.get();
}

PRM_Template *
SOP_CreateRodConstraints::buildTemplates()
{
    static PRM_TemplateBuilder templ("SOP_CreateRodConstraints.cpp"_sh, theDsFile);
    return templ.templates();
}

OP_Node *
SOP_CreateRodConstraints::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_CreateRodConstraints(net, name, op);
}

SOP_CreateRodConstraints::SOP_CreateRodConstraints(OP_Network *net, const char *name, OP_Operator *op)
    : SOP_Node(net, name, op)
{
    // NOTE: must manage data IDs if using verb
    mySopFlags.setManagesDataIDs(true);
}

SOP_CreateRodConstraints::~SOP_CreateRodConstraints() {}

OP_ERROR
SOP_CreateRodConstraints::cookMySop(OP_Context &context)
{
    return cookMyselfAsVerb(context);
}

const char *
SOP_CreateRodConstraints::inputLabel(unsigned idx) const
{
    switch(idx) {
        case 0:
            return "Sim Geo";
        default:
            return "Invalid Source";
    }
}

int
SOP_CreateRodConstraints::isRefInput(unsigned i) const
{
    UT_ASSERT(i >= 0);
    // First input is reference
    return (i == 0);
}
