/**
 * 
 * Create Rod constraints SOP
 */

#include "SOP_CreateRodConstraints.h"

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
        name    "doStretchStrain"
        cppname "DoStretchStrain"
        label   "Stretch-Strain"
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

    // Input attributes
    // ?
    GA_Offset nPoints = simgeo_input->getNumPointOffsets();
    
    // Output attributes
    auto typeAttr = output_geo->addStringTuple(GA_ATTRIB_POINT, "type", 1);
    auto targetAttr = output_geo->addIntArray(GA_ATTRIB_POINT, "target", 1);

    GA_RWHandleS typeHandle(typeAttr);
    GA_RWHandleIA targetHandle(targetAttr);

    bool doStretchStrain = sopparms.getDoStretchStrain();
    bool doBendTwist = sopparms.getDoBendTwist();

    // Stretch - Strain
    if (doStretchStrain) {
        GA_Offset simPtoff;
        GA_FOR_ALL_PTOFF(simgeo_input, simPtoff)
        {
            if ((simPtoff + 1) < nPoints)
            {
                // FIXME: ptoff or index?
                int targetId = simgeo_input->pointIndex(simPtoff);
                UT_Int32Array targets({targetId, targetId + 1});

                GA_Offset newPt = output_geo->appendPoint();
                typeHandle.set(newPt, "rod_ss");
                targetHandle.set(newPt, targets);
            }
        }
    }

    // Bend - Twist
    if (doBendTwist) {
        GA_Offset simPtoff;
        GA_FOR_ALL_PTOFF(simgeo_input, simPtoff)
        {
            if ((simPtoff + 1) < nPoints)
            {
                // FIXME: ptoff or index?
                int targetId = simgeo_input->pointIndex(simPtoff);
                UT_Int32Array targets({targetId, targetId + 1});

                GA_Offset newPt = output_geo->appendPoint();
                typeHandle.set(newPt, "rod_bt");
                targetHandle.set(newPt, targets);
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
