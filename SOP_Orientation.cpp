/**
 * 
 * Orientation SOP
 */

#include "SOP_Orientation.h"
#include "MathUtils.h"

#include <OP/OP_Operator.h>
#include <OP/OP_AutoLockInputs.h>
#include <PRM/PRM_TemplateBuilder.h>

using namespace HDK_PBD;
using namespace UT::Literal;

OP_Operator*
SOP_CalculateDarboux::getOperator()
{
    return new OP_Operator(
        "calculate_darboux",
        "Calculate Darboux",
        SOP_CalculateDarboux::myConstructor,
        SOP_CalculateDarboux::buildTemplates(),
        1,
        1,
        0
    );
}


static const char *theDsFile = R"THEDSFILE(
{
    name    parameters
    parm {
        name    "new_attr_name"
        cppname "NewAttributeName"
        label   "New Attribute Name"
        type    string
        default { "darboux" }
    }
    parm {
        name    "orientation_attr_name"
        cppname "OrientationAttributeName"
        label   "Orientation Attribute Name"
        type    string
        default { "orient" }
    }
    parm {
        name    "length_attr_name"
        cppname "LengthAttributeName"
        label   "Length Attribute Name"
        type    string
        default { "length" }
    }
}
)THEDSFILE";


PRM_Template *
SOP_CalculateDarboux::buildTemplates()
{
    static PRM_TemplateBuilder templ("SOP_Orientation.cpp"_sh, theDsFile);
    return templ.templates();
}


OP_Node *
SOP_CalculateDarboux::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_CalculateDarboux(net, name, op);
}

SOP_CalculateDarboux::SOP_CalculateDarboux(OP_Network *net, const char *name, OP_Operator *op)
    : SOP_Node(net, name, op)
{
    // NOTE: uncomment if using verb
    // mySopFlags.setManagesDataIDs(true);
}

SOP_CalculateDarboux::~SOP_CalculateDarboux() {}

OP_ERROR
SOP_CalculateDarboux::cookMySop(OP_Context &context)
{
    OP_AutoLockInputs inputs(this);
    if (inputs.lock(context) >= UT_ERROR_ABORT)
        return error();

    duplicateSource(0, context);
    GU_Detail* geo = gdp;

    UT_StringHolder newAttrName, orientAttrName, lengthAttrName;
    evalString(newAttrName, "new_attr_name", 0, context.getTime());
    evalString(orientAttrName, "orientation_attr_name", 0, context.getTime());
    evalString(lengthAttrName, "length_attr_name", 0, context.getTime());

    // Existing attributes
    GA_ROHandleV4 orientHandle(geo, GA_ATTRIB_POINT, orientAttrName);
    GA_ROHandleF  lengthHandle(geo, GA_ATTRIB_POINT, lengthAttrName);

    if (!orientHandle.isValid()) {
        addError(SOP_ERR_INVALID_ATTRIBUTE_NAME, "Invalid orientation attribute");
        return error();
    }
    else if (!lengthHandle.isValid()) {
        addError(SOP_ERR_INVALID_ATTRIBUTE_NAME, "Invalid length attribute");
        return error();
    }

    // Get or create darboux handle
    GA_RWHandleV3 darbouxHandle(geo, GA_ATTRIB_POINT, newAttrName);
    if (darbouxHandle.isInvalid()) {
        auto darbouxAttr = geo->addFloatTuple(GA_ATTRIB_POINT, newAttrName, 3);
        darbouxHandle = GA_RWHandleV3(darbouxAttr);
    }

    {
        GA_Offset ptoff;
        GA_FOR_ALL_PTOFF(geo, ptoff)
        {
            // FIXME: next ptoff might not be contiguous
            GA_Offset nextPtoff = ptoff + 1;
            if (nextPtoff >= geo->getNumPoints()) {
                continue;
            }
            UT_Vector4 q = orientHandle.get(ptoff);
            UT_Vector4 u = orientHandle.get(nextPtoff);
            float length = lengthHandle.get(ptoff);
                    
            UT_Vector3 darboux = MathUtils::darbouxVector(q, u, length);
            darbouxHandle.set(ptoff, darboux);
        }
    }

    return error();
}

const char *
SOP_CalculateDarboux::inputLabel(unsigned idx) const
{
    switch(idx) {
        case 0:
            return "Input Geo";
        default:
            return "Invalid Source";
    }
}