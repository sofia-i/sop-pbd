
#include "SOP_ApplyUpdates.h"
#include "MathUtils.h"
#include "Integration.h"

#include <OP/OP_Operator.h>
#include <OP/OP_AutoLockInputs.h>
#include <PRM/PRM_TemplateBuilder.h>

using namespace HDK_PBD;
using namespace UT::Literal;
using qm = MathUtils;

OP_Operator*
SOP_ApplyUpdates::getOperator()
{
    return new OP_Operator(
        "pbd_apply_corrections",
        "PBD Apply Corrections",
        SOP_ApplyUpdates::myConstructor,
        SOP_ApplyUpdates::buildTemplates(),
        1,
        1,
        0
    );
}


static const char *theDsFile = R"THEDSFILE(
{
    name    parameters
    parm {
        name    "timestep"
        cppname "Timestep"
        label   "Timestep"
        type    float
        default { "1/($FPS * chs(\"../../substep\"))" }
    }
}
)THEDSFILE";


PRM_Template *
SOP_ApplyUpdates::buildTemplates()
{
    static PRM_TemplateBuilder templ("SOP_ApplyUpdates.cpp"_sh, theDsFile);
    return templ.templates();
}


OP_Node *
SOP_ApplyUpdates::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_ApplyUpdates(net, name, op);
}


SOP_ApplyUpdates::SOP_ApplyUpdates(OP_Network *net, const char *name, OP_Operator *op)
    : SOP_Node(net, name, op)
{

}


SOP_ApplyUpdates::~SOP_ApplyUpdates() {}


OP_ERROR
SOP_ApplyUpdates::cookMySop(OP_Context &context)
{
    OP_AutoLockInputs inputs(this);
    if (inputs.lock(context) >= UT_ERROR_ABORT)
        return error();

    duplicateSource(0, context);
    GU_Detail* geo = gdp;

    // Parameters
    double timestep = evalFloat("timestep", 0, context.getTime());

    // Read attributes
    GA_ROHandleV3 propPosHandle(geo, GA_ATTRIB_POINT, "propp");
    GA_ROHandleV4 propOriHandle(geo, GA_ATTRIB_POINT, "propo");

    // Write attributes
    GA_RWHandleV3 prevPosHandle(geo, GA_ATTRIB_POINT, "prevP");
    GA_RWHandleV3 prevVelHandle(geo, GA_ATTRIB_POINT, "prevV");
    GA_RWHandleV3 posHandle(geo, GA_ATTRIB_POINT, "P");
    GA_RWHandleV3 velHandle(geo, GA_ATTRIB_POINT, "v");
    GA_RWHandleV3 angVelHandle(geo, GA_ATTRIB_POINT, "w");
    GA_RWHandleV4 oriHandle(geo, GA_ATTRIB_POINT, "orient");

    {
        GA_Offset ptoff;
        GA_FOR_ALL_PTOFF(gdp, ptoff)
        {
            UT_Vector3F propPos = propPosHandle.get(ptoff);
            UT_Vector3F pos = posHandle.get(ptoff);
            UT_Vector3F vel = velHandle.get(ptoff);
            UT_Vector4F propOri = propOriHandle.get(ptoff);
            UT_Vector4F ori = oriHandle.get(ptoff);

            // Cache old values
            prevPosHandle.set(ptoff, pos);
            prevVelHandle.set(ptoff, vel);
        
            // Update position from projection
            UT_Vector3F newPos = propPos;
            posHandle.set(ptoff, newPos);

            // Set orientation
            // UT_Vector4F temp = 2. * (1. / timestep) * qm::quatProd(qm::quatConjugate(ori), propOri);
            // UT_Vector3F newAngVel{temp.x(), temp.y(), temp.z()};
            UT_Vector3F newAngVel = getAngularVelocityUpdate(ori, propOri, timestep);
            UT_Vector4F newOrient = propOri;

            angVelHandle.set(ptoff, newAngVel);
            oriHandle.set(ptoff, newOrient);

            // Set velocity
            // UT_Vector3F newVel = (1. / timestep) * (newPos - pos);
            UT_Vector3F newVel = getVelocityUpdate(pos, newPos, timestep);
            velHandle.set(ptoff, newVel);
        }
    }

    return error();
}

const char *
SOP_ApplyUpdates::inputLabel(unsigned idx) const
{
    switch(idx) {
        case 0:
            return "Input Geo";
        default:
            return "Invalid Source";
    }
}