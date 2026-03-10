/**
 * Solve Velocity SOP
 */

#include "SOP_SolveVelocity.h"

#include <OP/OP_Operator.h>
#include <OP/OP_AutoLockInputs.h>
#include <PRM/PRM_TemplateBuilder.h>

using namespace HDK_PBD;
using namespace UT::Literal;

OP_Operator*
SOP_SolveVelocity::getOperator()
{
    return new OP_Operator(
        "pbd_solve_velocity",
        "PBD Solve Velocity",
        SOP_SolveVelocity::myConstructor,
        SOP_SolveVelocity::buildTemplates(),
        1,
        1,
        0
    );
}

// TODO: parameters
// name of collided attr

static const char *theDsFile = R"THEDSFILE(
{
    name    parameters
}
)THEDSFILE";

PRM_Template*
SOP_SolveVelocity::buildTemplates()
{
    static PRM_TemplateBuilder templ("SOP_SolveVelocity.cpp"_sh, theDsFile);
    return templ.templates();
}

OP_Node *
SOP_SolveVelocity::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_SolveVelocity(net, name, op);
}

SOP_SolveVelocity::SOP_SolveVelocity(OP_Network *net, const char *name, OP_Operator *op)
    : SOP_Node(net, name, op)
{
    // NOTE: must manage data IDs if using verb
    // mySopFlags.setManagesDataIDs(true);
}

SOP_SolveVelocity::~SOP_SolveVelocity() {}

OP_ERROR
SOP_SolveVelocity::cookMySop(OP_Context &context)
{
    OP_AutoLockInputs inputs(this);
    if (inputs.lock(context) >= UT_ERROR_ABORT)
        return error();

    // Duplicate incoming geometry
    duplicateSource(0, context);
    GU_Detail* sim_geo = gdp;

    // Attributes to be read/written
    // TODO: replace strings with variables
    GA_RWHandleV3 velocityHandle(sim_geo, GA_ATTRIB_POINT, "v");
    GA_ROHandleI collidedHandle(sim_geo, GA_ATTRIB_POINT, "collided");
    GA_ROHandleV3 prevVelocityHandle(sim_geo, GA_ATTRIB_POINT, "prevV");
    GA_ROHandleV3 contactNormalHandle(sim_geo, GA_ATTRIB_POINT, "cN");
    GA_ROHandleF restitutionHandle(sim_geo, GA_ATTRIB_POINT, "restitution_e");

    {
        GA_Offset ptOff;
        GA_FOR_ALL_PTOFF(sim_geo, ptOff)
        {
            if (!collidedHandle.get(ptOff))
                continue;

            float e = restitutionHandle.get(ptOff);
            UT_Vector3F v = velocityHandle.get(ptOff);
            UT_Vector3F vPrev = prevVelocityHandle.get(ptOff);
            UT_Vector3F contactN = contactNormalHandle.get(ptOff);
            UT_Vector3F n = contactN;

            float vn     = dot(n, v);
            float vnPrev = dot(n, vPrev);
            
            UT_Vector3F correction = n * (-vn + std::max(-vnPrev * e, 0.f));

            UT_Vector3F newV = v + correction;
            velocityHandle.set(ptOff, newV);
        }
    }

    return error();
}

const char *
SOP_SolveVelocity::inputLabel(unsigned idx) const
{
    switch(idx) {
        case 0:
            return "Sim Geo";
        default:
            return "Invalid Source";
    }
}
