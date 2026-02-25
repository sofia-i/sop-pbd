
#include "SOP_IntegrateStep.h"
#include "MathUtils.h"
#include "Integration.h"

#include <OP/OP_Operator.h>
#include <OP/OP_AutoLockInputs.h>
#include <PRM/PRM_TemplateBuilder.h>

using namespace HDK_PBD;
using namespace UT::Literal;

using qm = MathUtils;

OP_Operator*
SOP_Integrate::getOperator()
{
    return new OP_Operator(
        "pbd_integrate",
        "PBD Integrate",
        SOP_Integrate::myConstructor,
        SOP_Integrate::buildTemplates(),
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
SOP_Integrate::buildTemplates()
{
    static PRM_TemplateBuilder templ("SOP_IntegrateStep.cpp"_sh, theDsFile);
    return templ.templates();
}


OP_Node *
SOP_Integrate::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_Integrate(net, name, op);
}


SOP_Integrate::SOP_Integrate(OP_Network *net, const char *name, OP_Operator *op)
    : SOP_Node(net, name, op)
{
    
}


SOP_Integrate::~SOP_Integrate() {}


OP_ERROR
SOP_Integrate::cookMySop(OP_Context &context)
{
    OP_AutoLockInputs inputs(this);
    if (inputs.lock(context) >= UT_ERROR_ABORT)
        return error();

    duplicateSource(0, context);
    GU_Detail* geo = gdp;

    // Parameters
    double timestep = evalFloat("timestep", 0, context.getTime());

    // Read attributes
    GA_ROHandleV3   exForceHandle(geo, GA_ATTRIB_POINT, "external_force");
    GA_ROHandleV3   posHandle(geo, GA_ATTRIB_POINT, "P");
    GA_ROHandleV4   oriHandle(geo, GA_ATTRIB_POINT, "orient");
    GA_ROHandleF    massHandle(geo, GA_ATTRIB_POINT, "mass");
    GA_ROHandleF    oriInvMassHandle(geo, GA_ATTRIB_POINT, "oriInvMass");

    if(exForceHandle.isInvalid()) {
        addError(SOP_ERR_INVALID_ATTRIBUTE_NAME, "Invalid external force attribute");
        return error();
    }
    if(massHandle.isInvalid()) {
        addError(SOP_ERR_INVALID_ATTRIBUTE_NAME, "Invalid mass attribute");
        return error();
    }
    if(oriInvMassHandle.isInvalid()) {
        addError(SOP_ERR_INVALID_ATTRIBUTE_NAME, "Invalid orientation inverse mass attribute");
        return error();
    }
    

    // Read and write attributes
    GA_RWHandleV3   velHandle(geo, GA_ATTRIB_POINT, "v");
    GA_RWHandleV3   proppHandle(geo, GA_ATTRIB_POINT, "propp");
    GA_RWHandleV4   propoHandle(geo, GA_ATTRIB_POINT, "propo");
    GA_RWHandleV3   angVelHandle(geo, GA_ATTRIB_POINT, "w");

    if(velHandle.isInvalid()) {
        addError(SOP_ERR_INVALID_ATTRIBUTE_NAME, "Invalid velocity attribute");
        return error();
    }
    if(posHandle.isInvalid()) {
        addError(SOP_ERR_INVALID_ATTRIBUTE_NAME, "Invalid position attribute");
        return error();
    }
    if(proppHandle.isInvalid()) {
        addError(SOP_ERR_INVALID_ATTRIBUTE_NAME, "Invalid proposed position attribute");
        return error();
    }
    if(angVelHandle.isInvalid()) {
        addError(SOP_ERR_INVALID_ATTRIBUTE_NAME, "Invalid angular velocity attribute");
        return error();
    }

   
    {
        GA_Offset ptoff;
        GA_FOR_ALL_PTOFF(gdp, ptoff)
        {
            float mass = massHandle.get(ptoff);
            float oriInvMass = oriInvMassHandle.get(ptoff);
            UT_Vector3F exForce = exForceHandle.get(ptoff);
            UT_Vector3F vel = velHandle.get(ptoff);
            UT_Vector3F pos = posHandle.get(ptoff);
            UT_Vector3F torque  = {0., 0., 0.};
            UT_Vector3F angVel = angVelHandle.get(ptoff);
            UT_Vector4F orient = oriHandle.get(ptoff);

            if (mass > 0.) {
                // Integrate velocity
                // UT_Vector3F acceleration = exForce / mass;
                // UT_Vector3F newVel = vel + acceleration * timestep;
                // velHandle.set(ptoff, newVel);
                UT_Vector3F newVel = integrateVelocity(vel, exForce, mass, timestep);
            
                // Integrate position
                // UT_Vector3F newPropp = pos + newVel * timestep;
                UT_Vector3F newPropp = integratePosition(pos, newVel, timestep);
                proppHandle.set(ptoff, newPropp);
            }

            // Integrate orientation
            if (oriInvMass > 0.) {
                // UT_Vector3F newAngVel = angVel + timestep * oriInvMass * (torque - cross(angVel, (1. / oriInvMass) * angVel));
                UT_Vector3F newAngVel = integrateAngularVelocity(angVel, torque, oriInvMass, timestep);
                angVelHandle.set(ptoff, newAngVel);

                // UT_Vector4F newOrientation = orient + UT_Vector4F(qm::quatProd(orient, qm::quatEmbed(newAngVel)) * 0.5 * timestep);
                // newOrientation.normalize();
                UT_Vector4F newOrientation = integrateOrientation(orient, newAngVel, timestep);
                propoHandle.set(ptoff, newOrientation);
            }
        }
    }

    return error();
}

const char *
SOP_Integrate::inputLabel(unsigned idx) const
{
    switch(idx) {
        case 0:
            return "Input Geo";
        default:
            return "Invalid Source";
    }
}