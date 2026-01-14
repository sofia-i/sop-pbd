/**
 * 
 * Create Distance Constraints SOP
 */

#ifndef __SOP_CreateDistanceConstraints_h__
#define __SOP_CreateDistanceConstraints_h__

#include "SOP_CreateDistanceConstraints.proto.h"

#include <SOP/SOP_Node.h>
#include <SOP/SOP_NodeVerb.h>

class OP_Network;
class OP_Operator;

namespace HDK_PBD {

class SOP_CreateDistanceConstraintsVerb : public SOP_NodeVerb
{
public:
    SOP_NodeParms *allocParms() const override { return new SOP_CreateDistanceConstraintsParms(); }
    UT_StringHolder name() const override { return theSOPTypeName; }

    CookMode cookMode(const SOP_NodeParms *parms) const override { return COOK_GENERIC; }

    void cook(const CookParms &cookparms) const override;

    static const UT_StringHolder theSOPTypeName;
    static const SOP_NodeVerb::Register<SOP_CreateDistanceConstraintsVerb> theVerb;
};

class SOP_CreateDistanceConstraints : public SOP_Node 
{
public:
    SOP_CreateDistanceConstraints(OP_Network *net, const char *name, OP_Operator *op);
        ~SOP_CreateDistanceConstraints() override;

    static PRM_Template *buildTemplates();
    static OP_Node      *myConstructor(OP_Network*, const char *, OP_Operator *);
    static OP_Operator  *getOperator();

protected:
    const SOP_NodeVerb  *cookVerb() const override;
    
    const char *inputLabel(unsigned idx) const override;
    OP_ERROR    cookMySop(OP_Context &context) override;

    int isRefInput(unsigned i) const override;
};

}  // end namespace HDK_PBD

#endif  // __SOP_CreateDistanceConstraints_h__