/**
 * 
 * Create Rod Constraints SOP
 */

#ifndef __SOP_CreateRodConstraints_h__
#define __SOP_CreateRodConstraints_h__

#include "SOP_CreateRodConstraints.proto.h"

#include <SOP/SOP_Node.h>
#include <SOP/SOP_NodeVerb.h>

namespace HDK_PBD {

class SOP_CreateRodConstraintsVerb : public SOP_NodeVerb
{
public:
    SOP_NodeParms *allocParms() const override { return new SOP_CreateRodConstraintsParms(); }
    UT_StringHolder name() const override { return theSOPTypeName; }

    CookMode cookMode(const SOP_NodeParms *parms) const override { return COOK_GENERIC; }

    void cook(const CookParms &cookparms) const override;

    static const UT_StringHolder theSOPTypeName;
    static const SOP_NodeVerb::Register<SOP_CreateRodConstraintsVerb> theVerb;
};

class SOP_CreateRodConstraints : public SOP_Node
{
public:
    SOP_CreateRodConstraints(OP_Network *net, const char *name, OP_Operator *op);
        ~SOP_CreateRodConstraints() override;
    
    static PRM_Template *buildTemplates();
    static OP_Node      *myConstructor(OP_Network*, const char*, OP_Operator*);
    static OP_Operator  *getOperator();

protected:
    const SOP_NodeVerb *cookVerb() const override;

    const char *inputLabel(unsigned idx) const override;
    OP_ERROR    cookMySop(OP_Context &context) override;

    int isRefInput(unsigned i) const override;
};


}  // end namespace HDK_PBD


#endif  // __SOP_CreateRodConstraints_h__