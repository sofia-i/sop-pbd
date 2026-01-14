/**
 * 
 * Solve Velocity SOP
 * Apply restitution
 */

#ifndef __SOP_SolveVelocity_h__
#define __SOP_SolveVelocity_h__

#include <SOP/SOP_Node.h>

namespace HDK_PBD {

class SOP_SolveVelocity : public SOP_Node 
{
public:
    SOP_SolveVelocity(OP_Network *net, const char *name, OP_Operator *op);
        ~SOP_SolveVelocity() override;

    static PRM_Template *buildTemplates();
    static OP_Node      *myConstructor(OP_Network*, const char*, OP_Operator*);
    static OP_Operator  *getOperator();

protected:
    const char *inputLabel(unsigned idx) const override;
    OP_ERROR    cookMySop(OP_Context &context) override;

    // int isRefInput(unsigned i) const override;
};

}


#endif  // __SOP_SolveVelocity_h__