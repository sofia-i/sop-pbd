#ifndef __SOP_Integrate_Step_h__
#define __SOP_Integrate_Step_h__

#include <SOP/SOP_Node.h>

namespace HDK_PBD 
{

class SOP_Integrate : public SOP_Node
{
public:
    SOP_Integrate(OP_Network *net, const char *name, OP_Operator *op);
        ~SOP_Integrate() override;

    static PRM_Template *buildTemplates();
    static OP_Node      *myConstructor(OP_Network*, const char*, OP_Operator*);
    static OP_Operator  *getOperator();

protected:
    const char  *inputLabel(unsigned idx) const override;
    OP_ERROR    cookMySop(OP_Context &context) override;
};

}


#endif  // __SOP_Integrate_Step_h__