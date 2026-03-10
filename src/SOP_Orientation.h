/**
 * 
 * Orientation SOP
 */

#ifndef __SOP_Orientation_h__
#define __SOP_Orientation_h__

#include <SOP/SOP_Node.h>

namespace HDK_PBD {

class SOP_CalculateDarboux : public SOP_Node
{
public:
    SOP_CalculateDarboux(OP_Network *net, const char *name, OP_Operator *op);
        ~SOP_CalculateDarboux() override;

    static PRM_Template *buildTemplates();
    static OP_Node      *myConstructor(OP_Network*, const char*, OP_Operator*);
    static OP_Operator  *getOperator();

protected:
    const char *inputLabel(unsigned idx) const override;
    OP_ERROR    cookMySop(OP_Context &context) override;
};


}  // end namespace HDK_PBD

 #endif  // __SOP_Orientation_h__