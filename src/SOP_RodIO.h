#ifndef SOP_ROD_IO_H
#define SOP_ROD_IO_H

#include "SOP/SOP_Node.h"

namespace HDK_PBD {

class SOP_ReadRod : public SOP_Node {
public:
    SOP_ReadRod(OP_Network *net, const char *name, OP_Operator *op);
        ~SOP_ReadRod() override;

    static PRM_Template *buildTemplates();
    static OP_Node      *myConstructor(OP_Network*, const char*, OP_Operator*);
    static OP_Operator  *getOperator();

protected:
    OP_ERROR cookMySop(OP_Context &context) override;
    bool readRod(const std::string& filename, std::vector<UT_Vector3F>& positions,
        std::vector<UT_Vector4F>& orientations, std::vector<unsigned int>& nRodPoints);
};

class SOP_RodWriter {

};

}  // end namespace HDK_PBD


#endif  // SOP_ROD_IO_H