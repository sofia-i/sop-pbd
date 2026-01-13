/*
 * Copyright (c) 2025
 *	Side Effects Software Inc.  All rights reserved.
 *
 * Redistribution and use of Houdini Development Kit samples in source and
 * binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. The name of Side Effects Software may not be used to endorse or
 *    promote products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY SIDE EFFECTS SOFTWARE `AS IS' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO EVENT SHALL SIDE EFFECTS SOFTWARE BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *----------------------------------------------------------------------------
 * Create Constraints SOP.  
 */


#ifndef __SOP_CreateConstraints_h__
#define __SOP_CreateConstraints_h__

#include <SOP/SOP_Node.h>
#include <OP/OP_DataTypes.h>
// #include <SOP/SOP_NodeVerb.h>

class GU_RayIntersect;
class GU_RayInfo;

namespace HDK_PBD {

class SOP_CreateCollisionConstraints : public SOP_Node
{
public:
    SOP_CreateCollisionConstraints(OP_Network *net, const char *name, OP_Operator *op);
        ~SOP_CreateCollisionConstraints() override;

    static PRM_Template *buildTemplates();
    static OP_Node      *myConstructor(OP_Network*, const char *,
                                    OP_Operator *);
    static OP_Operator  *getOperator();

protected:
    // const SOP_NodeVerb *cookVerb() const override;

    const char *inputLabel(unsigned idx) const override;
    OP_ERROR    cookMySop(OP_Context &context) override;

    int isRefInput(unsigned i) const override;

private:
    // void createPointAttributes(GU_Detail* out_geo);
    bool checkCollision(GU_RayIntersect *coll, UT_Vector3 start, UT_Vector3 dir, 
                                double step_t, const std::string& source, GU_RayInfo &hitInfo);
    void addCollConstraintFromHit(GU_Detail* out_geo, int target, 
                                  UT_Vector3 start, UT_Vector3 dir,
                                  GU_RayInfo hitInfo, const std::string& source);
    void addCollConstraint(GU_Detail* out_geo, int target, UT_Vector3 hit_p,
                           UT_Vector3 hit_n, const std::string& source);

    GA_RWHandleIA targetHandle;
    GA_RWHandleS typeHandle;
    GA_RWHandleV3 posHandle;
    GA_RWHandleV3 normalHandle;
    GA_RWHandleS sourceHandle;

};

} // end HDK_PBD namespace

#endif // __SOP_CreateConstraints_h__