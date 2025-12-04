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
 * Project Constraints SOP.  
 */


#ifndef __SOP_ProjectConstraints_h__
#define __SOP_ProjectConstraints_h__

#include <SOP/SOP_Node.h>
#include <SOP/SOP_NodeVerb.h>
#include <UT/UT_DSOVersion.h>
#include <UT/UT_StringHolder.h>
#include <UT/UT_IStream.h>

namespace HDK_PBD {

using namespace UT::Literal;

extern const UT_StringHolder attachment_type;
extern const UT_StringHolder dist_type;
extern const UT_StringHolder coll_type;

class SOP_ProjectConstraintsVerb : public SOP_NodeVerb
{
public:
    SOP_NodeParms *allocParms() const override { return new SOP_ProjectConstraintsParms(); }
    UT_StringHolder name() const override { return theSOPTypeName; }

    CookMode cookMode(const SOP_NodeParms *parms) const override { return COOK_GENERIC; }

    void cook(const CookParms &cookparms) const override;

    static const UT_StringHolder theSOPTypeName;
    static const SOP_NodeVerb::Register<SOP_ProjectConstraintsVerb> theVerb;
};

const UT_StringHolder SOP_ProjectConstraintsVerb::theSOPTypeName("hdk_projectconstraints"_sh);
const SOP_NodeVerb::Register<SOP_ProjectConstraintsVerb> SOP_ProjectConstraintsVerb::theVerb;

class SOP_ProjectConstraints : public SOP_Node
{
public:
	     SOP_ProjectConstraints(OP_Network *net, const char *name, OP_Operator *op);
            ~SOP_ProjectConstraints() override;

    static PRM_Template *buildTemplates();
    static OP_Node		*myConstructor(OP_Network*, const char *,
							    OP_Operator *);

protected:
    const SOP_NodeVerb *cookVerb() const override;

    const char         *inputLabel(unsigned idx) const override;
    OP_ERROR            cookMySop(OP_Context &context) override;


};
} // End HDK_PBD namespace

#endif
