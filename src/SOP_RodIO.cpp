/**
 * 
 */

#include "SOP_RodIO.h"

#include <OP/OP_Operator.h>
#include <OP/OP_AutoLockInputs.h>
#include <PRM/PRM_TemplateBuilder.h>
#include <GEO/GEO_PrimPoly.h>
#include <FS/FS_Reader.h>
#include <FS/FS_Info.h>
#include <fstream>

using namespace HDK_PBD;
using namespace UT::Literal;

OP_Operator*
SOP_ReadRod::getOperator()
{
    return new OP_Operator(
        "pbd_read_rod",
        "PBD Read Rod",
        SOP_ReadRod::myConstructor,
        SOP_ReadRod::buildTemplates(),
        nullptr,
        0,
        1,
        0
    );
}

static const char *theDsFile_read = R"THEDSFILE(
{
    name    parameters
    multiparm {
        name    "numfiles"
        label   "Number of Files"
        default 1
        parmtag { "multistartoffset" "1" }

        parm {
            name    "filename#"
            label   "Filename"
            type    file
            default { "" }
        }
    }
    parm {
        name    "currentFile"
        cppname "CurrentFile"
        label   "Current File: "
        type    string
        default { "" }
    }
})THEDSFILE";

PRM_Template*
SOP_ReadRod::buildTemplates()
{
    // TODO: make string different for read and write?
    static PRM_TemplateBuilder templ("SOP_RodIO.cpp"_sh, theDsFile_read);
    return templ.templates();
}

OP_Node*
SOP_ReadRod::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_ReadRod(net, name, op);
}

SOP_ReadRod::SOP_ReadRod(OP_Network *net, const char *name, OP_Operator *op)
    : SOP_Node(net, name, op)
{

}

SOP_ReadRod::~SOP_ReadRod() {}

OP_ERROR
SOP_ReadRod::cookMySop(OP_Context &context)
{
    flags().setTimeDep(true);
    OP_AutoLockInputs inputs(this);
    if (inputs.lock(context) >= UT_ERROR_ABORT) {
        return error();
    }

    gdp->clearAndDestroy();

    // Parameters
    UT_StringHolder filename;
    int numfiles = evalInt("numfiles", 0, context.getTime());
    int frame = context.getFrame();
    int fileIndex = std::min(frame - 1, numfiles - 1);

    int start_idx = getParm("numfiles").getMultiStartOffset();
    fileIndex += start_idx;

    evalStringInst("filename#", &fileIndex, filename, 0, context.getTime());

    setString(filename, CH_STRING_LITERAL, "currentFile", 0, context.getTime());

    // Create attributes
    GA_Attribute* orientAttr = gdp->addFloatTuple(GA_ATTRIB_POINT, "orient", 4);

    // Attribute read handles
    GA_RWHandleV3 posHandle(gdp, GA_ATTRIB_POINT, "P");
    GA_RWHandleV4 oriHandle(orientAttr);

    std::vector<UT_Vector3F> positions;
    std::vector<UT_Vector4F> orientations;
    std::vector<unsigned int> nRodPoints;

    if (!readRod(filename.c_str(), positions, orientations, nRodPoints))
    {
        addError(SOP_ERR_FILEGEO, "Unable to open file.");
        return error();
    }

    unsigned int offset = 0;
    unsigned int oriOffset = 0;
    for (size_t i = 0; i < nRodPoints.size(); ++i)
    {
        GEO_PrimPoly* prim = static_cast<GEO_PrimPoly*>(gdp->appendPrimitive(GEO_PRIMPOLY));

        for (size_t j = 0; j < nRodPoints[i]; ++j)
        {
            UT_Vector3F pos = positions[offset + j];
            UT_Vector4F ori = orientations[oriOffset + j];
            if (j == (nRodPoints[i] - 1)) {
                // no orientation
                ori = {0., 0. , 0., 0.};
            }

            GA_Offset ptoff = gdp->appendPointOffset();
            posHandle.set(ptoff, pos);
            oriHandle.set(ptoff, ori);
            prim->appendVertex(ptoff);
        }
        
        offset += nRodPoints[i];
        oriOffset += nRodPoints[i] - 1;
    }

    return error();
}

bool 
SOP_ReadRod::readRod(const std::string& filename, std::vector<UT_Vector3F>& positions,
        std::vector<UT_Vector4F>& orientations, std::vector<unsigned int>& nRodPoints)
{
    // TODO: use FS_Reader
    std::ifstream infile;
    infile.open(filename);

    if(!infile)
        return false;

    unsigned int nRods;
    infile >> nRods;

    nRodPoints.resize(nRods);

    unsigned int nPoints, nEdges;
    for (unsigned int i = 0; i < nRods; ++i) {
        infile >> nPoints;
        infile >> nEdges;

        nRodPoints[i] = nPoints;

        positions.reserve(positions.size() + nPoints);
        orientations.reserve(orientations.size() + nEdges);

        float px, py, pz;
        for (unsigned int j = 0; j < nPoints; ++j) {
            infile >> px >> py >> pz;
            positions.push_back({px, py, pz});
        }
        float qx, qy, qz, qw;
        for (unsigned int j = 0; j < nEdges; ++j) {
            infile >> qx >> qy >> qz >> qw;
            orientations.push_back({qx, qy, qz, qw});
        }
    }
    return true;
}