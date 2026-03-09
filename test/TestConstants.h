#ifndef __Test_Constants_h__
#define __Test_Constants_h__

#include <UT/UT_Vector3.h>
#include <UT/UT_Vector4.h>

namespace PBD_TEST {

const float eps = 1.E-6;

const UT_Vector4F QUAT_A = {0.557293, 0.557293, -0.435229, 0.435229};
const UT_Vector4F QUAT_B = {0.94373, -0.127704, 0.144805, 0.268509};
const UT_Vector4F QUAT_C = {0.938315, -0.104257, 0.104257, 0.312772};

const UT_Vector3F UNIT_VEC_A = {0.199007, 0.398015, 0.895533};
const UT_Vector3F VEC_A = {0., -0.9, 0.};
const UT_Vector3F VEC_B = {0., -0.8, 0.};

}


#endif  // __Test_Constants_h__