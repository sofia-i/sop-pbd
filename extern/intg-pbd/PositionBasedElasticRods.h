#ifndef POSITION_BASED_ELASTIC_RODS_H
#define POSITION_BASED_ELASTIC_RODS_H

#include "MathFunctions.h"

#define _USE_MATH_DEFINES
#include "math.h"

using namespace PBD;

static const Real eps = static_cast<Real>(1e-6);

static const int permutation[3][3] = {
	0, 2, 1,
	1, 0, 2,
	2, 1, 0
};

bool solve_StretchShearConstraint(
    const Vector3r& p0, Real invMass0,
	const Vector3r& p1, Real invMass1,
	const Quaternionr& q0, Real invMassq0,
	const Vector3r& stretchingAndShearingKs,
	const Real restLength,
	Vector3r& corr0, Vector3r& corr1, Quaternionr& corrq0);

// ----------------------------------------------------------------------------------------------
bool solve_BendTwistConstraint(
	const Quaternionr& q0, Real invMassq0,
	const Quaternionr& q1, Real invMassq1,
	const Vector3r& bendingAndTwistingKs,
	const Quaternionr& restDarbouxVector,
	Quaternionr& corrq0, Quaternionr& corrq1);

#endif  // POSITION_BASED_ELASTIC_RODS_H