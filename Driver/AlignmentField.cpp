// PCH-free on purpose: SolverTests compiles this file standalone.
#include "AlignmentField.h"

#include <cmath>

namespace alignfield
{

void BlendAt(const protocol::SetAlignmentField &field, const double (&basePos)[3],
             vr::HmdQuaternion_t &rotOut, double (&transOut)[3])
{
	// Identity contributes only to the total weight and the quaternion's w
	// component; its translation is zero.
	double wSum = IdentityFloorWeight;
	double q[4] = { IdentityFloorWeight, 0.0, 0.0, 0.0 };   // w, x, y, z
	double t[3] = { 0.0, 0.0, 0.0 };

	double sigma = field.sigmaMeters > 0.01 ? field.sigmaMeters : 1.5;
	double invTwoSigmaSq = 1.0 / (2.0 * sigma * sigma);

	uint32_t count = field.anchorCount < protocol::SetAlignmentField::MaxAnchors
		? field.anchorCount : protocol::SetAlignmentField::MaxAnchors;

	for (uint32_t i = 0; i < count; ++i)
	{
		const protocol::FieldAnchor &a = field.anchors[i];

		double dx = basePos[0] - a.position[0];
		double dz = basePos[2] - a.position[2];
		double w = std::exp(-(dx * dx + dz * dz) * invTwoSigmaSq);

		// Hemisphere alignment: q and -q are the same rotation; deltas are
		// small, so the w >= 0 representative is always the short way. This
		// also keeps the accumulated w component >= floor > 0, so the
		// normalization below can never divide by ~zero.
		double sign = a.rotationDelta.w < 0.0 ? -1.0 : 1.0;

		q[0] += sign * w * a.rotationDelta.w;
		q[1] += sign * w * a.rotationDelta.x;
		q[2] += sign * w * a.rotationDelta.y;
		q[3] += sign * w * a.rotationDelta.z;

		for (int k = 0; k < 3; ++k)
			t[k] += w * a.translationDelta[k];

		wSum += w;
	}

	double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	rotOut = { q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm };

	for (int k = 0; k < 3; ++k)
		transOut[k] = t[k] / wSum;
}

void Evaluate(const protocol::SetAlignmentField &field, const double (&basePos)[3],
              double nowSeconds, EvalState &state)
{
	vr::HmdQuaternion_t targetRot;
	double targetTrans[3];
	BlendAt(field, basePos, targetRot, targetTrans);
	SlewToward(targetRot, targetTrans, nowSeconds, FieldSlewLimits, field.generation, state);
}

void SlewToward(const vr::HmdQuaternion_t &targetRot, const double (&targetTrans)[3],
                double nowSeconds, const SlewLimits &limits, uint32_t generation,
                EvalState &state)
{
	double dt = nowSeconds - state.lastTime;
	state.lastTime = nowSeconds;

	bool snap = !state.hasCurrent
		|| state.generation != generation
		|| dt <= 0.0 || dt > limits.maxGapSeconds;

	state.hasCurrent = true;
	state.generation = generation;

	if (snap)
	{
		state.rot = targetRot;
		for (int k = 0; k < 3; ++k)
			state.trans[k] = targetTrans[k];
		return;
	}

	// Translation: cap the step length.
	{
		double step[3], len2 = 0.0;
		for (int k = 0; k < 3; ++k)
		{
			step[k] = targetTrans[k] - state.trans[k];
			len2 += step[k] * step[k];
		}
		double len = std::sqrt(len2);
		double maxStep = limits.maxTranslationPerSec * dt;
		double f = len > maxStep ? maxStep / len : 1.0;
		for (int k = 0; k < 3; ++k)
			state.trans[k] += f * step[k];
	}

	// Rotation: cap the angular step, then nlerp (both quaternions are near
	// identity and near each other, so nlerp error is negligible).
	{
		double dot = state.rot.w * targetRot.w + state.rot.x * targetRot.x
			+ state.rot.y * targetRot.y + state.rot.z * targetRot.z;
		double sign = dot < 0.0 ? -1.0 : 1.0;
		double c = sign * dot;
		if (c > 1.0)
			c = 1.0;

		double angle = 2.0 * std::acos(c);
		double maxStep = limits.maxRotationPerSec * dt;
		double f = angle > maxStep ? maxStep / angle : 1.0;

		double q[4] = {
			(1.0 - f) * state.rot.w + f * sign * targetRot.w,
			(1.0 - f) * state.rot.x + f * sign * targetRot.x,
			(1.0 - f) * state.rot.y + f * sign * targetRot.y,
			(1.0 - f) * state.rot.z + f * sign * targetRot.z,
		};
		double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
		state.rot = { q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm };
	}
}

} // namespace alignfield
