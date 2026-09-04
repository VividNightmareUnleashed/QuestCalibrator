// PCH-free on purpose: SolverTests compiles this file standalone.
#include "AlignmentField.h"
#include "PoseTransform.h"

#include <algorithm>
#include <cmath>

namespace alignfield
{

void BlendAt(const protocol::SetAlignmentField &field, const double (&basePos)[3],
             vr::HmdQuaternion_t &rotOut, double (&transOut)[3])
{
	// Blend where each transform sends the query point. Averaging translation
	// coefficients instead makes the answer depend on the playspace origin.
	double wSum = IdentityFloorWeight;
	double q[4] = { IdentityFloorWeight, 0.0, 0.0, 0.0 };   // w, x, y, z
	double p[3] = { IdentityFloorWeight * basePos[0],
		IdentityFloorWeight * basePos[1], IdentityFloorWeight * basePos[2] };

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

		auto rotated = questcal::driverpose::RotateVector(a.rotationDelta, basePos);
		for (int k = 0; k < 3; ++k)
			p[k] += w * (rotated.v[k] + a.translationDelta[k]);

		wSum += w;
	}

	double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	rotOut = { q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm };

	auto rotated = questcal::driverpose::RotateVector(rotOut, basePos);
	for (int k = 0; k < 3; ++k)
		transOut[k] = p[k] / wSum - rotated.v[k];
}

void Evaluate(const protocol::SetAlignmentField &field, const double (&basePos)[3],
              double nowSeconds, EvalState &state)
{
	vr::HmdQuaternion_t targetRot;
	double targetTrans[3];
	BlendAt(field, basePos, targetRot, targetTrans);
	SlewTowardAt(targetRot, targetTrans, basePos, nowSeconds, FieldSlewLimits, field.generation, state);
}

void SlewToward(const vr::HmdQuaternion_t &targetRot, const double (&targetTrans)[3],
                double nowSeconds, const SlewLimits &limits, uint32_t generation,
                EvalState &state)
{
	const double origin[3] = {};
	SlewTowardAt(targetRot, targetTrans, origin, nowSeconds, limits, generation, state);
}

void SlewTowardAt(const vr::HmdQuaternion_t &targetRot, const double (&targetTrans)[3],
                  const double (&position)[3], double nowSeconds,
                  const SlewLimits &limits, uint32_t generation, EvalState &state)
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

	// Limit the device's displacement, not transform coefficients: a rotation
	// about a distant pivot needs translation that cancels its orbital motion.
	auto currentPoint = questcal::driverpose::RotateVector(state.rot, position);
	auto targetPoint = questcal::driverpose::RotateVector(targetRot, position);
	double step[3], len2 = 0.0;
	for (int k = 0; k < 3; ++k)
	{
		currentPoint.v[k] += state.trans[k];
		step[k] = targetPoint.v[k] + targetTrans[k] - currentPoint.v[k];
		len2 += step[k] * step[k];
	}
	double len = std::sqrt(len2);
	double maxStep = limits.maxTranslationPerSec * dt;
	double f = len > maxStep ? maxStep / len : 1.0;

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
		double maxAngle = limits.maxRotationPerSec * dt;
		if (angle > maxAngle)
			f = std::min(f, maxAngle / angle);

		double q[4] = {
			(1.0 - f) * state.rot.w + f * sign * targetRot.w,
			(1.0 - f) * state.rot.x + f * sign * targetRot.x,
			(1.0 - f) * state.rot.y + f * sign * targetRot.y,
			(1.0 - f) * state.rot.z + f * sign * targetRot.z,
		};
		double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
		state.rot = { q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm };
	}
	auto rotated = questcal::driverpose::RotateVector(state.rot, position);
	for (int k = 0; k < 3; ++k)
		state.trans[k] = currentPoint.v[k] + f * step[k] - rotated.v[k];
}

} // namespace alignfield
