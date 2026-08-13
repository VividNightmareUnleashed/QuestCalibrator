#pragma once

#include "PoseScale.h"

#include <openvr_driver.h>

// Pure driver-pose transform algebra shared by vrserver's hot path and the
// synthetic test harness. Keeping this here makes the exact shipped
// composition testable without constructing an OpenVR driver context.
namespace questcal
{
namespace driverpose
{

inline vr::HmdQuaternion_t Multiply(const vr::HmdQuaternion_t &lhs,
	const vr::HmdQuaternion_t &rhs)
{
	return {
		(lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z),
		(lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
		(lhs.w * rhs.y) + (lhs.y * rhs.w) + (lhs.z * rhs.x) - (lhs.x * rhs.z),
		(lhs.w * rhs.z) + (lhs.z * rhs.w) + (lhs.x * rhs.y) - (lhs.y * rhs.x)
	};
}

inline vr::HmdVector3d_t RotateVector(const vr::HmdQuaternion_t &quat,
	const double (&vector)[3])
{
	vr::HmdQuaternion_t vectorQuat{ 0.0, vector[0], vector[1], vector[2] };
	vr::HmdQuaternion_t conjugate{ quat.w, -quat.x, -quat.y, -quat.z };
	vr::HmdQuaternion_t rotated = Multiply(Multiply(quat, vectorQuat), conjugate);
	return { rotated.x, rotated.y, rotated.z };
}

// Component-wise value helpers so the pose path can express composition as
// named steps instead of hand-written component expressions, where a transposed
// index is invisible. Inline, allocation-free and side-effect free: these run on
// vrserver's pose threads, which must never wait or allocate.
inline vr::HmdVector3d_t Add(const double (&lhs)[3], const double (&rhs)[3])
{
	return { { lhs[0] + rhs[0], lhs[1] + rhs[1], lhs[2] + rhs[2] } };
}

inline vr::HmdVector3d_t Scale(const double (&value)[3], double scale)
{
	return { { value[0] * scale, value[1] * scale, value[2] * scale } };
}

// Apply the final calibration transform after base/field composition:
//
//   world' = Rcal * (scale * worldRaw) + Tcal
//
// where worldRaw = Rwfd * pDriver + Twfd. Linear derivatives share the same
// scale; angular quantities and the device-local orientation are unchanged.
//
// DECISION - vecDriverFromHeadTranslation is deliberately NOT scaled. `scale`
// corrects a mismatch between two tracking systems' universe scales; the rigid
// offset from a device's tracked origin to its head/render origin is that
// device's real physical geometry and stays real-size. The consequence is
// recorded rather than hidden: vrserver composes
// worldFromDriver x driverPose x driverFromHead, so a device with a non-zero
// driverFromHead offset lands (1-s) times that offset away from the solver
// model - well under a millimetre at a realistic 1-2% correction, a few
// millimetres at s = 0.9. Reversing this decision means scaling the term here
// AND adding it to the solver's model; changing one without the other is a
// silent bias.
inline void Apply(vr::DriverPose_t &pose, const vr::HmdQuaternion_t &calibrationRotation,
	const double (&calibrationTranslation)[3], double scale, double timeOffset)
{
	pose.qWorldFromDriverRotation =
		Multiply(calibrationRotation, pose.qWorldFromDriverRotation);

	ScaleLinearPose(scale, pose.vecPosition, pose.vecVelocity, pose.vecAcceleration);

	double scaledWorldFromDriverTranslation[3] = {
		pose.vecWorldFromDriverTranslation[0] * scale,
		pose.vecWorldFromDriverTranslation[1] * scale,
		pose.vecWorldFromDriverTranslation[2] * scale,
	};
	vr::HmdVector3d_t rotatedTranslation =
		RotateVector(calibrationRotation, scaledWorldFromDriverTranslation);
	for (int i = 0; i < 3; ++i)
		pose.vecWorldFromDriverTranslation[i] =
			rotatedTranslation.v[i] + calibrationTranslation[i];

	pose.poseTimeOffset += timeOffset;
}

} // namespace driverpose
} // namespace questcal
