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

// Apply the final calibration transform after base/field composition:
//
//   world' = Rcal * (scale * worldRaw) + Tcal
//
// where worldRaw = Rwfd * pDriver + Twfd. Linear derivatives share the same
// scale; angular quantities and the device-local orientation are unchanged.
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
