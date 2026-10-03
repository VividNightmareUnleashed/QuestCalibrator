#pragma once

#include "CalibrationRun.h"
#include "RingPoseMath.h"
#include "RingSampleGate.h"

// The flattened OpenVR headers cannot share a TU (see ChaperoneMath.h).
#if !defined(_OPENVR_API) && !defined(_OPENVR_DRIVER_API)
#include <openvr.h>
#endif

#include <vector>

// Where a calibration run's samples come from. The raw pose channel is the
// source: stamped at capture, with the driver's own velocities. A run falls
// back to runtime poses only when the channel cannot serve the selected pair:
// it is closed (its mapping could not be opened), or a selected device had no
// fresh trusted sample in it, as for a device whose driver updates its pose
// outside the hooked call (HookCoverage.h), or moved its world-from-driver
// during the check. An older driver never gets that far, since the handshake
// refuses any other protocol version; the fallback needs the driver too, to
// neutralize the selected slots, whose runtime poses may already carry the
// profile.
namespace questcal
{

// Whether the channel's batch holds a fresh (under 0.5 s old), trusted sample
// of both selected devices, each in one world-from-driver, which it records
// in the run. `reason` says which, for the log.
inline bool PreflightPoseRing(CalibrationRun &run,
	const std::vector<protocol::DevicePoseSample> &samples, double qpcNow, double qpcToSeconds,
	const char *&reason)
{
	CalibrationRun::Universe reference;
	CalibrationRun::Universe target;
	for (const auto &sample : samples)
	{
		if (!IsTrustedRingSample(sample, qpcToSeconds) ||
			!ringpose::IsFreshCaptureTime(
				RingCaptureTime(sample, qpcToSeconds), qpcNow, 0.5))
			continue;

		auto parts = UnpackRingSample(sample);
		auto *universe = sample.deviceId == run.referenceId ? &reference :
			sample.deviceId == run.targetId ? &target : nullptr;
		if (universe && !universe->Accept(parts.wfdRot, parts.wfdTrans))
		{
			reason = "selected device changed world-from-driver during preflight";
			return false;
		}
	}
	if (!reference.valid || !target.valid)
	{
		reason = !reference.valid && !target.valid ? "neither selected device had fresh trusted samples"
			: !reference.valid ? "reference had no fresh trusted samples" : "target had no fresh trusted samples";
		return false;
	}
	reason = "fresh trusted pair available";
	run.referenceUniverse = reference;
	run.targetUniverse = target;
	return true;
}

// A runtime pose as a calibration sample at `now`, the UI clock, since the
// runtime keeps no capture time. False for a pose that is not valid and
// Running_OK, or not usable (non-finite or out of range).
inline bool RuntimePoseSample(const vr::TrackedDevicePose_t &pose, double now, PoseSample &out)
{
	if (!pose.bPoseIsValid || pose.eTrackingResult != vr::TrackingResult_Running_OK)
		return false;

	const auto &m = pose.mDeviceToAbsoluteTracking.m;
	Eigen::Matrix3d rot;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			rot(i, j) = m[i][j];

	out.time = now;
	out.rot = Eigen::Quaterniond(rot).normalized();
	out.pos = Eigen::Vector3d(m[0][3], m[1][3], m[2][3]);
	out.vel = Eigen::Vector3d(pose.vVelocity.v[0], pose.vVelocity.v[1], pose.vVelocity.v[2]);
	out.angVel = Eigen::Vector3d(pose.vAngularVelocity.v[0], pose.vAngularVelocity.v[1], pose.vAngularVelocity.v[2]);
	return IsUsableComposedSample(out);
}

} // namespace questcal
