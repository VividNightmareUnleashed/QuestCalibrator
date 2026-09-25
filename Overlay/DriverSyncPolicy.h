#pragma once

#include "../common/Protocol.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <string>

// Pure per-slot policy for deriving a complete driver state from an enumerated
// device and validated calibration. OpenVR facts cross this boundary as values;
// no I/O or runtime header dependency is required.
namespace questcal
{

inline vr::HmdQuaternion_t WireQuaternion(const Eigen::Quaterniond &q)
{
	vr::HmdQuaternion_t out;
	out.w = q.w();
	out.x = q.x();
	out.y = q.y();
	out.z = q.z();
	return out;
}

inline vr::HmdVector3d_t WireVector(const Eigen::Vector3d &meters)
{
	vr::HmdVector3d_t out;
	out.v[0] = meters(0);
	out.v[1] = meters(1);
	out.v[2] = meters(2);
	return out;
}

// The vr::ETrackedDeviceClass values a decision reads. Invalid is what OpenVR
// reports for an id it no longer exposes; the driver slot outlives that, so it
// still has to be retired.
enum class SyncDeviceClass
{
	Invalid,
	Hmd,
	Other,
};

// One enumerated device, as the decision reads it. A failed property read
// ("known" false) retires the slot; an empty string would instead be compared
// against the profile and could match an empty profile field.
struct SyncDevice
{
	uint32_t id = vr::k_unTrackedDeviceIndexInvalid;
	SyncDeviceClass deviceClass = SyncDeviceClass::Invalid;
	bool trackingSystemKnown = false;
	std::string trackingSystem;
	bool serialKnown = false;
	std::string serial;

	bool operator==(const SyncDevice &other) const
	{
		return id == other.id && deviceClass == other.deviceClass &&
			trackingSystemKnown == other.trackingSystemKnown &&
			trackingSystem == other.trackingSystem &&
			serialKnown == other.serialKnown && serial == other.serial;
	}
};

// The live calibration reduced to what a slot decision reads. Its transform is
// finite and bounded before this value reaches the policy.
struct DriverSyncDesired
{
	std::string referenceTrackingSystem;
	std::string targetTrackingSystem;
	Eigen::Quaterniond rotation{ 1, 0, 0, 0 };
	Eigen::Vector3d translationMeters{ 0, 0, 0 };
	double scale = 1.0;
	// Seconds added to the target devices' poseTimeOffset (runtime latency
	// re-prediction), already clamped and validated by the caller.
	double timeShift = 0.0;
	// SetDeviceTransform::generation. CalibrationContext::SetCalibration bumps
	// it for every intentional change; continuous corrections leave it alone.
	uint32_t baseGeneration = 0;
	bool continuousArmed = false;
	bool hideMountedTracker = false;
	// The HMD-mounted tracker's persisted identity. Ids are not stable across
	// sessions, so the tracker is re-resolved by serial on every scan.
	std::string continuousTrackerSerial;

	bool operator==(const DriverSyncDesired &other) const
	{
		return referenceTrackingSystem == other.referenceTrackingSystem &&
			targetTrackingSystem == other.targetTrackingSystem &&
			rotation.coeffs() == other.rotation.coeffs() &&
			translationMeters == other.translationMeters &&
			scale == other.scale && timeShift == other.timeShift &&
			baseGeneration == other.baseGeneration &&
			continuousArmed == other.continuousArmed &&
			hideMountedTracker == other.hideMountedTracker &&
			continuousTrackerSerial == other.continuousTrackerSerial;
	}
};

enum class SlotAction { None, ApplyTransform };

struct SlotDecision
{
	SlotAction action = SlotAction::None;
	// Complete wire message, valid only when action == ApplyTransform.
	protocol::SetDeviceTransform transform;
	// Runtime device masks for this slot; false when the tracking system could
	// not be read.
	bool referenceDevice = false;
	bool targetDevice = false;
	bool continuousTracker = false;
	// The headset reports a different tracking system than the profile's
	// reference. Reported rather than acted on because the caller owns
	// ctx.enabled.
	bool disableProfile = false;
};

// Whether resolving this slot needs its serial number, so the enumerator can
// skip the property read. The decision uses the same predicate.
inline bool SlotNeedsSerial(const DriverSyncDesired &desired, const SyncDevice &device)
{
	return !desired.continuousTrackerSerial.empty() &&
		device.id != vr::k_unTrackedDeviceIndex_Hmd &&
		device.deviceClass != SyncDeviceClass::Invalid &&
		device.trackingSystemKnown &&
		device.trackingSystem == desired.targetTrackingSystem;
}

// `device.id` is the slot index, below k_unMaxTrackedDeviceCount.
inline SlotDecision DecideSlot(const DriverSyncDesired &desired,
	const SyncDevice &device)
{
	SlotDecision decision;
	const uint32_t id = device.id;
	if (device.deviceClass == SyncDeviceClass::Invalid || !device.trackingSystemKnown)
		return decision;

	decision.referenceDevice =
		device.trackingSystem == desired.referenceTrackingSystem;
	decision.targetDevice = device.trackingSystem == desired.targetTrackingSystem;

	// The headset defines the reference universe; it is never a target for the
	// transform, whatever its tracking system says.
	if (id == vr::k_unTrackedDeviceIndex_Hmd)
	{
		decision.disableProfile = !decision.referenceDevice;
		return decision;
	}

	if (!decision.targetDevice)
		return decision;

	decision.continuousTracker = SlotNeedsSerial(desired, device) &&
		device.serialKnown && device.serial == desired.continuousTrackerSerial;

	decision.action = SlotAction::ApplyTransform;
	decision.transform = protocol::SetDeviceTransform(id, true,
		WireVector(desired.translationMeters), WireQuaternion(desired.rotation),
		desired.scale, desired.timeShift);
	decision.transform.generation = desired.baseGeneration;
	// An unarmed tracker must remain visible because nothing is maintaining its
	// displaced alignment.
	decision.transform.hidden = desired.continuousArmed &&
		desired.hideMountedTracker && decision.continuousTracker;
	return decision;
}

} // namespace questcal
