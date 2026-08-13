#pragma once

#include "../common/Protocol.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <string>

// Which driver slot gets which transform, and which slots must be retired.
//
// This is the decision half of SynchronizeDriverState, lifted out of
// Calibration.cpp so the harness can drive it. It is the only place
// baseGeneration reaches a wire message and the only owner of the conservative
// slot ledger, and both were unreachable from any test while they sat fused to
// vr::VRSystem() and IPCClient::SendBlocking. Nothing here performs I/O:
// Calibration.cpp enumerates OpenVR, calls this, and sends what comes back.
//
// Deliberately free of openvr.h. Protocol.h already picks openvr.h or
// openvr_driver.h from whichever the including translation unit carries (the
// two conflict, and the harness carries the driver one), and the device facts
// arrive as a plain enum plus strings so no OpenVR type crosses the seam.
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

// Device class reduced to the three cases that change a decision. Invalid is
// the one OpenVR reports for an id it no longer exposes, which is not the same
// as "absent": the driver slot outlives the disappearance, so it still has to
// be retired.
enum class SyncDeviceClass
{
	Invalid,
	Hmd,
	Other,
};

// One enumerated device, as the decision reads it. The "known" flags are
// separate from empty strings because a failed property read is not a device on
// an unnamed system: the first retires the slot conservatively, the second
// would be compared against the profile and could match an empty profile field.
struct SyncDevice
{
	uint32_t id = vr::k_unTrackedDeviceIndexInvalid;
	SyncDeviceClass deviceClass = SyncDeviceClass::Invalid;
	bool trackingSystemKnown = false;
	std::string trackingSystem;
	bool serialKnown = false;
	std::string serial;
};

// The live calibration, reduced to what a slot decision reads. Assembled by
// SynchronizeDriverState after its own validity gates have run, so by the time
// this is built the transform is already known finite and in range.
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
	// Snap/slew discriminator (protocol v5). CalibrationContext::SetCalibration
	// bumps it so every intentional change snaps; only continuous-calibration
	// corrections leave it alone so the driver slews. Losing this assignment
	// would route every recalibration, jump compensation and profile edit
	// through the driver's slew path, smearing a whole recalibration delta over
	// seconds of visibly drifting world.
	uint32_t baseGeneration = 0;
	bool continuousArmed = false;
	bool hideMountedTracker = false;
	// The HMD-mounted tracker's persisted identity. Ids are not stable across
	// sessions, so the tracker is re-resolved by serial on every scan.
	std::string continuousTrackerSerial;
};

enum class SlotAction
{
	None,
	// Retire whatever this slot may still be applying.
	Disable,
	ApplyTransform,
};

struct SlotDecision
{
	SlotAction action = SlotAction::None;
	// Complete wire message, valid only when action == ApplyTransform. The whole
	// message is built here rather than field-by-field at the call site so there
	// is no per-field copy for `generation` to fall out of.
	protocol::SetDeviceTransform transform;
	// Runtime device masks for this slot. False whenever the tracking system
	// could not be read, which matches the caller clearing both masks up front.
	bool referenceDevice = false;
	bool targetDevice = false;
	bool continuousTracker = false;
	// The headset reports a different tracking system than the profile's
	// reference: this is a different rig and the profile must stop being applied
	// to it. Reported rather than acted on because the caller owns ctx.enabled.
	bool disableProfile = false;
};

// Whether resolving this slot needs its serial number. Reading one is an OpenVR
// string property read per device, so the enumerator asks this before paying it
// for all 64 slots. Same predicate the decision uses, so an enumerator that
// skips a read can never disagree with a decision that expected one.
inline bool SlotNeedsSerial(const DriverSyncDesired &desired, const SyncDevice &device)
{
	return !desired.continuousTrackerSerial.empty() &&
		device.id < vr::k_unMaxTrackedDeviceCount &&
		device.id != vr::k_unTrackedDeviceIndex_Hmd &&
		device.deviceClass != SyncDeviceClass::Invalid &&
		device.trackingSystemKnown &&
		device.trackingSystem == desired.targetTrackingSystem;
}

// Conservative knowledge of driver slot state across scans, plus the decisions
// that read it.
//
// A failed batch does not erase the ledger: a request may have reached the
// driver immediately before a pipe failure, so only a confirmed disable (or a
// complete connection-wide neutralization) proves a slot is no longer live.
class DriverSlotPolicy
{
public:
	bool SlotMayBeEnabled(uint32_t id) const
	{
		// Ids reaching this loop come from an OpenVR scan and from IPC-derived
		// state; every entry point bounds-checks before indexing.
		return id < vr::k_unMaxTrackedDeviceCount && slotMayBeEnabled[id];
	}

	// A disable the driver acknowledged. The only per-slot way out of the
	// ledger: an unacknowledged one leaves the slot marked on purpose.
	void NoteSlotDisabled(uint32_t id)
	{
		if (id < vr::k_unMaxTrackedDeviceCount)
			slotMayBeEnabled[id] = false;
	}

	// A complete, acknowledged neutralization of one driver connection.
	void ForgetEverySlot()
	{
		for (bool &mayBeEnabled : slotMayBeEnabled)
			mayBeEnabled = false;
	}

	// The profile is not being applied. Converging to neutral needs no OpenVR
	// device scan at all: the ledger already names every transform that may
	// still be live, response-loss uncertainty included.
	SlotDecision DecideNeutralSlot(uint32_t id) const
	{
		SlotDecision decision;
		if (SlotMayBeEnabled(id))
			decision.action = SlotAction::Disable;
		return decision;
	}

	// The profile is being applied: decide this slot from the enumerated facts.
	//
	// Marks the ledger for an enable BEFORE returning, so the caller cannot send
	// first and mark after. That ordering looks like a bug and is not: a lost
	// response leaves an accepted enable indistinguishable from a failed
	// request, and only a slot that stays marked gets disabled when the id later
	// disappears or is reassigned to a reference-side device. Marking on success
	// instead would leave an enabled transform on a slot the overlay believes is
	// neutral, and the next device to inherit that id flies to the target
	// position with no error anywhere.
	SlotDecision DecideSlot(const DriverSyncDesired &desired, const SyncDevice &device)
	{
		SlotDecision decision;
		const uint32_t id = device.id;
		if (id >= vr::k_unMaxTrackedDeviceCount)
			return decision;

		// No class, or no tracking system: OpenVR is no longer describing this
		// device, but the driver slot survives. Retire anything a prior
		// successful or uncertain enable may have left behind.
		if (device.deviceClass == SyncDeviceClass::Invalid || !device.trackingSystemKnown)
			return DecideNeutralSlot(id);

		decision.referenceDevice =
			device.trackingSystem == desired.referenceTrackingSystem;
		decision.targetDevice = device.trackingSystem == desired.targetTrackingSystem;

		// The headset defines the reference universe; it is never a target for
		// the transform, whatever its tracking system says.
		if (id == vr::k_unTrackedDeviceIndex_Hmd)
		{
			decision.disableProfile = !decision.referenceDevice;
			if (SlotMayBeEnabled(id))
				decision.action = SlotAction::Disable;
			return decision;
		}

		// Reference-side and third-system devices must carry nothing. A device
		// that changed tracking system since the last scan is exactly the case
		// the ledger exists for.
		if (!decision.targetDevice)
		{
			if (SlotMayBeEnabled(id))
				decision.action = SlotAction::Disable;
			return decision;
		}

		decision.continuousTracker = SlotNeedsSerial(desired, device) &&
			device.serialKnown && device.serial == desired.continuousTrackerSerial;

		decision.action = SlotAction::ApplyTransform;
		decision.transform = protocol::SetDeviceTransform(id, true,
			WireVector(desired.translationMeters), WireQuaternion(desired.rotation),
			desired.scale, desired.timeShift);
		decision.transform.generation = desired.baseGeneration;
		// Only the mounted tracker is displaced out of games' reach, and only
		// while the feature is actually armed: gating on the weaker half used to
		// hide the tracker while nothing maintained the alignment.
		decision.transform.hidden = desired.continuousArmed &&
			desired.hideMountedTracker && decision.continuousTracker;

		slotMayBeEnabled[id] = true;
		return decision;
	}

private:
	bool slotMayBeEnabled[vr::k_unMaxTrackedDeviceCount] = {};
};

} // namespace questcal
