#pragma once

#include "AlignmentField.h"
#include "IPCServer.h"
#include "../common/PoseChannel.h"

#include <openvr_driver.h>

#include <atomic>
#include <cstdint>

class ServerTrackedDeviceProvider : public vr::IServerTrackedDeviceProvider
{
public:
	////// Start vr::IServerTrackedDeviceProvider functions

	/** initializes the driver. This will be called before any other methods are called. */
	virtual vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext) override;

	/** cleans up the driver right before it is unloaded */
	virtual void Cleanup() override;

	/** Returns the version of the ITrackedDeviceServerDriver interface used by this driver */
	virtual const char * const *GetInterfaceVersions() { return vr::k_InterfaceVersions; }

	/** Allows the driver do to some work in the main loop of the server. */
	virtual void RunFrame() { }

	/** Returns true if the driver wants to block Standby mode. */
	virtual bool ShouldBlockStandbyMode() { return false; }

	/** Called when the system is entering Standby mode. The driver should switch itself into whatever sort of low-power
	* state it has. */
	virtual void EnterStandby() { }

	/** Called when the system is leaving Standby mode. The driver should switch itself back to
	full operation. */
	virtual void LeaveStandby() { }

	////// End vr::IServerTrackedDeviceProvider functions

	ServerTrackedDeviceProvider() : server(this) { }
	void SetDeviceTransform(const protocol::SetDeviceTransform &newTransform);
	void SetAlignmentField(const protocol::SetAlignmentField &newField);
	bool HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t &pose);

private:
	IPCServer server;

	// A zeroed quaternion or scale is a degenerate transform, not a neutral one:
	// these defaults are load-bearing and the slots must never be memset.
	struct DeviceTransform
	{
		bool enabled = false;
		vr::HmdVector3d_t translation{ { 0.0, 0.0, 0.0 } };
		vr::HmdQuaternion_t rotation{ 1.0, 0.0, 0.0, 0.0 };
		double scale = 1.0;
		double timeOffset = 0.0;   // seconds added to poseTimeOffset (see Protocol.h)
		uint32_t generation = 0;   // snap/slew discriminator (see Protocol.h)
		bool hidden = false;       // displace forwarded pose out of games' reach
	};

	// Seqlock-protected slot: the IPC pipe thread writes, vrserver's pose thread
	// reads on every pose update. Sequence is even when stable, odd mid-write.
	struct TransformSlot
	{
		std::atomic<uint32_t> sequence{ 0 };
		DeviceTransform transform;
	};

	// Returns false if a consistent snapshot could not be taken (racing writes);
	// `out` then holds the last consistent snapshot instead.
	bool ReadDeviceTransform(uint32_t openVRID, DeviceTransform &out);

	TransformSlot transforms[vr::k_unMaxTrackedDeviceCount];

	// Per-device fallback when a read keeps racing a write. Poses for a given
	// device always arrive on that device driver's own thread, so each entry has
	// a single effective writer even though different devices update concurrently.
	DeviceTransform lastGood[vr::k_unMaxTrackedDeviceCount];

	// Spatial correction field (protocol v4). Stored under its own seqlock;
	// the pose path blends the anchor deltas per device (see AlignmentField.h).
	struct AlignmentFieldSlot
	{
		std::atomic<uint32_t> sequence{ 0 };
		protocol::SetAlignmentField field;
	};
	AlignmentFieldSlot alignmentField;

	// Returns false if a consistent snapshot could not be taken; the caller
	// then keeps the device's previously applied delta for this frame.
	bool ReadAlignmentField(protocol::SetAlignmentField &out);

	// Per-device blend/slew state, owned by that device's pose thread (same
	// single-writer argument as lastGood).
	alignfield::EvalState fieldState[vr::k_unMaxTrackedDeviceCount];

	// Per-device slew state for the BASE calibration transform (continuous
	// calibration, protocol v5). Same ownership argument as fieldState. An
	// unchanged generation slews small continuous corrections; a bumped
	// generation (recalibration, universe jump, edit) snaps.
	alignfield::EvalState baseState[vr::k_unMaxTrackedDeviceCount];

	double qpcToSeconds = 0.0;

	// Publishes every raw (pre-transform) pose for the overlay's solver.
	protocol::PoseRingWriter poseRing;
};
