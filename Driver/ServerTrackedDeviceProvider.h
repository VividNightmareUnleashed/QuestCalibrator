#pragma once

#include "AlignmentField.h"
#include "AtomicSnapshot.h"
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
	virtual void RunFrame() override;

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
	bool TrySetDeviceTransform(const protocol::SetDeviceTransform &newTransform);
	bool TrySetAlignmentField(const protocol::SetAlignmentField &newField);
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
	// Payload scalars are individually lock-free atomic so a discarded mixed
	// generation is still race-free under the C++ memory model.
	struct TransformSlot
	{
		std::atomic<uint32_t> sequence{ 0 };
		std::atomic<uint32_t> enabled{ 0 };
		questcal::atomicsnapshot::Vector3 translation;
		questcal::atomicsnapshot::Quaternion rotation;
		questcal::atomicsnapshot::Double scale{ 1.0 };
		questcal::atomicsnapshot::Double timeOffset;
		std::atomic<uint32_t> generation{ 0 };
		std::atomic<uint32_t> hidden{ 0 };

		void Store(const DeviceTransform &source) noexcept
		{
			enabled.store(source.enabled ? 1u : 0u, std::memory_order_release);
			translation.Store(source.translation.v);
			rotation.Store(source.rotation);
			scale.Store(source.scale);
			timeOffset.Store(source.timeOffset);
			generation.store(source.generation, std::memory_order_release);
			hidden.store(source.hidden ? 1u : 0u, std::memory_order_release);
		}

		DeviceTransform Load() const noexcept
		{
			DeviceTransform result;
			result.enabled = enabled.load(std::memory_order_acquire) != 0;
			// Hidden is independent of calibration enablement, so it remains part
			// of every coherent snapshot. The other payload is irrelevant while
			// disabled and DeviceTransform's neutral defaults are load-bearing.
			result.hidden = hidden.load(std::memory_order_acquire) != 0;
			if (!result.enabled)
				return result;

			translation.Load(result.translation.v);
			result.rotation = rotation.Load<vr::HmdQuaternion_t>();
			result.scale = scale.Load();
			result.timeOffset = timeOffset.Load();
			result.generation = generation.load(std::memory_order_acquire);
			return result;
		}
	};

	// Returns false if a consistent snapshot could not be taken (racing writes);
	// `out` then holds the last consistent snapshot instead.
	bool ReadDeviceTransform(uint32_t openVRID, DeviceTransform &out);

	TransformSlot transforms[vr::k_unMaxTrackedDeviceCount];

	// Per-device fallback when a read keeps racing a write. Poses for a given
	// device always arrive on that device driver's own thread, so each entry has
	// a single effective writer even though different devices update concurrently.
	DeviceTransform lastGood[vr::k_unMaxTrackedDeviceCount];

	struct AtomicFieldAnchor
	{
		questcal::atomicsnapshot::Vector3 position;
		questcal::atomicsnapshot::Quaternion rotationDelta;
		questcal::atomicsnapshot::Vector3 translationDelta;

		void Store(const protocol::FieldAnchor &source) noexcept
		{
			position.Store(source.position);
			rotationDelta.Store(source.rotationDelta);
			translationDelta.Store(source.translationDelta);
		}

		void Load(protocol::FieldAnchor &destination) const noexcept
		{
			position.Load(destination.position);
			destination.rotationDelta = rotationDelta.Load<vr::HmdQuaternion_t>();
			translationDelta.Load(destination.translationDelta);
		}
	};

	struct AtomicAlignmentField
	{
		std::atomic<uint32_t> enabled{ 0 };
		std::atomic<uint32_t> generation{ 0 };
		std::atomic<uint32_t> anchorCount{ 0 };
		questcal::atomicsnapshot::Double sigmaMeters{ 1.5 };
		AtomicFieldAnchor anchors[protocol::SetAlignmentField::MaxAnchors];

		void Store(const protocol::SetAlignmentField &source) noexcept
		{
			enabled.store(source.enabled ? 1u : 0u, std::memory_order_release);
			generation.store(source.generation, std::memory_order_release);
			anchorCount.store(source.anchorCount, std::memory_order_release);
			sigmaMeters.Store(source.sigmaMeters);
			for (uint32_t i = 0; i < protocol::SetAlignmentField::MaxAnchors; ++i)
				anchors[i].Store(source.anchors[i]);
		}

		protocol::SetAlignmentField Load() const noexcept
		{
			protocol::SetAlignmentField result;
			result.enabled = enabled.load(std::memory_order_acquire) != 0;
			if (!result.enabled)
				return result;

			result.generation = generation.load(std::memory_order_acquire);
			uint32_t count = anchorCount.load(std::memory_order_acquire);
			if (count > protocol::SetAlignmentField::MaxAnchors)
				count = protocol::SetAlignmentField::MaxAnchors;
			result.anchorCount = count;
			result.sigmaMeters = sigmaMeters.Load();
			for (uint32_t i = 0; i < count; ++i)
				anchors[i].Load(result.anchors[i]);
			return result;
		}
	};

	// Spatial correction field (protocol v4). Stored under its own seqlock;
	// the pose path blends the anchor deltas per device (see AlignmentField.h).
	struct AlignmentFieldSlot
	{
		std::atomic<uint32_t> sequence{ 0 };
		AtomicAlignmentField field;
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
	// Init may encounter a stale overlay-held mapping while a prior writer is
	// disappearing. RunFrame retries off the latency-sensitive pose threads;
	// this release/acquire flag publishes a completed Create to those threads.
	std::atomic<bool> poseRingReady{ false };
	uint64_t lastPoseRingCreateAttemptMs = 0;
};
