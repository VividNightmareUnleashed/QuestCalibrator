#pragma once

#include "AlignmentField.h"
#include "AtomicSnapshot.h"
#include "IPCServer.h"
#include "../common/PoseChannel.h"

#include <openvr_driver.h>

#include <atomic>
#include <cstdint>
#include <mutex>

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

	bool TrySetDeviceTransform(const protocol::SetDeviceTransform &newTransform);
	bool TrySetRuntimeState(const protocol::SetRuntimeState &newState);
	void HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t &pose);

#ifdef QUESTCAL_DRIVER_PROVIDER_TEST_SEAM
	void SetPoseTimeForTest(double seconds) { poseTimeForTest = seconds; }
#endif

private:
	// The one unwind path shared by Cleanup and every Init failure. Init must not
	// leave detours enabled behind a failure return, and the ring must not be
	// unmapped while a pose thread can still be inside the publish path.
	void Teardown();
	vr::EVRInitError FailInit(vr::EVRInitError error);

	// The envelope: fields the pose path honours whether or not calibration is
	// enabled for the device.
	struct DeviceControl
	{
		bool enabled = false;
		bool hidden = false;       // displace forwarded pose out of games' reach
		uint32_t generation = 0;   // snap/slew discriminator (see Protocol.h)
	};

	// One coherent slot read. `calibration` is meaningful only while
	// `control.enabled`; otherwise it keeps the protocol's neutral defaults
	// (identity quaternion, scale 1.0), so the slots must never be memset.
	struct DeviceTransform
	{
		DeviceControl control;
		protocol::SetDeviceTransform calibration;
	};

	// Protected by runtimeSequence: the IPC thread writes, pose callbacks read.
	// Payload scalars are individually lock-free atomic so a discarded mixed
	// generation is still race-free under the C++ memory model.
	struct TransformSlot
	{
		// Envelope - always stored, always loaded.
		std::atomic<uint32_t> enabled{ 0 };
		std::atomic<uint32_t> hidden{ 0 };
		std::atomic<uint32_t> generation{ 0 };
		// Calibration payload - loaded only while enabled. Explicit initialisers
		// (see atomicsnapshot::Double).
		questcal::atomicsnapshot::Vector3 translation;
		questcal::atomicsnapshot::Quaternion rotation;
		questcal::atomicsnapshot::Double scale{ 1.0 };
		questcal::atomicsnapshot::Double timeOffset{ 0.0 };

		void Store(const protocol::SetDeviceTransform &source) noexcept
		{
			enabled.store(source.enabled ? 1u : 0u, std::memory_order_release);
			hidden.store(source.hidden ? 1u : 0u, std::memory_order_release);
			generation.store(source.generation, std::memory_order_release);
			translation.Store(source.translation.v);
			rotation.Store(source.rotation);
			scale.store(source.scale, std::memory_order_release);
			timeOffset.store(source.timeOffset, std::memory_order_release);
		}

		DeviceTransform Load() const noexcept
		{
			DeviceTransform result;
			result.control.enabled = enabled.load(std::memory_order_acquire) != 0;
			result.control.hidden = hidden.load(std::memory_order_acquire) != 0;
			result.control.generation = generation.load(std::memory_order_acquire);
			result.calibration.enabled = result.control.enabled ? 1u : 0u;
			result.calibration.hidden = result.control.hidden ? 1u : 0u;
			result.calibration.generation = result.control.generation;
			if (!result.control.enabled)
				return result;   // payload stays at its neutral defaults

			translation.Load(result.calibration.translation.v);
			result.calibration.rotation = rotation.Load<vr::HmdQuaternion_t>();
			result.calibration.scale = scale.load(std::memory_order_acquire);
			result.calibration.timeOffset = timeOffset.load(std::memory_order_acquire);
			return result;
		}

		// Store/Load mirror the wire struct by hand; this turns a new protocol
		// field into a build break rather than a silent default on the pose path.
		static_assert(sizeof(protocol::SetDeviceTransform) == 88,
			"protocol::SetDeviceTransform changed; mirror the new field in TransformSlot::Store/Load "
			"and place it in DeviceControl or the calibration payload");
	};

	// Bounded reads fall back to the last consistent base/field pair.
	void ReadRuntimeState(uint32_t openVRID, DeviceTransform &transform,
		protocol::SetAlignmentField &field);

	// One IPC transaction publishes the base and field together. A reader must
	// never compose a base from one transaction with a field from another.
	std::atomic<uint32_t> runtimeSequence{ 0 };
	TransformSlot transforms[vr::k_unMaxTrackedDeviceCount];
	// Different devices remain fully concurrent. Same-device callbacks share
	// lastGood and both slew states, so serialize that narrow ownership domain.
	std::mutex poseMutexes[vr::k_unMaxTrackedDeviceCount];

	// Per-device fallback when a read keeps racing a write, under the matching
	// pose mutex.
	DeviceTransform lastGood[vr::k_unMaxTrackedDeviceCount];
	protocol::SetAlignmentField lastGoodField[vr::k_unMaxTrackedDeviceCount];

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
		questcal::atomicsnapshot::Double sigmaMeters{ 1.5 };   // the protocol default
		AtomicFieldAnchor anchors[protocol::SetAlignmentField::MaxAnchors];

		void Store(const protocol::SetAlignmentField &source) noexcept
		{
			enabled.store(source.enabled ? 1u : 0u, std::memory_order_release);
			generation.store(source.generation, std::memory_order_release);
			anchorCount.store(source.anchorCount, std::memory_order_release);
			sigmaMeters.store(source.sigmaMeters, std::memory_order_release);
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
			// Only validated counts (<= MaxAnchors) are ever stored, and this is one
			// atomic, so even a torn snapshot cannot read past the array.
			result.anchorCount = anchorCount.load(std::memory_order_acquire);
			result.sigmaMeters = sigmaMeters.load(std::memory_order_acquire);
			for (uint32_t i = 0; i < result.anchorCount; ++i)
				anchors[i].Load(result.anchors[i]);
			return result;
		}
	};

	AtomicAlignmentField alignmentField;

	// Per-device slew state for the field and the base transform, under the
	// matching pose mutex.
	alignfield::EvalState fieldState[vr::k_unMaxTrackedDeviceCount];
	alignfield::EvalState baseState[vr::k_unMaxTrackedDeviceCount];

	double qpcToSeconds = 0.0;
#ifdef QUESTCAL_DRIVER_PROVIDER_TEST_SEAM
	double poseTimeForTest = -1.0;
#endif

	// Publishes every raw (pre-transform) pose for the overlay's solver.
	protocol::PoseRingWriter poseRing;
	// Publishes a completed Create (in Init or a RunFrame retry) to the pose
	// threads. Publish runs only while this is set.
	std::atomic<bool> poseRingReady{ false };
	uint64_t lastPoseRingCreateAttemptMs = 0;

	// Declared last so it is destroyed first: ~IPCServer joins the pipe thread,
	// which writes into the members above, in case the DLL unloads without
	// Cleanup().
	IPCServer server;
};
