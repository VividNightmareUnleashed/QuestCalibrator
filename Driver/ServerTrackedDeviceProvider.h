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
	bool HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t &pose);

#ifdef QUESTCAL_DRIVER_PROVIDER_TEST_SEAM
	void SetPoseTimeForTest(double seconds) { poseTimeForTest = seconds; }
#endif

private:
	// The one unwind path shared by Cleanup and every Init failure. Init must not
	// leave detours enabled behind a failure return, and the ring must not be
	// unmapped while a pose thread can still be inside the publish path.
	void Teardown();
	vr::EVRInitError FailInit(vr::EVRInitError error);

	// The envelope: fields the pose path must honour whether or not calibration
	// is enabled for the device. Membership of this struct - not a field's
	// position relative to an early return - is what decides whether a value
	// survives a disabled slot, so a new protocol field is placed by answering
	// one question: does it still mean something while calibration is off?
	struct DeviceControl
	{
		bool enabled = false;
		bool hidden = false;       // displace forwarded pose out of games' reach
		uint32_t generation = 0;   // snap/slew discriminator (see Protocol.h)
	};

	// One coherent slot read. `calibration` is the wire struct itself rather
	// than a third hand-copied mirror of it; it is meaningful only while
	// `control.enabled`, and while disabled it keeps protocol's neutral defaults
	// (identity quaternion, scale 1.0). Those defaults are load-bearing - a
	// zeroed quaternion is degenerate, not neutral - so the slots must never be
	// memset. Load keeps `calibration`'s own envelope members in lockstep with
	// `control`, so the two can never disagree.
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
		// Calibration payload - loaded only while enabled. Every atomic here
		// carries an explicit initialiser: std::atomic does not value-initialise
		// before C++20 and the neutral defaults are contract.
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

		// Store/Load above are the only hand-maintained mirror of the wire struct
		// left, and this is what turns "a new protocol field silently reads as
		// its default on the pose path" into a build break. When it fires: decide
		// whether the new field is envelope (declare it in DeviceControl, store
		// and load it above the enabled check) or calibration payload (store and
		// load it below), then update the expected size. x64 is the only build
		// target, so the layout is deterministic.
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

	// Per-device fallback when a read keeps racing a write. The matching pose
	// mutex makes this a true single-writer value even if a driver dispatches
	// concurrent callbacks for one OpenVR slot.
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
		// Explicit initialiser required: std::atomic does not value-initialise
		// before C++20 and this default is the protocol's neutral sigma.
		questcal::atomicsnapshot::Double sigmaMeters{ 1.5 };
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
			uint32_t count = anchorCount.load(std::memory_order_acquire);
			if (count > protocol::SetAlignmentField::MaxAnchors)
				count = protocol::SetAlignmentField::MaxAnchors;
			result.anchorCount = count;
			result.sigmaMeters = sigmaMeters.load(std::memory_order_acquire);
			for (uint32_t i = 0; i < count; ++i)
				anchors[i].Load(result.anchors[i]);
			return result;
		}
	};

	AtomicAlignmentField alignmentField;

	// Per-device blend/slew state, under the matching pose mutex.
	alignfield::EvalState fieldState[vr::k_unMaxTrackedDeviceCount];

	// Per-device slew state for the BASE calibration transform (continuous
	// calibration, protocol v5). Same ownership argument as fieldState. An
	// unchanged generation slews small continuous corrections; a bumped
	// generation (recalibration, universe jump, edit) snaps.
	alignfield::EvalState baseState[vr::k_unMaxTrackedDeviceCount];

	double qpcToSeconds = 0.0;
#ifdef QUESTCAL_DRIVER_PROVIDER_TEST_SEAM
	double poseTimeForTest = -1.0;
#endif

	// Publishes every raw (pre-transform) pose for the overlay's solver.
	protocol::PoseRingWriter poseRing;
	// Init may encounter a stale overlay-held mapping while a prior writer is
	// disappearing. RunFrame retries on vrserver's driver frame loop with a zero
	// wait budget, so a retry never blocks that loop; this release/acquire flag
	// publishes a completed Create to the pose threads.
	std::atomic<bool> poseRingReady{ false };
	uint64_t lastPoseRingCreateAttemptMs = 0;

	// DECLARED LAST ON PURPOSE, so it is destroyed FIRST. ~IPCServer is what
	// joins the pipe thread, and that thread dispatches into the transform
	// slots, the alignment field, the last-good array and the pose ring above.
	// Declared first, it would outlive every one of them: if the DLL is ever
	// unloaded without vrserver calling Cleanup(), the IPC thread would be
	// writing into objects whose lifetimes have ended. Cleanup's explicit
	// server.Stop() remains the primary mechanism; this is the backstop.
	IPCServer server;
};
