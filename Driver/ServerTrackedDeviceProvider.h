#pragma once

#include "AlignmentField.h"
#include "RuntimeSnapshot.h"
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

	// False when the request is refused, and then `reason` (if given) says why.
	bool TrySetDeviceTransform(const protocol::SetDeviceTransform &newTransform,
		protocol::RejectReason *reason = nullptr);
	bool TrySetRuntimeState(const protocol::SetRuntimeState &newState,
		protocol::RejectReason *reason = nullptr);
	void HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t &pose);
	// IPC-thread owned checkpoint, separate from temporary slot neutralization.
	void GetRuntimeState(protocol::Response &response) const
	{
		response.driverSessionId = driverSessionId;
		response.runtimeState = recoveryState;
	}

#ifdef QUESTCAL_DRIVER_PROVIDER_TEST_SEAM
	void SetSessionForTest(uint64_t id) { driverSessionId = id; recoveryState = {}; }
	void SetPoseTimeForTest(double seconds) { poseTimeForTest = seconds; }
#endif

private:
	// The one unwind path shared by Cleanup and every Init failure. Init must not
	// leave detours enabled behind a failure return, and the ring must not be
	// unmapped while a pose thread can still be inside the publish path.
	void Teardown();
	vr::EVRInitError FailInit(vr::EVRInitError error);

	using DeviceTransform = questcal::runtimesnapshot::DeviceTransform;
    using TransformSlot = questcal::runtimesnapshot::TransformSlot;
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
	questcal::runtimesnapshot::Snapshot lastGood[vr::k_unMaxTrackedDeviceCount];

	using AtomicAlignmentField = questcal::runtimesnapshot::AtomicAlignmentField;

	AtomicAlignmentField alignmentField;

	// Per-device slew state for the field and the base transform, under the
	// matching pose mutex.
	alignfield::EvalState fieldState[vr::k_unMaxTrackedDeviceCount];
	alignfield::EvalState baseState[vr::k_unMaxTrackedDeviceCount];
	protocol::FrameCorrection lastLoggedFrame[vr::k_unMaxTrackedDeviceCount];

	// A device's pose right after its frame correction changed, recorded by
	// its pose callback and written to the log by the IPC thread
	// (FlushFrameLog), so a pose callback never waits on the log file. The flag
	// is set only while a record waits to be written.
	struct FrameLogRecord
	{
		long long qpc = 0;
		int connected = 0;
		uint32_t generation = 0;
		vr::HmdVector3d_t inputPosition{};
		vr::HmdQuaternion_t inputRotation{};
		vr::HmdVector3d_t outputPosition{};
		vr::HmdQuaternion_t outputRotation{};
	};
	FrameLogRecord frameLog[vr::k_unMaxTrackedDeviceCount];
	std::atomic<bool> frameLogReady[vr::k_unMaxTrackedDeviceCount] = {};
	void FlushFrameLog();

	uint64_t driverSessionId = 0;
	protocol::SetRuntimeState recoveryState;
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
