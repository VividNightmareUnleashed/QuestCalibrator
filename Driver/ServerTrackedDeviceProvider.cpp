#include "ServerTrackedDeviceProvider.h"
#include "Logging.h"
#include "InterfaceHookInjector.h"
#include "RuntimePose.h"
#include "ProtocolValidation.h"

#include <utility>

// Vertical displacement applied to a hidden device's forwarded pose.
static constexpr double HiddenPoseOffsetY = 1000.0;   // meters
static constexpr uint64_t PoseRingRetryIntervalMs = 1000;

vr::EVRInitError ServerTrackedDeviceProvider::Init(vr::IVRDriverContext *pDriverContext)
{
	TRACE("ServerTrackedDeviceProvider::Init()");
	// Pose callbacks can begin on other driver threads as soon as the global
	// host detour is enabled. Initialize every callback-visible member first.
	LARGE_INTEGER started;
	QueryPerformanceCounter(&started);
	driverSessionId = static_cast<uint64_t>(started.QuadPart);
	recoveryState = {};
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);   // never fails on XP or later
	qpcToSeconds = 1.0 / static_cast<double>(freq.QuadPart);

	bool poseRingCreated = poseRing.Create(QUESTCALIBRATOR_SHMEM_NAME);
	poseRingReady.store(poseRingCreated, std::memory_order_release);
	lastPoseRingCreateAttemptMs = GetTickCount64();
	if (!poseRingCreated)
	{
		// Non-fatal: calibration transforms still apply, but the overlay will
		// fall back to runtime-predicted poses instead of raw driver poses.
		LOG("Failed to create pose ring shared memory (error %u)", GetLastError());
	}

	// Install the context detour before OpenVR initializes its cached interfaces.
	// InitServerDriverContext requests IVRServerDriverHost_006 through the
	// detour, so the check below proves a pose hook exists, not that every
	// device driver is routed through it: a driver that resolved its host
	// interface before this detour existed forwards poses untouched. Only the
	// `01questcalibrator` manifest name orders this driver first.
	if (!InjectHooks(this, pDriverContext))
		return FailInit(vr::VRInitError_Driver_Failed);

	vr::EVRInitError contextError = vr::InitServerDriverContext(pDriverContext);
	if (contextError != vr::VRInitError_None)
		return FailInit(contextError);

	if (!IsPoseUpdateHookInstalled())
	{
		LOG("No supported IVRServerDriverHost pose hook was installed");
		return FailInit(vr::VRInitError_Driver_Failed);
	}

	IPCServer::RequestSink sink;
	sink.setDeviceTransform = [this](const protocol::SetDeviceTransform &transform)
	{
		protocol::RejectReason reason = protocol::RejectReason::None;
		TrySetDeviceTransform(transform, &reason);
		return reason;
	};
	sink.setRuntimeState = [this](const protocol::SetRuntimeState &state)
	{
		protocol::RejectReason reason = protocol::RejectReason::None;
		TrySetRuntimeState(state, &reason);
		return reason;
	};
	sink.poseHookMask = [] { return PoseUpdateHookMask(); };
	sink.getRuntimeState = [this](protocol::Response &response) { GetRuntimeState(response); };
	if (!server.Run(std::move(sink)))
	{
		LOG("IPC server could not establish its control listener");
		return FailInit(vr::VRInitError_Driver_Failed);
	}

	return vr::VRInitError_None;
}

void ServerTrackedDeviceProvider::Teardown()
{
	server.Stop();
	// Hooks may be live even when Init failed (InjectHooks refuses while a
	// previous Init's hooks remain). Wait for the detours to leave the driver
	// before the ring goes away, or a pose thread still publishing would write
	// into an unmapped view.
	const bool released = DisableHooks();
	poseRingReady.store(false, std::memory_order_release);
	// The IPC thread has stopped; write what it would have.
	FlushFrameLog();
	if (released)
		poseRing.Close();
	else
	{
		// A pose thread may still reach the ring, so leak the mapping; the next
		// writer's PID + creation-time liveness proof covers an owner that never
		// closes.
		LOG("The pose ring mapping is retained for the pose callback still inside the driver");
	}
}

vr::EVRInitError ServerTrackedDeviceProvider::FailInit(vr::EVRInitError error)
{
	Teardown();
	vr::CleanupDriverContext();
	return error;
}

void ServerTrackedDeviceProvider::Cleanup()
{
	TRACE("ServerTrackedDeviceProvider::Cleanup()");
	Teardown();
	VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

void ServerTrackedDeviceProvider::RunFrame()
{
	if (poseRingReady.load(std::memory_order_acquire))
		return;

	uint64_t now = GetTickCount64();
	if (now - lastPoseRingCreateAttemptMs < PoseRingRetryIntervalMs)
		return;
	// Zero wait budget: this is vrserver's driver frame loop, shared with every
	// other driver.
	if (poseRing.Create(QUESTCALIBRATOR_SHMEM_NAME, 0))
	{
		poseRingReady.store(true, std::memory_order_release);
		LOG("Pose ring shared memory became available after retry");
	}
	// Stamped after the attempt so a slow failure does not eat its own interval.
	lastPoseRingCreateAttemptMs = GetTickCount64();
}

void ServerTrackedDeviceProvider::FlushFrameLog()
{
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		if (!frameLogReady[id].load(std::memory_order_acquire))
			continue;
		const FrameLogRecord record = frameLog[id];
		frameLogReady[id].store(false, std::memory_order_release);
		if (!LogFile)
			continue;
		LOG("frame applied device %u QPC %lld connected %d: raw p %.6f %.6f %.6f q %.6f %.6f %.6f %.6f; "
			"output before hiding p %.6f %.6f %.6f q %.6f %.6f %.6f %.6f (base generation %u)",
			id, record.qpc, record.connected,
			record.inputPosition.v[0], record.inputPosition.v[1], record.inputPosition.v[2],
			record.inputRotation.w, record.inputRotation.x, record.inputRotation.y, record.inputRotation.z,
			record.outputPosition.v[0], record.outputPosition.v[1], record.outputPosition.v[2],
			record.outputRotation.w, record.outputRotation.x, record.outputRotation.y, record.outputRotation.z,
			record.generation);
	}
}

bool ServerTrackedDeviceProvider::TrySetDeviceTransform(const protocol::SetDeviceTransform &newTransform,
	protocol::RejectReason *reason)
{
	FlushFrameLog();
	protocol::SetDeviceTransform sanitized;
	if (!questcal::driverinput::ValidateAndSanitize(newTransform, sanitized))
	{
		LOG("SetDeviceTransform: rejected invalid transform for device id %u", newTransform.openVRID);
		if (reason)
			*reason = protocol::RejectReason::InvalidValues;
		return false;
	}
	if (reason)
		*reason = protocol::RejectReason::None;

	auto &slot = transforms[sanitized.openVRID];
	// The IPC thread is the single writer. Odd means a coherent generation is
	// in flight; payload stores are atomic so a failed reader attempt is safe.
	runtimeSequence.fetch_add(1, std::memory_order_acq_rel);
	slot.Store(sanitized);
	runtimeSequence.fetch_add(1, std::memory_order_release);
	return true;
}

bool ServerTrackedDeviceProvider::TrySetRuntimeState(const protocol::SetRuntimeState &newState,
	protocol::RejectReason *reason)
{
	// The overlay sends its complete state about once a second, so a frame
	// change reaches the log within about that.
	FlushFrameLog();
	if (newState.expectedSessionId != 0 && newState.expectedSessionId != driverSessionId)
	{
		if (reason)
			*reason = protocol::RejectReason::StaleSession;
		return false;
	}
	protocol::SetRuntimeState sanitized;
	if (!questcal::driverinput::ValidateAndSanitize(newState, sanitized))
	{
		LOG("SetRuntimeState: rejected invalid state (enabled=%llx, hidden=%llx, anchors=%u)",
			static_cast<unsigned long long>(newState.enabledMask),
			static_cast<unsigned long long>(newState.hiddenMask),
			newState.field.anchorCount);
		if (reason)
			*reason = protocol::RejectReason::InvalidValues;
		return false;
	}
	if (reason)
		*reason = protocol::RejectReason::None;

	if (sanitized.enabledMask != 0 && sanitized.frameProfileKey != 0)
		recoveryState = sanitized;
	runtimeSequence.fetch_add(1, std::memory_order_acq_rel);
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		protocol::SetDeviceTransform transform = sanitized.transform;
		transform.openVRID = id;
		transform.enabled = (sanitized.enabledMask >> id) & 1;
		transform.hidden = (sanitized.hiddenMask >> id) & 1;
		auto &slot = transforms[id];
		slot.Store(transform, sanitized.frames[id]);
	}

	alignmentField.Store(sanitized.field);
	runtimeSequence.fetch_add(1, std::memory_order_release);
	return true;
}

void ServerTrackedDeviceProvider::ReadRuntimeState(uint32_t openVRID,
	DeviceTransform &out, protocol::SetAlignmentField &field)
{
    const auto snapshot = questcal::runtimesnapshot::Read(runtimeSequence,
        transforms[openVRID], alignmentField, lastGood[openVRID]);
    out = snapshot.transform;
    field = snapshot.field;
}

void ServerTrackedDeviceProvider::HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t &pose)
{
	if (openVRID >= vr::k_unMaxTrackedDeviceCount)
		return;
	std::lock_guard<std::mutex> poseLock(poseMutexes[openVRID]);

	// Publish the raw driver-space pose for the solver, stamped at capture.
	//
	// INVARIANT: this publish must stay ABOVE every pose mutation below,
	// including the poseTimeOffset shift. If the ring recorded rewritten poses,
	// the next solve would find ~zero residual offset and the full-replace
	// transform would erase the latency correction on every recalibration.
	LARGE_INTEGER now;
	QueryPerformanceCounter(&now);

	if (poseRingReady.load(std::memory_order_acquire))
	{
		protocol::DevicePoseSample sample;
		sample.sampleTimeQpc = now.QuadPart;
		sample.deviceId = openVRID;
		sample.trackingResult = static_cast<uint32_t>(pose.result);
		sample.poseIsValid = pose.poseIsValid;
		sample.deviceIsConnected = pose.deviceIsConnected;
		sample.poseTimeOffset = pose.poseTimeOffset;
		sample.worldFromDriverRotation = pose.qWorldFromDriverRotation;
		sample.rotation = pose.qRotation;
		for (int i = 0; i < 3; ++i)
		{
			sample.worldFromDriverTranslation[i] = pose.vecWorldFromDriverTranslation[i];
			sample.position[i] = pose.vecPosition[i];
			sample.velocity[i] = pose.vecVelocity[i];
			sample.angularVelocity[i] = pose.vecAngularVelocity[i];
		}
		poseRing.Publish(sample);
	}

	DeviceTransform tf;
	protocol::SetAlignmentField field;
	ReadRuntimeState(openVRID, tf, field);
	const bool logFrame = LogFile && tf.control.enabled && pose.poseIsValid &&
		!(lastLoggedFrame[openVRID] == tf.frame) &&
		!frameLogReady[openVRID].load(std::memory_order_acquire);
	vr::HmdVector3d_t inputPosition{};
	vr::HmdQuaternion_t inputRotation{};
	if (logFrame)
	{
		const auto rotated = questcal::driverpose::RotateVector(pose.qWorldFromDriverRotation, pose.vecPosition);
		inputPosition = questcal::driverpose::Add(rotated.v, pose.vecWorldFromDriverTranslation);
		inputRotation = questcal::driverpose::Multiply(pose.qWorldFromDriverRotation, pose.qRotation);
	}

	double nowSeconds = static_cast<double>(now.QuadPart) * qpcToSeconds;
#ifdef QUESTCAL_DRIVER_PROVIDER_TEST_SEAM
	if (poseTimeForTest >= 0.0)
		nowSeconds = poseTimeForTest;
#endif

	questcal::driverpose::ApplyRuntimePose(pose, tf.calibration, tf.frame,
        field, nowSeconds, baseState[openVRID], fieldState[openVRID]);

	if (logFrame)
	{
		// Written by the IPC thread on its next request (FlushFrameLog).
		auto &record = frameLog[openVRID];
		const auto rotated = questcal::driverpose::RotateVector(pose.qWorldFromDriverRotation, pose.vecPosition);
		record.qpc = static_cast<long long>(now.QuadPart);
		record.connected = pose.deviceIsConnected;
		record.generation = tf.control.generation;
		record.inputPosition = inputPosition;
		record.inputRotation = inputRotation;
		record.outputPosition = questcal::driverpose::Add(rotated.v, pose.vecWorldFromDriverTranslation);
		record.outputRotation = questcal::driverpose::Multiply(pose.qWorldFromDriverRotation, pose.qRotation);
		frameLogReady[openVRID].store(true, std::memory_order_release);
		lastLoggedFrame[openVRID] = tf.frame;
	}

	// Hide the HMD-mounted continuous-calibration tracker from applications:
	// it keeps a valid pose (an invalid one would make SteamVR flag it lost) but
	// far above the play area, out of reach of games' tracker auto-assignment.
	if (tf.control.hidden)
		pose.vecWorldFromDriverTranslation[1] += HiddenPoseOffsetY;
}
