#include "ServerTrackedDeviceProvider.h"
#include "Logging.h"
#include "InterfaceHookInjector.h"
#include "PoseTransform.h"
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
		return TrySetDeviceTransform(transform);
	};
	sink.setRuntimeState = [this](const protocol::SetRuntimeState &state)
	{
		return TrySetRuntimeState(state);
	};
	sink.poseHookMask = [] { return PoseUpdateHookMask(); };
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
	// previous Init's hooks remain). Quiesce the detours before the ring goes
	// away, or a pose thread still publishing would write into an unmapped view.
	bool quiesced = DisableHooks();
	poseRingReady.store(false, std::memory_order_release);
	if (quiesced)
		poseRing.Close();
	else
	{
		// A pose thread may still reach the ring, so leak the mapping; the next
		// writer's PID + creation-time liveness proof covers an owner that never
		// closes.
		LOG("Hook quiescence could not be proven; the pose ring mapping is retained");
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

bool ServerTrackedDeviceProvider::TrySetDeviceTransform(const protocol::SetDeviceTransform &newTransform)
{
	protocol::SetDeviceTransform sanitized;
	if (!questcal::driverinput::ValidateAndSanitize(newTransform, sanitized))
	{
		LOG("SetDeviceTransform: rejected invalid transform for device id %u", newTransform.openVRID);
		return false;
	}

	auto &slot = transforms[sanitized.openVRID];
	// The IPC thread is the single writer. Odd means a coherent generation is
	// in flight; payload stores are atomic so a failed reader attempt is safe.
	runtimeSequence.fetch_add(1, std::memory_order_acq_rel);
	slot.Store(sanitized);
	runtimeSequence.fetch_add(1, std::memory_order_release);
	return true;
}

bool ServerTrackedDeviceProvider::TrySetRuntimeState(const protocol::SetRuntimeState &newState)
{
	protocol::SetRuntimeState sanitized;
	if (!questcal::driverinput::ValidateAndSanitize(newState, sanitized))
	{
		LOG("SetRuntimeState: rejected invalid state (enabled=%llx, hidden=%llx, anchors=%u)",
			static_cast<unsigned long long>(newState.enabledMask),
			static_cast<unsigned long long>(newState.hiddenMask),
			newState.field.anchorCount);
		return false;
	}

	runtimeSequence.fetch_add(1, std::memory_order_acq_rel);
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		protocol::SetDeviceTransform transform = sanitized.transform;
		transform.openVRID = id;
		transform.enabled = (sanitized.enabledMask >> id) & 1;
		transform.hidden = (sanitized.hiddenMask >> id) & 1;
		auto &slot = transforms[id];
		slot.Store(transform);
	}

	alignmentField.Store(sanitized.field);
	runtimeSequence.fetch_add(1, std::memory_order_release);
	return true;
}

void ServerTrackedDeviceProvider::ReadRuntimeState(uint32_t openVRID,
	DeviceTransform &out, protocol::SetAlignmentField &field)
{
	auto &slot = transforms[openVRID];

	for (int attempt = 0; attempt < 8; ++attempt)
	{
		uint32_t before = runtimeSequence.load(std::memory_order_acquire);
		if (before & 1)
			continue;

		out = slot.Load();
		field = alignmentField.Load();

		uint32_t after = runtimeSequence.load(std::memory_order_acquire);
		if (before == after)
		{
			lastGood[openVRID] = out;
			lastGoodField[openVRID] = field;
			return;
		}
	}

	// A writer kept racing us; use the last consistent snapshot rather than
	// stalling vrserver's pose thread.
	out = lastGood[openVRID];
	field = lastGoodField[openVRID];
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

	double nowSeconds = static_cast<double>(now.QuadPart) * qpcToSeconds;
#ifdef QUESTCAL_DRIVER_PROVIDER_TEST_SEAM
	if (poseTimeForTest >= 0.0)
		nowSeconds = poseTimeForTest;
#endif

	if (tf.control.enabled)
	{
		const protocol::SetDeviceTransform &cal = tf.calibration;
		vr::HmdVector3d_t scaledPosition = questcal::driverpose::Scale(pose.vecPosition, cal.scale);
		vr::HmdVector3d_t rotatedPosition = questcal::driverpose::RotateVector(
			pose.qWorldFromDriverRotation, scaledPosition.v);
		vr::HmdVector3d_t scaledOrigin = questcal::driverpose::Scale(
			pose.vecWorldFromDriverTranslation, cal.scale);
		vr::HmdVector3d_t rawWorld = questcal::driverpose::Add(rotatedPosition.v, scaledOrigin.v);
		bool usablePosition = pose.poseIsValid && pose.deviceIsConnected &&
			questcal::numeric::IsBoundedVector3(rawWorld.v,
				cal.scale * protocol::limits::MaxAbsPosePositionMeters);

		// Base-calibration slew or snap, by generation (see Protocol.h). A read
		// that fell back to lastGood may briefly carry an older generation; the
		// next consistent read then snaps to where it was slewing, which is benign.
		auto &bs = baseState[openVRID];
		if (usablePosition)
			alignfield::SlewTowardAt(cal.rotation, cal.translation.v, rawWorld.v, nowSeconds,
				alignfield::BaseSlewLimits, tf.control.generation, bs);
		else if (!bs.hasCurrent)
			alignfield::SlewToward(cal.rotation, cal.translation.v, nowSeconds,
				alignfield::BaseSlewLimits, tf.control.generation, bs);
		// Invalid positions must not enter persistent smoothing state. Hold the
		// previous transform and clock; recovery after a long gap snaps normally.
		vr::HmdQuaternion_t baseRot = bs.rot;
		vr::HmdVector3d_t baseTrans{ { bs.trans[0], bs.trans[1], bs.trans[2] } };

		// Spatial correction field: blend the anchor deltas at this device's
		// own base-calibrated position and left-compose the result onto the
		// base calibration (world = delta(base(raw))). Blending on the
		// device's own position keeps a static tripod tracker static while
		// the user walks, and needs no cross-thread HMD position cache.
		vr::HmdQuaternion_t calRot = baseRot;
		vr::HmdVector3d_t calTrans = baseTrans;

		auto &fs = fieldState[openVRID];
		if (!field.enabled || field.anchorCount == 0)
		{
			fs.hasCurrent = false;   // re-enabling later snaps
		}
		else if (usablePosition)
		{
			vr::HmdVector3d_t rotatedRawWorld =
				questcal::driverpose::RotateVector(baseRot, rawWorld.v);
			vr::HmdVector3d_t basePos =
				questcal::driverpose::Add(rotatedRawWorld.v, baseTrans.v);

			alignfield::Evaluate(field, basePos.v, nowSeconds, fs);
		}
		// Invalid pose: keep the previous delta without advancing the
		// slew clock; Evaluate's gap check snaps after a long loss.

		if (fs.hasCurrent)
		{
			calRot = questcal::driverpose::Multiply(fs.rot, baseRot);
			vr::HmdVector3d_t rotatedBase =
				questcal::driverpose::RotateVector(fs.rot, baseTrans.v);
			calTrans = questcal::driverpose::Add(rotatedBase.v, fs.trans);
		}

		// Apply the exact solver model to the composed raw-world pose, including
		// scale on worldFromDriver translation and every linear derivative.
		questcal::driverpose::Apply(
			pose, calRot, calTrans.v, cal.scale, cal.timeOffset);
	}
	else
	{
		// Re-enabling later must snap, not slew from a stale state.
		baseState[openVRID].hasCurrent = false;
		fieldState[openVRID].hasCurrent = false;
	}

	// Hide the HMD-mounted continuous-calibration tracker from applications:
	// it keeps a valid pose (an invalid one would make SteamVR flag it lost) but
	// far above the play area, out of reach of games' tracker auto-assignment.
	if (tf.control.hidden)
		pose.vecWorldFromDriverTranslation[1] += HiddenPoseOffsetY;
}
