#include "ServerTrackedDeviceProvider.h"
#include "Logging.h"
#include "InterfaceHookInjector.h"
#include "PoseTransform.h"
#include "ProtocolValidation.h"

// Vertical displacement applied to a hidden device's forwarded pose.
static constexpr double HiddenPoseOffsetY = 1000.0;   // meters
static constexpr uint64_t PoseRingRetryIntervalMs = 1000;

vr::EVRInitError ServerTrackedDeviceProvider::Init(vr::IVRDriverContext *pDriverContext)
{
	TRACE("ServerTrackedDeviceProvider::Init()");
	// Pose callbacks can begin on other driver threads as soon as the global
	// host detour is enabled. Initialize every callback-visible member first.
	LARGE_INTEGER freq;
	if (!QueryPerformanceFrequency(&freq) || freq.QuadPart <= 0)
	{
		LOG("QueryPerformanceFrequency failed (error %u)", GetLastError());
		return vr::VRInitError_Driver_Failed;
	}
	qpcToSeconds = 1.0 / static_cast<double>(freq.QuadPart);

	lastPoseRingCreateAttemptMs = GetTickCount64();
	bool poseRingCreated = poseRing.Create(QUESTCALIBRATOR_SHMEM_NAME);
	poseRingReady.store(poseRingCreated, std::memory_order_release);
	if (!poseRingCreated)
	{
		// Non-fatal: calibration transforms still apply, but the overlay will
		// fall back to runtime-predicted poses instead of raw driver poses.
		LOG("Failed to create pose ring shared memory (error %u)", GetLastError());
	}

	// Install the context detour before OpenVR initializes its cached interfaces.
	// InitServerDriverContext requests IVRServerDriverHost through the detour, so
	// a successful return proves that one of the supported 005/006 pose hooks is
	// active rather than letting the overlay handshake with a no-op driver.
	if (!InjectHooks(this, pDriverContext))
	{
		poseRing.Close();
		return vr::VRInitError_Driver_Failed;
	}

	vr::EVRInitError contextError = vr::InitServerDriverContext(pDriverContext);
	if (contextError != vr::VRInitError_None)
	{
		DisableHooks();
		poseRing.Close();
		vr::CleanupDriverContext();
		return contextError;
	}

	if (!IsPoseUpdateHookInstalled())
	{
		LOG("No supported IVRServerDriverHost pose hook was installed");
		DisableHooks();
		poseRing.Close();
		vr::CleanupDriverContext();
		return vr::VRInitError_Driver_Failed;
	}

	if (!server.Run())
	{
		LOG("IPC server could not establish its control listener");
		server.Stop();
		DisableHooks();
		poseRing.Close();
		vr::CleanupDriverContext();
		return vr::VRInitError_Driver_Failed;
	}

	return vr::VRInitError_None;
}

void ServerTrackedDeviceProvider::Cleanup()
{
	TRACE("ServerTrackedDeviceProvider::Cleanup()");
	server.Stop();
	DisableHooks();
	poseRingReady.store(false, std::memory_order_release);
	poseRing.Close();
	VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

void ServerTrackedDeviceProvider::RunFrame()
{
	if (poseRingReady.load(std::memory_order_acquire))
		return;

	uint64_t now = GetTickCount64();
	if (now - lastPoseRingCreateAttemptMs < PoseRingRetryIntervalMs)
		return;
	lastPoseRingCreateAttemptMs = now;
	if (poseRing.Create(QUESTCALIBRATOR_SHMEM_NAME))
	{
		poseRingReady.store(true, std::memory_order_release);
		LOG("Pose ring shared memory became available after retry");
	}
}

bool ServerTrackedDeviceProvider::TrySetDeviceTransform(const protocol::SetDeviceTransform &newTransform)
{
	protocol::SetDeviceTransform sanitized;
	if (!questcal::driverinput::ValidateAndSanitize(newTransform, sanitized))
	{
		LOG("SetDeviceTransform: rejected invalid transform for device id %u", newTransform.openVRID);
		return false;
	}

	DeviceTransform tf;
	tf.enabled = sanitized.enabled;
	tf.translation = sanitized.translation;
	tf.rotation = sanitized.rotation;
	tf.scale = sanitized.scale;
	tf.timeOffset = sanitized.timeOffset;
	tf.generation = sanitized.generation;
	tf.hidden = sanitized.hidden;

	auto &slot = transforms[sanitized.openVRID];
	// The IPC thread is the single writer. Odd means a coherent generation is
	// in flight; payload stores are atomic so a failed reader attempt is safe.
	slot.sequence.fetch_add(1, std::memory_order_acq_rel);
	slot.Store(tf);
	slot.sequence.fetch_add(1, std::memory_order_release);
	return true;
}

bool ServerTrackedDeviceProvider::TrySetAlignmentField(const protocol::SetAlignmentField &newField)
{
	protocol::SetAlignmentField sanitized;
	if (!questcal::driverinput::ValidateAndSanitize(newField, sanitized))
	{
		LOG("SetAlignmentField: rejected invalid field (enabled=%d, anchors=%u)",
			newField.enabled ? 1 : 0, newField.anchorCount);
		return false;
	}

	// Same seqlock discipline as the transform slots: IPC thread writes, pose
	// threads read.
	alignmentField.sequence.fetch_add(1, std::memory_order_acq_rel);
	alignmentField.field.Store(sanitized);
	alignmentField.sequence.fetch_add(1, std::memory_order_release);
	return true;
}

bool ServerTrackedDeviceProvider::ReadAlignmentField(protocol::SetAlignmentField &out)
{
	for (int attempt = 0; attempt < 8; ++attempt)
	{
		uint32_t before = alignmentField.sequence.load(std::memory_order_acquire);
		if (before & 1)
			continue;

		out = alignmentField.field.Load();

		uint32_t after = alignmentField.sequence.load(std::memory_order_acquire);
		if (before == after)
			return true;
	}
	return false;
}

bool ServerTrackedDeviceProvider::ReadDeviceTransform(uint32_t openVRID, DeviceTransform &out)
{
	auto &slot = transforms[openVRID];

	for (int attempt = 0; attempt < 8; ++attempt)
	{
		uint32_t before = slot.sequence.load(std::memory_order_acquire);
		if (before & 1)
			continue;

		out = slot.Load();

		uint32_t after = slot.sequence.load(std::memory_order_acquire);
		if (before == after)
		{
			lastGood[openVRID] = out;
			return true;
		}
	}

	// A writer kept racing us; use the last consistent snapshot rather than
	// stalling vrserver's pose thread.
	out = lastGood[openVRID];
	return false;
}

bool ServerTrackedDeviceProvider::HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t &pose)
{
	if (openVRID >= vr::k_unMaxTrackedDeviceCount)
		return true;

	// Publish the raw driver-space pose (pre-transform) for the solver, stamped
	// at capture. This is the overlay's time base for cross-system alignment.
	//
	// INVARIANT: this publish must stay ABOVE every pose mutation below,
	// including the poseTimeOffset shift. If the ring ever records post-rewrite
	// values, the next solve sees pre-shifted timelines, finds ~zero residual
	// offset, and the full-replace transform silently erases the latency
	// correction on every recalibration.
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
	ReadDeviceTransform(openVRID, tf);

	double nowSeconds = static_cast<double>(now.QuadPart) * qpcToSeconds;

	if (tf.enabled)
	{
		// Base-calibration slew (protocol v5): continuous-calibration
		// corrections arrive with an unchanged generation and are rate-limited
		// here so the world never visibly steps; intentional discontinuities
		// (recalibration, universe jump, profile edit) bump the generation and
		// snap. A seqlock read that fell back to lastGood may briefly carry an
		// older generation; the later consistent read then snaps to the value
		// it was already slewing toward, which is benign.
		auto &bs = baseState[openVRID];
		alignfield::SlewToward(tf.rotation, tf.translation.v, nowSeconds,
			alignfield::BaseSlewLimits, tf.generation, bs);
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
		protocol::SetAlignmentField field;
		if (ReadAlignmentField(field))
		{
			if (!field.enabled || field.anchorCount == 0)
			{
				fs.hasCurrent = false;   // re-enabling later snaps
			}
			else if (pose.poseIsValid)
			{
				double scaled[3] = {
					pose.vecPosition[0] * tf.scale,
					pose.vecPosition[1] * tf.scale,
					pose.vecPosition[2] * tf.scale,
				};
				vr::HmdVector3d_t world =
					questcal::driverpose::RotateVector(pose.qWorldFromDriverRotation, scaled);
				double raw[3] = {
					world.v[0] + pose.vecWorldFromDriverTranslation[0] * tf.scale,
					world.v[1] + pose.vecWorldFromDriverTranslation[1] * tf.scale,
					world.v[2] + pose.vecWorldFromDriverTranslation[2] * tf.scale,
				};
				vr::HmdVector3d_t based =
					questcal::driverpose::RotateVector(baseRot, raw);
				double basePos[3] = {
					based.v[0] + baseTrans.v[0],
					based.v[1] + baseTrans.v[1],
					based.v[2] + baseTrans.v[2],
				};

				alignfield::Evaluate(field, basePos, nowSeconds, fs);
			}
			// Invalid pose: keep the previous delta without advancing the
			// slew clock; Evaluate's gap check snaps after a long loss.
		}
		// A racing field write keeps the previous delta for this frame.

		if (fs.hasCurrent)
		{
			calRot = questcal::driverpose::Multiply(fs.rot, baseRot);
			vr::HmdVector3d_t rotatedBase =
				questcal::driverpose::RotateVector(fs.rot, baseTrans.v);
			calTrans.v[0] = rotatedBase.v[0] + fs.trans[0];
			calTrans.v[1] = rotatedBase.v[1] + fs.trans[1];
			calTrans.v[2] = rotatedBase.v[2] + fs.trans[2];
		}

		// Apply the exact solver model to the composed raw-world pose, including
		// scale on worldFromDriver translation and every linear derivative.
		questcal::driverpose::Apply(
			pose, calRot, calTrans.v, tf.scale, tf.timeOffset);
	}
	else
	{
		// Re-enabling later must snap, not slew from a stale state.
		baseState[openVRID].hasCurrent = false;
	}

	// Hide the HMD-mounted continuous-calibration tracker from applications:
	// the device keeps reporting a valid, tracking pose (invalid poses would
	// make SteamVR flag the device as lost) but lives far above the play area,
	// out of reach of proximity-based tracker auto-assignment in games. The
	// raw pose was already published to the ring above, so the overlay's
	// solver always sees the true pose.
	if (tf.hidden)
		pose.vecWorldFromDriverTranslation[1] += HiddenPoseOffsetY;

	return true;
}
