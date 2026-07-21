#include "ServerTrackedDeviceProvider.h"
#include "Logging.h"
#include "InterfaceHookInjector.h"

// Vertical displacement applied to a hidden device's forwarded pose.
static constexpr double HiddenPoseOffsetY = 1000.0;   // meters

vr::EVRInitError ServerTrackedDeviceProvider::Init(vr::IVRDriverContext *pDriverContext)
{
	TRACE("ServerTrackedDeviceProvider::Init()");
	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	qpcToSeconds = 1.0 / static_cast<double>(freq.QuadPart);

	if (!poseRing.Create(QUESTCALIBRATOR_SHMEM_NAME))
	{
		// Non-fatal: calibration transforms still apply, but the overlay will
		// fall back to runtime-predicted poses instead of raw driver poses.
		LOG("Failed to create pose ring shared memory (error %u)", GetLastError());
	}

	InjectHooks(this, pDriverContext);
	server.Run();

	return vr::VRInitError_None;
}

void ServerTrackedDeviceProvider::Cleanup()
{
	TRACE("ServerTrackedDeviceProvider::Cleanup()");
	server.Stop();
	DisableHooks();
	poseRing.Close();
	VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

inline vr::HmdQuaternion_t operator*(const vr::HmdQuaternion_t &lhs, const vr::HmdQuaternion_t &rhs) {
	return {
		(lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z),
		(lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
		(lhs.w * rhs.y) + (lhs.y * rhs.w) + (lhs.z * rhs.x) - (lhs.x * rhs.z),
		(lhs.w * rhs.z) + (lhs.z * rhs.w) + (lhs.x * rhs.y) - (lhs.y * rhs.x)
	};
}

inline vr::HmdVector3d_t quaternionRotateVector(const vr::HmdQuaternion_t& quat, const double(&vector)[3]) {
	vr::HmdQuaternion_t vectorQuat = { 0.0, vector[0], vector[1] , vector[2] };
	vr::HmdQuaternion_t conjugate = { quat.w, -quat.x, -quat.y, -quat.z };
	auto rotatedVectorQuat = quat * vectorQuat * conjugate;
	return { rotatedVectorQuat.x, rotatedVectorQuat.y, rotatedVectorQuat.z };
}

void ServerTrackedDeviceProvider::SetDeviceTransform(const protocol::SetDeviceTransform &newTransform)
{
	// The id arrives over the pipe from another process; never index with it unchecked.
	if (newTransform.openVRID >= vr::k_unMaxTrackedDeviceCount)
	{
		LOG("SetDeviceTransform: rejected out-of-range device id %u", newTransform.openVRID);
		return;
	}

	auto &slot = transforms[newTransform.openVRID];
	auto &tf = slot.transform;

	// Seqlock write; single writer (the IPC thread). Odd sequence = write in flight.
	slot.sequence.fetch_add(1, std::memory_order_acq_rel);

	tf.enabled = newTransform.enabled;
	tf.translation = newTransform.translation;
	tf.rotation = newTransform.rotation;
	tf.scale = newTransform.scale;
	tf.timeOffset = newTransform.timeOffset;
	tf.generation = newTransform.generation;
	tf.hidden = newTransform.hidden;

	slot.sequence.fetch_add(1, std::memory_order_acq_rel);
}

void ServerTrackedDeviceProvider::SetAlignmentField(const protocol::SetAlignmentField &newField)
{
	// Same seqlock discipline as the transform slots: IPC thread writes, pose
	// threads read.
	alignmentField.sequence.fetch_add(1, std::memory_order_acq_rel);
	alignmentField.field = newField;
	alignmentField.sequence.fetch_add(1, std::memory_order_acq_rel);
}

bool ServerTrackedDeviceProvider::ReadAlignmentField(protocol::SetAlignmentField &out)
{
	for (int attempt = 0; attempt < 8; ++attempt)
	{
		uint32_t before = alignmentField.sequence.load(std::memory_order_acquire);
		if (before & 1)
			continue;

		out = alignmentField.field;

		std::atomic_thread_fence(std::memory_order_acquire);
		uint32_t after = alignmentField.sequence.load(std::memory_order_relaxed);
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

		out = slot.transform;

		std::atomic_thread_fence(std::memory_order_acquire);
		uint32_t after = slot.sequence.load(std::memory_order_relaxed);
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
				vr::HmdVector3d_t world = quaternionRotateVector(pose.qWorldFromDriverRotation, scaled);
				double raw[3] = {
					world.v[0] + pose.vecWorldFromDriverTranslation[0],
					world.v[1] + pose.vecWorldFromDriverTranslation[1],
					world.v[2] + pose.vecWorldFromDriverTranslation[2],
				};
				vr::HmdVector3d_t based = quaternionRotateVector(baseRot, raw);
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
			calRot = fs.rot * baseRot;
			vr::HmdVector3d_t rotatedBase = quaternionRotateVector(fs.rot, baseTrans.v);
			calTrans.v[0] = rotatedBase.v[0] + fs.trans[0];
			calTrans.v[1] = rotatedBase.v[1] + fs.trans[1];
			calTrans.v[2] = rotatedBase.v[2] + fs.trans[2];
		}

		pose.qWorldFromDriverRotation = calRot * pose.qWorldFromDriverRotation;

		pose.vecPosition[0] *= tf.scale;
		pose.vecPosition[1] *= tf.scale;
		pose.vecPosition[2] *= tf.scale;

		vr::HmdVector3d_t rotatedTranslation = quaternionRotateVector(calRot, pose.vecWorldFromDriverTranslation);
		pose.vecWorldFromDriverTranslation[0] = rotatedTranslation.v[0] + calTrans.v[0];
		pose.vecWorldFromDriverTranslation[1] = rotatedTranslation.v[1] + calTrans.v[1];
		pose.vecWorldFromDriverTranslation[2] = rotatedTranslation.v[2] + calTrans.v[2];

		// Align this device's timeline with the reference system: a positive
		// shift declares the pose newer, so vrserver predicts less and the
		// device is presented slightly in the past (matching a laggy wireless
		// reference). Sign convention pinned by the live spike; see
		// questcal::ComputeAppliedTimeOffset.
		pose.poseTimeOffset += tf.timeOffset;
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
