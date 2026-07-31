#include "stdafx.h"
#include "Calibration.h"
#include "CalibrationEngine.h"
#include "ChaperoneMath.h"
#include "Configuration.h"
#include "DriftMonitor.h"
#include "FieldMath.h"
#include "IPCClient.h"
#include "JumpDetector.h"
#include "PoseStreamHub.h"
#include "ProfileValidation.h"
#include "../common/PoseChannel.h"
#include "../common/Version.h"

#include <Eigen/Dense>

#include <ctime>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

static IPCClient Driver;
static PoseStreamHub PoseHub;
static int CollectorConsumer = -1;
static std::vector<protocol::DevicePoseSample> CollectorScratch;
static double QpcToSeconds = 0.0;

static std::unique_ptr<JumpDetector> Jumps;
static std::unique_ptr<DriftMonitor> Drift;
static int MonitorConsumer = -1;
static std::vector<protocol::DevicePoseSample> MonitorScratch;
static bool MonitorActive = false;
static bool MonitorHasComposedTime[vr::k_unMaxTrackedDeviceCount] = {};
static double MonitorLastComposedTime[vr::k_unMaxTrackedDeviceCount] = {};

// Conservative knowledge of driver slot state across scans. A failed batch
// does not erase it: a request may have reached the driver immediately before
// a pipe failure, so only a confirmed disable (or a complete connection-wide
// neutralization) proves a slot is no longer live.
static bool DriverSlotMayBeEnabled[vr::k_unMaxTrackedDeviceCount] = {};
static uint64_t SynchronizedDriverConnectionGeneration = 0;

// Dedicated consumer/cache for the HMD's raw-universe transform. It drains
// even without a profile or protected room, so captures always bind to a
// current validated worldFromDriver baseline.
static int ChaperoneMonitorConsumer = -1;
static std::vector<protocol::DevicePoseSample> ChaperoneMonitorScratch;

struct HmdUniverseObservation
{
	enum class State
	{
		Empty,
		EndpointOnly,
		Usable
	};

	State state = State::Empty;
	Eigen::Quaterniond rotation{ 1, 0, 0, 0 };
	Eigen::Vector3d translation{ 0, 0, 0 };
	double sampleTime = -1e9;
	double captureTime = -1e9;
	ringpose::DriverLocalPoseSample localPose;

	bool HasEndpoint() const noexcept
	{
		return state != State::Empty;
	}

	bool IsUsable() const noexcept
	{
		return state == State::Usable;
	}

	void Reset() noexcept
	{
		*this = HmdUniverseObservation();
	}

	void BreakContinuity() noexcept
	{
		if (HasEndpoint())
			state = State::EndpointOnly;
	}

	void Accept(const Eigen::Quaterniond &acceptedRotation,
		const Eigen::Vector3d &acceptedTranslation, double acceptedSampleTime,
		double acceptedCaptureTime,
		const ringpose::DriverLocalPoseSample &acceptedLocalPose)
	{
		state = State::Usable;
		rotation = acceptedRotation;
		translation = acceptedTranslation;
		sampleTime = acceptedSampleTime;
		captureTime = acceptedCaptureTime;
		localPose = acceptedLocalPose;
	}
};

static HmdUniverseObservation CurrentHmdObservation;

struct HmdWorldTransition
{
	bool worldChanged = false;
	bool localPoseContinuous = false;
	Eigen::Quaterniond previousRotation{ 1, 0, 0, 0 };
	Eigen::Vector3d previousTranslation{ 0, 0, 0 };
	Eigen::Quaterniond currentRotation{ 1, 0, 0, 0 };
	Eigen::Vector3d currentTranslation{ 0, 0, 0 };
};

static std::unique_ptr<questcal::ContinuousAlignment> Continuous;
static int ContinuousConsumer = -1;
static std::vector<protocol::DevicePoseSample> ContinuousScratch;
static bool ContinuousActive = false;
static double LastDriverRequestErrorTime = -1e9;

CalibrationContext CalCtx;

static void AbortCalibration(CalibrationContext &ctx, const std::string &reason);
static void ChaperoneMonitorTick(CalibrationContext &ctx, double now);

static bool SaveDirtyPersistence(CalibrationContext &ctx)
{
	// Coupled changes always write Config first.  A crash after that first
	// write is detected by the shared revision at the next launch.
	if (ctx.profileSaveDirty)
	{
		if (!ctx.validProfile || !SaveProfile(ctx))
			return false;
		ctx.profileSaveDirty = false;
	}
	if (ctx.settingsSaveDirty && !SaveSettings(ctx))
		return false;
	return true;
}

static void PersistenceTick(CalibrationContext &ctx, double now)
{
	if (!ctx.HasDirtyPersistence())
		return;

	if (now - ctx.persistenceDirtyTime <= 5.0)
		return;

	if (!SaveDirtyPersistence(ctx))
		ctx.DelayPersistenceRetry(now);
}

static void ResetMonitorIngestionTimes()
{
	for (bool &hasTime : MonitorHasComposedTime)
		hasTime = false;
}

// ---------------------------------------------------------------------------
// Session log (see Calibration.h)

static std::ofstream SessionLog;
static size_t SessionLogBytes = 0;
// Hard cap so a pathological log loop can never eat a user's disk.
static constexpr size_t SessionLogMaxBytes = 4 * 1024 * 1024;

void InitSessionLog()
{
	wchar_t base[MAX_PATH];
	DWORD len = GetEnvironmentVariableW(L"LOCALAPPDATA", base, MAX_PATH);
	if (len == 0 || len >= MAX_PATH)
		return;

	std::wstring dir = std::wstring(base) + L"\\QuestCalibrator";
	CreateDirectoryW(dir.c_str(), nullptr);

	std::wstring current = dir + L"\\QuestCalibrator.log";
	std::wstring previous = dir + L"\\QuestCalibrator.prev.log";
	// One-generation rotation: bounded disk use, but the session that ended in
	// a problem survives the restart that usually precedes the bug report.
	MoveFileExW(current.c_str(), previous.c_str(), MOVEFILE_REPLACE_EXISTING);

	SessionLog.open(current.c_str(), std::ios::out | std::ios::trunc);
	if (!SessionLog.is_open())
		return;

	char date[32] = { 0 };
	std::time_t now = std::time(nullptr);
	std::tm tm;
	if (localtime_s(&tm, &now) == 0)
		std::strftime(date, sizeof date, "%Y-%m-%d %H:%M:%S", &tm);
	SessionLog << "QuestCalibrator " << QUESTCAL_VERSION_STRING
		<< " session started " << date << "\n";
	SessionLog.flush();
}

void AppendSessionLog(const std::string &msg)
{
	if (!SessionLog.is_open() || msg.empty() || SessionLogBytes >= SessionLogMaxBytes)
		return;

	char stamp[16] = { 0 };
	std::time_t now = std::time(nullptr);
	std::tm tm;
	if (localtime_s(&tm, &now) == 0)
		std::strftime(stamp, sizeof stamp, "[%H:%M:%S] ", &tm);

	SessionLog << stamp << msg;
	if (msg.back() != '\n')
		SessionLog << '\n';

	SessionLogBytes += msg.size() + 12;
	if (SessionLogBytes >= SessionLogMaxBytes)
		SessionLog << stamp << "session log size cap reached -- further messages dropped\n";

	// Flushed per line so a crashed or killed session keeps everything.
	SessionLog.flush();
}

void InitCalibrator()
{
	Driver.Connect();

	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	QpcToSeconds = 1.0 / static_cast<double>(freq.QuadPart);

	// The hub keeps draining the driver's shmem ring on its own thread even
	// while this (UI) thread stalls; it retries the open internally until the
	// driver creates the ring.
	PoseHub.Start(QUESTCALIBRATOR_SHMEM_NAME);
	CollectorConsumer = PoseHub.CreateConsumer();

	Jumps = std::make_unique<JumpDetector>(QpcToSeconds);
	Drift = std::make_unique<DriftMonitor>(QpcToSeconds);
	MonitorConsumer = PoseHub.CreateConsumer();

	ChaperoneMonitorConsumer = PoseHub.CreateConsumer();

	Continuous = std::make_unique<questcal::ContinuousAlignment>();
	ContinuousConsumer = PoseHub.CreateConsumer();
}

void ShutdownCalibrator(bool cleanExit)
{
	// A quit inside the save-debounce window must not lose a runtime
	// compensation update.
	if (CalCtx.HasDirtyPersistence())
		SaveDirtyPersistence(CalCtx);
	PoseHub.Stop();
	if (cleanExit)
		AppendSessionLog("session ended cleanly");
}

PoseStreamHub &GetPoseHub()
{
	return PoseHub;
}

double GetQpcToSeconds()
{
	return QpcToSeconds;
}

static vr::HmdQuaternion_t VRQuat(const Eigen::Quaterniond &q)
{
	vr::HmdQuaternion_t out;
	out.w = q.w();
	out.x = q.x();
	out.y = q.y();
	out.z = q.z();
	return out;
}

static vr::HmdVector3d_t VRVec(const Eigen::Vector3d &meters)
{
	vr::HmdVector3d_t out;
	out.v[0] = meters(0);
	out.v[1] = meters(1);
	out.v[2] = meters(2);
	return out;
}

static bool SendDriverRequest(CalibrationContext &ctx, const protocol::Request &request,
	const char *operation, uint64_t *batchConnectionGeneration = nullptr)
{
	try
	{
		protocol::Response response = Driver.SendBlocking(request);
		bool accepted = response.type == protocol::ResponseSuccess ||
			(request.type == protocol::RequestHandshake &&
				response.type == protocol::ResponseHandshake &&
				response.protocol.version == protocol::Version);
		if (accepted)
		{
			if (batchConnectionGeneration)
			{
				uint64_t generation = Driver.ConnectionGeneration();
				if (*batchConnectionGeneration == 0)
					*batchConnectionGeneration = generation;
				else if (*batchConnectionGeneration != generation)
					return false;
			}
			return true;
		}
		if (ctx.timeLastTick - LastDriverRequestErrorTime >= 30.0)
		{
			ctx.ReportError(std::string("QuestCalibrator driver rejected ") + operation +
				"; the requested live state was not applied\n",
				CalibrationContext::ErrorSource::Driver);
			LastDriverRequestErrorTime = ctx.timeLastTick;
		}
	}
	catch (const std::exception &e)
	{
		if (ctx.timeLastTick - LastDriverRequestErrorTime >= 30.0)
		{
			ctx.ReportError(std::string("QuestCalibrator driver communication failed while ") +
				operation + ": " + e.what() + "\n",
				CalibrationContext::ErrorSource::Driver);
			LastDriverRequestErrorTime = ctx.timeLastTick;
		}
	}
	return false;
}

static bool ResetAndDisableOffsets(CalibrationContext &ctx, uint32_t id,
	uint64_t *batchConnectionGeneration = nullptr)
{
	protocol::Request req(protocol::RequestSetDeviceTransform);
	req.setDeviceTransform = protocol::SetDeviceTransform(id, false);
	return SendDriverRequest(ctx, req, "disabling a device transform",
		batchConnectionGeneration);
}

static_assert(vr::k_unTrackedDeviceIndex_Hmd == 0, "HMD index expected to be 0");

// Ship the spatial correction field. Deltas are derived here against the
// current base calibration: delta_i = anchor_i o base^-1 is the correction
// that, applied after the base calibration, reproduces the absolute solve at
// that anchor's spot.
static bool SendAlignmentField(CalibrationContext &ctx, bool baseBatchComplete,
	uint64_t *batchConnectionGeneration)
{
	protocol::Request req(protocol::RequestSetAlignmentField);
	auto &f = req.setAlignmentField;

	f.enabled = baseBatchComplete && ctx.enabled && ctx.fieldEnabled &&
		!ctx.fieldAnchors.empty();
	f.generation = ctx.fieldGeneration;
	if (!f.enabled)
	{
		// Disabled messages are canonical and independent of live calibration
		// numerics. This guarantees a bad base can still clear a stale field.
		f.anchorCount = 0;
		return SendDriverRequest(ctx, req, "disabling the alignment field",
			batchConnectionGeneration);
	}
	f.anchorCount = static_cast<uint32_t>(std::min(
		ctx.fieldAnchors.size(), static_cast<size_t>(protocol::SetAlignmentField::MaxAnchors)));

	Eigen::Quaterniond baseInv = ctx.calibratedRotationQ.conjugate();
	Eigen::Vector3d baseT = ctx.TranslationMeters();
	for (uint32_t i = 0; i < f.anchorCount; ++i)
	{
		const auto &a = ctx.fieldAnchors[i];
		Eigen::Quaterniond dR;
		Eigen::Vector3d dT;
		questcal::AnchorDelta(a.rotation, a.translationMeters, baseInv, baseT, dR, dT);

		for (int k = 0; k < 3; ++k)
		{
			f.anchors[i].position[k] = a.position(k);
			f.anchors[i].translationDelta[k] = dT(k);
		}
		f.anchors[i].rotationDelta = VRQuat(dR);
	}

	return SendDriverRequest(ctx, req, "applying the alignment field",
		batchConnectionGeneration);
}

// Put a single, positively identified driver connection into canonical neutral
// state. A pipe can reconnect during any request; restart from slot zero so no
// generation ever receives only a suffix of the reset pass. Retries are bounded
// so a flapping vrserver cannot stall the UI tick indefinitely.
static bool NeutralizeDriverConnection(CalibrationContext &ctx,
	uint64_t &neutralizedConnectionGeneration)
{
	constexpr int MaxNeutralizationAttempts = 3;
	neutralizedConnectionGeneration = 0;

	for (int attempt = 0; attempt < MaxNeutralizationAttempts; ++attempt)
	{
		uint64_t passConnectionGeneration = 0;
		protocol::Request handshake(protocol::RequestHandshake);
		if (!SendDriverRequest(ctx, handshake,
			"checking the driver connection before neutralizing it",
			&passConnectionGeneration))
			continue;

		bool complete = SendAlignmentField(ctx, false,
			&passConnectionGeneration);
		bool generationChanged =
			Driver.ConnectionGeneration() != passConnectionGeneration;
		for (uint32_t id = 0;
			id < vr::k_unMaxTrackedDeviceCount && !generationChanged; ++id)
		{
			complete = ResetAndDisableOffsets(ctx, id,
				&passConnectionGeneration) && complete;
			generationChanged =
				Driver.ConnectionGeneration() != passConnectionGeneration;
		}

		if (generationChanged)
			continue;
		if (!complete)
			continue;

		for (bool &mayBeEnabled : DriverSlotMayBeEnabled)
			mayBeEnabled = false;
		neutralizedConnectionGeneration = passConnectionGeneration;
		return true;
	}

	return false;
}

static bool ReadTrackedDeviceString(uint32_t id,
	vr::ETrackedDeviceProperty property, std::string &value)
{
	value.clear();
	auto system = vr::VRSystem();
	if (!system || id >= vr::k_unMaxTrackedDeviceCount)
		return false;

	char buffer[vr::k_unMaxPropertyStringSize] = {};
	vr::ETrackedPropertyError error = vr::TrackedProp_Success;
	uint32_t size = system->GetStringTrackedDeviceProperty(id, property, buffer,
		static_cast<uint32_t>(sizeof buffer), &error);
	if (error != vr::TrackedProp_Success || size <= 1 || size > sizeof buffer ||
		buffer[size - 1] != '\0')
		return false;

	value.assign(buffer, size - 1);
	return !value.empty();
}

static bool ReadCurrentHmdIdentity(std::string &trackingSystem, std::string &serial)
{
	return ReadTrackedDeviceString(vr::k_unTrackedDeviceIndex_Hmd,
		vr::Prop_TrackingSystemName_String, trackingSystem) &&
		ReadTrackedDeviceString(vr::k_unTrackedDeviceIndex_Hmd,
			vr::Prop_SerialNumber_String, serial);
}

static bool CacheHmdWorldFromDriver(
	const protocol::DevicePoseSample &sample, HmdWorldTransition &transition)
{
	transition = HmdWorldTransition();
	if (sample.deviceId != vr::k_unTrackedDeviceIndex_Hmd ||
		!sample.poseIsValid ||
		sample.trackingResult != static_cast<uint32_t>(vr::TrackingResult_Running_OK) ||
		!IsUsableRingSample(sample, QpcToSeconds))
		return false;

	double sampleTime = RingSampleTime(sample, QpcToSeconds);
	if (CurrentHmdObservation.HasEndpoint() &&
		sampleTime <= CurrentHmdObservation.sampleTime)
		return false;

	RingSampleParts parts = UnpackRingSample(sample);
	ringpose::DriverLocalPoseSample localPose;
	localPose.time = sampleTime;
	localPose.rotation = parts.drvRot;
	localPose.position = parts.drvPos;
	localPose.velocity = Eigen::Vector3d(
		sample.velocity[0], sample.velocity[1], sample.velocity[2]);
	localPose.angularVelocity = Eigen::Vector3d(
		sample.angularVelocity[0], sample.angularVelocity[1],
		sample.angularVelocity[2]);

	if (CurrentHmdObservation.HasEndpoint() &&
		questcal::WorldFromDriverChanged(
			CurrentHmdObservation.rotation,
			CurrentHmdObservation.translation,
			parts.wfdRot, parts.wfdTrans))
	{
		transition.worldChanged = true;
		transition.previousRotation = CurrentHmdObservation.rotation;
		transition.previousTranslation = CurrentHmdObservation.translation;
		transition.currentRotation = parts.wfdRot;
		transition.currentTranslation = parts.wfdTrans;
		transition.localPoseContinuous = CurrentHmdObservation.IsUsable() &&
			ringpose::IsDriverLocalPoseContinuous(
				CurrentHmdObservation.localPose, localPose);
	}

	CurrentHmdObservation.Accept(parts.wfdRot, parts.wfdTrans, sampleTime,
		RingCaptureTime(sample, QpcToSeconds), localPose);
	return true;
}

static bool HasFreshHmdWorldFromDriver()
{
	LARGE_INTEGER qpcNow;
	if (!QueryPerformanceCounter(&qpcNow))
		return false;
	double sampleClockNow = static_cast<double>(qpcNow.QuadPart) * QpcToSeconds;
	return PoseHub.RingOpen() && CurrentHmdObservation.IsUsable() &&
		ringpose::IsFreshCaptureTime(
			CurrentHmdObservation.captureTime, sampleClockNow, 2.0);
}

static bool ChaperoneBaselineIsCurrent(
	const CalibrationContext::Chaperone &snapshot)
{
	return snapshot.worldFromDriverValid &&
		snapshot.baselineVerifiedThisSession && HasFreshHmdWorldFromDriver() &&
		!questcal::WorldFromDriverChanged(
			snapshot.worldFromDriverRotation, snapshot.worldFromDriverTranslation,
			CurrentHmdObservation.rotation, CurrentHmdObservation.translation);
}

static bool CopyCurrentHmdWorldFromDriver(
	CalibrationContext::Chaperone &snapshot)
{
	if (!HasFreshHmdWorldFromDriver())
		return false;
	snapshot.worldFromDriverRotation = CurrentHmdObservation.rotation;
	snapshot.worldFromDriverTranslation = CurrentHmdObservation.translation;
	snapshot.worldFromDriverValid = true;
	snapshot.baselineVerifiedThisSession = true;
	return true;
}

static void SynchronizeDriverState(CalibrationContext &ctx)
{
	ctx.enabled = ctx.validProfile && !ctx.profileUniverseUnsafe;
	uint64_t batchConnectionGeneration = 0;
	protocol::Request handshake(protocol::RequestHandshake);
	bool connectionReady = SendDriverRequest(ctx, handshake,
		"checking the driver connection", &batchConnectionGeneration);
	bool driverSynchronized = connectionReady;
	if (ctx.enabled && !questcal::IsValidTrackingSystemPair(
		ctx.referenceTrackingSystem, ctx.targetTrackingSystem))
	{
		ctx.ReportError(
			"The live profile has invalid tracking-system identities and was disabled before sending it to the driver\n");
		ctx.enabled = false;
	}

	// Resolve the reference HMD before any target transform can be enabled.
	// Device 0 may be absent/invalid during startup; relying on the loop below
	// would skip that check and could briefly apply a profile to the wrong rig.
	if (ctx.enabled)
	{
		std::string hmdTrackingSystem;
		std::string hmdSerial;
		if (vr::VRSystem()->GetTrackedDeviceClass(vr::k_unTrackedDeviceIndex_Hmd) !=
				vr::TrackedDeviceClass_HMD ||
			!ReadCurrentHmdIdentity(hmdTrackingSystem, hmdSerial) ||
			hmdTrackingSystem != ctx.referenceTrackingSystem)
		{
			ctx.enabled = false;
		}
	}

	// Timeline shift for target devices (runtime latency re-prediction). The
	// manual override bypasses the solved value; it exists to pin the sign
	// convention against a live SteamVR session.
	double timeShift = 0.0;
	if (ctx.useManualTimeOffset)
		timeShift = ctx.manualTimeOffsetMs / 1000.0;
	else if (ctx.applyTimeOffset)
		timeShift = questcal::ComputeAppliedTimeOffset(ctx.calibratedTimeOffset);
	if (!std::isfinite(timeShift) ||
		std::abs(timeShift) > protocol::limits::MaxAbsTimeOffsetSeconds ||
		(ctx.enabled && !questcal::IsValidCalibrationTransform(ctx.calibratedRotationQ,
			ctx.TranslationMeters(), ctx.calibratedScale)))
	{
		ctx.ReportError("The live calibration contains invalid numeric values and was disabled before sending it to the driver\n");
		ctx.enabled = false;
		timeShift = 0.0;
	}
	ctx.appliedTimeOffset = timeShift;

	// A new pipe generation may be a restarted vrserver (fresh slots) or a new
	// pipe to the same provider (retained slots). Treat both alike: neutralize
	// the complete connection before rebuilding desired state. This one-time
	// pass is intentionally not paid on steady-state scans.
	bool newlyObservedConnection = connectionReady &&
		batchConnectionGeneration != SynchronizedDriverConnectionGeneration;
	if (newlyObservedConnection)
	{
		uint64_t neutralizedConnectionGeneration = 0;
		bool neutralized = NeutralizeDriverConnection(ctx,
			neutralizedConnectionGeneration);
		connectionReady = neutralized;
		driverSynchronized = neutralized;
		if (neutralized)
			batchConnectionGeneration = neutralizedConnectionGeneration;
	}

	ctx.continuousTrackerId = vr::k_unTrackedDeviceIndexInvalid;
	bool desiredMutationAttempted = false;
	auto disableSlot = [&](uint32_t id)
	{
		desiredMutationAttempted = true;
		bool disabled = ResetAndDisableOffsets(ctx, id,
			&batchConnectionGeneration);
		if (disabled)
			DriverSlotMayBeEnabled[id] = false;
		if (Driver.ConnectionGeneration() != batchConnectionGeneration)
			connectionReady = false;
		driverSynchronized = disabled && driverSynchronized;
	};

	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		ctx.referenceDeviceMask[id] = false;
		ctx.targetDeviceMask[id] = false;
	}

	if (!ctx.enabled)
	{
		// No OpenVR device scan is needed to converge to neutral state. The
		// conservative slot ledger already identifies every transform that may
		// still be live, including response-loss uncertainty.
		for (uint32_t id = 0;
			id < vr::k_unMaxTrackedDeviceCount && connectionReady && driverSynchronized;
			++id)
		{
			if (DriverSlotMayBeEnabled[id])
				disableSlot(id);
		}
	}
	else
	{
		for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
		{
			if (!connectionReady || !driverSynchronized)
				continue;
			if (!ctx.enabled)
			{
				if (DriverSlotMayBeEnabled[id])
					disableSlot(id);
				continue;
			}

			auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
			if (deviceClass == vr::TrackedDeviceClass_Invalid)
			{
				// OpenVR no longer exposes this ID, but the driver slot survives the
				// disappearance. Retire any transform that a prior successful or
				// uncertain enable may have left behind.
				if (DriverSlotMayBeEnabled[id])
					disableSlot(id);
				continue;
			}

			std::string trackingSystem;
			if (!ReadTrackedDeviceString(id, vr::Prop_TrackingSystemName_String,
				trackingSystem))
			{
				if (DriverSlotMayBeEnabled[id])
					disableSlot(id);
				continue;
			}

			ctx.referenceDeviceMask[id] =
				trackingSystem == ctx.referenceTrackingSystem;
			ctx.targetDeviceMask[id] = trackingSystem == ctx.targetTrackingSystem;

			if (id == vr::k_unTrackedDeviceIndex_Hmd)
			{
				if (trackingSystem != ctx.referenceTrackingSystem)
				{
					// Currently using an HMD with a different tracking system than the calibration.
					ctx.enabled = false;
				}

				if (DriverSlotMayBeEnabled[id])
					disableSlot(id);
				continue;
			}

			if (trackingSystem != ctx.targetTrackingSystem)
			{
				if (DriverSlotMayBeEnabled[id])
					disableSlot(id);
				continue;
			}

			// The continuous-calibration tracker is identified by serial (ids are
			// not stable across sessions).
			if (!ctx.continuousTrackerSerial.empty())
			{
				std::string serial;
				if (ReadTrackedDeviceString(id, vr::Prop_SerialNumber_String, serial) &&
					ctx.continuousTrackerSerial == serial)
					ctx.continuousTrackerId = id;
			}

			protocol::Request req(protocol::RequestSetDeviceTransform);
			req.setDeviceTransform = {
				id,
				true,
				VRVec(ctx.TranslationMeters()),
				VRQuat(ctx.calibratedRotationQ),
				ctx.calibratedScale,
				timeShift
			};
			req.setDeviceTransform.generation = ctx.baseGeneration;
			req.setDeviceTransform.hidden =
				ctx.continuousEnabled && ctx.hideMountedTracker &&
				id == ctx.continuousTrackerId;
			// Mark before sending: a response loss can leave an accepted enable
			// indistinguishable from a failed request, so future disappearance must
			// conservatively issue a disable.
			DriverSlotMayBeEnabled[id] = true;
			desiredMutationAttempted = true;
			bool applied = SendDriverRequest(ctx, req,
				"applying a device transform", &batchConnectionGeneration);
			if (Driver.ConnectionGeneration() != batchConnectionGeneration)
			{
				connectionReady = false;
				// The enable may have landed just before the response path exposed
				// the reconnect. Neutralize that one known slot on the new pipe now;
				// the generation-wide recovery pass below clears every other slot.
				if (ResetAndDisableOffsets(ctx, id, nullptr))
					DriverSlotMayBeEnabled[id] = false;
			}
			driverSynchronized = applied && driverSynchronized;
		}
	}

	// Re-asserted alongside the per-device transforms so a restarted driver
	// converges without special casing. On a universe jump the base transforms
	// land first and the field one pipe round-trip later; the mixed window is
	// bounded by the (small) delta magnitudes and the generation bump snaps
	// driver-side smoothing when it arrives.
	// A spatial field is meaningful only on top of a complete base-transform
	// batch from this same driver connection. Never send a field enable after a
	// failed base request. If any mutation failed or reconnected, immediately
	// neutralize the whole current connection before retrying desired state on
	// the next periodic scan.
	bool fieldSynchronized = false;
	if (connectionReady && driverSynchronized)
	{
		desiredMutationAttempted = true;
		fieldSynchronized = SendAlignmentField(ctx, true,
			&batchConnectionGeneration);
	}
	driverSynchronized = fieldSynchronized && driverSynchronized;
	if (!driverSynchronized && desiredMutationAttempted)
	{
		uint64_t neutralizedConnectionGeneration = 0;
		if (NeutralizeDriverConnection(ctx, neutralizedConnectionGeneration))
		{
			// The profile batch still failed, but the connection is now known
			// neutral. Next scan can safely apply desired state without another
			// generation-wide reset.
			SynchronizedDriverConnectionGeneration =
				neutralizedConnectionGeneration;
		}
	}

	// Never let the jump/drift/continuous monitors infer that the live driver
	// matches the profile after a partial pipe failure. The next periodic scan
	// rebuilds and retries the complete desired state, including after vrserver
	// restarts; until then, all device identities are deliberately unavailable.
	if (!driverSynchronized)
	{
		ctx.enabled = false;
		ctx.continuousTrackerId = vr::k_unTrackedDeviceIndexInvalid;
		for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
		{
			ctx.referenceDeviceMask[id] = false;
			ctx.targetDeviceMask[id] = false;
		}
	}
	else
	{
		SynchronizedDriverConnectionGeneration = batchConnectionGeneration;
		ctx.ClearError(CalibrationContext::ErrorSource::Driver);
		LastDriverRequestErrorTime = -1e9;
	}
}

static void CheckProtectedChaperone(CalibrationContext &ctx)
{
	// Auto-restore of the protected chaperone snapshot. The trigger compares
	// wall GEOMETRY content only: quads are standing-frame, so both universe
	// jumps and play-space movers (which move the standing center, not the
	// walls) leave them untouched — deliberate adjustments never trip this.
	// A count-only check would miss a reset that lands on the same number of
	// walls (two 4-wall rectangles), hence the corner-wise comparison.
	// An empty snapshot never auto-restores: it would stomp any bounds the
	// runtime imports later (e.g. a Guardian arriving after session start).
	const bool shouldCheckChaperone = ctx.chaperone.valid &&
		ctx.chaperone.autoApply && !ctx.chaperone.geometry.empty() &&
		ChaperoneBaselineIsCurrent(ctx.chaperone);
	if (!shouldCheckChaperone)
		ctx.ClearError(CalibrationContext::ErrorSource::ChaperoneMonitor);
	else
	{
		static double lastSetupFailure = -1e9;
		static double lastReadFailure = -1e9;
		auto setup = vr::VRChaperoneSetup();
		if (!setup)
		{
			if (ctx.timeLastTick - lastSetupFailure >= 30.0)
			{
				ctx.ReportError(
					"Protected chaperone could not be checked because OpenVR chaperone setup is unavailable\n",
					CalibrationContext::ErrorSource::ChaperoneMonitor);
				lastSetupFailure = ctx.timeLastTick;
			}
			return;
		}
		lastSetupFailure = -1e9;

		uint32_t quadCount = 0;
		bool liveRead = setup->GetLiveCollisionBoundsInfo(nullptr, &quadCount);

		bool differs = liveRead && quadCount != ctx.chaperone.geometry.size();
		if (liveRead && !differs)
		{
			std::vector<vr::HmdQuad_t> live(quadCount);
			if (setup->GetLiveCollisionBoundsInfo(live.data(), &quadCount))
				differs = !questcal::QuadsMatch(live, ctx.chaperone.geometry, 0.002f);
			else
				liveRead = false;
		}
		if (!liveRead)
		{
			if (ctx.timeLastTick - lastReadFailure >= 30.0)
			{
				ctx.ReportError(
					"Could not read the live chaperone; protected bounds were not changed\n",
					CalibrationContext::ErrorSource::ChaperoneMonitor);
				lastReadFailure = ctx.timeLastTick;
			}
			return;
		}
		lastReadFailure = -1e9;
		ctx.ClearError(CalibrationContext::ErrorSource::ChaperoneMonitor);

		// Cooldown so a runtime that keeps reasserting its own bounds is
		// contested every few seconds at worst, not at the 1 Hz scan rate.
		if (differs && ctx.timeLastTick - ctx.chaperone.lastRestoreTime >= 5.0)
		{
			// Advance the cooldown for failures too. A runtime that rejects or
			// rewrites bounds must not be hammered once per profile scan.
			ctx.chaperone.lastRestoreTime = ctx.timeLastTick;
			if (ApplyChaperoneBounds(false))
			{
				char buf[128];
				snprintf(buf, sizeof buf,
					"Chaperone changed outside the app (%u live / %zu saved walls); restored the protected bounds\n",
					quadCount, ctx.chaperone.geometry.size());
				ctx.Log(buf);
			}
		}
	}
}

// ---------------------------------------------------------------------------
// Sample collection

// Compose the raw-world pose (worldFromDriver * driver pose) from a shmem
// sample and convert its capture timestamp to seconds on the QPC clock.
static questcal::PoseSample EngineSampleFromRing(const protocol::DevicePoseSample &s)
{
	RingSampleParts p = UnpackRingSample(s);

	questcal::PoseSample out;
	// poseTimeOffset is the driver's own estimate of how far the pose's validity
	// time differs from the submit time; folding it in tightens the alignment.
	out.time = static_cast<double>(s.sampleTimeQpc) * QpcToSeconds + s.poseTimeOffset;
	out.rot = (p.wfdRot * p.drvRot).normalized();
	out.pos = p.wfdRot * p.drvPos + p.wfdTrans;
	out.vel = p.wfdRot * Eigen::Vector3d(s.velocity[0], s.velocity[1], s.velocity[2]);
	out.angVel = p.wfdRot * Eigen::Vector3d(s.angularVelocity[0], s.angularVelocity[1], s.angularVelocity[2]);
	return out;
}

// A misbehaving driver can publish poseIsValid=true with non-finite fields;
// gate them out at ingestion so no consumer (solver, continuous alignment)
// ever sees one.
static bool IsUsableEngineSample(const questcal::PoseSample &s)
{
	return std::isfinite(s.time) && questcal::IsValidRotation(s.rot) &&
		questcal::IsBoundedVector(s.pos,
			protocol::limits::MaxAbsPosePositionMeters) &&
		questcal::IsBoundedVector(s.vel,
			protocol::limits::MaxAbsLinearVelocityMetersPerSecond) &&
		questcal::IsBoundedVector(s.angVel,
			protocol::limits::MaxAbsAngularVelocityRadiansPerSecond);
}

// Drop the collector's backlog so a new collection starts fresh.
static void DiscardPoseRingBacklog()
{
	PoseHub.DiscardBacklog(CollectorConsumer);
}

static bool CollectFromPoseRing(CalibrationContext &ctx, double now)
{
	uint64_t dropped = PoseHub.Drain(CollectorConsumer, CollectorScratch);
	if (dropped > 0)
	{
		AbortCalibration(ctx,
			"Pose stream overran during collection; retry calibration so no samples are missing");
		return false;
	}
	for (const auto &s : CollectorScratch)
	{
		// Strict validity: a pose the driver flagged invalid never enters the
		// solve, even if the tracking result still claims "running OK".
		if (!s.poseIsValid ||
			s.trackingResult != static_cast<uint32_t>(vr::TrackingResult_Running_OK) ||
			!IsUsableRingSample(s, QpcToSeconds))
			continue;

		// The composed time folds in the driver's jittery poseTimeOffset, so
		// the odd inversion occurs in healthy data; drop it here rather than
		// let the solver fail the whole collection (same policy as
		// ContinuousAlignment::PushReference).
		if (s.deviceId == ctx.calibrationReferenceID)
		{
			questcal::PoseSample sample = EngineSampleFromRing(s);
			if (!IsUsableEngineSample(sample))
				continue;
			if (!ctx.refSamples.empty() && sample.time <= ctx.refSamples.back().time)
				continue;
			ctx.refSamples.push_back(sample);
			ctx.lastRefSampleTime = now;
		}
		else if (s.deviceId == ctx.calibrationTargetID)
		{
			questcal::PoseSample sample = EngineSampleFromRing(s);
			if (!IsUsableEngineSample(sample))
				continue;
			if (!ctx.targetSamples.empty() && sample.time <= ctx.targetSamples.back().time)
				continue;
			ctx.targetSamples.push_back(sample);
			ctx.lastTargetSampleTime = now;
		}
	}
	return true;
}

// Fallback when the shmem channel is unavailable (e.g. running against an old
// driver build): sample runtime poses at tick rate. Loses per-device capture
// timestamps and driver velocities, so alignment quality is reduced.
static void CollectFromRuntimePoses(CalibrationContext &ctx, double now)
{
	auto push = [&](uint32_t id, std::vector<questcal::PoseSample> &into, double &lastTime)
	{
		if (id >= vr::k_unMaxTrackedDeviceCount)
			return;
		const auto &p = ctx.devicePoses[id];
		if (!p.bPoseIsValid || p.eTrackingResult != vr::TrackingResult_Running_OK)
			return;

		const auto &m = p.mDeviceToAbsoluteTracking.m;
		Eigen::Matrix3d rot;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				rot(i, j) = m[i][j];

		questcal::PoseSample s;
		s.time = now;
		s.rot = Eigen::Quaterniond(rot).normalized();
		s.pos = Eigen::Vector3d(m[0][3], m[1][3], m[2][3]);
		s.vel = Eigen::Vector3d(p.vVelocity.v[0], p.vVelocity.v[1], p.vVelocity.v[2]);
		s.angVel = Eigen::Vector3d(p.vAngularVelocity.v[0], p.vAngularVelocity.v[1], p.vAngularVelocity.v[2]);
		if (!IsUsableEngineSample(s) || (!into.empty() && s.time <= into.back().time))
			return;
		into.push_back(s);
		lastTime = now;
	};

	push(ctx.calibrationReferenceID, ctx.refSamples, ctx.lastRefSampleTime);
	push(ctx.calibrationTargetID, ctx.targetSamples, ctx.lastTargetSampleTime);
}

enum class ChaperoneOwnerStatus
{
	Match,
	Mismatch,
	Unavailable
};

static ChaperoneOwnerStatus CurrentChaperoneOwner(
	const CalibrationContext::Chaperone &snapshot)
{
	if (snapshot.ownerTrackingSystem.empty() || snapshot.ownerHmdSerial.empty())
		return ChaperoneOwnerStatus::Mismatch;

	std::string trackingSystem;
	std::string serial;
	if (!ReadCurrentHmdIdentity(trackingSystem, serial))
		return ChaperoneOwnerStatus::Unavailable;
	return trackingSystem == snapshot.ownerTrackingSystem && serial == snapshot.ownerHmdSerial
		? ChaperoneOwnerStatus::Match
		: ChaperoneOwnerStatus::Mismatch;
}

// ---------------------------------------------------------------------------
// Runtime compensation: folding deltas into the live calibration

// Fold a left delta D into the calibration (newCal = D o oldCal) and move
// everything anchored in reference-raw coordinates with it. `snap` marks an
// intentional discontinuity (universe jump): the base generation bump makes
// the driver snap, and the field smoothing snaps with it. Continuous
// corrections pass snap=false — the driver slews the base, and the mm-scale
// anchor-delta change is absorbed by the field slew without a generation bump.
static void ApplyAlignmentDelta(CalibrationContext &ctx, const Eigen::Quaterniond &dR,
                                const Eigen::Vector3d &dT, bool snap, double now)
{
	Eigen::Quaterniond newRot = (dR * ctx.calibratedRotationQ).normalized();
	Eigen::Vector3d newTrans = dR * ctx.TranslationMeters() + dT;
	if (snap)
		ctx.SetCalibration(newRot, newTrans, ctx.calibratedScale);
	else
		ctx.SetCalibrationContinuous(newRot, newTrans, ctx.calibratedScale);

	// The field anchors live in reference space: shift them by D so the field
	// moves with the universe. The per-anchor deltas conjugate automatically
	// (delta' = D delta D^-1) because SendAlignmentField re-derives them from
	// the absolute transforms.
	for (auto &a : ctx.fieldAnchors)
	{
		a.position = dR * a.position + dT;
		a.rotation = (dR * a.rotation).normalized();
		a.translationMeters = dR * a.translationMeters + dT;
	}
	if (snap && !ctx.fieldAnchors.empty())
		ctx.fieldGeneration++;

	if (snap)
	{
		ctx.AdvancePersistenceRevision();
		// The chaperone snapshot's standing center maps the standing frame into
		// the (just re-based) raw frame, so it re-anchors by D like everything
		// else raw-frame; the wall quads are standing-frame and stay put.
		if (ctx.chaperone.valid)
			ctx.chaperone.standingCenter =
				questcal::DeltaTimesPose(dR, dT, ctx.chaperone.standingCenter);
		ctx.MarkSettingsDirty(now);
	}

	ctx.MarkProfileDirty(now);
	SynchronizeDriverState(ctx);
}

// ---------------------------------------------------------------------------
// Runtime monitoring: universe-jump compensation

// Fold an accepted universe delta into the calibration: the reference universe
// moved by D in one frame, so target devices must follow to stay aligned.
static void ApplyUniverseDelta(CalibrationContext &ctx, const JumpDetector::UniverseDelta &d, double now)
{
	ApplyAlignmentDelta(ctx, d.rotation, d.translation, /*snap=*/true, now);
	if (d.exact && ctx.chaperone.valid)
	{
		// Bind to this accepted HMD sample's exact endpoint, never to the
		// consumer's global latest value: a drained backlog may already contain
		// a second rebase which retrigger hysteresis intentionally suppressed.
		ctx.chaperone.worldFromDriverRotation =
			d.worldFromDriverRotation.normalized();
		ctx.chaperone.worldFromDriverTranslation =
			d.worldFromDriverTranslation;
		ctx.chaperone.worldFromDriverValid = true;
		ctx.chaperone.baselineVerifiedThisSession = true;
	}

	ctx.jumpsCompensated++;
	ctx.lastJumpUiTime = now;
	ctx.jumpTiltResidualDeg += d.residualTiltRad * 180.0 / EIGEN_PI;
	ctx.jumpSpreadResidualM += d.residualSpread;

	double yawDeg = 2.0 * std::atan2(d.rotation.y(), d.rotation.w()) * 180.0 / EIGEN_PI;
	char buf[256];
	snprintf(buf, sizeof buf, "Universe jump compensated (%s): yaw %+.2f deg, shift %.3f m, %d device(s)\n",
		d.exact ? "exact" : "estimated", yawDeg, d.translation.norm(), d.devicesAgreeing);
	ctx.Log(buf);
}

static void ChaperoneMonitorTick(CalibrationContext &ctx, double now)
{
	std::vector<HmdWorldTransition> transitions;

	// Drain independently of profile/snapshot state. The first valid HMD sample
	// after startup or a ring outage is compared with the persisted baseline
	// before any protected bounds are eligible for restoration.
	if (!PoseHub.RingOpen())
	{
		CurrentHmdObservation.Reset();
		ctx.chaperone.baselineVerifiedThisSession = false;
	}
	else
	{
		uint64_t dropped = PoseHub.Drain(
			ChaperoneMonitorConsumer, ChaperoneMonitorScratch);
		if (dropped > 0)
		{
			ctx.chaperone.baselineVerifiedThisSession = false;
			// Keep the last WFD endpoint so a later mismatch is still visible,
			// but never call the pose pair adjacent across an overrun or let the
			// old observation immediately re-verify the baseline below. A later
			// accepted HMD frame restores both freshness and adjacency.
			CurrentHmdObservation.BreakContinuity();
		}
		for (const auto &sample : ChaperoneMonitorScratch)
		{
			HmdWorldTransition transition;
			bool accepted = CacheHmdWorldFromDriver(sample, transition);
			if (!accepted)
			{
				if (sample.deviceId == vr::k_unTrackedDeviceIndex_Hmd)
				{
					// Invalid, malformed, and composed-time-inverted HMD frames are
					// observed discontinuities, not invisible absence. Retain the last
					// WFD endpoint/sample time so a later mismatch remains detectable,
					// but invalidate freshness as well as adjacency. Otherwise the old,
					// still-fresh endpoint can re-verify the baseline later in this tick.
					CurrentHmdObservation.BreakContinuity();
					ctx.chaperone.baselineVerifiedThisSession = false;
				}
				continue;
			}
			if (transition.worldChanged)
			{
				if (!transition.localPoseContinuous)
					ctx.chaperone.baselineVerifiedThisSession = false;
				transitions.push_back(std::move(transition));
			}
		}
		if (!HasFreshHmdWorldFromDriver())
			ctx.chaperone.baselineVerifiedThisSession = false;
	}

	// Device properties are stable for a session but comparatively expensive
	// OpenVR calls; the tick runs at up to 50 Hz. Recheck once per second, or
	// immediately when a newly captured/loaded snapshot changes ownership.
	static double lastOwnerCheck = -1e9;
	static std::string checkedTrackingSystem;
	static std::string checkedHmdSerial;
	static ChaperoneOwnerStatus owner = ChaperoneOwnerStatus::Unavailable;
	if (!ctx.chaperone.valid)
	{
		owner = ChaperoneOwnerStatus::Unavailable;
		checkedTrackingSystem.clear();
		checkedHmdSerial.clear();
		lastOwnerCheck = -1e9;
	}
	else if (ctx.chaperone.ownerTrackingSystem != checkedTrackingSystem ||
		ctx.chaperone.ownerHmdSerial != checkedHmdSerial ||
		now - lastOwnerCheck >= 1.0)
	{
		checkedTrackingSystem = ctx.chaperone.ownerTrackingSystem;
		checkedHmdSerial = ctx.chaperone.ownerHmdSerial;
		owner = CurrentChaperoneOwner(ctx.chaperone);
		lastOwnerCheck = now;
	}
	if (ctx.chaperone.valid && owner == ChaperoneOwnerStatus::Mismatch)
	{
		ctx.DisarmChaperone();
		ctx.MarkSettingsDirty(now);
		ctx.ReportError(
			"The protected chaperone belongs to a different headset/runtime and has been disarmed. "
			"Capture it again for the current headset.\n",
			CalibrationContext::ErrorSource::Chaperone);
		SaveSettings(ctx);
		return;
	}

	if (!ctx.chaperone.valid || owner != ChaperoneOwnerStatus::Match ||
		!HasFreshHmdWorldFromDriver())
		return;

	if (!ctx.chaperone.worldFromDriverValid)
	{
		ctx.DisarmChaperone();
		ctx.MarkSettingsDirty(now);
		ctx.ReportError(
			"The protected chaperone has no raw-universe baseline and was disarmed. "
			"Capture it again before restoring it.\n",
			CalibrationContext::ErrorSource::Chaperone);
		SaveSettings(ctx);
		return;
	}

	if (!questcal::WorldFromDriverChanged(
		ctx.chaperone.worldFromDriverRotation,
		ctx.chaperone.worldFromDriverTranslation,
		CurrentHmdObservation.rotation,
		CurrentHmdObservation.translation))
	{
		// Keep the persisted baseline fixed so sub-epsilon changes accumulate
		// instead of being silently chased one sample at a time.
		ctx.chaperone.baselineVerifiedThisSession = true;
		return;
	}

	if (ctx.validProfile)
	{
		// Across a monitor discontinuity we know the reference universe moved,
		// but not whether the target universe also moved. Applying only the HMD
		// delta could silently corrupt alignment; keep both profile and room
		// fail-closed until a fresh base calibration establishes the relation.
		bool newlyUnsafe = !ctx.profileUniverseUnsafe;
		bool autoApplyChanged = ctx.chaperone.autoApply;
		ctx.profileUniverseUnsafe = true;
		ctx.enabled = false;
		ctx.chaperone.autoApply = false;
		ctx.chaperone.baselineVerifiedThisSession = false;
		if (newlyUnsafe)
		{
			ctx.ReportError(
				"The headset raw universe changed while calibration monitoring had no continuity. "
				"The profile and protected chaperone are disabled until you run a new base calibration.\n",
				CalibrationContext::ErrorSource::Chaperone);
			// Fail closed in the driver immediately; waiting for the periodic
			// one-second scan would leave the previous transforms live.
			SynchronizeDriverState(ctx);
		}
		if (autoApplyChanged)
		{
			ctx.MarkSettingsDirty(now);
			SaveSettings(ctx);
		}
		return;
	}

	// With no calibration profile there is no target-space evidence available.
	// Re-anchor the protected room only along a complete chain of adjacent HMD
	// samples whose driver-local pose stayed continuous across each WFD change.
	// A startup, ring outage, overrun, or inverse local-pose rewrite deliberately
	// leaves no such chain and therefore requires a fresh room capture.
	bool reanchored = false;
	bool continuityLost = false;
	double accumulatedShift = 0.0;
	for (const auto &transition : transitions)
	{
		if (questcal::WorldFromDriverChanged(
			ctx.chaperone.worldFromDriverRotation,
			ctx.chaperone.worldFromDriverTranslation,
			transition.previousRotation, transition.previousTranslation))
			continue;

		if (!transition.localPoseContinuous)
		{
			continuityLost = true;
			break;
		}

		Eigen::Quaterniond deltaRotation;
		Eigen::Vector3d deltaTranslation;
		if (!questcal::WorldFromDriverDelta(
			transition.previousRotation, transition.previousTranslation,
			transition.currentRotation, transition.currentTranslation,
			deltaRotation, deltaTranslation))
		{
			continuityLost = true;
			break;
		}

		ctx.chaperone.standingCenter = questcal::DeltaTimesPose(
			deltaRotation, deltaTranslation, ctx.chaperone.standingCenter);
		ctx.chaperone.worldFromDriverRotation =
			transition.currentRotation.normalized();
		ctx.chaperone.worldFromDriverTranslation =
			transition.currentTranslation;
		ctx.chaperone.worldFromDriverValid = true;
		ctx.chaperone.baselineVerifiedThisSession = true;
		accumulatedShift += deltaTranslation.norm();
		reanchored = true;
	}

	if (continuityLost || questcal::WorldFromDriverChanged(
		ctx.chaperone.worldFromDriverRotation,
		ctx.chaperone.worldFromDriverTranslation,
		CurrentHmdObservation.rotation,
		CurrentHmdObservation.translation))
	{
		ctx.DisarmChaperone();
		ctx.MarkSettingsDirty(now);
		ctx.ReportError(
			"The headset raw universe changed without an adjacent continuous HMD pose pair. "
			"The protected chaperone was disarmed; capture it again before restoring it.\n",
			CalibrationContext::ErrorSource::Chaperone);
		SaveSettings(ctx);
		return;
	}

	if (!reanchored)
		return;

	ctx.AdvancePersistenceRevision();
	ctx.MarkSettingsDirty(now);

	char buf[192];
	snprintf(buf, sizeof buf,
		"Protected chaperone re-anchored after HMD universe rebase (shift %.3f m)\n",
		accumulatedShift);
	ctx.Log(buf);
	if (SaveSettings(ctx))
		ctx.ClearError(CalibrationContext::ErrorSource::Chaperone);
	else
	{
		ctx.chaperone.autoApply = false;
		ctx.chaperone.baselineVerifiedThisSession = false;
		ctx.MarkSettingsDirty(now);
	}
}

// ---------------------------------------------------------------------------
// Runtime monitoring: drift staleness (detect + notify only, never corrects)

// One-shot log + VR toast; each caller owns its re-arm flag.
static void NotifyOnce(CalibrationContext &ctx, bool &notified, const char *logLine, const char *toast)
{
	if (notified)
		return;
	notified = true;

	ctx.Log(std::string(logLine) + "\n");

	vr::VROverlayHandle_t overlay = GetMainOverlayHandle();
	if (overlay && vr::VRNotifications())
	{
		vr::VRNotificationId notifId = 0;
		vr::VRNotifications()->CreateNotification(
			overlay, 0, vr::EVRNotificationType_Transient,
			toast, vr::EVRNotificationStyle_Application, nullptr, &notifId);
	}
}

// One notification per calibration: re-armed only by a successful solve.
static bool StaleNotified = false;

// One notification per freeze episode: re-armed by resume or a new solve.
static bool FreezeNotified = false;

// One "observations unstable" notification per calibration: the condition is
// benign and self-healing (bad lighthouse geometry while lying down), so the
// log records every episode but the toast never repeats.
static bool UnstableNotified = false;

static void NotifyStaleAlignment(CalibrationContext &ctx)
{
	NotifyOnce(ctx, StaleNotified,
		"Calibration quality looks poor -- recalibrating is recommended",
		"QuestCalibrator: calibration quality looks poor. Recalibrating is recommended.");
}

static void UpdateDriftScore(CalibrationContext &ctx)
{
	// Age alone can only ever say "aging": it saturates at 0.5 after 24 h,
	// below the stale threshold. Observed evidence is what crosses the line.
	// A continuously maintained calibration is not aging: each auto-applied
	// correction re-measures the alignment, so age counts from the later of
	// the manual solve and the last correction.
	double ageScore = 0.0;
	double ageBase = std::max(ctx.calibrationUnixTime, ctx.lastAutoCorrectionUnixTime);
	if (ageBase > 0.0)
	{
		double ageHours = (static_cast<double>(std::time(nullptr)) - ageBase) / 3600.0;
		ageScore = std::min(std::max(ageHours, 0.0) / 24.0, 1.0) * 0.5;
	}

	// A large slide dominates; repeated small events accumulate. Loss glitches
	// are weaker evidence than observed slides.
	double slideScore = std::min(ctx.driftMaxSlideM / 0.04, 1.0);
	double repeatScore = std::min((ctx.driftSlideEvents + 0.5 * ctx.discontinuousLossEvents) / 8.0, 1.0);
	double evidence = std::max(slideScore, repeatScore);

	// Corroboration gate: one slide window on one device is a measurement,
	// not a verdict — a resting body's posture creep passes the rest gates and
	// produces exactly one such window, and used to flip the rating straight
	// to Stale/Very Poor while the alignment looked visibly fine. Genuine
	// detector-band drift persists, so the monitor re-fires within ~one window
	// (its window clears per event) and the cap lifts almost immediately;
	// until then a lone event can only ever say "aging".
	if (ctx.driftSlideEvents + ctx.discontinuousLossEvents <= 1)
		evidence = std::min(evidence, 0.5);

	// Soft-OR: independent evidence compounds without exceeding 1.
	ctx.driftScore = 1.0 - (1.0 - ageScore) * (1.0 - evidence);

	ctx.alignment =
		ctx.driftScore >= 0.65 ? CalibrationContext::AlignmentHealth::Stale :
		ctx.driftScore >= 0.30 ? CalibrationContext::AlignmentHealth::Aging :
		CalibrationContext::AlignmentHealth::Fresh;

	// A healthy continuous loop re-measures the alignment constantly; the
	// rating already skips staleness for it, and toasting "quality looks poor"
	// while it is visibly being maintained is pure noise.
	bool maintained = ctx.continuousEnabled && ctx.mountExtrinsic.valid &&
		ctx.continuousState ==
			static_cast<int>(questcal::ContinuousAlignment::State::Tracking);
	if (ctx.alignment == CalibrationContext::AlignmentHealth::Stale && !maintained)
		NotifyStaleAlignment(ctx);
}

static void RuntimeMonitorTick(CalibrationContext &ctx, double now)
{
	// Score even while the monitors are parked so the UI's health readout
	// tracks calibration age from the moment a profile loads. Age alone
	// saturates below the stale threshold, so this can't notify by itself.
	if (ctx.validProfile)
		UpdateDriftScore(ctx);

	bool shouldRun = ctx.state == CalibrationState::None &&
		ctx.enabled && ctx.validProfile && PoseHub.RingOpen();

	if (!shouldRun)
	{
		if (MonitorActive)
		{
			Jumps->Reset();
			Drift->Reset();
			ResetMonitorIngestionTimes();
			MonitorActive = false;
		}
		PoseHub.DiscardBacklog(MonitorConsumer);
		return;
	}

	if (!MonitorActive)
	{
		PoseHub.DiscardBacklog(MonitorConsumer);
		ResetMonitorIngestionTimes();
		MonitorActive = true;
	}

	uint64_t dropped = PoseHub.Drain(MonitorConsumer, MonitorScratch);
	if (dropped > 0)
	{
		// We lost part of our own observation window; baselines across the
		// hole are unsafe. For the drift monitor a drain hole would read as a
		// tracking loss and could fake a discontinuity event.
		Jumps->Reset();
		Drift->Reset();
		ResetMonitorIngestionTimes();
	}

	// The HMD's latest raw position, for the worn-device heuristic below.
	// Persisted across ticks; only rough currency is needed.
	static Eigen::Vector3d lastHmdRawPos;
	static double lastHmdRawTime = -1e9;

	for (const auto &s : MonitorScratch)
	{
		if (s.deviceId >= vr::k_unMaxTrackedDeviceCount)
			continue;
		// JumpDetector owns the observation-continuity policy and must see bad
		// frames as well as good ones. Drift/HMD caches below remain valid-only.
		if (ctx.referenceDeviceMask[s.deviceId])
			Jumps->Push(s);

		bool sampleValid = s.poseIsValid &&
			s.trackingResult == static_cast<uint32_t>(vr::TrackingResult_Running_OK) &&
			IsUsableRingSample(s, QpcToSeconds);
		double composedTime = 0.0;
		questcal::PoseSample sample;
		if (sampleValid)
		{
			composedTime = RingSampleTime(s, QpcToSeconds);
			if (MonitorHasComposedTime[s.deviceId] &&
				composedTime <= MonitorLastComposedTime[s.deviceId])
				sampleValid = false;
			else
			{
				sample = EngineSampleFromRing(s);
				sampleValid = IsUsableEngineSample(sample);
			}
		}
		if (!sampleValid)
			continue;
		MonitorHasComposedTime[s.deviceId] = true;
		MonitorLastComposedTime[s.deviceId] = composedTime;

		if (sampleValid && s.deviceId == vr::k_unTrackedDeviceIndex_Hmd &&
			ctx.referenceDeviceMask[s.deviceId])
		{
			lastHmdRawPos = sample.pos;
			lastHmdRawTime = composedTime;
		}

		// Drift evidence must come from devices that anchor a universe. On the
		// reference side that is the HMD alone: its SLAM map IS the reference
		// universe, while reference-side peripherals (e.g. set-down Touch
		// controllers) IMU-coast with slowly sliding poses that still report
		// Running_OK -- centimeters of "slide" that say nothing about
		// alignment. Every lighthouse device is rigid to the target universe,
		// so target-system devices stay eligible -- except the HMD-mounted
		// tracker (rides a human head), and any lighthouse device within
		// arm's reach of the headset: that is a worn tracker (hips, feet) or a
		// device the user is right next to, and a supported resting body
		// passes the stationarity gates then "slides" with slow posture creep.
		bool anchorsUniverse =
			(s.deviceId == vr::k_unTrackedDeviceIndex_Hmd && ctx.referenceDeviceMask[s.deviceId]) ||
			(ctx.targetDeviceMask[s.deviceId] && s.deviceId != ctx.continuousTrackerId);

		if (anchorsUniverse && s.deviceId != vr::k_unTrackedDeviceIndex_Hmd &&
			composedTime - lastHmdRawTime < 3.0)
		{
			Eigen::Vector3d refPos = ctx.calibratedRotationQ
				* (ctx.calibratedScale * sample.pos) + ctx.TranslationMeters();
			if ((refPos - lastHmdRawPos).norm() < 1.2)
				anchorsUniverse = false;
		}

		if (anchorsUniverse)
			Drift->Push(s,
				ctx.targetDeviceMask[s.deviceId] ? ctx.calibratedScale : 1.0);
	}

	std::string note;
	while (Jumps->PollNote(note))
		ctx.Log(note + "\n");

	JumpDetector::GapEvent gap;
	while (Jumps->PollGap(gap))
	{
		ctx.referenceGapEvents++;
		char buf[256];
		snprintf(buf, sizeof buf,
			"Reference tracking gap (%.1f s) on device %u -- the universe may have moved; recalibrate if alignment looks off\n",
			gap.duration, gap.deviceId);
		ctx.Log(buf);
	}

	JumpDetector::UniverseDelta delta;
	bool jumped = false;
	while (Jumps->PollDelta(delta))
	{
		ApplyUniverseDelta(ctx, delta, now);
		jumped = true;
	}

	// A compensated jump moved the raw stream under the drift windows (an
	// exact wfd rebase can be small enough to read as a "slide" while still
	// leaving the window stationary); it was corrected, so it is not
	// staleness evidence. Jump acceptance always lands before the slide
	// window can conclude (~0.25 s vs ~2.5 s), so dropping the window here
	// also discards any queued event from the same discontinuity. The
	// continuous window straddles the same rebase and must refill too.
	if (jumped)
	{
		Drift->Reset();
		Continuous->Reset();
	}

	DriftMonitor::Event drift;
	while (Drift->PollEvent(drift))
	{
		char buf[256];
		if (drift.type == DriftMonitor::Event::StationarySlide)
		{
			ctx.driftSlideEvents++;
			ctx.driftMaxSlideM = std::max(ctx.driftMaxSlideM, drift.magnitude);
			snprintf(buf, sizeof buf,
				"Stationary device %u slid %.1f cm -- drift evidence\n",
				drift.deviceId, drift.magnitude * 100.0);
		}
		else
		{
			ctx.discontinuousLossEvents++;
			snprintf(buf, sizeof buf,
				"Device %u recovered %.1f cm away from where tracking dropped -- possible re-localization\n",
				drift.deviceId, drift.magnitude * 100.0);
		}
		ctx.Log(buf);
	}

}

// ---------------------------------------------------------------------------
// Continuous calibration (HMD-mounted tracker)

static void ContinuousTick(CalibrationContext &ctx, double now)
{
	bool shouldRun = ctx.state == CalibrationState::None &&
		ctx.enabled && ctx.validProfile && PoseHub.RingOpen() &&
		ctx.continuousEnabled && ctx.mountExtrinsic.valid &&
		ctx.continuousTrackerId < vr::k_unMaxTrackedDeviceCount &&
		ctx.referenceDeviceMask[vr::k_unTrackedDeviceIndex_Hmd];

	if (!shouldRun)
	{
		if (ContinuousActive)
		{
			Continuous->Reset();
			ContinuousActive = false;
		}
		PoseHub.DiscardBacklog(ContinuousConsumer);
		ctx.continuousState = static_cast<int>(Continuous->GetState());
		return;
	}

	if (!ContinuousActive)
	{
		PoseHub.DiscardBacklog(ContinuousConsumer);
		ContinuousActive = true;
	}

	// Cheap unconditional sync: the extrinsic changes only on recalibration
	// or profile load, but re-copying it every tick needs no bookkeeping.
	Continuous->SetExtrinsic(ctx.mountExtrinsic);
	Continuous->SetLatencyReestimation(ctx.continuousLatencyReestimation);

	uint64_t dropped = PoseHub.Drain(ContinuousConsumer, ContinuousScratch);
	if (dropped > 0)
	{
		// A drain hole could fake a discontinuity; baselines across it are unsafe.
		Continuous->Reset();
	}

	// The mounted tracker's latest raw position anchors the field lookup
	// below; it only has to be roughly current (the field varies over meters).
	static Eigen::Vector3d lastTrackerRawPos;
	static bool hasTrackerRawPos = false;

	for (const auto &s : ContinuousScratch)
	{
		if (s.deviceId >= vr::k_unMaxTrackedDeviceCount)
			continue;
		if (!s.poseIsValid ||
			s.trackingResult != static_cast<uint32_t>(vr::TrackingResult_Running_OK) ||
			!IsUsableRingSample(s, QpcToSeconds))
			continue;

		if (s.deviceId == vr::k_unTrackedDeviceIndex_Hmd)
		{
			questcal::PoseSample sample = EngineSampleFromRing(s);
			if (!IsUsableEngineSample(sample))
				continue;
			Continuous->PushReference(sample);
		}
		else if (s.deviceId == ctx.continuousTrackerId)
		{
			questcal::PoseSample sample = EngineSampleFromRing(s);
			if (!IsUsableEngineSample(sample))
				continue;
			Continuous->PushTarget(sample);
			lastTrackerRawPos = sample.pos;
			hasTrackerRawPos = true;
		}
	}

	// The deviation baseline is the transform expected AT THE TRACKER'S SPOT:
	// with field anchors active the true local alignment differs from base by
	// each anchor's own delta BY DESIGN, so comparing against raw base would
	// freeze on healthy anchors ("check the mount" while lying at an anchored
	// spot) and, below the freeze threshold, emit corrections that drag the
	// base -- and every anchor with it -- toward one spot's local deformation.
	// Corrections stay exact under this baseline: ApplyAlignmentDelta shifts
	// base and anchors together, so the blended expectation moves by exactly
	// the applied delta.
	Eigen::Quaterniond expectedRot = ctx.calibratedRotationQ;
	Eigen::Vector3d expectedTrans = ctx.TranslationMeters();
	if (ctx.fieldEnabled && !ctx.fieldAnchors.empty() && hasTrackerRawPos)
	{
		Eigen::Vector3d basePos = ctx.calibratedRotationQ
			* (ctx.calibratedScale * lastTrackerRawPos) + ctx.TranslationMeters();
		questcal::BlendedFieldCalibration(ctx.fieldAnchors, ctx.calibratedRotationQ,
			ctx.TranslationMeters(), basePos, expectedRot, expectedTrans);
	}

	// The engine's clock is the ring's (QPC seconds), not the UI clock.
	LARGE_INTEGER qnow;
	QueryPerformanceCounter(&qnow);
	double ringNow = static_cast<double>(qnow.QuadPart) * QpcToSeconds;
	Continuous->Update(ringNow, expectedRot, expectedTrans,
		ctx.calibratedScale, ctx.calibratedTimeOffset);

	questcal::ContinuousAlignment::Correction corr;
	while (Continuous->PollCorrection(corr))
	{
		ApplyAlignmentDelta(ctx, corr.rotation, corr.translation, /*snap=*/false, now);
		ctx.lastAutoCorrectionUnixTime = static_cast<double>(std::time(nullptr));
		ctx.autoCorrectionsApplied++;

		// The evidence these counters accumulated was just acted on (same
		// rationale as the post-jump reset); the monitor windows themselves
		// stay valid because a correction never moves raw poses.
		ctx.driftSlideEvents = 0;
		ctx.driftMaxSlideM = 0.0;
		ctx.discontinuousLossEvents = 0;
	}

	questcal::ContinuousAlignment::Event ev;
	while (Continuous->PollEvent(ev))
	{
		char buf[256];
		switch (ev.type)
		{
		case questcal::ContinuousAlignment::Event::FrozenLargeDeviation:
			if (ev.deviation.valid)
				snprintf(buf, sizeof buf,
					"Continuous calibration frozen: deviation yaw %.2f deg, tilt %.2f deg, %.1f cm\n",
					ev.deviation.yawDeg, ev.deviation.tiltDeg, ev.deviation.posM * 100.0);
			else
				snprintf(buf, sizeof buf,
					"Continuous calibration frozen: observation scatter %.2f deg / %.1f cm stays far above the tracking noise -- mount fault signature\n",
					Continuous->ScatterRotRmsDeg(), Continuous->ScatterPosRmsM() * 100.0);
			ctx.Log(buf);
			NotifyOnce(ctx, FreezeNotified,
				"The headset-mounted tracker moved or lost tracking -- alignment updates are on hold",
				"QuestCalibrator: the headset-mounted tracker moved or lost tracking. Alignment updates are on hold -- recalibrate to re-learn the mount.");
			break;
		case questcal::ContinuousAlignment::Event::ObservationsUnstable:
			snprintf(buf, sizeof buf,
				"Mounted tracker observations unstable (scatter %.2f deg / %.1f cm) -- alignment updates paused until tracking settles\n",
				Continuous->ScatterRotRmsDeg(), Continuous->ScatterPosRmsM() * 100.0);
			ctx.Log(buf);
			NotifyOnce(ctx, UnstableNotified,
				"Mounted tracker tracking is unstable -- alignment updates paused until it settles",
				"QuestCalibrator: the headset-mounted tracker's tracking looks unstable here. Alignment updates are paused and will resume on their own.");
			break;
		case questcal::ContinuousAlignment::Event::Resumed:
			ctx.Log("Continuous calibration resumed\n");
			FreezeNotified = false;
			break;
		case questcal::ContinuousAlignment::Event::TrackerLost:
			ctx.Log("Mounted tracker not tracking -- continuous calibration holding\n");
			break;
		case questcal::ContinuousAlignment::Event::TrackerRecovered:
			ctx.Log("Mounted tracker recovered -- continuous calibration gathering\n");
			break;
		}
	}

	// Opt-in online latency: EWMA toward the fresh measurement with a hard
	// per-update step clamp -- latency drifts slowly, and one bad correlation
	// must never yank the timeline. The applied poseTimeOffset only re-sends
	// when it moved perceptibly; sub-ms steps ride along with the next scan.
	double measuredOffset = 0.0;
	if (Continuous->PollTimeOffset(measuredOffset) && ctx.continuousLatencyReestimation)
	{
		double target = 0.75 * ctx.calibratedTimeOffset + 0.25 * measuredOffset;
		double step = target - ctx.calibratedTimeOffset;
		if (step > 0.002) step = 0.002;
		if (step < -0.002) step = -0.002;
		double updated = ctx.calibratedTimeOffset + step;
		if (updated > 0.060) updated = 0.060;
		if (updated < -0.060) updated = -0.060;

		double appliedBefore = questcal::ComputeAppliedTimeOffset(ctx.calibratedTimeOffset);
		double appliedAfter = questcal::ComputeAppliedTimeOffset(updated);
		ctx.calibratedTimeOffset = updated;
		ctx.MarkProfileDirty(now);
		if (std::abs(appliedAfter - appliedBefore) > 0.0005)
			SynchronizeDriverState(ctx);
	}

	ctx.continuousState = static_cast<int>(Continuous->GetState());
	ctx.continuousDeviation = Continuous->CurrentDeviation();
	ctx.continuousScatterRotDeg = Continuous->ScatterRotRmsDeg();
	ctx.continuousScatterPosM = Continuous->ScatterPosRmsM();
}

static void AbortCalibration(CalibrationContext &ctx, const std::string &reason)
{
	ctx.Log(reason + "\nAborting calibration!\n");
	ctx.state = CalibrationState::None;
	ctx.collectAsAnchor = false;
	ctx.ClearSampleBuffers();
}

// Store an anchor solve into the field. The anchor keeps the absolute solved
// transform; its delta against the base calibration is derived at send time.
static void StoreFieldAnchor(CalibrationContext &ctx, const questcal::EngineResult &result,
                             const Eigen::Vector3d &targetCentroid)
{
	// Where the fitted data actually lives, in reference space.
	Eigen::Vector3d centroidRef = result.rotation * targetCentroid + result.translation;

	// Magnitude gate: the field corrects small SLAM-map deformation. A large
	// disagreement with the base calibration means the universe moved since
	// the base solve -- recalibrating the base is the honest fix, not a huge
	// local patch.
	double rotDeltaDeg = result.rotation.angularDistance(ctx.calibratedRotationQ) * 180.0 / EIGEN_PI;
	Eigen::Vector3d baseHere = ctx.calibratedRotationQ * targetCentroid + ctx.TranslationMeters();
	double posDeltaM = (centroidRef - baseHere).norm();

	char buf[256];
	if (rotDeltaDeg > 8.0 || posDeltaM > 0.30)
	{
		snprintf(buf, sizeof buf,
			"Anchor rejected: %.1f deg / %.1f cm from the base calibration -- recalibrate the base instead\n",
			rotDeltaDeg, posDeltaM * 100.0);
		ctx.Log(buf);
		return;
	}

	// Replace an anchor measured near this spot (horizontal distance, matching
	// the driver's XZ blend metric); otherwise append.
	CalibrationContext::FieldAnchor anchor{ centroidRef, result.rotation, result.translation };
	auto previousAnchors = ctx.fieldAnchors;
	uint32_t previousGeneration = ctx.fieldGeneration;
	size_t slot = ctx.fieldAnchors.size();
	for (size_t i = 0; i < ctx.fieldAnchors.size(); ++i)
	{
		Eigen::Vector3d d = ctx.fieldAnchors[i].position - centroidRef;
		if (std::sqrt(d.x() * d.x() + d.z() * d.z()) < 1.0)
		{
			slot = i;
			break;
		}
	}

	if (slot == ctx.fieldAnchors.size())
	{
		if (ctx.fieldAnchors.size() >= protocol::SetAlignmentField::MaxAnchors)
		{
			ctx.Log("Anchor limit reached (8): collect within 1 m of an existing anchor to replace it, or clear the field\n");
			return;
		}
		ctx.fieldAnchors.push_back(anchor);
	}
	else
	{
		ctx.fieldAnchors[slot] = anchor;
	}

	ctx.fieldGeneration++;
	if (!SaveProfile(ctx))
	{
		ctx.fieldAnchors = std::move(previousAnchors);
		ctx.fieldGeneration = previousGeneration;
		ctx.Log("Field anchor was not applied because the updated profile could not be saved\n");
		return;
	}
	SynchronizeDriverState(ctx);

	snprintf(buf, sizeof buf, "Field anchor %zu stored at (%.2f, %.2f): %.2f deg / %.1f cm from base\n",
		slot + 1, centroidRef.x(), centroidRef.z(), rotDeltaDeg, posDeltaM * 100.0);
	ctx.Log(buf);
}

static void FinishCalibration(CalibrationContext &ctx)
{
	bool asAnchor = ctx.collectAsAnchor && ctx.validProfile;
	ctx.collectAsAnchor = false;

	questcal::EngineConfig config;
	// Scale stays global from the base solve; an anchor solve fits R,T on
	// target samples pre-scaled by it, so its absolute transform composes with
	// the driver's scale-then-transform application unchanged.
	config.solveScale = ctx.solveScale && !asAnchor;
	if (asAnchor && ctx.calibratedScale != 1.0)
	{
		for (auto &s : ctx.targetSamples)
		{
			s.pos *= ctx.calibratedScale;
			s.vel *= ctx.calibratedScale;
		}
	}

	char buf[512];
	snprintf(buf, sizeof buf, "Collected %zu reference / %zu target samples, solving%s...\n",
		ctx.refSamples.size(), ctx.targetSamples.size(), asAnchor ? " (field anchor)" : "");
	ctx.Log(buf);

	questcal::EngineResult result = questcal::CalibrationEngine::Solve(ctx.refSamples, ctx.targetSamples, config);
	ctx.lastResult = result;

	Eigen::Vector3d targetCentroid = Eigen::Vector3d::Zero();
	if (!ctx.targetSamples.empty())
	{
		for (const auto &s : ctx.targetSamples)
			targetCentroid += s.pos;
		targetCentroid /= static_cast<double>(ctx.targetSamples.size());
	}

	// Mount extrinsic for continuous calibration: a head-referenced base
	// solve's buffers directly measure the tracker's pose in the HMD frame.
	// Must run before the buffers are cleared. The rigidity gate keeps a
	// hand-held calibration from arming continuous mode; on failure any
	// previously learned mount (which did not move just because this solve
	// happened without the tracker) is kept.
	if (result.valid && !asAnchor && ctx.calibrationReferenceID == vr::k_unTrackedDeviceIndex_Hmd)
	{
		questcal::ContinuousAlignment::Config contCfg;
		questcal::MountExtrinsic extrinsic;
		if (questcal::ContinuousAlignment::DeriveMountExtrinsic(
			ctx.refSamples, ctx.targetSamples, result, contCfg, extrinsic))
		{
			std::string serial;
			if (ReadTrackedDeviceString(ctx.calibrationTargetID,
				vr::Prop_SerialNumber_String, serial))
			{
				// The learned transform and physical-device identity are one
				// credential. Commit neither if the checked serial read failed.
				ctx.mountExtrinsic = extrinsic;
				ctx.continuousTrackerSerial = serial;
				snprintf(buf, sizeof buf,
					"Mount offset learned for continuous calibration (%.2f deg / %.1f mm spread, %zu pairs)\n",
					extrinsic.rotRmsDeg, extrinsic.posRmsM * 1000.0, extrinsic.pairs);
				ctx.Log(buf);
			}
			else
			{
				ctx.Log("The mounted target's serial could not be read safely -- keeping the previous mount offset\n");
			}
		}
		else if (ctx.continuousEnabled)
		{
			ctx.Log(ctx.mountExtrinsic.valid
				? "The target device was not rigid on the headset -- keeping the previous mount offset\n"
				: "The target device was not rigid on the headset -- continuous calibration needs a mounted tracker\n");
		}
	}

	ctx.ClearSampleBuffers();
	ctx.state = CalibrationState::None;

	snprintf(buf, sizeof buf,
		"Rotation residual %.2f deg, position residual %.1f cm\n"
		"Time offset %+.1f ms, axis spread %.3f, %zu pairs (%zu rejected), %zu samples gated\n",
		result.rotationRmsDeg, result.translationRmsMeters * 100.0,
		result.timeOffset * 1000.0, result.axisSpread,
		result.pairsUsed, result.pairsRejected, result.samplesGated);
	ctx.Log(buf);

	// Scale-artifact diagnostic: flat gains = genuine metric difference; fine
	// below gross = the reference stream smooths motion and the solved scale
	// is contaminated (the guard then takes scale from the gross band).
	if (result.motionGainValid)
	{
		snprintf(buf, sizeof buf,
			"Motion gain (reference vs target): %.3f gross / %.3f fine%s\n",
			result.motionGainLow, result.motionGainHigh,
			result.scaleFromGrossMotion
				? " -- fine motion attenuated; scale taken from clean gross motion"
				: (result.scaleNeutralizedForSmoothing
					? " -- gross and fine motion attenuated; scale held at neutral 1.0"
					: (result.motionSmoothingDetected
						? " -- streamed-pose smoothing detected"
						: "")));
		ctx.Log(buf);
	}

	if (!result.valid)
	{
		ctx.Log("Calibration failed: " + result.message + "\n");
		return;
	}

	if (asAnchor)
	{
		StoreFieldAnchor(ctx, result, targetCentroid);
		return;
	}

	// Commit the tracking-system identity atomically with the successful base
	// solve. The UI's pending selection must never redirect an old transform.
	ctx.referenceTrackingSystem = ctx.calibrationReferenceTrackingSystem;
	ctx.targetTrackingSystem = ctx.calibrationTargetTrackingSystem;
	ctx.SetCalibration(result.rotation, result.translation, result.scale);
	ctx.calibratedTimeOffset = result.timeOffset;
	ctx.validProfile = true;

	// A base recalibration re-measures the whole alignment; anchors solved
	// against the previous alignment would fight it. Start the field fresh.
	if (!ctx.fieldAnchors.empty())
	{
		ctx.fieldAnchors.clear();
		ctx.Log("Field anchors cleared (base recalibration)\n");
	}
	ctx.fieldGeneration++;

	// A fresh solve resets the staleness clock and all accumulated drift
	// evidence, and re-arms the one-shot stale notification.
	ctx.calibrationUnixTime = static_cast<double>(std::time(nullptr));
	ctx.driftSlideEvents = 0;
	ctx.driftMaxSlideM = 0.0;
	ctx.discontinuousLossEvents = 0;
	ctx.driftScore = 0.0;
	ctx.alignment = CalibrationContext::AlignmentHealth::Fresh;
	StaleNotified = false;
	FreezeNotified = false;
	UnstableNotified = false;
	ctx.lastAutoCorrectionUnixTime = 0.0;
	ctx.autoCorrectionsApplied = 0;
	Drift->Reset();

	bool priorUniverseUnsafe = ctx.profileUniverseUnsafe;
	ctx.profileUniverseUnsafe = false;
	ctx.AdvancePersistenceRevision();
	if (ctx.chaperone.valid &&
		(ctx.chaperone.ownerTrackingSystem != ctx.referenceTrackingSystem ||
			priorUniverseUnsafe ||
			!ChaperoneBaselineIsCurrent(ctx.chaperone)))
	{
		ctx.DisarmChaperone();
		ctx.ReportError(
			"The protected chaperone is not anchored to the newly calibrated reference universe. "
			"It has been disarmed; capture it again for this profile.\n",
			CalibrationContext::ErrorSource::Chaperone);
	}
	ctx.MarkProfileAndSettingsDirty(ctx.timeLastTick);
	bool saved = SaveDirtyPersistence(ctx);
	SynchronizeDriverState(ctx);
	ctx.Log(saved ? "Finished calibration, profile and settings saved\n"
		: "Finished calibration and applied it for this session, but all persistence writes did not complete\n");

	if (result.scale != 1.0)
	{
		snprintf(buf, sizeof buf, "Playspace scale: %.4f\n", result.scale);
		ctx.Log(buf);
	}
}

bool StartCalibration()
{
	if (!questcal::IsValidTrackingSystemPair(
		CalCtx.pendingReferenceTrackingSystem,
		CalCtx.pendingTargetTrackingSystem))
	{
		CalCtx.ReportError("Choose two different tracking systems before starting calibration\n");
		return false;
	}

	CalCtx.calibrationReferenceTrackingSystem = CalCtx.pendingReferenceTrackingSystem;
	CalCtx.calibrationTargetTrackingSystem = CalCtx.pendingTargetTrackingSystem;
	CalCtx.calibrationReferenceID = CalCtx.referenceID;
	CalCtx.calibrationTargetID = CalCtx.targetID;
	CalCtx.collectAsAnchor = false;
	CalCtx.state = CalibrationState::Begin;
	CalCtx.wantedUpdateInterval = 0.0;
	CalCtx.messages.clear();
	return true;
}

bool StartAnchorCalibration()
{
	if (CalCtx.pendingReferenceTrackingSystem != CalCtx.referenceTrackingSystem ||
		CalCtx.pendingTargetTrackingSystem != CalCtx.targetTrackingSystem)
	{
		CalCtx.ReportError("Select the active profile's tracking systems before collecting a field anchor\n");
		return false;
	}
	if (!StartCalibration())
		return false;
	CalCtx.collectAsAnchor = true;
	return true;
}

void CalibrationTick(double time)
{
	auto &ctx = CalCtx;
	// Persistence is independent of SteamVR, profile validity, and the current
	// UI/calibration state. In particular, settings-only retries must continue
	// while the runtime is unavailable or the advanced editor is open.
	PersistenceTick(ctx, time);

	if (!vr::VRSystem())
		return;

	if ((time - ctx.timeLastTick) < 0.02)
		return;

	ctx.timeLastTick = time;

	// RawAndUncalibrated is deliberate: the solve must see pre-calibration
	// poses. "Fixing" this to Standing breaks calibration entirely.
	vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0.0f, ctx.devicePoses, vr::k_unMaxTrackedDeviceCount);

	RuntimeMonitorTick(ctx, time);
	ChaperoneMonitorTick(ctx, time);
	// After the monitors: an accepted jump must land (and reset the continuous
	// window) before the continuous loop reads the calibration this tick.
	ContinuousTick(ctx, time);

	if (ctx.state == CalibrationState::None)
	{
		ctx.wantedUpdateInterval = 1.0;

		if ((time - ctx.timeLastScan) >= 1.0)
		{
			SynchronizeDriverState(ctx);
			CheckProtectedChaperone(ctx);
			ctx.timeLastScan = time;
		}
		return;
	}

	if (ctx.state == CalibrationState::Editing)
	{
		ctx.wantedUpdateInterval = 1.0;

		if ((time - ctx.timeLastScan) >= 1.0)
		{
			// Editor values are draft-only. Keep reasserting the last committed
			// profile in case vrserver restarts while the editor is open.
			SynchronizeDriverState(ctx);
			CheckProtectedChaperone(ctx);
			ctx.timeLastScan = time;
		}
		return;
	}

	if (ctx.state == CalibrationState::Begin)
	{
		bool ok = true;

		if (ctx.calibrationReferenceID >= vr::k_unMaxTrackedDeviceCount)
		{
			ctx.Log("Missing reference device\n");
			ok = false;
		}
		else if (!ctx.devicePoses[ctx.calibrationReferenceID].bPoseIsValid)
		{
			ctx.Log("Reference device is not tracking\n");
			ok = false;
		}

		if (ctx.calibrationTargetID >= vr::k_unMaxTrackedDeviceCount)
		{
			ctx.Log("Missing target device\n");
			ok = false;
		}
		else if (!ctx.devicePoses[ctx.calibrationTargetID].bPoseIsValid)
		{
			ctx.Log("Target device is not tracking\n");
			ok = false;
		}

		if (!ok)
		{
			ctx.state = CalibrationState::None;
			ctx.Log("Aborting calibration!\n");
			return;
		}

		auto matchesCapturedSystem = [](uint32_t id, const std::string &expected)
		{
			std::string system;
			return ReadTrackedDeviceString(id,
				vr::Prop_TrackingSystemName_String, system) && expected == system;
		};
		if (!matchesCapturedSystem(ctx.calibrationReferenceID,
			ctx.calibrationReferenceTrackingSystem) ||
			!matchesCapturedSystem(ctx.calibrationTargetID,
				ctx.calibrationTargetTrackingSystem))
		{
			ctx.state = CalibrationState::None;
			ctx.Log("Selected devices no longer belong to the chosen tracking systems; calibration aborted\n");
			return;
		}

		std::string referenceSerial;
		std::string targetSerial;
		ReadTrackedDeviceString(ctx.calibrationReferenceID,
			vr::Prop_SerialNumber_String, referenceSerial);
		ReadTrackedDeviceString(ctx.calibrationTargetID,
			vr::Prop_SerialNumber_String, targetSerial);

		char buf[256];
		snprintf(buf, sizeof buf, "Reference device ID: %u, serial: %s\n",
			ctx.calibrationReferenceID, referenceSerial.c_str());
		ctx.Log(buf);
		snprintf(buf, sizeof buf, "Target device ID: %u, serial: %s\n",
			ctx.calibrationTargetID, targetSerial.c_str());
		ctx.Log(buf);

		if (!ResetAndDisableOffsets(ctx, ctx.calibrationTargetID))
		{
			ctx.state = CalibrationState::None;
			ctx.Log("Could not disable the existing target transform; calibration aborted\n");
			return;
		}

		ctx.ClearSampleBuffers();
		if (PoseHub.RingOpen())
		{
			DiscardPoseRingBacklog();
			ctx.Log("Sampling raw driver poses (timestamped)\n");
		}
		else
		{
			ctx.Log("Pose channel unavailable, falling back to runtime poses\n");
		}

		ctx.collectionStart = time;
		ctx.lastRefSampleTime = time;
		ctx.lastTargetSampleTime = time;
		ctx.state = CalibrationState::Collecting;
		ctx.wantedUpdateInterval = 0.0;

		if (ctx.calibrationReferenceID == vr::k_unTrackedDeviceIndex_Hmd)
			ctx.Log("Wear the headset with the tracker firmly mounted.\nSlowly turn and tilt your head in wide arcs, and lean side to side.\n");
		else
			ctx.Log("Hold the devices firmly together.\nSlowly move them in wide circles, both side to side and up and down.\n");
		return;
	}

	// ---- Collecting ----
	if (PoseHub.RingOpen())
	{
		if (!CollectFromPoseRing(ctx, time))
			return;
	}
	else
		CollectFromRuntimePoses(ctx, time);

	if (time - ctx.lastRefSampleTime > 2.0)
	{
		AbortCalibration(ctx, "Reference device stopped tracking");
		return;
	}
	if (time - ctx.lastTargetSampleTime > 2.0)
	{
		AbortCalibration(ctx, "Target device stopped tracking");
		return;
	}

	double duration = ctx.CollectionSeconds();
	double elapsed = time - ctx.collectionStart;
	ctx.Progress(static_cast<int>(elapsed * 100.0), static_cast<int>(duration * 100.0));

	if (elapsed >= duration)
		FinishCalibration(ctx);
}

static bool FiniteChaperone(const CalibrationContext::Chaperone &snapshot)
{
	return questcal::IsPlausibleChaperone(snapshot.geometry,
		snapshot.standingCenter, snapshot.playSpaceSize);
}

static bool FailClosedChaperoneCapture(const std::string &message)
{
	// A failed replacement means the previously captured room may now be
	// stale. Keep its data for diagnosis/manual recapture, but make it neither
	// auto- nor manually restorable until a current baseline is verified.
	bool persistDisarm = CalCtx.chaperone.valid;
	if (persistDisarm)
	{
		CalCtx.chaperone.autoApply = false;
		CalCtx.chaperone.baselineVerifiedThisSession = false;
		CalCtx.MarkSettingsDirty(CalCtx.timeLastTick);
	}
	CalCtx.ReportError(message, CalibrationContext::ErrorSource::Chaperone);
	if (persistDisarm && !SaveSettings(CalCtx))
	{
		// SaveSettings preserves dirty on a failed record write; restate it for
		// failures that occurred before the low-level Settings write as well.
		CalCtx.MarkSettingsDirty(CalCtx.timeLastTick);
	}
	return false;
}

bool LoadChaperoneBounds()
{
	if (CalCtx.profileUniverseUnsafe)
		return FailClosedChaperoneCapture(
			"Could not protect the chaperone: run a new base calibration first because the previous profile lost raw-universe continuity\n");

	auto setup = vr::VRChaperoneSetup();
	if (!setup)
		return FailClosedChaperoneCapture(
			"Could not protect the chaperone: OpenVR chaperone setup is unavailable\n");

	setup->RevertWorkingCopy();
	CalibrationContext::Chaperone snapshot;
	snapshot.autoApply = true;
	if (!ReadCurrentHmdIdentity(snapshot.ownerTrackingSystem, snapshot.ownerHmdSerial))
		return FailClosedChaperoneCapture(
			"Could not protect the chaperone: the current headset/runtime identity is unavailable\n");
	if (CalCtx.validProfile &&
		snapshot.ownerTrackingSystem != CalCtx.referenceTrackingSystem)
		return FailClosedChaperoneCapture(
			"Could not protect the chaperone: the active profile uses a different reference "
			"tracking system than the headset which owns this play area\n");
	if (!CopyCurrentHmdWorldFromDriver(snapshot))
		return FailClosedChaperoneCapture(
			"Could not protect the chaperone: no fresh validated HMD raw-universe pose is available yet\n");

	uint32_t quadCount = 0;
	if (!setup->GetLiveCollisionBoundsInfo(nullptr, &quadCount) || quadCount > 16384)
		return FailClosedChaperoneCapture(
			"Could not protect the chaperone: failed to read the live wall count\n");

	snapshot.geometry.resize(quadCount);

	if (quadCount != 0)
	{
		uint32_t returnedCount = quadCount;
		if (!setup->GetLiveCollisionBoundsInfo(snapshot.geometry.data(), &returnedCount) ||
			returnedCount > quadCount)
			return FailClosedChaperoneCapture(
				"Could not protect the chaperone: failed to read the live walls\n");
		snapshot.geometry.resize(returnedCount);
		quadCount = returnedCount;
	}

	if (!setup->GetWorkingStandingZeroPoseToRawTrackingPose(&snapshot.standingCenter) ||
		!setup->GetWorkingPlayAreaSize(&snapshot.playSpaceSize.v[0], &snapshot.playSpaceSize.v[1]) ||
		!FiniteChaperone(snapshot))
		return FailClosedChaperoneCapture(
			"Could not protect the chaperone: play-area data is missing or invalid\n");

	std::time_t copyTime = std::time(nullptr);
	double copyUnixTime = static_cast<double>(copyTime);
	if (copyTime == static_cast<std::time_t>(-1) ||
		!std::isfinite(copyUnixTime) || copyUnixTime < 0.0 ||
		copyUnixTime > protocol::limits::MaxPlausibleUnixTimeSeconds)
		return FailClosedChaperoneCapture(
			"Could not protect the chaperone: the capture time is unavailable or invalid\n");

	snapshot.copyUnixTime = copyUnixTime;
	snapshot.valid = true;

	// Replacement is a two-stage transaction. Persistently disarm any valid old
	// snapshot before replacing it in memory, so a failure writing the new
	// snapshot cannot resurrect an older armed record after restart. Do this even
	// when the in-memory copy is already disarmed: its prior disarm write may have
	// failed, leaving the registry copy armed.
	if (CalCtx.chaperone.valid)
	{
		CalCtx.chaperone.autoApply = false;
		CalCtx.chaperone.baselineVerifiedThisSession = false;
		CalCtx.MarkSettingsDirty(CalCtx.timeLastTick);
		if (!SaveSettings(CalCtx))
		{
			CalCtx.MarkSettingsDirty(CalCtx.timeLastTick);
			CalCtx.ReportError(
				"Could not protect the chaperone: the existing snapshot could not be "
				"safely disarmed before replacement; no new snapshot was installed\n",
				CalibrationContext::ErrorSource::Chaperone);
			return false;
		}
	}

	CalCtx.chaperone = std::move(snapshot);
	if (!SaveSettings(CalCtx))
	{
		// Do not resurrect the previous (possibly stale and armed) snapshot.
		// Retain the newly captured data only in a fail-closed state so the
		// centralized retry can persist it without ever auto-restoring it first.
		CalCtx.chaperone.autoApply = false;
		CalCtx.chaperone.baselineVerifiedThisSession = false;
		CalCtx.MarkSettingsDirty(CalCtx.timeLastTick);
		CalCtx.Log("The new chaperone snapshot was kept disarmed because it could not be persisted\n");
		return false;
	}
	CalCtx.ClearError(CalibrationContext::ErrorSource::Chaperone);

	char buf[128];
	if (quadCount == 0)
		snprintf(buf, sizeof buf,
			"Chaperone snapshot saved: play area center only (no chaperone walls yet) -- auto-restore stays inactive\n");
	else
		snprintf(buf, sizeof buf,
			"Chaperone snapshot saved: %u wall segment(s), %.1f x %.1f m play area\n",
			quadCount, CalCtx.chaperone.playSpaceSize.v[0], CalCtx.chaperone.playSpaceSize.v[1]);
	CalCtx.Log(buf);
	return true;
}

bool ApplyChaperoneBounds(bool logSuccess)
{
	if (!CalCtx.chaperone.valid || !FiniteChaperone(CalCtx.chaperone))
	{
		CalCtx.ReportError("Could not restore the chaperone: the protected snapshot is invalid\n",
			CalibrationContext::ErrorSource::Chaperone);
		return false;
	}
	ChaperoneOwnerStatus owner = CurrentChaperoneOwner(CalCtx.chaperone);
	if (owner == ChaperoneOwnerStatus::Unavailable)
	{
		CalCtx.ReportError(
			"Could not restore the chaperone: the current headset/runtime identity is unavailable\n",
			CalibrationContext::ErrorSource::Chaperone);
		return false;
	}
	if (owner == ChaperoneOwnerStatus::Mismatch)
	{
		CalCtx.DisarmChaperone();
		CalCtx.MarkSettingsDirty(CalCtx.timeLastTick);
		CalCtx.ReportError(
			"Could not restore the chaperone: the snapshot belongs to a different headset/runtime. "
			"It has been disarmed; capture it again for this headset.\n",
			CalibrationContext::ErrorSource::Chaperone);
		SaveSettings(CalCtx);
		return false;
	}
	if (!ChaperoneBaselineIsCurrent(CalCtx.chaperone))
	{
		CalCtx.ReportError(
			"Could not restore the chaperone: its raw-universe baseline has not been verified against a current HMD pose\n",
			CalibrationContext::ErrorSource::Chaperone);
		return false;
	}
	auto setup = vr::VRChaperoneSetup();
	if (!setup)
	{
		CalCtx.ReportError("Could not restore the chaperone: OpenVR chaperone setup is unavailable\n",
			CalibrationContext::ErrorSource::Chaperone);
		return false;
	}

	setup->RevertWorkingCopy();
	if (!CalCtx.chaperone.geometry.empty())
		setup->SetWorkingCollisionBoundsInfo(CalCtx.chaperone.geometry.data(),
			static_cast<uint32_t>(CalCtx.chaperone.geometry.size()));
	setup->SetWorkingStandingZeroPoseToRawTrackingPose(&CalCtx.chaperone.standingCenter);
	setup->SetWorkingPlayAreaSize(CalCtx.chaperone.playSpaceSize.v[0], CalCtx.chaperone.playSpaceSize.v[1]);
	if (!setup->CommitWorkingCopy(vr::EChaperoneConfigFile_Live))
	{
		setup->RevertWorkingCopy();
		CalCtx.ReportError("Could not restore the chaperone: SteamVR rejected the update\n",
			CalibrationContext::ErrorSource::Chaperone);
		return false;
	}

	// Verify every value that OpenVR exposes after commit. Setters are void,
	// so this is the only truthful way to know the live update landed.
	bool verified = true;
	if (!CalCtx.chaperone.geometry.empty())
	{
		uint32_t count = 0;
		verified = setup->GetLiveCollisionBoundsInfo(nullptr, &count) &&
			count == CalCtx.chaperone.geometry.size();
		if (verified)
		{
			std::vector<vr::HmdQuad_t> live(count);
			verified = setup->GetLiveCollisionBoundsInfo(live.data(), &count) &&
				questcal::QuadsMatch(live, CalCtx.chaperone.geometry, 0.002f);
		}
	}
	setup->RevertWorkingCopy();
	vr::HmdMatrix34_t standing = {};
	vr::HmdVector2_t size = {};
	verified = verified && setup->GetWorkingStandingZeroPoseToRawTrackingPose(&standing) &&
		setup->GetWorkingPlayAreaSize(&size.v[0], &size.v[1]);
	for (int row = 0; verified && row < 3; ++row)
		for (int column = 0; verified && column < 4; ++column)
			verified = std::abs(standing.m[row][column] -
				CalCtx.chaperone.standingCenter.m[row][column]) <= 0.002f;
	verified = verified && std::abs(size.v[0] - CalCtx.chaperone.playSpaceSize.v[0]) <= 0.002f &&
		std::abs(size.v[1] - CalCtx.chaperone.playSpaceSize.v[1]) <= 0.002f;
	if (!verified)
	{
		CalCtx.ReportError("Could not verify the restored chaperone; check SteamVR Room Setup before relying on it\n",
			CalibrationContext::ErrorSource::Chaperone);
		return false;
	}
	if (logSuccess)
	{
		CalCtx.Log("Protected chaperone restored successfully\n");
	}
	CalCtx.ClearError(CalibrationContext::ErrorSource::Chaperone);
	return true;
}
