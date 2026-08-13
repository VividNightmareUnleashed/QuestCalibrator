#include "stdafx.h"
#include "Calibration.h"
#include "CalibrationEngine.h"
#include "ChaperoneMath.h"
#include "Configuration.h"
#include "DriftMonitor.h"
#include "DriverSession.h"
#include "DriverSyncPolicy.h"
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
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

static IPCClient Driver;
static PoseStreamHub PoseHub;
static int CollectorConsumer = -1;
static std::vector<protocol::DevicePoseSample> CollectorScratch;
static double QpcToSeconds = 0.0;

// The sample source pinned at the Begin transition. The ring stamps ring/QPC
// seconds and the runtime-pose fallback stamps the UI clock into the same
// buffers, so a channel that opens or closes mid-run would concatenate two
// timelines behind a monotonicity guard that cannot tell them apart.
static bool CollectionUsesPoseRing = false;
// UI-clock time of the last frozen-pair identity re-check during Collecting.
static double LastCollectionIdentityCheck = -1e9;

static std::unique_ptr<JumpDetector> Jumps;
static std::unique_ptr<DriftMonitor> Drift;
static int MonitorConsumer = -1;
static std::vector<protocol::DevicePoseSample> MonitorScratch;
static bool MonitorActive = false;

enum class ChaperoneOwnerStatus
{
	Match,
	Mismatch,
	// The snapshot records no owner at all. Distinct from Mismatch: it is an
	// incomplete record, not another headset's room, and saying "belongs to a
	// different headset" for it contradicts what the load path reports.
	Unowned,
	Unavailable
};

// Everything the per-tick runtime monitors keep between ticks that does not
// live inside a detector object. Collected here so "what is the monitor's
// state" has one answer and the half that is inferred from an unbroken pose
// stream is cleared wherever the detectors themselves are reset.
struct MonitorState
{
	// Per-device composed-time watermarks (see RuntimeMonitorTick).
	bool hasComposedTime[vr::k_unMaxTrackedDeviceCount] = {};
	double lastComposedTime[vr::k_unMaxTrackedDeviceCount] = {};

	// The HMD's latest raw position, for the worn-device heuristic. Only rough
	// currency is needed, but a stale one suppresses drift evidence, so it must
	// not survive a hole the detectors were reset across.
	Eigen::Vector3d hmdRawPosition{ 0, 0, 0 };
	double hmdRawTime = -1e9;

	// Chaperone owner check: device properties are stable for a session but
	// comparatively expensive, so the verdict is cached and rechecked at 1 Hz
	// or whenever the snapshot's recorded ownership changes.
	double lastOwnerCheck = -1e9;
	std::string checkedTrackingSystem;
	std::string checkedHmdSerial;
	ChaperoneOwnerStatus owner = ChaperoneOwnerStatus::Unavailable;

	// 30 s rate limits on the protected-chaperone reader's two failure banners.
	double lastSetupFailure = -1e9;
	double lastReadFailure = -1e9;

	// One notification per calibration: re-armed only by a successful solve
	// (the freeze one also by a resume event).
	bool staleNotified = false;
	bool freezeNotified = false;
	bool unstableNotified = false;

	// Only the stream-derived half. The owner cache invalidates on its own key,
	// the failure cooldowns are session-rate policy, and the one-shot flags are
	// per-calibration policy -- none of them describe the observation window, so
	// a drain hole must not clear them.
	void ResetObservations()
	{
		for (bool &hasTime : hasComposedTime)
			hasTime = false;
		hmdRawPosition = Eigen::Vector3d::Zero();
		hmdRawTime = -1e9;
	}

	void ReArmNotifications()
	{
		staleNotified = false;
		freezeNotified = false;
		unstableNotified = false;
	}
};

static MonitorState Monitors;

// The whole driver conversation: the slot ledger, the connection generation it
// last converged, the error debounce and the send sequencing (DriverSession.h).
// This file owns only the two things the session cannot name -- the real pipe
// and the OpenVR enumeration -- and hands them over as injected seams.
//
// The IPCClient instance stays here rather than inside the session because
// DriverSession.h has to remain free of windows.h: the test translation unit
// includes it beside openvr_driver.h, and IPCClient.h needs HANDLE and DWORD.
static questcal::DriverSession DriverLink;

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

CalibrationContext CalCtx;

static constexpr const char *CalibrationAbortedMessage =
	"Calibration aborted to avoid using bad data. Please try again.\n";
static void AbortCalibration(CalibrationContext &ctx, const std::string &reason);
static void ChaperoneMonitorTick(CalibrationContext &ctx, double now);

static bool SaveDirtyPersistence(CalibrationContext &ctx)
{
	// SaveSettings owns the ordering: it commits a dirty Config first, and only
	// a coupled revision bump lets a Config failure hold the Settings half back.
	// A crash (or a failure) after that first write is detected by the shared
	// revision at the next launch.
	if (ctx.settingsSaveDirty)
		return SaveSettings(ctx);
	if (ctx.profileSaveDirty)
	{
		if (!ctx.validProfile)
		{
			ctx.ReportError(PendingProfileWithoutValidProfileMessage,
				CalibrationContext::ErrorSource::ProfilePersistence);
			return false;
		}
		if (!SaveProfile(ctx))
			return false;
	}
	ctx.persistenceCoupled = false;
	return true;
}

static void PersistenceTick(CalibrationContext &ctx, double now)
{
	if (!ctx.HasDirtyPersistence())
		return;

	// The quiet period avoids a registry write per continuous correction, but a
	// steadily drifting universe never goes quiet: force the write once the
	// dirty streak passes the ceiling so a crash cannot discard a whole session.
	if (now - ctx.persistenceDirtyTime <= 5.0 &&
		now - ctx.persistenceFirstDirtyTime <= 60.0)
		return;

	if (!SaveDirtyPersistence(ctx))
		ctx.DelayPersistenceRetry(now);
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
	// "Driver not reachable" is an observable property, not a startup
	// precondition: SteamVR auto-launch can beat vrserver's driver load, and a
	// throw here would kill the overlay before LoadProfile, exactly when the
	// user needs the UI to inspect or clear the profile. SendBlocking
	// reconnects (with the same exact-version handshake) on the first request,
	// and until then SynchronizeDriverState fails closed with enabled = false
	// and the Driver banner reports it, as after any mid-session restart.
	try
	{
		Driver.Connect();
	}
	catch (const std::exception &e)
	{
		AppendSessionLog(std::string("Driver not reachable at startup: ") + e.what());
	}

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

static_assert(vr::k_unTrackedDeviceIndex_Hmd == 0, "HMD index expected to be 0");

// "Which anchors is the driver blending right now" has exactly one answer, and
// both the sender and the continuous loop must read it from here: they used to
// spell the predicate out separately and already disagreed, so a toggle could
// leave the loop comparing against raw base while the driver still applied
// anchor deltas.
static const std::vector<CalibrationContext::FieldAnchor> &ActiveFieldAnchors(
	const CalibrationContext &ctx)
{
	static const std::vector<CalibrationContext::FieldAnchor> none;
	return ctx.fieldEnabled ? ctx.fieldAnchors : none;
}

// The spatial correction field as a value, not a send: deltas are derived here
// against the current base calibration, and DriverSession decides whether the
// batch earned the right to ship them. delta_i = anchor_i o base^-1 is the
// correction that, applied after the base calibration, reproduces the absolute
// solve at that anchor's spot.
//
// `enabled` carries only this side's half of the predicate -- the profile is
// live and there is something to blend. The session ANDs in "a complete base
// batch reached one connection" and builds the canonical disable itself, so the
// anchor derivation below is never even attempted against a base the validity
// gates already rejected.
static protocol::SetAlignmentField BuildAlignmentField(const CalibrationContext &ctx)
{
	protocol::SetAlignmentField f;

	const auto &anchors = ActiveFieldAnchors(ctx);
	f.enabled = ctx.enabled && !anchors.empty();
	f.generation = ctx.fieldGeneration;
	// Ship the width explicitly rather than leaning on the struct default
	// coinciding with what the overlay blends: the driver shapes its field from
	// whatever arrives here, so this assignment is what keeps the driver's
	// blend and ContinuousTick's expectation the same function.
	f.sigmaMeters = questcal::FieldBlendSigmaMeters;
	if (!f.enabled)
		return f;

	f.anchorCount = static_cast<uint32_t>(std::min(
		anchors.size(), static_cast<size_t>(protocol::SetAlignmentField::MaxAnchors)));

	Eigen::Quaterniond baseInv = ctx.calibratedRotationQ.conjugate();
	Eigen::Vector3d baseT = ctx.TranslationMeters();
	for (uint32_t i = 0; i < f.anchorCount; ++i)
	{
		const auto &a = anchors[i];
		Eigen::Quaterniond dR;
		Eigen::Vector3d dT;
		questcal::AnchorDelta(a.rotation, a.translationMeters, baseInv, baseT, dR, dT);

		for (int k = 0; k < 3; ++k)
		{
			f.anchors[i].position[k] = a.position(k);
			f.anchors[i].translationDelta[k] = dT(k);
		}
		f.anchors[i].rotationDelta = questcal::WireQuaternion(dR);
	}

	return f;
}

static bool ReadTrackedDeviceString(uint32_t id,
	vr::ETrackedDeviceProperty property, std::string &value)
{
	value.clear();
	auto system = vr::VRSystem();
	if (!system || id >= vr::k_unMaxTrackedDeviceCount)
		return false;

	// Sized for what this function is actually used to read -- tracking-system
	// names, serials and icon paths, all tens of bytes -- rather than for
	// k_unMaxPropertyStringSize, which value-initialises a 32 KB stack frame on
	// every one of the ~70 property reads each 1 Hz scan makes. An oversized
	// property degrades into the buffer-too-small error the check below already
	// fails closed on. The zero-initialisation stays: the NUL check reads
	// buffer[size - 1], which a misbehaving runtime could point at a byte it
	// never wrote.
	char buffer[256] = {};
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

// The whole OpenVR half of one slot's reconciliation: everything the slot
// policy needs, and nothing else. The serial is a string property read per
// device, so it is paid only for the slots SlotNeedsSerial names — the same
// predicate the decision applies, so the two cannot disagree about which device
// the serial is supposed to identify.
static questcal::SyncDevice EnumerateSyncDevice(uint32_t id,
	const questcal::DriverSyncDesired &desired)
{
	questcal::SyncDevice device;
	if (id >= vr::k_unMaxTrackedDeviceCount)
		return device;

	device.id = id;
	auto system = vr::VRSystem();
	// No runtime means no description of this device, which is the same
	// conservative answer as a slot OpenVR stopped exposing: retire it.
	if (!system)
		return device;

	switch (system->GetTrackedDeviceClass(id))
	{
	case vr::TrackedDeviceClass_Invalid:
		device.deviceClass = questcal::SyncDeviceClass::Invalid;
		return device;
	case vr::TrackedDeviceClass_HMD:
		device.deviceClass = questcal::SyncDeviceClass::Hmd;
		break;
	default:
		device.deviceClass = questcal::SyncDeviceClass::Other;
		break;
	}

	device.trackingSystemKnown = ReadTrackedDeviceString(id,
		vr::Prop_TrackingSystemName_String, device.trackingSystem);
	if (questcal::SlotNeedsSerial(desired, device))
		device.serialKnown = ReadTrackedDeviceString(id,
			vr::Prop_SerialNumber_String, device.serial);
	return device;
}

// Point the session's three injected seams at this process's real pipe, real
// OpenVR enumeration and this context's error banner.
//
// Re-bound at every entry rather than once at startup so the session can never
// report a driver failure into a context other than the one being synchronized.
// Nothing here allocates: the transport lambda captures nothing, the enumerator
// is a function pointer, and the error sinks capture one reference each.
static void BindDriverSession(CalibrationContext &ctx)
{
	DriverLink.SetTransport([](const protocol::Request &request)
	{
		questcal::DriverTransportResult result;
		try
		{
			result.response = Driver.SendBlocking(request);
			result.completed = true;
		}
		catch (const std::exception &e)
		{
			result.error = e.what();
		}
		// Read after the attempt either way. SendBlocking reconnects and replays
		// internally, so a request can be accepted on a pipe the batch did not
		// start on, and even a request that ultimately failed can have advanced
		// the generation on the way -- both are what the batch rules react to.
		result.connectionGeneration = Driver.ConnectionGeneration();
		return result;
	});
	DriverLink.SetDeviceEnumerator(EnumerateSyncDevice);
	DriverLink.SetErrorSink(
		[&ctx](const std::string &message)
		{
			ctx.ReportError(message, CalibrationContext::ErrorSource::Driver);
		},
		[&ctx]() { ctx.ClearError(CalibrationContext::ErrorSource::Driver); });
}

static bool CacheHmdWorldFromDriver(
	const protocol::DevicePoseSample &sample, HmdWorldTransition &transition)
{
	transition = HmdWorldTransition();
	if (sample.deviceId != vr::k_unTrackedDeviceIndex_Hmd ||
		!IsTrustedRingSample(sample, QpcToSeconds))
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

// Derive what the driver should be applying, hand it to the session, and mirror
// back what the session says the rest of the overlay may now believe.
//
// Everything below is derivation: no request is built or sent here. The batch
// itself -- the handshake, the ledger, the connection generation, the send
// ordering and the fail-closed rule -- belongs to DriverSession.h, where it can
// be driven from a test.
static void SynchronizeDriverState(CalibrationContext &ctx)
{
	BindDriverSession(ctx);

	ctx.enabled = ctx.validProfile && !ctx.profileUniverseUnsafe;
	ctx.disableReason = ctx.enabled
		? CalibrationContext::DisableReason::None
		: CalibrationContext::DisableReason::UniverseUnsafe;

	// The connection check happens before the validity gates below, exactly
	// where it did when this function owned the pipe. Both halves can report an
	// error in the same tick and the banner keeps the last one, so this ordering
	// is what decides which failure the user is actually told about.
	questcal::DriverBatch batch = DriverLink.Begin(ctx.timeLastTick);

	if (ctx.enabled && !questcal::IsValidTrackingSystemPair(
		ctx.referenceTrackingSystem, ctx.targetTrackingSystem))
	{
		ctx.ReportError(
			"The live profile has invalid tracking-system identities and was disabled before sending it to the driver\n");
		ctx.enabled = false;
		ctx.disableReason = CalibrationContext::DisableReason::InvalidIdentity;
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
			ctx.disableReason = CalibrationContext::DisableReason::HmdMismatch;
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
		ctx.disableReason = CalibrationContext::DisableReason::InvalidTransform;
		timeShift = 0.0;
	}
	ctx.appliedTimeOffset = timeShift;

	// What the driver should be applying, as the slot policy reads it. Built
	// once here, after the validity gates above have run, so every slot decides
	// against the same transform.
	questcal::DriverApplyRequest request;
	request.enabled = ctx.enabled;
	questcal::DriverSyncDesired &desired = request.desired;
	desired.referenceTrackingSystem = ctx.referenceTrackingSystem;
	desired.targetTrackingSystem = ctx.targetTrackingSystem;
	desired.rotation = ctx.calibratedRotationQ;
	desired.translationMeters = ctx.TranslationMeters();
	desired.scale = ctx.calibratedScale;
	desired.timeShift = timeShift;
	desired.baseGeneration = ctx.baseGeneration;
	desired.continuousArmed = ctx.ContinuousArmed();
	desired.hideMountedTracker = ctx.hideMountedTracker;
	desired.continuousTrackerSerial = ctx.continuousTrackerSerial;
	request.field = BuildAlignmentField(ctx);

	questcal::DriverApplyResult result =
		DriverLink.Apply(batch, request, ctx.timeLastTick);

	// The result is already fail-closed, so this mirror is a plain assignment:
	// on anything short of a complete batch the masks and the tracker id come
	// back cleared, and no monitor can infer that the live driver matches the
	// profile. Each cause is kept distinct because the UI switches on it and
	// sends the user somewhere different for each.
	ctx.enabled = result.enabled;
	switch (result.cause)
	{
	case questcal::DriverDisableCause::HmdMismatch:
		// Currently using an HMD with a different tracking system than the calibration.
		ctx.disableReason = CalibrationContext::DisableReason::HmdMismatch;
		break;
	case questcal::DriverDisableCause::DriverUnreachable:
		ctx.disableReason = CalibrationContext::DisableReason::DriverUnreachable;
		break;
	case questcal::DriverDisableCause::None:
		// The profile's own validity gates above already recorded their cause.
		break;
	}
	ctx.continuousTrackerId = result.continuousTrackerId;
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		ctx.referenceDeviceMask[id] = result.referenceDeviceMask[id];
		ctx.targetDeviceMask[id] = result.targetDeviceMask[id];
	}
}

void ResyncDriverState()
{
	// Same guard as CalibrationTick: with no runtime (UI preview) there is no
	// driver to synchronize, and SynchronizeDriverState reads OpenVR directly.
	if (!vr::VRSystem())
		return;
	SynchronizeDriverState(CalCtx);
}

// One tolerance for every live-vs-snapshot chaperone comparison. The monitor
// that triggers a restore and the verification that judges one must loosen or
// tighten together: contest with a tighter bound than the verifier accepts and
// the app restores in a loop; the reverse verifies a restore it will contest on
// the next scan.
static constexpr float ChaperoneCompareTolerance = 0.002f;

// Does the live wall geometry still match the snapshot? Both call sites used to
// spell this out and both got the same detail wrong: the sizing call reports
// capacity, the filling call writes back what it actually produced, and a count
// that shrank in between means the bounds changed under the read. That is "could
// not read consistently", not "differs" -- reporting it as a difference restores
// a stale snapshot over the edit the user just made in Room Setup.
static bool LiveGeometryMatches(vr::IVRChaperoneSetup *setup,
	const std::vector<vr::HmdQuad_t> &snapshot, uint32_t &liveQuadCount,
	bool &readOk)
{
	readOk = false;
	liveQuadCount = 0;

	uint32_t quadCount = 0;
	if (!setup->GetLiveCollisionBoundsInfo(nullptr, &quadCount))
		return false;
	liveQuadCount = quadCount;
	if (quadCount != snapshot.size())
	{
		readOk = true;
		return false;
	}

	std::vector<vr::HmdQuad_t> live(quadCount);
	uint32_t returnedCount = quadCount;
	if (!setup->GetLiveCollisionBoundsInfo(live.data(), &returnedCount))
		return false;
	// Only compare what the second call actually wrote. Any disagreement with
	// the sized count leaves the tail zero-initialised (or the buffer overrun),
	// so there is no consistent live geometry to judge this snapshot against.
	if (returnedCount != quadCount)
	{
		liveQuadCount = returnedCount;
		return false;
	}
	readOk = true;
	return questcal::QuadsMatch(live, snapshot, ChaperoneCompareTolerance);
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
	{
		ctx.ClearError(CalibrationContext::ErrorSource::ChaperoneMonitor);
		return;
	}

	auto setup = vr::VRChaperoneSetup();
	if (!setup)
	{
		if (ctx.timeLastTick - Monitors.lastSetupFailure >= 30.0)
		{
			ctx.ReportError(
				"Protected chaperone could not be checked because OpenVR chaperone setup is unavailable\n",
				CalibrationContext::ErrorSource::ChaperoneMonitor);
			Monitors.lastSetupFailure = ctx.timeLastTick;
		}
		return;
	}
	Monitors.lastSetupFailure = -1e9;

	uint32_t quadCount = 0;
	bool liveRead = false;
	bool differs = !LiveGeometryMatches(setup, ctx.chaperone.geometry, quadCount,
		liveRead);
	if (!liveRead)
	{
		if (ctx.timeLastTick - Monitors.lastReadFailure >= 30.0)
		{
			ctx.ReportError(
				"Could not read the live chaperone; protected bounds were not changed\n",
				CalibrationContext::ErrorSource::ChaperoneMonitor);
			Monitors.lastReadFailure = ctx.timeLastTick;
		}
		return;
	}
	Monitors.lastReadFailure = -1e9;
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

// ---------------------------------------------------------------------------
// Sample collection

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
		// The composed time folds in the driver's jittery poseTimeOffset, so
		// the odd inversion occurs in healthy data; drop it here rather than
		// let the solver fail the whole collection (same policy as
		// ContinuousAlignment::PushReference). This guard is buffer-relative on
		// purpose: it compares against this collection's own tail, not any
		// device-global watermark the monitors keep.
		questcal::PoseSample sample;
		if (s.deviceId == ctx.calibrationReferenceID)
		{
			if (!TryComposeRingSample(s, QpcToSeconds, sample))
				continue;
			if (!ctx.refSamples.empty() && sample.time <= ctx.refSamples.back().time)
				continue;
			ctx.refSamples.push_back(sample);
			ctx.lastRefSampleTime = now;
		}
		else if (s.deviceId == ctx.calibrationTargetID)
		{
			if (!TryComposeRingSample(s, QpcToSeconds, sample))
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
		if (!IsUsableComposedSample(s) || (!into.empty() && s.time <= into.back().time))
			return;
		into.push_back(s);
		lastTime = now;
	};

	push(ctx.calibrationReferenceID, ctx.refSamples, ctx.lastRefSampleTime);
	push(ctx.calibrationTargetID, ctx.targetSamples, ctx.lastTargetSampleTime);
}

static ChaperoneOwnerStatus CurrentChaperoneOwner(
	const CalibrationContext::Chaperone &snapshot)
{
	// A snapshot that predates owner recording (or whose write never filled it)
	// is an incomplete record, not another headset's room. Folding it into
	// Mismatch told the user their room belonged to a different headset while
	// the load path described the same state accurately.
	if (snapshot.ownerTrackingSystem.empty() || snapshot.ownerHmdSerial.empty())
		return ChaperoneOwnerStatus::Unowned;

	std::string trackingSystem;
	std::string serial;
	if (!ReadCurrentHmdIdentity(trackingSystem, serial))
		return ChaperoneOwnerStatus::Unavailable;
	return trackingSystem == snapshot.ownerTrackingSystem && serial == snapshot.ownerHmdSerial
		? ChaperoneOwnerStatus::Match
		: ChaperoneOwnerStatus::Mismatch;
}

// The one recovery policy for "this snapshot can no longer be trusted": disarm
// it, restate the settings dirty bit, tell the user why, and commit immediately
// so a crash before the debounce elapses cannot hand back an armed record. The
// error source is Chaperone at every site, so it is not a parameter.
//
// Deliberately not shared with FailClosedChaperoneCapture: that path keeps the
// snapshot's data for diagnosis and only clears autoApply, and with the load
// path's conservative in-memory disarm, which must NOT persist over a
// recoverable legacy Config.
static void DisarmChaperoneAndPersist(CalibrationContext &ctx, double now,
	const char *reason)
{
	ctx.DisarmChaperone();
	ctx.MarkSettingsDirty(now);
	ctx.ReportError(reason, CalibrationContext::ErrorSource::Chaperone);
	SaveSettings(ctx);
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

	// Snap-independent, so it sits above the one branch below: the field anchors
	// live in reference space and shift by D either way, and the loop reads only
	// D, never the calibration. The per-anchor deltas conjugate automatically
	// (delta' = D delta D^-1) because SendAlignmentField re-derives them from
	// the absolute transforms.
	for (auto &a : ctx.fieldAnchors)
	{
		a.position = dR * a.position + dT;
		a.rotation = (dR * a.rotation).normalized();
		a.translationMeters = dR * a.translationMeters + dT;
	}

	// Every consequence of the intentional-discontinuity/continuous-correction
	// distinction, in one place: a fourth one added later cannot land under the
	// wrong conditional because there is only one.
	if (snap)
	{
		ctx.SetCalibration(newRot, newTrans, ctx.calibratedScale);
		// Only a field that exists has smoothing to snap.
		if (!ctx.fieldAnchors.empty())
			ctx.fieldGeneration++;
		ctx.AdvancePersistenceRevision();
		// The chaperone snapshot's standing center maps the standing frame into
		// the (just re-based) raw frame, so it re-anchors by D like everything
		// else raw-frame; the wall quads are standing-frame and stay put.
		if (ctx.chaperone.valid)
			ctx.chaperone.standingCenter =
				questcal::DeltaTimesPose(dR, dT, ctx.chaperone.standingCenter);
		ctx.MarkSettingsDirty(now);
	}
	else
		ctx.SetCalibrationContinuous(newRot, newTrans, ctx.calibratedScale);

	ctx.MarkProfileDirty(now);
	SynchronizeDriverState(ctx);
}

// ---------------------------------------------------------------------------
// Runtime monitoring: universe-jump compensation

// How long the universe check defers its verdict once it first sees a mismatch,
// and how long a compensated-but-endpointless jump may claim the observation
// that follows it. The jump detector's heuristic path can only accept between
// its window (0.2 s) and agreeWindow (0.25 s) after the discontinuity, so a
// verdict on the first observation always lands ~0.2 s before the compensation
// it would have to account for — and parking the monitor discards the pending
// candidate, so the jump is never compensated at all. Fail-closed survives; it
// is only this much later. Kept below the detector's 1 s retrigger hold so a
// second, suppressed rebase still reads as unobserved.
static constexpr double UniverseVerdictGraceSeconds = 0.5;

// UI-clock time the current mismatch was first observed; negative = none.
static double UniverseMismatchSince = -1e9;

// Ring time of a compensated jump whose endpoint the detector could not report
// (the heuristic path regresses a heading from velocity-compensated windows; it
// never reads the new worldFromDriver), so the next observation is the
// compensated endpoint rather than evidence of an unobserved rebase.
static double CompensatedJumpAwaitingEndpoint = -1e9;

static bool UniverseVerdictPending()
{
	return UniverseMismatchSince >= 0.0;
}

// Bind both universe baselines to the endpoint we are looking at, if it is the
// one a heuristic jump compensation was waiting for.
static bool AdoptObservedUniverseAfterJump(CalibrationContext &ctx, double now)
{
	if (CurrentHmdObservation.sampleTime < CompensatedJumpAwaitingEndpoint ||
		CurrentHmdObservation.sampleTime - CompensatedJumpAwaitingEndpoint >
			UniverseVerdictGraceSeconds)
		return false;
	CompensatedJumpAwaitingEndpoint = -1e9;

	ctx.profileWorldFromDriverRotation = CurrentHmdObservation.rotation;
	ctx.profileWorldFromDriverTranslation = CurrentHmdObservation.translation;
	ctx.profileUniverseValid = true;
	ctx.MarkProfileDirty(now);
	if (ctx.chaperone.valid)
	{
		ctx.chaperone.worldFromDriverRotation = CurrentHmdObservation.rotation;
		ctx.chaperone.worldFromDriverTranslation = CurrentHmdObservation.translation;
		ctx.chaperone.worldFromDriverValid = true;
		ctx.chaperone.baselineVerifiedThisSession = true;
		ctx.MarkSettingsDirty(now);
	}
	return true;
}

// Fold an accepted universe delta into the calibration: the reference universe
// moved by D in one frame, so target devices must follow to stay aligned.
static void ApplyUniverseDelta(CalibrationContext &ctx, const JumpDetector::UniverseDelta &d, double now)
{
	ApplyAlignmentDelta(ctx, d.rotation, d.translation, /*snap=*/true, now);
	if (d.exact)
	{
		// Bind to this accepted HMD sample's exact endpoint, never to the
		// consumer's global latest value: a drained backlog may already contain
		// a second rebase which retrigger hysteresis intentionally suppressed.
		if (ctx.profileUniverseValid)
		{
			ctx.profileWorldFromDriverRotation =
				d.worldFromDriverRotation.normalized();
			ctx.profileWorldFromDriverTranslation =
				d.worldFromDriverTranslation;
		}
		if (ctx.chaperone.valid)
		{
			ctx.chaperone.worldFromDriverRotation =
				d.worldFromDriverRotation.normalized();
			ctx.chaperone.worldFromDriverTranslation =
				d.worldFromDriverTranslation;
			ctx.chaperone.worldFromDriverValid = true;
			ctx.chaperone.baselineVerifiedThisSession = true;
		}
	}
	else
		CompensatedJumpAwaitingEndpoint = d.time;

	ctx.jumpsCompensated++;

	double yawDeg = 2.0 * std::atan2(d.rotation.y(), d.rotation.w()) * 180.0 / EIGEN_PI;
	char buf[256];
	snprintf(buf, sizeof buf, "Universe jump compensated (%s): yaw %+.2f deg, shift %.3f m, %d device(s)\n",
		d.exact ? "exact" : "estimated", yawDeg, d.translation.norm(), d.devicesAgreeing);
	ctx.Log(buf);
}

// Does the live calibration still describe the universe it was solved in? The
// profile owns that question: it carries its own persisted headset identity and
// worldFromDriver baseline, so the check works with no protected room and
// survives a restart. The chaperone snapshot references the same verdict rather
// than deciding it. Runs on the drained observation, before any snapshot logic.
static void ProfileUniverseTick(CalibrationContext &ctx, double now)
{
	// Same state gate as the runtime monitors: a rebase seen mid-collection is
	// not a verdict, because the solve in flight rebinds the baseline itself
	// (and a failed solve leaves this to decide on the next idle tick).
	if (ctx.state != CalibrationState::None || !ctx.validProfile ||
		ctx.profileUniverseUnsafe)
	{
		UniverseMismatchSince = -1e9;
		return;
	}
	// Losing freshness deliberately does NOT restart the grace below: a flapping
	// ring would otherwise postpone the verdict indefinitely.
	if (!HasFreshHmdWorldFromDriver())
		return;

	// Steady state first: an unchanged baseline decides nothing and must not
	// pay for the device-property reads below, which are comparatively
	// expensive OpenVR calls on a tick that runs at up to 50 Hz.
	bool changed = !ctx.profileUniverseValid || questcal::WorldFromDriverChanged(
		ctx.profileWorldFromDriverRotation, ctx.profileWorldFromDriverTranslation,
		CurrentHmdObservation.rotation, CurrentHmdObservation.translation);
	if (!changed)
	{
		UniverseMismatchSince = -1e9;
		return;
	}

	// Scoped to one physical headset. SynchronizeDriverState only matches the
	// tracking-system name, so a same-system spare headset is a different
	// universe rather than a rebase of this one, and an identity that cannot be
	// read proves nothing either way.
	std::string trackingSystem;
	std::string serial;
	if (!ReadCurrentHmdIdentity(trackingSystem, serial) ||
		trackingSystem != ctx.referenceTrackingSystem)
		return;

	if (!ctx.profileUniverseValid || ctx.profileHmdSerial != serial)
	{
		// A profile saved before this baseline existed adopts one: absence of a
		// baseline is not evidence of a rebase. A protected room captured by
		// this same headset already carries the universe the profile was
		// calibrated in, so prefer it and keep that protection across the
		// migration instead of silently adopting a universe that may have
		// already moved.
		bool fromSnapshot = ctx.chaperone.valid &&
			ctx.chaperone.worldFromDriverValid &&
			ctx.chaperone.ownerTrackingSystem == trackingSystem &&
			ctx.chaperone.ownerHmdSerial == serial;
		ctx.profileHmdSerial = serial;
		ctx.profileWorldFromDriverRotation = fromSnapshot
			? ctx.chaperone.worldFromDriverRotation : CurrentHmdObservation.rotation;
		ctx.profileWorldFromDriverTranslation = fromSnapshot
			? ctx.chaperone.worldFromDriverTranslation : CurrentHmdObservation.translation;
		ctx.profileUniverseValid = true;
		ctx.MarkProfileDirty(now);
		UniverseMismatchSince = -1e9;
		if (!fromSnapshot)
			return;
	}

	if (!questcal::WorldFromDriverChanged(
		ctx.profileWorldFromDriverRotation, ctx.profileWorldFromDriverTranslation,
		CurrentHmdObservation.rotation, CurrentHmdObservation.translation))
	{
		UniverseMismatchSince = -1e9;
		return;
	}

	// A rebase the jump detector compensated moved the calibration, the field
	// anchors and the standing center by the same D. It was observed, so it is
	// not this failure.
	if (AdoptObservedUniverseAfterJump(ctx, now))
	{
		UniverseMismatchSince = -1e9;
		return;
	}

	// Give the jump detector its window before answering. Its heuristic path
	// cannot produce a delta until ~0.2 s after the discontinuity, so a verdict
	// on the first observation is always premature — and because the verdict
	// parks the monitor, it also destroys the compensation it was judging.
	if (!UniverseVerdictPending())
		UniverseMismatchSince = now;
	if (now - UniverseMismatchSince < UniverseVerdictGraceSeconds)
	{
		// Nothing may be restored against a baseline still being adjudicated.
		ctx.chaperone.baselineVerifiedThisSession = false;
		return;
	}
	UniverseMismatchSince = -1e9;

	// We know the reference universe moved, but not whether the target universe
	// moved with it. Applying only the HMD delta could silently corrupt
	// alignment; keep both profile and room fail-closed until a fresh base
	// calibration establishes the relation.
	ctx.profileUniverseUnsafe = true;
	ctx.enabled = false;
	ctx.MarkProfileDirty(now);
	bool autoApplyChanged = ctx.chaperone.autoApply;
	ctx.chaperone.autoApply = false;
	ctx.chaperone.baselineVerifiedThisSession = false;
	if (autoApplyChanged)
		ctx.MarkSettingsDirty(now);
	ctx.ReportError(autoApplyChanged
		? "The headset raw universe changed while calibration monitoring had no continuity. "
			"The profile and protected chaperone are disabled until you run a new base calibration.\n"
		: "The headset raw universe changed while calibration monitoring had no continuity. "
			"The profile is disabled until you run a new base calibration.\n",
		CalibrationContext::ErrorSource::Chaperone);
	// Fail closed in the driver immediately; waiting for the periodic
	// one-second scan would leave the previous transforms live.
	SynchronizeDriverState(ctx);
	// The latch is worth nothing if a crash before the debounce elapses hands
	// the profile back enabled; the dirty bits survive a failed write and retry.
	SaveDirtyPersistence(ctx);
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

	// The profile's own universe check runs on this observation first, so the
	// snapshot logic below sees a verdict (and, after a compensated jump, an
	// already-advanced baseline) rather than deciding the same question again.
	ProfileUniverseTick(ctx, now);

	// Device properties are stable for a session but comparatively expensive
	// OpenVR calls; the tick runs at up to 50 Hz. Recheck once per second, or
	// immediately when a newly captured/loaded snapshot changes ownership.
	if (!ctx.chaperone.valid)
	{
		Monitors.owner = ChaperoneOwnerStatus::Unavailable;
		Monitors.checkedTrackingSystem.clear();
		Monitors.checkedHmdSerial.clear();
		Monitors.lastOwnerCheck = -1e9;
	}
	else if (ctx.chaperone.ownerTrackingSystem != Monitors.checkedTrackingSystem ||
		ctx.chaperone.ownerHmdSerial != Monitors.checkedHmdSerial ||
		now - Monitors.lastOwnerCheck >= 1.0)
	{
		Monitors.checkedTrackingSystem = ctx.chaperone.ownerTrackingSystem;
		Monitors.checkedHmdSerial = ctx.chaperone.ownerHmdSerial;
		Monitors.owner = CurrentChaperoneOwner(ctx.chaperone);
		Monitors.lastOwnerCheck = now;
	}
	if (ctx.chaperone.valid && Monitors.owner == ChaperoneOwnerStatus::Mismatch)
	{
		DisarmChaperoneAndPersist(ctx, now,
			"The protected chaperone belongs to a different headset/runtime and has been disarmed. "
			"Capture it again for the current headset.\n");
		return;
	}
	if (ctx.chaperone.valid && Monitors.owner == ChaperoneOwnerStatus::Unowned)
	{
		// Same disarm, but the record is incomplete rather than foreign; this is
		// the wording the load path uses for the identical state.
		DisarmChaperoneAndPersist(ctx, now,
			"The protected chaperone has no complete headset/universe baseline. "
			"It has been disarmed; capture it again before enabling auto-restore.\n");
		return;
	}

	if (!ctx.chaperone.valid || Monitors.owner != ChaperoneOwnerStatus::Match ||
		!HasFreshHmdWorldFromDriver())
		return;

	if (!ctx.chaperone.worldFromDriverValid)
	{
		DisarmChaperoneAndPersist(ctx, now,
			"The protected chaperone has no raw-universe baseline and was disarmed. "
			"Capture it again before restoring it.\n");
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
		// The profile owns the universe verdict and has already acted on this
		// same observation (latched, or re-bound after a compensated jump). All
		// that is left here is the room: this snapshot's baseline no longer
		// describes the live universe, so it is not restorable until recaptured.
		// Re-anchoring it from HMD continuity alone is only safe with no
		// profile — with one, the target universe's relation is unknown.
		if (UniverseVerdictPending())
		{
			// Still deferred: block restores, but do not persist a disarm that a
			// compensation landing inside the grace window would make wrong.
			ctx.chaperone.baselineVerifiedThisSession = false;
			return;
		}
		bool autoApplyChanged = ctx.chaperone.autoApply;
		ctx.chaperone.autoApply = false;
		ctx.chaperone.baselineVerifiedThisSession = false;
		if (autoApplyChanged)
		{
			ctx.MarkSettingsDirty(now);
			ctx.ReportError(
				"The protected chaperone no longer matches the headset raw universe and was disarmed. "
				"Capture it again before restoring it.\n",
				CalibrationContext::ErrorSource::Chaperone);
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
		DisarmChaperoneAndPersist(ctx, now,
			"The headset raw universe changed without an adjacent continuous HMD pose pair. "
			"The protected chaperone was disarmed; capture it again before restoring it.\n");
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

// Supplied by the app shell (see Calibration.h). An absent sink is a normal
// state, not an error: -uipreview never installs one, and it degrades to
// log-only exactly as a not-yet-created overlay handle did.
static std::function<void(const char *)> ToastSink;

void SetToastSink(std::function<void(const char *)> sink)
{
	ToastSink = std::move(sink);
}

// One-shot log + optional VR toast; each caller owns its re-arm flag.
static void NotifyOnce(CalibrationContext &ctx, bool &notified, const char *logLine,
	const char *toast, bool showToast = true)
{
	if (notified)
		return;
	notified = true;

	// Unconditional and first: "the user turned toasts off" and "there is
	// nowhere to toast" must both still leave the episode in the session log,
	// which is the bug-report surface for a GUI binary.
	ctx.Log(std::string(logLine) + "\n");

	if (showToast && ToastSink)
		ToastSink(toast);
}

// The three one-shot flags live on MonitorState with the rest of the monitors'
// between-tick state: one notification per calibration (freeze also re-arms on
// a resume event), and "observations unstable" is benign and self-healing (bad
// lighthouse geometry while lying down), so the log records every episode but
// the toast never repeats.

static void NotifyStaleAlignment(CalibrationContext &ctx)
{
	NotifyOnce(ctx, Monitors.staleNotified,
		"Calibration quality looks poor -- recalibrating is recommended",
		"QuestCalibrator: calibration quality looks poor. Recalibrating is recommended.",
		ctx.notifyPoorCalibration);
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

	// Soft-OR: independent evidence compounds without exceeding 1.
	ctx.driftScore = 1.0 - (1.0 - ageScore) * (1.0 - evidence);

	// Corroboration gate: one slide window on one device is a measurement,
	// not a verdict — a resting body's posture creep passes the rest gates and
	// produces exactly one such window, and used to flip the rating straight
	// to Stale/Very Poor while the alignment looked visibly fine. The cap is on
	// the combined score, not the evidence term: age saturates at 0.5, so a
	// capped-at-0.5 evidence term still soft-ORs to 0.75 and rates every
	// day-old calibration Stale off one benign event. Genuine detector-band
	// drift persists, so the monitor re-fires within ~one window (its window
	// clears per event) and the cap lifts almost immediately; until then a lone
	// event can only ever say "aging".
	if (ctx.driftSlideEvents + ctx.discontinuousLossEvents <= 1)
		ctx.driftScore = std::min(ctx.driftScore, 0.5);

	ctx.alignment =
		ctx.driftScore >= 0.65 ? CalibrationContext::AlignmentHealth::Stale :
		ctx.driftScore >= 0.30 ? CalibrationContext::AlignmentHealth::Aging :
		CalibrationContext::AlignmentHealth::Fresh;

	// A healthy continuous loop re-measures the alignment constantly; the
	// rating already skips staleness for it, and toasting "quality looks poor"
	// while it is visibly being maintained is pure noise.
	bool maintained = ctx.ContinuousArmed() &&
		ctx.continuousState == questcal::ContinuousAlignment::State::Tracking;
	if (ctx.alignment == CalibrationContext::AlignmentHealth::Stale && !maintained)
		NotifyStaleAlignment(ctx);
}

// Notes and gaps the detector has already queued describe observations it made
// before the reset, so they must reach the log first: Reset() drops the deque
// and re-seeds every lastValidTime, and a drain hole or a deactivated profile is
// exactly when "the universe may have moved" is both most likely and least
// observable. Resetting the baselines across the hole is still correct -- only
// the observation already made survives it.
static void DrainJumpObservations(CalibrationContext &ctx)
{
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
			DrainJumpObservations(ctx);
			Jumps->Reset();
			Drift->Reset();
			Monitors.ResetObservations();
			MonitorActive = false;
		}
		PoseHub.DiscardBacklog(MonitorConsumer);
		return;
	}

	if (!MonitorActive)
	{
		PoseHub.DiscardBacklog(MonitorConsumer);
		Monitors.ResetObservations();
		MonitorActive = true;
	}

	uint64_t dropped = PoseHub.Drain(MonitorConsumer, MonitorScratch);
	if (dropped > 0)
	{
		// We lost part of our own observation window; baselines across the
		// hole are unsafe. For the drift monitor a drain hole would read as a
		// tracking loss and could fake a discontinuity event.
		DrainJumpObservations(ctx);
		Jumps->Reset();
		Drift->Reset();
		Monitors.ResetObservations();
	}

	for (const auto &s : MonitorScratch)
	{
		if (s.deviceId >= vr::k_unMaxTrackedDeviceCount)
			continue;
		// JumpDetector owns the observation-continuity policy and must see bad
		// frames as well as good ones. Drift/HMD caches below remain valid-only.
		if (ctx.referenceDeviceMask[s.deviceId])
			Jumps->Push(s);

		questcal::PoseSample sample;
		if (!TryComposeRingSample(s, QpcToSeconds, sample))
			continue;
		// Device-relative monotonicity: this monitor keeps a per-device
		// watermark across drains, unlike the collector's buffer-relative guard
		// and the HMD observation's single-endpoint one. Same shape, three
		// different domain rules — deliberately not one shared guard.
		double composedTime = sample.time;
		if (Monitors.hasComposedTime[s.deviceId] &&
			composedTime <= Monitors.lastComposedTime[s.deviceId])
			continue;
		Monitors.hasComposedTime[s.deviceId] = true;
		Monitors.lastComposedTime[s.deviceId] = composedTime;

		if (s.deviceId == vr::k_unTrackedDeviceIndex_Hmd &&
			ctx.referenceDeviceMask[s.deviceId])
		{
			Monitors.hmdRawPosition = sample.pos;
			Monitors.hmdRawTime = composedTime;
		}

		// Feed-selection policy (HMD-only on the reference side, mounted-tracker
		// exclusion, worn-device proximity) lives in ringpose::AnchorsUniverse
		// so it is one pure predicate a test can drive; see its comment for why
		// all three rules are load-bearing.
		ringpose::DriftFeedCandidate candidate;
		candidate.deviceId = s.deviceId;
		candidate.referenceSide = ctx.referenceDeviceMask[s.deviceId];
		candidate.targetSide = ctx.targetDeviceMask[s.deviceId];
		candidate.mountedTrackerId = ctx.continuousTrackerId;
		candidate.rawPosition = sample.pos;
		candidate.composedTime = composedTime;
		candidate.hmdRawPosition = Monitors.hmdRawPosition;
		candidate.hmdRawTime = Monitors.hmdRawTime;
		candidate.calibratedRotation = ctx.calibratedRotationQ;
		candidate.calibratedTranslationMeters = ctx.TranslationMeters();
		candidate.calibratedScale = ctx.calibratedScale;

		if (ringpose::AnchorsUniverse(candidate))
			Drift->Push(s,
				ctx.targetDeviceMask[s.deviceId] ? ctx.calibratedScale : 1.0);
	}

	DrainJumpObservations(ctx);

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
		ctx.ContinuousArmed() &&
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
		ctx.continuousState = Continuous->GetState();
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

	for (const auto &s : ContinuousScratch)
	{
		if (s.deviceId >= vr::k_unMaxTrackedDeviceCount)
			continue;

		questcal::PoseSample sample;
		if (s.deviceId == vr::k_unTrackedDeviceIndex_Hmd)
		{
			if (!TryComposeRingSample(s, QpcToSeconds, sample))
				continue;
			Continuous->PushReference(sample);
		}
		else if (s.deviceId == ctx.continuousTrackerId)
		{
			if (!TryComposeRingSample(s, QpcToSeconds, sample))
				continue;
			Continuous->PushTarget(sample);
		}
	}

	// The engine's clock is the ring's (QPC seconds), not the UI clock. Checked
	// like every other QueryPerformanceCounter call in this file: the documented
	// failure return leaves the value indeterminate, and it feeds the window trim
	// cutoff, the coast-gap freshness test and every confirm timer in Decide.
	// Drained samples are already in the engine; skipping this tick's update
	// only defers the decision.
	LARGE_INTEGER qnow;
	if (!QueryPerformanceCounter(&qnow))
		return;
	double ringNow = static_cast<double>(qnow.QuadPart) * QpcToSeconds;

	// Re-evaluate the current field for each retained observation. Comparing a
	// multi-position history with only the latest spot turns healthy anchor
	// gradients into apparent temporal drift as the user walks through them.
	// Always passed, from the same active set the driver is blending: with no
	// active anchors the blend returns the base calibration exactly, so there
	// is no second way to spell "the field is off" that could disagree.
	auto expectedAt = [&](const Eigen::Vector3d &targetRawPos,
		Eigen::Quaterniond &rotationOut, Eigen::Vector3d &translationOut)
	{
		// The field is looked up at the tracker's BASE-CALIBRATED world
		// position, so the calibrated scale multiplies the raw position first -
		// shared with the drift feed's proximity test so there is one spelling
		// of where that factor goes.
		Eigen::Vector3d basePos = ringpose::BaseCalibratedPosition(
			ctx.calibratedRotationQ, ctx.TranslationMeters(), ctx.calibratedScale,
			targetRawPos);
		// Same width SendAlignmentField put on the wire - the expectation has to
		// be the field the driver is actually applying, not a similar one.
		questcal::BlendedFieldCalibration(ActiveFieldAnchors(ctx), ctx.calibratedRotationQ,
			ctx.TranslationMeters(), basePos, rotationOut, translationOut,
			questcal::FieldBlendSigmaMeters);
	};
	Continuous->Update(ringNow, ctx.calibratedRotationQ, ctx.TranslationMeters(),
		ctx.calibratedScale, ctx.calibratedTimeOffset, expectedAt);

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
			snprintf(buf, sizeof buf,
				"Continuous calibration frozen: deviation yaw %.2f deg, tilt %.2f deg, %.1f cm\n",
				ev.deviation.yawDeg, ev.deviation.tiltDeg, ev.deviation.posM * 100.0);
			ctx.Log(buf);
			NotifyOnce(ctx, Monitors.freezeNotified,
				"The headset-mounted tracker moved or lost tracking -- alignment updates are on hold",
				"QuestCalibrator: the headset-mounted tracker moved or lost tracking. Alignment updates are on hold -- recalibrate to re-learn the mount.");
			break;
		case questcal::ContinuousAlignment::Event::FrozenMountScatter:
			// The event now carries the scatter it froze on, so this no longer
			// re-reads live accessors that have moved on since it was raised.
			snprintf(buf, sizeof buf,
				"Continuous calibration frozen: observation scatter %.2f deg / %.1f cm stays far above the tracking noise -- mount fault signature\n",
				ev.scatterRotDeg, ev.scatterPosM * 100.0);
			ctx.Log(buf);
			NotifyOnce(ctx, Monitors.freezeNotified,
				"The headset-mounted tracker moved or lost tracking -- alignment updates are on hold",
				"QuestCalibrator: the headset-mounted tracker moved or lost tracking. Alignment updates are on hold -- recalibrate to re-learn the mount.");
			break;
		case questcal::ContinuousAlignment::Event::ObservationsUnstable:
			snprintf(buf, sizeof buf,
				"Mounted tracker observations unstable (scatter %.2f deg / %.1f cm) -- alignment updates paused until tracking settles\n",
				Continuous->ScatterRotRmsDeg(), Continuous->ScatterPosRmsM() * 100.0);
			ctx.Log(buf);
			NotifyOnce(ctx, Monitors.unstableNotified,
				"Mounted tracker tracking is unstable -- alignment updates paused until it settles",
				"QuestCalibrator: the headset-mounted tracker's tracking looks unstable here. Alignment updates are paused and will resume on their own.");
			break;
		case questcal::ContinuousAlignment::Event::Resumed:
			ctx.Log("Continuous calibration resumed\n");
			Monitors.freezeNotified = false;
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

	ctx.continuousState = Continuous->GetState();
	ctx.continuousDeviation = Continuous->CurrentDeviation();
	ctx.continuousScatterRotDeg = Continuous->ScatterRotRmsDeg();
	ctx.continuousScatterPosM = Continuous->ScatterPosRmsM();
}

static void AbortCalibration(CalibrationContext &ctx, const std::string &reason)
{
	AppendSessionLog("Calibration aborted: " + reason + "\n");
	ctx.Log(CalibrationAbortedMessage);
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

	// Decided before the transaction opens: a refused anchor is not a failed
	// save, so it must not bump the generation or roll anything back.
	bool append = slot == ctx.fieldAnchors.size();
	if (append && ctx.fieldAnchors.size() >= protocol::SetAlignmentField::MaxAnchors)
	{
		ctx.Log("Anchor limit reached (8): collect within 1 m of an existing anchor to replace it, or clear the field\n");
		return;
	}

	if (!SaveProfileFieldEdit(ctx,
		[&](questcal::ProfileRecord &candidate) {
			if (append)
				candidate.fieldAnchors.push_back(PersistedAnchor(anchor));
			else
				candidate.fieldAnchors[slot] = PersistedAnchor(anchor);
		},
		true))
	{
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
		questcal::MountExtrinsic extrinsic;
		if (questcal::ContinuousAlignment::DeriveMountExtrinsic(
			ctx.refSamples, ctx.targetSamples, result, extrinsic))
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
	if (!result.valid)
	{
		AppendSessionLog("Calibration failed: " + result.message + "\n");
		ctx.Log(CalibrationAbortedMessage);
		return;
	}

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
		const char *guardNote = "";
		switch (result.scaleGuard)
		{
		case questcal::ScaleGuard::FromGrossMotion:
			guardNote = " -- fine motion attenuated; scale taken from clean gross motion";
			break;
		case questcal::ScaleGuard::NeutralizedForSmoothing:
			guardNote = " -- gross and fine motion attenuated; scale held at neutral 1.0";
			break;
		case questcal::ScaleGuard::NotApplied:
			// The smoothing diagnostic is independent of the guard and fires
			// even with scale solving off, so it only speaks when it is alone.
			if (result.motionSmoothingDetected)
				guardNote = " -- streamed-pose smoothing detected";
			break;
		}
		snprintf(buf, sizeof buf,
			"Motion gain (reference vs target): %.3f gross / %.3f fine%s\n",
			result.motionGainLow, result.motionGainHigh, guardNote);
		ctx.Log(buf);
	}

	if (asAnchor)
	{
		StoreFieldAnchor(ctx, result, targetCentroid);
		return;
	}

	// Only a successful base solve publishes its residuals: the advanced row
	// labels them "Last calibration" and ComputeCalibrationRating caps the
	// rating from them, so a deliberately short, spatially local anchor solve
	// (or a failure, which leaves the old profile live) must not replace them.
	ctx.lastResult = result;

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
	// evidence, and re-arms the one-shot notifications.
	ctx.calibrationUnixTime = static_cast<double>(std::time(nullptr));
	ctx.driftSlideEvents = 0;
	ctx.driftMaxSlideM = 0.0;
	ctx.discontinuousLossEvents = 0;
	ctx.driftScore = 0.0;
	ctx.alignment = CalibrationContext::AlignmentHealth::Fresh;
	Monitors.ReArmNotifications();
	ctx.lastAutoCorrectionUnixTime = 0.0;
	ctx.autoCorrectionsApplied = 0;
	Drift->Reset();

	bool priorUniverseUnsafe = ctx.profileUniverseUnsafe;
	ctx.profileUniverseUnsafe = false;

	// This solve defines the reference universe the profile now lives in, so it
	// is the one place the persisted identity is (re-)bound. If the ring is
	// unavailable there is nothing to bind to; the monitor adopts the first
	// fresh observation instead, exactly as it does for an older profile.
	ctx.profileUniverseValid = false;
	ctx.profileHmdSerial.clear();
	std::string universeHmdSerial;
	if (HasFreshHmdWorldFromDriver() &&
		ReadTrackedDeviceString(vr::k_unTrackedDeviceIndex_Hmd,
			vr::Prop_SerialNumber_String, universeHmdSerial))
	{
		ctx.profileHmdSerial = universeHmdSerial;
		ctx.profileWorldFromDriverRotation = CurrentHmdObservation.rotation;
		ctx.profileWorldFromDriverTranslation = CurrentHmdObservation.translation;
		ctx.profileUniverseValid = true;
	}

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
	CalCtx.ClearMessages();
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
	// Channel health is independent of SteamVR and of the profile: the UI must
	// report it even on the ticks that return early below.
	ctx.poseRingOpen = PoseHub.RingOpen();
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

		// Frozen alongside the ids: the id is a slot, and the collection window
		// is long enough for SteamVR to free one and hand it to a different
		// physical device. Read once here (properties are expensive) and
		// re-checked at 1 Hz while collecting.
		ReadTrackedDeviceString(ctx.calibrationReferenceID,
			vr::Prop_SerialNumber_String, ctx.calibrationReferenceSerial);
		ReadTrackedDeviceString(ctx.calibrationTargetID,
			vr::Prop_SerialNumber_String, ctx.calibrationTargetSerial);
		LastCollectionIdentityCheck = time;

		char buf[256];
		snprintf(buf, sizeof buf, "Reference device ID: %u, serial: %s\n",
			ctx.calibrationReferenceID, ctx.calibrationReferenceSerial.c_str());
		ctx.Log(buf);
		snprintf(buf, sizeof buf, "Target device ID: %u, serial: %s\n",
			ctx.calibrationTargetID, ctx.calibrationTargetSerial.c_str());
		ctx.Log(buf);

		// Outside any reconciliation batch, and deliberately outside the ledger:
		// this clears a stale transform off the device about to be sampled, it
		// does not speak for what the live profile left enabled.
		BindDriverSession(ctx);
		if (!DriverLink.DisableDeviceTransform(ctx.calibrationTargetID, ctx.timeLastTick))
		{
			ctx.state = CalibrationState::None;
			ctx.Log("Could not disable the existing target transform; calibration aborted\n");
			return;
		}

		ctx.ClearSampleBuffers();
		// Pin the sample source for the whole run. The ring stamps ring/QPC
		// seconds and the runtime-pose fallback stamps the UI clock into the
		// same buffers, and the only guard is monotonicity against the buffer's
		// own tail, which cannot tell two origins apart: a channel that appears
		// or disappears mid-run would leave the solver a window it silently
		// rejects, losing the latency correction the run existed to measure.
		CollectionUsesPoseRing = PoseHub.RingOpen();
		if (CollectionUsesPoseRing)
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

		ctx.Log("Keep the selected devices rigidly together.\n"
			"Move them through wide, varied rotations around at least two different axes and across the play area.\n");
		return;
	}

	// ---- Collecting ----
	// Channel-specific, because the generic abort reads as a device fault: the
	// reverse transition used to surface as "Reference device stopped tracking".
	if (PoseHub.RingOpen() != CollectionUsesPoseRing)
	{
		AbortCalibration(ctx, CollectionUsesPoseRing
			? "The driver pose channel closed mid-collection; the remaining samples would carry a different clock"
			: "The driver pose channel opened mid-collection; the remaining samples would carry a different clock");
		return;
	}

	// The frozen pair is an OpenVR index. Re-read the two serials at 1 Hz --
	// never per sample, these are expensive property reads -- so a device that
	// power-cycled into another device's slot cannot have its poses concatenated
	// into one buffer and fitted as a single rigid body. A serial that cannot be
	// read proves nothing; the no-sample timeouts below stay the liveness check.
	if (time - LastCollectionIdentityCheck >= 1.0)
	{
		LastCollectionIdentityCheck = time;
		std::string serial;
		auto deviceReplaced = [&](uint32_t id, const std::string &frozen)
		{
			return !frozen.empty() &&
				ReadTrackedDeviceString(id, vr::Prop_SerialNumber_String, serial) &&
				serial != frozen;
		};
		if (deviceReplaced(ctx.calibrationReferenceID, ctx.calibrationReferenceSerial) ||
			deviceReplaced(ctx.calibrationTargetID, ctx.calibrationTargetSerial))
		{
			AbortCalibration(ctx,
				"A selected device was replaced in its slot mid-collection");
			return;
		}
	}

	if (CollectionUsesPoseRing)
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

	// IsPlausibleChaperone by its own name at the call site: this is the named
	// trust boundary for room geometry, not the finiteness check a local
	// forwarder's name suggested.
	if (!setup->GetWorkingStandingZeroPoseToRawTrackingPose(&snapshot.standingCenter) ||
		!setup->GetWorkingPlayAreaSize(&snapshot.playSpaceSize.v[0], &snapshot.playSpaceSize.v[1]) ||
		!questcal::IsPlausibleChaperone(snapshot.geometry, snapshot.standingCenter,
			snapshot.playSpaceSize))
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
	if (!CalCtx.chaperone.valid || !questcal::IsPlausibleChaperone(
		CalCtx.chaperone.geometry, CalCtx.chaperone.standingCenter,
		CalCtx.chaperone.playSpaceSize))
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
		DisarmChaperoneAndPersist(CalCtx, CalCtx.timeLastTick,
			"Could not restore the chaperone: the snapshot belongs to a different headset/runtime. "
			"It has been disarmed; capture it again for this headset.\n");
		return false;
	}
	if (owner == ChaperoneOwnerStatus::Unowned)
	{
		// Same disarm as a foreign snapshot, but the honest reason: the record
		// never carried an owner, which is what the load path reports too.
		DisarmChaperoneAndPersist(CalCtx, CalCtx.timeLastTick,
			"Could not restore the chaperone: the snapshot has no complete headset/universe baseline. "
			"It has been disarmed; capture it again before enabling auto-restore.\n");
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
		// A geometry read that could not be made consistently is not a verified
		// restore, so both outcomes of the shared reader fail the same way here.
		uint32_t liveQuadCount = 0;
		bool readOk = false;
		verified = LiveGeometryMatches(setup, CalCtx.chaperone.geometry,
			liveQuadCount, readOk);
	}
	setup->RevertWorkingCopy();
	vr::HmdMatrix34_t standing = {};
	vr::HmdVector2_t size = {};
	verified = verified && setup->GetWorkingStandingZeroPoseToRawTrackingPose(&standing) &&
		setup->GetWorkingPlayAreaSize(&size.v[0], &size.v[1]);
	for (int row = 0; verified && row < 3; ++row)
		for (int column = 0; verified && column < 4; ++column)
			verified = std::abs(standing.m[row][column] -
				CalCtx.chaperone.standingCenter.m[row][column]) <= ChaperoneCompareTolerance;
	verified = verified &&
		std::abs(size.v[0] - CalCtx.chaperone.playSpaceSize.v[0]) <= ChaperoneCompareTolerance &&
		std::abs(size.v[1] - CalCtx.chaperone.playSpaceSize.v[1]) <= ChaperoneCompareTolerance;
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
