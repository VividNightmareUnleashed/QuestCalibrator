#include "stdafx.h"
#include "Calibration.h"
#include "CalibrationInternal.h"
#include "CalibrationActionPolicy.h"
#include "CalibrationDriver.h"
#include "CalibrationSpace.h"
#include "CollectionSource.h"
#include "Configuration.h"
#include "DriftMonitor.h"
#include "LighthouseFrameWatch.h"
#include "Diagnostics.h"
#include "PoseStreamHub.h"
#include "ProfileValidation.h"
#include "RingPoseMath.h"
#include "SessionLogTrim.h"
#include "StreamEvents.h"
#include "../common/Version.h"

#include <Eigen/Dense>

#include <ctime>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

using namespace calibration_internal;

PoseStreamHub calibration_internal::PoseHub;
static uint64_t PoseHubFailuresLogged = 0;
static int CollectorConsumer = -1;
static std::vector<protocol::DevicePoseSample> CollectorScratch;
double calibration_internal::QpcToSeconds = 0.0;

// The ring's clock (QPC seconds). QueryPerformanceCounter cannot fail on
// Windows XP or later.
double calibration_internal::QpcNowSeconds()
{
	LARGE_INTEGER now;
	QueryPerformanceCounter(&now);
	return static_cast<double>(now.QuadPart) * QpcToSeconds;
}

std::unique_ptr<DriftMonitor> calibration_internal::Drift;

LighthouseFrameWatch calibration_internal::FrameWatch;
int calibration_internal::MonitorConsumer = -1;

MonitorState calibration_internal::Monitors;

std::unique_ptr<questcal::ContinuousAlignment> calibration_internal::Continuous;
int calibration_internal::ContinuousConsumer = -1;
// The hub can publish between the monitor and continuous drains. Retain that
// tail until the frame watch has seen it; never solve an unexamined frame.
std::vector<protocol::DevicePoseSample> calibration_internal::ContinuousPending;
int64_t calibration_internal::FrameObservedThrough[vr::k_unMaxTrackedDeviceCount] = {};
bool calibration_internal::FrameCapturePending = false;
double calibration_internal::FrameCaptureDue = 0.0;
double calibration_internal::LastFrameCapture = -1e9;

void calibration_internal::ResetContinuousObservations(CalibrationContext &ctx,
	questcal::ContinuousAlignment::ResetReason reason)
{
	Continuous->Reset(reason);
	ctx.continuousDiagnostics.engine = Continuous->GetDiagnostics();
	ctx.continuousCorrectionGate.Clear();
}

// The one place the windows hear about a stream or calibration event; what
// each event drops is the table in StreamEvents.h.
void calibration_internal::DispatchStreamEvent(CalibrationContext &ctx, questcal::StreamEvent event)
{
	const questcal::StreamResets resets = questcal::ResetsFor(event);
	if (resets.universeObservations)
		questcal::ResetUniverseObservations(ctx);
	if (resets.universeHoleNote)
		questcal::NoteUniverseStreamHole();
	if (resets.drift)
		Drift->Reset();
	if (resets.monitorObservations)
		Monitors.ResetObservations();
	if (resets.frameWatch)
		FrameWatch.Reset();
	if (resets.frameProgress)
	{
		std::fill(std::begin(FrameObservedThrough), std::end(FrameObservedThrough), 0);
		// The tail held back for the frame watch was read against the old frames.
		ContinuousPending.clear();
	}
	if (resets.continuous)
		ResetContinuousObservations(ctx, resets.continuousReason);
}

CalibrationContext CalCtx;

DiagnosticCapture CaptureCalibrationDiagnostics()
{
	DiagnosticCapture capture{ PoseHub.ReadDiagnostics(), questcal::CaptureDriverSyncDiagnostics() };
	capture.sampleClock = QpcNowSeconds();
	char runtimePath[4096]{};
	uint32_t needed = 0;
	if (vr::VR_GetRuntimePath(runtimePath, sizeof runtimePath, &needed))
		capture.steamVrRuntimePath = runtimePath;
	if (auto settings = vr::VRSettings())
		capture.steamVrWorldScale = settings->GetFloat(vr::k_pch_SteamVR_Section,
			vr::k_pch_SteamVR_WorldScale_Float, &capture.worldScaleError);
	return capture;
}

using questcal::ReadCurrentHmdIdentity;
using questcal::ReadTrackedDeviceString;
using questcal::SynchronizeCalibrationDriver;

static void PersistenceTick(CalibrationContext &ctx, double now)
{
	// Quiet period vs. maximum dirty age: see PersistenceState::Due.
	if (!ctx.persistence.Due(now))
		return;

	if (!SavePendingChanges(ctx))
		ctx.persistence.Retry(now);
}

// ---------------------------------------------------------------------------
// Session log (see Calibration.h)

static std::ofstream SessionLog;
static std::wstring SessionLogPath;
static std::mutex SessionLogMutex;
// The file never grows past 4 MiB, so a pathological log loop can never eat a
// user's disk; past that it keeps the session's first and latest megabyte.
static constexpr size_t SessionLogMaxBytes = 4 * 1024 * 1024;
static constexpr size_t SessionLogEndBytes = 1024 * 1024;
static constexpr size_t SessionLogLineMaxBytes = 64 * 1024;
static questcal::SessionLogTrim SessionLogBudget(SessionLogEndBytes, SessionLogEndBytes, SessionLogMaxBytes);

static std::string LocalTimeText(const char *format)
{
	char text[32] = { 0 };
	std::time_t now = std::time(nullptr);
	std::tm tm;
	if (localtime_s(&tm, &now) == 0)
		std::strftime(text, sizeof text, format, &tm);
	return text;
}

// Writes a line and, once the file reaches its budget, rewrites it as the head
// and tail SessionLogTrim keeps, through a temporary file so a crash mid-rewrite
// leaves the old log whole.
static void WriteSessionLogLine(const std::string &line)
{
	SessionLog << line;
	// Flushed per line so a crashed or killed session keeps everything.
	SessionLog.flush();
	if (!SessionLogBudget.Note(line))
		return;

	const std::string compacted = SessionLogBudget.Compacted();
	const std::wstring rewrite = SessionLogPath + L".tmp";
	std::ofstream file(rewrite.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);
	file.write(compacted.data(), static_cast<std::streamsize>(compacted.size()));
	file.close();
	SessionLog.close();
	if (file.good())
		MoveFileExW(rewrite.c_str(), SessionLogPath.c_str(), MOVEFILE_REPLACE_EXISTING);
	SessionLog.open(SessionLogPath.c_str(), std::ios::out | std::ios::app | std::ios::binary);
}

void InitSessionLog()
{
	std::lock_guard<std::mutex> lock(SessionLogMutex);
	wchar_t base[MAX_PATH];
	DWORD len = GetEnvironmentVariableW(L"LOCALAPPDATA", base, MAX_PATH);
	if (len == 0 || len >= MAX_PATH)
		return;

	std::wstring dir = std::wstring(base) + L"\\QuestCalibrator";
	CreateDirectoryW(dir.c_str(), nullptr);

	SessionLogPath = dir + L"\\QuestCalibrator.log";
	std::wstring previous = dir + L"\\QuestCalibrator.prev.log";
	// One-generation rotation: bounded disk use, but the session that ended in
	// a problem survives the restart that usually precedes the bug report.
	MoveFileExW(SessionLogPath.c_str(), previous.c_str(), MOVEFILE_REPLACE_EXISTING);

	SessionLog.open(SessionLogPath.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);
	if (!SessionLog.is_open())
		return;
	SessionLogBudget = questcal::SessionLogTrim(SessionLogEndBytes, SessionLogEndBytes, SessionLogMaxBytes);
	WriteSessionLogLine(std::string("QuestCalibrator ") + QUESTCAL_VERSION_STRING +
		" session started " + LocalTimeText("%Y-%m-%d %H:%M:%S") + "\n");
}

void AppendSessionLog(const std::string &msg)
{
	std::lock_guard<std::mutex> lock(SessionLogMutex);
	if (!SessionLog.is_open() || msg.empty())
		return;

	// Each line carries its date: a session can run past midnight.
	std::string line = LocalTimeText("[%Y-%m-%d %H:%M:%S] ");
	line.append(msg, 0, (std::min)(msg.size(), SessionLogLineMaxBytes));
	if (line.back() != '\n')
		line += '\n';
	WriteSessionLogLine(line);
}

void InitCalibrator()
{
	CalCtx.frameRecoveryPending = true;
	CalCtx.frameDriverSession = 0;
	LARGE_INTEGER freq{};
	QueryPerformanceFrequency(&freq);
	QpcToSeconds = 1.0 / static_cast<double>(freq.QuadPart);

	// The hub keeps draining the driver's shmem ring on its own thread even
	// while this (UI) thread stalls; it retries the open internally until the
	// driver creates the ring.
	PoseHub.Start(QUESTCALIBRATOR_SHMEM_NAME);
	CollectorConsumer = PoseHub.CreateConsumer();

	Drift = std::make_unique<DriftMonitor>(QpcToSeconds);
	MonitorConsumer = PoseHub.CreateConsumer();
	questcal::StartCalibrationSpace(PoseHub, QpcToSeconds);

	Continuous = std::make_unique<questcal::ContinuousAlignment>();
	CalCtx.continuousDiagnostics.engine = Continuous->GetDiagnostics();
	ContinuousConsumer = PoseHub.CreateConsumer();
	questcal::StartCalibrationDriver();
}

void ShutdownCalibrator(bool cleanExit)
{
	bool reconcileDriver = CalCtx.run.neutralizationSequence != 0;
	if (reconcileDriver)
		questcal::ReleaseCalibrationDeviceNeutralization();
	CalCtx.state = CalibrationState::None;
	CalCtx.run.Reset();
	if (reconcileDriver && vr::VRSystem())
		SynchronizeCalibrationDriver(CalCtx);

	// A quit inside the save-debounce window must not lose a runtime
	// compensation update.
	bool persisted = !CalCtx.persistence.HasDirty() || SavePendingChanges(CalCtx);
	questcal::StopCalibrationDriver();
	questcal::StopCalibrationSpace();
	PoseHub.Stop();
	if (cleanExit && persisted)
		AppendSessionLog("session ended cleanly");
	else if (cleanExit)
		AppendSessionLog("session ended with unsaved persistence changes");
}

static_assert(vr::k_unTrackedDeviceIndex_Hmd == 0, "HMD index expected to be 0");

void ResyncDriverState()
{
	// Same guard as CalibrationTick: with no runtime (UI preview) there is no
	// driver to synchronize, and synchronization reads OpenVR directly.
	if (!vr::VRSystem())
		return;
	SynchronizeCalibrationDriver(CalCtx);
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
	auto &run = ctx.run;
	const auto drained = PoseHub.DrainThroughGaps(CollectorConsumer, CollectorScratch);
	const bool crossedBoundary = PoseHub.StreamBoundaries() != run.streamBoundariesAtStart;
	if (!ringpose::CollectionGapTolerable(drained.largestGap, crossedBoundary))
	{
		char detail[160];
		snprintf(detail, sizeof detail,
			crossedBoundary ? "Driver pose session restarted during collection"
				: "Pose stream lost %llu samples in one gap during collection",
			static_cast<unsigned long long>(drained.largestGap));
		AbortCalibration(ctx, {
			"Some tracking data was dropped: the PC couldn't keep up.",
			"Try again; close capture or recording software if it repeats.",
			detail });
		return false;
	}
	run.toleratedLoss += drained.loss;
	run.toleratedGaps += drained.gaps;
	for (const auto &s : CollectorScratch)
	{
		if (s.deviceId == run.targetId && run.preserveTrackerFrames && !run.normalizationCaptured)
		{
			questcal::PoseSample first;
			if (s.sampleTimeQpc > FrameObservedThrough[s.deviceId] ||
				!TryComposeRingSample(s, QpcToSeconds, first) ||
				!ctx.trackerFrames.Normalize(s.deviceId, first))
				continue;
			const auto &frame = ctx.trackerFrames.Snapshot()[s.deviceId];
			run.targetNormalizationRotation = Eigen::Quaterniond(frame.rotation.w, frame.rotation.x, frame.rotation.y, frame.rotation.z);
			run.targetNormalizationTranslation = Eigen::Vector3d(frame.translation.v);
			run.normalizationCaptured = true;
		}
		if ((s.deviceId == run.referenceId || s.deviceId == run.targetId) &&
			IsTrustedRingSample(s, QpcToSeconds))
		{
			auto parts = UnpackRingSample(s);
			// A full calibration measures the target's frame as it finds it, so
			// a station re-solved mid-run is followed (CalibrationRun::
			// TargetFrame). An anchor refines a calibration it does not
			// re-measure, and stops as before.
			bool accepted = true;
			if (s.deviceId == run.targetId && !run.anchor)
			{
				const ringpose::DriverLocalPoseSample local{ RingSampleTime(s, QpcToSeconds), parts.drvRot,
					parts.drvPos, Eigen::Vector3d(s.velocity[0], s.velocity[1], s.velocity[2]),
					Eigen::Vector3d(s.angularVelocity[0], s.angularVelocity[1], s.angularVelocity[2]) };
				const auto verdict = run.targetFrame.Accept(local, parts.wfdRot, parts.wfdTrans);
				accepted = verdict != questcal::CalibrationRun::TargetFrame::Verdict::Changed;
				if (verdict == questcal::CalibrationRun::TargetFrame::Verdict::Moved)
					ctx.Diag("calibration: the target's lighthouse frame moved during the measurement; "
						"measuring on in the frame it began in");
			}
			else
			{
				accepted = run.AcceptUniverse(s.deviceId, parts.wfdRot, parts.wfdTrans);
			}
			if (!accepted)
			{
				bool reference = s.deviceId == run.referenceId;
				std::string name = DeviceName(ctx,
					reference ? run.referenceModel : run.targetModel,
					reference ? run.referenceSerial : run.targetSerial, reference);
				AbortCalibration(ctx, {
					name + "'s tracking reset during the measurement.",
					"Let tracking settle for a few seconds, then start again.",
					std::string(reference ? "Reference" : "Target") + " world-from-driver changed during collection" });
				return false;
			}
		}
		// The composed time folds in the driver's jittery poseTimeOffset, so
		// the odd inversion occurs in healthy data; drop it here (relative to
		// this collection's tail) rather than let the solver refuse the whole
		// collection.
		questcal::PoseSample sample;
		if (s.deviceId == run.referenceId)
		{
			if (!TryComposeRingSample(s, QpcToSeconds, sample))
				continue;
			if (!run.referenceSamples.empty() && sample.time <= run.referenceSamples.back().time)
				continue;
			run.referenceSamples.push_back(sample);
			run.lastReferenceSample = now;
		}
		else if (s.deviceId == run.targetId)
		{
			if (!TryComposeRingSample(s, QpcToSeconds, sample))
				continue;
			run.targetFrame.ToStart(sample);
			if (run.anchor && !ctx.trackerFrames.Normalize(s.deviceId, sample))
				continue;
			if (!run.targetSamples.empty() && sample.time <= run.targetSamples.back().time)
				continue;
			run.targetSamples.push_back(sample);
			run.lastTargetSample = now;
		}
	}
	return true;
}

// The fallback when the raw pose channel cannot serve the selected pair (see
// CollectionSource.h): samples runtime poses at tick rate. Loses per-device
// capture timestamps and driver velocities, so alignment quality is reduced.
static void CollectFromRuntimePoses(CalibrationContext &ctx, double now)
{
	auto &run = ctx.run;
	// The run's ids were bounds-checked when it began.
	auto push = [&](uint32_t id, std::vector<questcal::PoseSample> &into, double &lastTime)
	{
		questcal::PoseSample s;
		if (!questcal::RuntimePoseSample(ctx.devicePoses[id], now, s) ||
			(!into.empty() && s.time <= into.back().time))
			return;
		if (run.anchor && id == run.targetId)
		{
			// Runtime fallback uses the UI clock, not QPC; only the transform
			// applies here (the selected driver slot was neutralized).
			ctx.trackerFrames.ApplyToPose(id, s);
		}
		into.push_back(s);
		lastTime = now;
	};

	push(run.referenceId, run.referenceSamples, run.lastReferenceSample);
	push(run.targetId, run.targetSamples, run.lastTargetSample);
}

bool calibration_internal::EndCalibrationRun(CalibrationContext &ctx)
{
	const bool heldDriver = ctx.run.End([] { questcal::ReleaseCalibrationDeviceNeutralization(); });
	ctx.state = CalibrationState::None;
	return heldDriver;
}

// Called only from within CalibrationTick, after its VRSystem() check.
void calibration_internal::AbortCalibration(CalibrationContext &ctx, const StopReason &reason)
{
	ctx.lastRunPassed = false;
	ctx.Outcome("Calibration stopped", reason.body, reason.action, reason.detail,
		CalibrationContext::Tone::Warn);
	if (EndCalibrationRun(ctx))
		SynchronizeCalibrationDriver(ctx);
}

// The player's own name for the device when they gave it one, else the model
// when it could be read, else the role the pane gave it.
std::string calibration_internal::DeviceName(const CalibrationContext &ctx, const std::string &model,
	const std::string &serial, bool reference)
{
	auto named = ctx.deviceNames.find(serial);
	if (!serial.empty() && named != ctx.deviceNames.end() && !named->second.empty())
		return named->second;
	if (!model.empty())
		return model;
	return reference ? "The reference device" : "The target device";
}

// Live lighthouse restarts (LighthouseVisibility::Device::liveRestarts) of the
// device with this serial; 0 for a device the log never named.
uint32_t calibration_internal::LighthouseRestarts(const CalibrationContext &ctx, const std::string &serial)
{
	const LighthouseVisibility::Device *seen = serial.empty() ? nullptr : ctx.lighthouse.Find(serial);
	return seen ? seen->liveRestarts : 0;
}

// A calibration measures a lighthouse device only this long after its last
// new solution, and while it sees two stations and nothing disturbed it for
// LighthouseVisibility's disturbedSeconds; it waits at most
// CalibrationWaitSeconds for that (see CalibrationTick's Begin).
static constexpr double CalibrationSettleSeconds = 15.0;
static constexpr double CalibrationWaitSeconds = 30.0;

bool StartCalibration()
{
	if (!questcal::MayStartManualRun(CalCtx.state == CalibrationState::None,
		CalCtx.referenceID, CalCtx.targetID, vr::k_unMaxTrackedDeviceCount))
	{
		CalCtx.ReportError("Finish the current calibration and pick two different available devices before starting.\n");
		return false;
	}
	if (!questcal::IsValidTrackingSystemPair(
		CalCtx.pendingReferenceTrackingSystem,
		CalCtx.pendingTargetTrackingSystem))
	{
		CalCtx.ReportError("Pick a reference and a target from two different tracking systems.\n");
		return false;
	}

	CalCtx.run.Reset();
	CalCtx.run.referenceSystem = CalCtx.pendingReferenceTrackingSystem;
	CalCtx.run.targetSystem = CalCtx.pendingTargetTrackingSystem;
	CalCtx.run.referenceId = CalCtx.referenceID;
	CalCtx.run.targetId = CalCtx.targetID;
	CalCtx.state = CalibrationState::Begin;
	CalCtx.wantedUpdateInterval = 0.0;
	CalCtx.ClearMessages();
	return true;
}

bool StartAnchorCalibration()
{
	if (!questcal::MayStartAnchorRun(CalCtx.validProfile, CalCtx.profileUniverseUnsafe,
		CalCtx.pendingReferenceTrackingSystem == CalCtx.referenceTrackingSystem &&
		CalCtx.pendingTargetTrackingSystem == CalCtx.targetTrackingSystem))
	{
		CalCtx.ReportError("Anchors need the same reference and target systems as the current calibration. Pick those devices first.\n");
		return false;
	}
	if (!StartCalibration())
		return false;
	CalCtx.run.anchor = true;
	return true;
}

static void BeginCollection(CalibrationContext &ctx, double time)
{
	auto &run = ctx.run;
	// Disabled/unsafe profiles have no running frame monitor. Their repair
	// must remain a fresh solve rather than wait on an unadvancing watermark.
	run.preserveTrackerFrames = ctx.validProfile && ctx.enabled && !ctx.frameRecoveryPending && !run.anchor &&
		run.referenceSystem == ctx.referenceTrackingSystem && run.targetSystem == ctx.targetTrackingSystem;
	if (run.preserveTrackerFrames && !run.usesPoseRing)
	{
		const auto &frame = ctx.trackerFrames.Snapshot()[run.targetId];
		run.targetNormalizationRotation = Eigen::Quaterniond(frame.rotation.w, frame.rotation.x, frame.rotation.y, frame.rotation.z);
		run.targetNormalizationTranslation = Eigen::Vector3d(frame.translation.v);
		run.normalizationCaptured = true;
	}
	run.referenceRestartsAtStart = LighthouseRestarts(ctx, run.referenceSerial);
	run.targetRestartsAtStart = LighthouseRestarts(ctx, run.targetSerial);
	run.collectionStart = time;
	run.lastReferenceSample = time;
	run.lastTargetSample = time;
	ctx.state = CalibrationState::Collecting;
	ctx.wantedUpdateInterval = 0.0;
	if (run.referenceId == vr::k_unTrackedDeviceIndex_Hmd)
	{
		// A head-referenced run: the tracker is strapped to the headset, so
		// the motion is the head's.
		ctx.Instruct("Look around slowly.");
		ctx.Note("Look left and right, then up and down. Gently tilt your head to each side. Keep the tracker in view of the base stations.");
	}
	else
	{
		ctx.Instruct("Keep both devices firmly together.");
		ctx.Note("Move both devices in a figure eight, gently turning and tilting as you go. Keep them firmly together and in view of their tracking cameras or base stations.");
	}
}

void CalibrationTick(double time)
{
	auto &ctx = CalCtx;
	// Channel health is independent of SteamVR and of the profile: the UI must
	// report it even on the ticks that return early below.
	ctx.poseRingOpen = PoseHub.RingOpen();
	// The hub restarts a failed drain by itself. Say why the samples stopped,
	// or the outage reads as the driver going away.
	const uint64_t poseHubFailures = PoseHub.Failures();
	if (poseHubFailures != PoseHubFailuresLogged)
	{
		PoseHubFailuresLogged = poseHubFailures;
		ctx.Log("Pose stream failed (" + PoseHub.LastFailure() + "); restarting it\n");
	}
	// Persistence is independent of SteamVR, profile validity, and the current
	// UI/calibration state. In particular, settings-only retries must continue
	// while the runtime is unavailable or the advanced editor is open.
	PersistenceTick(ctx, time);
	questcal::PollCalibrationDriver(ctx);

	if (!vr::VRSystem())
		return;

	if ((time - ctx.timeLastTick) < 0.02)
		return;

	ctx.timeLastTick = time;

	// Before the monitors: a log line that explains this tick's drift event
	// must already be folded in when the event is scored.
	LighthouseTick(ctx, time);
	RuntimeMonitorTick(ctx, time);
	questcal::CalibrationSpaceTick(ctx, time);
	// After the monitors: an accepted jump must land (and reset the continuous
	// window) before the continuous loop reads the calibration this tick.
	ContinuousTick(ctx, time);
	if (!ctx.detailedLogging) FrameCapturePending = false;
	if (FrameCapturePending && time >= FrameCaptureDue)
	{
		FrameCapturePending = false;
		LastFrameCapture = time;
		ctx.Diag(DescribeFrameFailureCapture(ctx, vr::VRSystem(), CaptureCalibrationDiagnostics(), QpcToSeconds));
	}
	static double lastInputDiagTime = -1e9;
	if (ctx.detailedLogging && ctx.continuousEnabled && time - lastInputDiagTime >= 10.0)
	{
		lastInputDiagTime = time;
		ctx.Diag(DescribeContinuousDiagnostics(ctx, QpcNowSeconds()));
	}

	// Runtime poses are only the compatibility collection source; the raw-ring
	// path and the idle monitors have timestamped samples.
	if (ctx.state == CalibrationState::Begin ||
		(ctx.state == CalibrationState::Collecting && !ctx.run.usesPoseRing))
	{
		vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(
			vr::TrackingUniverseRawAndUncalibrated, 0.0f, ctx.devicePoses,
			vr::k_unMaxTrackedDeviceCount);
	}

	if (ctx.state == CalibrationState::None)
	{
		if ((time - ctx.timeLastScan) >= 1.0)
		{
			SynchronizeCalibrationDriver(ctx);
			questcal::CheckProtectedChaperone(ctx);
			ctx.timeLastScan = time;
		}
		ctx.wantedUpdateInterval = ctx.IdleUpdateInterval();
		return;
	}

	if (ctx.state == CalibrationState::Editing)
	{
		ctx.wantedUpdateInterval = ctx.IdleUpdateInterval();

		if ((time - ctx.timeLastScan) >= 1.0)
		{
			// Editor values are draft-only. Keep reasserting the last committed
			// profile in case vrserver restarts while the editor is open.
			SynchronizeCalibrationDriver(ctx);
			questcal::CheckProtectedChaperone(ctx);
			ctx.timeLastScan = time;
		}
		return;
	}

	if (ctx.state == CalibrationState::Neutralizing)
	{
		auto result = questcal::TakeCalibrationNeutralization(
			ctx.run.neutralizationSequence);
		if (!result)
			return;
		if (!result->succeeded)
		{
			AbortCalibration(ctx, {
				"QuestCalibrator can't talk to SteamVR.",
				"Restart SteamVR. If it repeats, re-run the installer.",
				"Raw pose traffic is unavailable and the selected device transforms could not be neutralized" });
			return;
		}

		ctx.Log(PoseHub.RingOpen()
			? "Raw pose channel does not have fresh trusted traffic for both selected devices; using neutralized runtime poses\n"
			: "Raw pose channel unavailable; using neutralized runtime poses\n");
		BeginCollection(ctx, time);
		return;
	}

	if (ctx.state == CalibrationState::Begin)
	{
		auto &run = ctx.run;

		// Names first, so every stop reason below names the hardware the
		// player picked. A failed read falls back to the role; the serials are
		// re-read strictly below, where a failure is its own stop reason.
		if (run.referenceId < vr::k_unMaxTrackedDeviceCount)
		{
			ReadTrackedDeviceString(run.referenceId, vr::Prop_ModelNumber_String, run.referenceModel);
			ReadTrackedDeviceString(run.referenceId, vr::Prop_SerialNumber_String, run.referenceSerial);
		}
		if (run.targetId < vr::k_unMaxTrackedDeviceCount)
		{
			ReadTrackedDeviceString(run.targetId, vr::Prop_ModelNumber_String, run.targetModel);
			ReadTrackedDeviceString(run.targetId, vr::Prop_SerialNumber_String, run.targetSerial);
		}

		// One stop reason per cause, so the player learns which device and
		// what to do rather than "both devices must report Running_OK".
		auto trackingOk = [&](uint32_t id)
		{
			return ctx.devicePoses[id].bPoseIsValid &&
				ctx.devicePoses[id].eTrackingResult == vr::TrackingResult_Running_OK;
		};
		if (run.referenceId >= vr::k_unMaxTrackedDeviceCount)
		{
			AbortCalibration(ctx, {
				"No reference device is selected.",
				"Pick a device that's switched on.",
				"Missing reference device" });
			return;
		}
		if (run.targetId >= vr::k_unMaxTrackedDeviceCount)
		{
			AbortCalibration(ctx, {
				"No target device is selected.",
				"Pick a device that's switched on.",
				"Missing target device" });
			return;
		}
		// A lighthouse solution measured before it settles puts its settling
		// into the calibration (live 2026-09-26: a run 6 s into the headset
		// tracker's new solution, 2.4 s after SteamVR re-tilted the universe
		// from that solution's first reading of a station, read 1.1 to 1.3 deg
		// of tilt against the tracker's mount for the next half hour; one 16 s
		// in read 0.14 to 0.35). So the run waits for both devices to track
		// and settle rather than refusing: the player who put the headset back
		// on and pressed Calibrate the moment the tracker woke (as on that
		// night, refused at 23:50:03) gets a calibration once it can be made.
		// A device that never tracks still stops the run; one that tracks but
		// never settles is measured anyway once the wait is over.
		{
			const double ringNow = QpcNowSeconds();
			auto settled = [&](const std::string &serial)
			{
				return serial.empty() ||
					ctx.lighthouse.SettledFor(serial, ringNow, CalibrationSettleSeconds);
			};
			const bool referenceTracking = trackingOk(run.referenceId);
			const bool targetTracking = trackingOk(run.targetId);
			const bool referenceSettled = settled(run.referenceSerial);
			const bool targetSettled = settled(run.targetSerial);
			if (!referenceTracking || !targetTracking || !referenceSettled || !targetSettled)
			{
				if (run.waitStart < 0.0)
					run.waitStart = time;
				const bool tracking = referenceTracking && targetTracking;
				if (time - run.waitStart < CalibrationWaitSeconds)
				{
					const bool reference = tracking ? !referenceSettled : !referenceTracking;
					const std::string name = reference
						? DeviceName(ctx, run.referenceModel, run.referenceSerial, true)
						: DeviceName(ctx, run.targetModel, run.targetSerial, false);
					const std::string instruction = tracking
						? name + "'s tracking is settling."
						: name + " isn't tracking yet.";
					if (instruction != run.waitInstruction)
					{
						run.waitInstruction = instruction;
						run.waitNote = tracking
							? "The calibration starts on its own in a few seconds. Keep it in view of its base stations."
							: "The calibration starts on its own once it tracks. Wake it and keep it in view.";
						ctx.Log("Calibration waiting: " + instruction + "\n");
					}
					return;
				}
				if (!referenceTracking)
				{
					AbortCalibration(ctx, {
						DeviceName(ctx, run.referenceModel, run.referenceSerial, true) + " isn't tracking.",
						"Check it's awake and in view of its tracking cameras or base stations, then try again.",
						"Reference device is not Running_OK" });
					return;
				}
				if (!targetTracking)
				{
					AbortCalibration(ctx, {
						DeviceName(ctx, run.targetModel, run.targetSerial, false) + " isn't tracking.",
						"Check it's awake and visible to its base stations, then try again.",
						"Target device is not Running_OK" });
					return;
				}
				char waited[160];
				snprintf(waited, sizeof waited,
					"Calibration starting after %.0f s although tracking has not settled\n", time - run.waitStart);
				ctx.Log(waited);
			}
			run.waitInstruction.clear();
			run.waitNote.clear();
		}

		auto matchesCapturedSystem = [](uint32_t id, const std::string &expected)
		{
			std::string system;
			return ReadTrackedDeviceString(id,
				vr::Prop_TrackingSystemName_String, system) && expected == system;
		};
		if (!matchesCapturedSystem(run.referenceId, run.referenceSystem) ||
			!matchesCapturedSystem(run.targetId, run.targetSystem))
		{
			AbortCalibration(ctx, {
				"A device changed tracking system as the run started.",
				"Re-pick both devices and start again.",
				"Selected devices no longer belong to the chosen tracking systems" });
			return;
		}

		// Frozen alongside the ids: the id is a slot, and the collection window
		// is long enough for SteamVR to free one and hand it to a different
		// physical device. Read once here (properties are expensive) and
		// re-checked at 1 Hz while collecting.
		if (!ReadTrackedDeviceString(run.referenceId,
				vr::Prop_SerialNumber_String, run.referenceSerial) ||
			!ReadTrackedDeviceString(run.targetId,
				vr::Prop_SerialNumber_String, run.targetSerial))
		{
			AbortCalibration(ctx, {
				"SteamVR stopped reporting one of the devices.",
				"Wake or reconnect it, then try again.",
				"Could not verify both selected device serials" });
			return;
		}
		run.lastIdentityCheck = time;
		std::string hmdSystem;
		if (!ReadCurrentHmdIdentity(hmdSystem, run.hmdSerial) ||
			hmdSystem != run.referenceSystem)
		{
			AbortCalibration(ctx, {
				"The reference device must be on the headset's tracking system.",
				"Pick the headset or one of its controllers as the reference.",
				"Could not verify that the current HMD owns the selected reference tracking system" });
			return;
		}

		char buf[256];
		snprintf(buf, sizeof buf, "Reference device ID: %u, serial: %s\n",
			run.referenceId, run.referenceSerial.c_str());
		ctx.Log(buf);
		snprintf(buf, sizeof buf, "Target device ID: %u, serial: %s\n",
			run.targetId, run.targetSerial.c_str());
		ctx.Log(buf);

		// A mapping alone does not prove that this driver's hook sees the selected
		// pair. Drain to now and require recent trusted traffic from both devices.
		// The collector sits idle between runs, so its backlog is whatever the hub
		// still holds, and any driver drop in it is a gap: drain through them, or
		// only the prefix before the first one comes back and nothing is fresh.
		run.streamBoundariesAtStart = PoseHub.StreamBoundaries();
		const uint64_t preflightDropped =
			PoseHub.DrainThroughGaps(CollectorConsumer, CollectorScratch).loss;
		const char *preflightReason = "raw channel closed";
		run.usesPoseRing = PoseHub.RingOpen() && questcal::PreflightPoseRing(
			run, CollectorScratch, QpcNowSeconds(), QpcToSeconds, preflightReason);
		// The loss is the idle collector's backlog overflowing since its last
		// run (every sample of a long session reads as lost), not stream health.
		snprintf(buf, sizeof buf, "pose preflight: %s; batch %zu samples, dropped while idle %llu",
			preflightReason, CollectorScratch.size(), static_cast<unsigned long long>(preflightDropped));
		ctx.Diag(buf);

		if (run.usesPoseRing)
		{
			DiscardPoseRingBacklog();
			ctx.Log("Sampling raw driver poses (timestamped)\n");
			BeginCollection(ctx, time);
		}
		else
		{
			// Runtime poses may already include the active profile. Neutralize both
			// selected slots so reversing calibration roles cannot solve against an
			// already transformed reference.
			run.neutralizationSequence = questcal::NeutralizeCalibrationDevices(
				{ run.referenceId, run.targetId }, time);
			ctx.state = CalibrationState::Neutralizing;
			ctx.Log("Preparing neutral runtime poses for calibration\n");
		}
		return;
	}

	// ---- Collecting ----
	// A closed raw channel is a source failure rather than a selected-device fault.
	auto &run = ctx.run;
	if (run.usesPoseRing && !PoseHub.RingOpen())
	{
		AbortCalibration(ctx, {
			"Lost contact with SteamVR mid-run.",
			"Restart SteamVR and try again.",
			"The driver pose channel closed mid-collection; the remaining samples would carry a different clock" });
		return;
	}

	// The frozen pair is an OpenVR index. Re-read the two serials at 1 Hz --
	// never per sample, these are expensive property reads -- so a device that
	// power-cycled into another device's slot cannot have its poses concatenated
	// into one buffer and fitted as a single rigid body. An unreadable identity
	// also stops collection, but is not evidence that the device was replaced.
	if (time - run.lastIdentityCheck >= 1.0)
	{
		run.lastIdentityCheck = time;
		auto identityMatches = [&](uint32_t id, const std::string &frozen,
			const std::string &model, bool reference)
		{
			std::string serial;
			bool readable = ReadTrackedDeviceString(id, vr::Prop_SerialNumber_String, serial);
			if (readable && serial == frozen)
				return true;
			std::string name = DeviceName(ctx, model, frozen, reference);
			AbortCalibration(ctx, {
				readable ? name + " was replaced by another device during the measurement."
					: "Could not verify the identity of " + name + " during the measurement.",
				"Keep both devices powered and try again.",
				std::string(reference ? "Reference" : "Target") +
					(readable ? " serial changed mid-collection" : " serial could not be read mid-collection") });
			return false;
		};
		if (!identityMatches(run.referenceId, run.referenceSerial, run.referenceModel, true) ||
			!identityMatches(run.targetId, run.targetSerial, run.targetModel, false))
			return;
	}

	if (run.usesPoseRing)
	{
		if (!CollectFromPoseRing(ctx, time))
			return;
	}
	else
		CollectFromRuntimePoses(ctx, time);

	// The ring-vs-runtime distinction is invisible to the player; it stays in
	// the detail line.
	if (time - run.lastReferenceSample > 2.0)
	{
		AbortCalibration(ctx, {
			DeviceName(ctx, run.referenceModel, run.referenceSerial, true) + " stopped tracking mid-run.",
			"Keep it tracking for the whole countdown.",
			run.usesPoseRing ? "No trusted Running_OK raw poses arrived for the reference device"
				: "Reference device stopped reporting Running_OK runtime poses" });
		return;
	}
	if (time - run.lastTargetSample > 2.0)
	{
		AbortCalibration(ctx, {
			DeviceName(ctx, run.targetModel, run.targetSerial, false) + " stopped tracking mid-run.",
			"Keep it in view of its base stations for the whole countdown.",
			run.usesPoseRing ? "No trusted Running_OK raw poses arrived for the target device"
				: "Target device stopped reporting Running_OK runtime poses" });
		return;
	}

	double duration = ctx.CollectionSeconds();
	double elapsed = time - run.collectionStart;
	ctx.Progress(static_cast<int>(elapsed * 100.0), static_cast<int>(duration * 100.0));

	if (elapsed >= duration)
		FinishCalibration(ctx);
}

void CancelCalibration()
{
	auto &ctx = CalCtx;
	if (ctx.state != CalibrationState::Begin && ctx.state != CalibrationState::Neutralizing &&
		ctx.state != CalibrationState::Collecting)
		return;
	ctx.lastRunPassed = false;
	ctx.Outcome("Calibration cancelled", "", "", "");
	if (EndCalibrationRun(ctx) && vr::VRSystem())
		SynchronizeCalibrationDriver(ctx);
}
