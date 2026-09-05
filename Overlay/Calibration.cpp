#include "stdafx.h"
#include "Calibration.h"
#include "CalibrationDriver.h"
#include "CalibrationEngine.h"
#include "CalibrationSpace.h"
#include "LegacyContinuous.h"
#include "Configuration.h"
#include "DriftMonitor.h"
#include "FieldMath.h"
#include "PoseStreamHub.h"
#include "ProfileValidation.h"
#include "RingPoseMath.h"
#include "../common/PoseChannel.h"
#include "../common/Version.h"

#include <Eigen/Dense>

#include <ctime>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

static PoseStreamHub PoseHub;
static int CollectorConsumer = -1;
static std::vector<protocol::DevicePoseSample> CollectorScratch;
static double QpcToSeconds = 0.0;

static std::unique_ptr<DriftMonitor> Drift;
static int MonitorConsumer = -1;
static std::vector<protocol::DevicePoseSample> MonitorScratch;
static bool MonitorActive = false;

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

static std::unique_ptr<questcal::ContinuousAlignment> Continuous;
static int ContinuousConsumer = -1;
static std::vector<protocol::DevicePoseSample> ContinuousScratch;
static bool ContinuousActive = false;

static void ResetContinuousObservations(CalibrationContext &ctx)
{
	Continuous->Reset();
	ctx.continuousCorrectionGate.Clear();
}

CalibrationContext CalCtx;

// Why a run stopped, in the player's words: what happened (naming what they
// can see) and one thing to do. `detail` is the engineer's reason, kept
// verbatim for the session log and the modal's details toggle.
struct StopReason
{
	std::string body;
	std::string action;
	std::string detail;
	// The picture the modal shows with it; None for environmental stops.
	CalibrationContext::GuideHint hint = CalibrationContext::GuideHint::None;
};

static void AbortCalibration(CalibrationContext &ctx, const StopReason &reason);
static std::string DeviceName(const CalibrationContext &ctx, const std::string &model,
                              const std::string &serial, bool reference);
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
static size_t SessionLogBytes = 0;
static std::mutex SessionLogMutex;
// Hard cap so a pathological log loop can never eat a user's disk.
static constexpr size_t SessionLogMaxBytes = 4 * 1024 * 1024;

void InitSessionLog()
{
	std::lock_guard<std::mutex> lock(SessionLogMutex);
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
	std::streampos written = SessionLog.tellp();
	SessionLogBytes = written > 0 ? static_cast<size_t>(written) : 0;
}

void AppendSessionLog(const std::string &msg)
{
	std::lock_guard<std::mutex> lock(SessionLogMutex);
	if (!SessionLog.is_open() || msg.empty() || SessionLogBytes >= SessionLogMaxBytes)
		return;

	char stamp[16] = { 0 };
	std::time_t now = std::time(nullptr);
	std::tm tm;
	if (localtime_s(&tm, &now) == 0)
		std::strftime(stamp, sizeof stamp, "[%H:%M:%S] ", &tm);

	const size_t stampBytes = strlen(stamp);
	const size_t framingBytes = stampBytes + 1;
	const size_t remaining = SessionLogMaxBytes - SessionLogBytes;
	if (remaining <= framingBytes)
	{
		SessionLogBytes = SessionLogMaxBytes;
		return;
	}

	const size_t count = (std::min)(msg.size(), remaining - framingBytes);
	SessionLog << stamp;
	SessionLog.write(msg.data(), count);
	if (msg[count - 1] != '\n')
		SessionLog << '\n';

	SessionLogBytes += stampBytes + count + (msg[count - 1] == '\n' ? 0 : 1);
	// Flushed per line so a crashed or killed session keeps everything.
	SessionLog.flush();
}

void InitCalibrator()
{
	LARGE_INTEGER freq{};
	if (!QueryPerformanceFrequency(&freq) || freq.QuadPart <= 0)
		throw std::runtime_error("QueryPerformanceFrequency failed");
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

static bool PreflightPoseRing(questcal::CalibrationRun &run,
	const std::vector<protocol::DevicePoseSample> &samples, double qpcNow)
{
	questcal::CalibrationRun::Universe reference;
	questcal::CalibrationRun::Universe target;
	for (const auto &sample : samples)
	{
		if (!IsTrustedRingSample(sample, QpcToSeconds) ||
			!ringpose::IsFreshCaptureTime(
				RingCaptureTime(sample, QpcToSeconds), qpcNow, 0.5))
			continue;

		auto parts = UnpackRingSample(sample);
		auto *universe = sample.deviceId == run.referenceId ? &reference :
			sample.deviceId == run.targetId ? &target : nullptr;
		if (universe && !universe->Accept(parts.wfdRot, parts.wfdTrans))
			return false;
	}
	if (!reference.valid || !target.valid)
		return false;
	run.referenceUniverse = reference;
	run.targetUniverse = target;
	return true;
}

static bool CollectFromPoseRing(CalibrationContext &ctx, double now)
{
	auto &run = ctx.run;
	uint64_t dropped = PoseHub.Drain(CollectorConsumer, CollectorScratch);
	if (dropped > 0)
	{
		AbortCalibration(ctx, {
			"Some tracking data was dropped: the PC couldn't keep up.",
			"Try again; close capture or recording software if it repeats.",
			"Pose stream overran during collection; retry calibration so no samples are missing" });
		return false;
	}
	for (const auto &s : CollectorScratch)
	{
		if ((s.deviceId == run.referenceId || s.deviceId == run.targetId) &&
			IsTrustedRingSample(s, QpcToSeconds))
		{
			auto parts = UnpackRingSample(s);
			if (!run.AcceptUniverse(s.deviceId, parts.wfdRot, parts.wfdTrans))
			{
				bool reference = s.deviceId == run.referenceId;
				std::string name = DeviceName(ctx,
					reference ? run.referenceModel : run.targetModel,
					reference ? run.referenceSerial : run.targetSerial, reference);
				AbortCalibration(ctx, {
					name + " changed tracking space during the measurement.",
					"Let tracking settle for a few seconds, then start again.",
					std::string(reference ? "Reference" : "Target") + " world-from-driver changed during collection",
					CalibrationContext::GuideHint::WaitForTracking });
				return false;
			}
		}
		// The composed time folds in the driver's jittery poseTimeOffset, so
		// the odd inversion occurs in healthy data; drop it here rather than
		// let the solver fail the whole collection (same policy as
		// ContinuousAlignment::PushReference). This guard is buffer-relative on
		// purpose: it compares against this collection's own tail, not any
		// device-global watermark the monitors keep.
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
			if (!run.targetSamples.empty() && sample.time <= run.targetSamples.back().time)
				continue;
			run.targetSamples.push_back(sample);
			run.lastTargetSample = now;
		}
	}
	return true;
}

// Fallback when the shmem channel is unavailable (e.g. running against an old
// driver build): sample runtime poses at tick rate. Loses per-device capture
// timestamps and driver velocities, so alignment quality is reduced.
static void CollectFromRuntimePoses(CalibrationContext &ctx, double now)
{
	auto &run = ctx.run;
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

	push(run.referenceId, run.referenceSamples, run.lastReferenceSample);
	push(run.targetId, run.targetSamples, run.lastTargetSample);
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

// One-shot player-facing line + optional VR toast; each caller owns its
// re-arm flag. The line goes through Tell so it reaches the main screen's
// activity feed as well as the log.
static void NotifyOnce(CalibrationContext &ctx, bool &notified, const char *line,
	CalibrationContext::Tone tone, const char *toast, bool showToast)
{
	if (notified)
		return;
	notified = true;

	// Unconditional and first: "the user turned toasts off" and "there is
	// nowhere to toast" must both still leave the episode in the session log,
	// which is the bug-report surface for a GUI binary.
	ctx.Tell(std::string(line) + "\n", tone);

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
		"Your calibration looks off. Run a new calibration.",
		CalibrationContext::Tone::Warn,
		"QuestCalibrator: your calibration looks off. Run a new calibration.",
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

	// One slide window is evidence rather than a verdict. Cap the combined score
	// until another event corroborates it; persistent drift quickly lifts the cap.
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
			questcal::ResetUniverseObservations(ctx);
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
		questcal::ResetUniverseObservations(ctx);
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
			questcal::ObserveUniversePose(s);

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

		// AnchorsUniverse excludes devices whose normal movement could be
		// mistaken for playspace drift, including the mounted tracker.
		ringpose::DriftFeedCandidate candidate;
		candidate.deviceId = s.deviceId;
		candidate.referenceSide = ctx.referenceDeviceMask[s.deviceId];
		candidate.targetSide = ctx.targetDeviceMask[s.deviceId];
		candidate.mountedTrackerId = ctx.continuousTrackerId;
		candidate.rawPosition = sample.pos;
		candidate.composedTime = composedTime;
		candidate.hmdRawPosition = Monitors.hmdRawPosition;
		candidate.hmdRawTime = Monitors.hmdRawTime;
		candidate.calibratedRotation = ctx.transform.rotation;
		candidate.calibratedTranslationMeters = ctx.transform.translationMeters;
		candidate.calibratedScale = ctx.transform.scale;

		if (ringpose::AnchorsUniverse(candidate))
			Drift->Push(s,
				ctx.targetDeviceMask[s.deviceId] ? ctx.transform.scale : 1.0);
	}

	const bool jumped = questcal::FinishUniverseObservations(ctx, now);

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
		ResetContinuousObservations(ctx);
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

// ---- Legacy loop: hyblocker's calculator driven the way its overlay drove
// it. The latest reference and tracker poses are paired at 20 Hz, the window
// is the calibration duration's sample count (100 / 250 / 500), every full
// window is re-solved, and a tenth of it is dropped afterwards. An accepted
// solve is applied as a delta over the current calibration through the same
// slewing path as the QuestCalibrator loop's corrections.
static questcal::legacy::CalibrationCalc LegacyCalc;
static double LegacyLastSampleTime = -1e9;
static bool LegacyHaveRef = false, LegacyHaveTarget = false;
static bool LegacyRefFresh = false, LegacyTargetFresh = false;
static double LegacyLastRefArrival = -1e9, LegacyLastTargetArrival = -1e9;
static questcal::PoseSample LegacyRef, LegacyTarget;
static Eigen::Vector3d LegacyPrevHmdPos = Eigen::Vector3d::Zero();
static Eigen::Quaterniond LegacyPrevHmdRot{ 1, 0, 0, 0 };
static bool LegacyHavePrevHmd = false;
static bool LegacyWaitingTrigger = false;
static ContinuousMode LastContinuousMode = ContinuousMode::Quest;
static bool LegacyBindingValid = false;
static uint32_t LegacyBoundTrackerId = vr::k_unTrackedDeviceIndexInvalid;
static uint32_t LegacyBoundBaseGeneration = 0;
static std::string LegacyBoundTrackerSerial;
static constexpr double LegacyPoseStaleSeconds = 0.5;
static constexpr double LegacyMaxPairSkewSeconds = 0.1;

static size_t LegacySampleWindow(const CalibrationContext &ctx)
{
	switch (ctx.calibrationSpeed)
	{
	case CalibrationContext::FAST: return 100;
	case CalibrationContext::SLOW: return 250;
	default: return 500;
	}
}

static void LegacyReset()
{
	LegacyCalc.Clear();
	LegacyHaveRef = LegacyHaveTarget = false;
	LegacyRefFresh = LegacyTargetFresh = false;
	LegacyLastRefArrival = LegacyLastTargetArrival = -1e9;
	LegacyLastSampleTime = -1e9;
	LegacyPrevHmdPos = Eigen::Vector3d::Zero();
	LegacyPrevHmdRot = Eigen::Quaterniond::Identity();
	LegacyHavePrevHmd = false;
	LegacyWaitingTrigger = false;
	LegacyBindingValid = false;
}

static bool AnyControllerTriggerPressed();

static void LegacyContinuousTick(CalibrationContext &ctx, double now)
{
	auto bindCurrentProfile = [&]()
	{
		LegacyBoundTrackerId = ctx.continuousTrackerId;
		LegacyBoundBaseGeneration = ctx.baseGeneration;
		LegacyBoundTrackerSerial = ctx.continuousTrackerSerial;
		LegacyBindingValid = true;
	};
	if (!LegacyBindingValid ||
		LegacyBoundTrackerId != ctx.continuousTrackerId ||
		LegacyBoundBaseGeneration != ctx.baseGeneration ||
		LegacyBoundTrackerSerial != ctx.continuousTrackerSerial)
	{
		LegacyReset();
		ctx.continuousCorrectionGate.Clear();
		bindCurrentProfile();
	}

	const uint64_t dropped = PoseHub.Drain(ContinuousConsumer, ContinuousScratch);
	if (dropped > 0)
	{
		LegacyReset();
		ctx.continuousCorrectionGate.Clear();
		bindCurrentProfile();
	}
	for (const auto &s : ContinuousScratch)
	{
		if (s.deviceId >= vr::k_unMaxTrackedDeviceCount)
			continue;
		questcal::PoseSample sample;
		if (s.deviceId == vr::k_unTrackedDeviceIndex_Hmd)
		{
			if (TryComposeRingSample(s, QpcToSeconds, sample))
			{
				LegacyRef = sample;
				LegacyHaveRef = true;
				LegacyRefFresh = true;
				LegacyLastRefArrival = now;
			}
		}
		else if (s.deviceId == ctx.continuousTrackerId)
		{
			if (TryComposeRingSample(s, QpcToSeconds, sample))
			{
				LegacyTarget = sample;
				LegacyHaveTarget = true;
				LegacyTargetFresh = true;
				LegacyLastTargetArrival = now;
			}
		}
	}
	if (!LegacyHaveRef || !LegacyHaveTarget ||
		now - LegacyLastRefArrival > LegacyPoseStaleSeconds ||
		now - LegacyLastTargetArrival > LegacyPoseStaleSeconds)
	{
		if (LegacyHaveRef || LegacyHaveTarget)
		{
			LegacyReset();
			ctx.continuousCorrectionGate.Clear();
			bindCurrentProfile();
		}
		ctx.continuousState = questcal::ContinuousAlignment::State::Inactive;
		return;
	}

	auto applyCorrection = [&](const questcal::ContinuousAlignment::Correction &correction)
	{
		if (!questcal::ApplyCalibrationDelta(ctx, correction.rotation,
			correction.translation, /*snap=*/false, now))
		{
			ctx.ReportError("A correction was too large to be safe and was skipped. If this keeps happening, recalibrate.\n");
			return;
		}
		ctx.lastAutoCorrectionUnixTime = static_cast<double>(std::time(nullptr));
		ctx.autoCorrectionsApplied++;
		ctx.driftSlideEvents = 0;
		ctx.driftMaxSlideM = 0.0;
		ctx.discontinuousLossEvents = 0;
	};

	const bool triggerPressed = ctx.continuousRequireTrigger &&
		AnyControllerTriggerPressed();
	questcal::ContinuousAlignment::Correction correction;
	if (ctx.continuousCorrectionGate.HasPending())
	{
		if (!ctx.continuousCorrectionGate.Take(
			ctx.continuousRequireTrigger, triggerPressed, correction))
		{
			if (!LegacyWaitingTrigger)
				ctx.Tell("Correction ready. Pull a trigger to apply it.\n");
			LegacyWaitingTrigger = true;
			return;
		}
		if (ctx.continuousRequireTrigger)
			ctx.Tell("Trigger pulled; applying the correction.\n");
		LegacyWaitingTrigger = false;
		applyCorrection(correction);
		return;
	}

	if (!LegacyRefFresh || !LegacyTargetFresh)
		return;

	const double pairSkew = LegacyRef.time - LegacyTarget.time;
	if (std::abs(pairSkew) > LegacyMaxPairSkewSeconds)
	{
		// Keep the newer observation and wait for the lagging device to catch up.
		if (pairSkew < 0.0)
			LegacyRefFresh = false;
		else
			LegacyTargetFresh = false;
		return;
	}

	if (now - LegacyLastSampleTime < CalibrationContext::ContinuousInputInterval)
		return;
	LegacyLastSampleTime = now;
	LegacyRefFresh = LegacyTargetFresh = false;

	// The original skipped a tick whose headset pose had not moved at all. Check
	// the complete pose so rotation-only calibration motion is not discarded.
	if (LegacyHavePrevHmd && LegacyRef.pos == LegacyPrevHmdPos &&
		LegacyRef.rot.coeffs() == LegacyPrevHmdRot.coeffs())
		return;
	LegacyPrevHmdPos = LegacyRef.pos;
	LegacyPrevHmdRot = LegacyRef.rot;
	LegacyHavePrevHmd = true;

	// Target positions pre-scaled, as everywhere else the calibration is
	// composed with a solved scale; the original had no scale.
	LegacyCalc.PushSample(questcal::legacy::Sample(
		questcal::legacy::Pose(LegacyRef.rot, LegacyRef.pos),
		questcal::legacy::Pose(LegacyTarget.rot, LegacyTarget.pos * ctx.transform.scale),
		now));

	const size_t window = LegacySampleWindow(ctx);
	if (LegacyCalc.SampleCount() < window)
	{
		ctx.continuousState = LegacyCalc.isValid()
			? questcal::ContinuousAlignment::State::Tracking
			: questcal::ContinuousAlignment::State::Inactive;
		return;
	}
	while (LegacyCalc.SampleCount() > window)
		LegacyCalc.ShiftSample();

	bool lerp = false;
	LegacyCalc.enableStaticRecalibration = false;   // the original's default
	LegacyCalc.lockRelativePosition = false;
	const bool updated = LegacyCalc.ComputeIncremental(lerp, 1.5, 0.005, false);

	if (updated && LegacyCalc.isValid())
	{
		const Eigen::AffineCompact3d est = LegacyCalc.Transformation();
		const Eigen::Quaterniond newRotation(est.rotation());
		const Eigen::Vector3d newTranslation = est.translation();
		Eigen::Quaterniond deltaRotation;
		Eigen::Vector3d deltaTranslation;
		questcal::legacy::DeltaBetweenCalibrations(
			ctx.transform.rotation, ctx.transform.translationMeters,
			newRotation, newTranslation, deltaRotation, deltaTranslation);
		correction.rotation = deltaRotation;
		correction.translation = deltaTranslation;
		ctx.continuousCorrectionGate.Offer(correction, triggerPressed);
		if (!ctx.continuousCorrectionGate.Take(
			ctx.continuousRequireTrigger, triggerPressed, correction))
		{
			if (!LegacyWaitingTrigger)
				ctx.Tell("Correction ready. Pull a trigger to apply it.\n");
			LegacyWaitingTrigger = true;
		}
		else
		{
			if (ctx.continuousRequireTrigger)
				ctx.Tell("Trigger pulled; applying the correction.\n");
			LegacyWaitingTrigger = false;
			applyCorrection(correction);
		}
	}
	ctx.continuousState = LegacyCalc.isValid()
		? questcal::ContinuousAlignment::State::Tracking
		: questcal::ContinuousAlignment::State::Inactive;

	char buf[192];
	snprintf(buf, sizeof buf, "legacy loop: %zu samples, solve %s, error %.1f cm, %u corrections",
		LegacyCalc.SampleCount(), updated ? "accepted" : "kept", LegacyCalc.m_lastError * 100.0,
		ctx.autoCorrectionsApplied);
	ctx.Diag(buf);

	for (size_t i = 0; i < window / 10; ++i)
		LegacyCalc.ShiftSample();
}

static bool AnyControllerTriggerPressed()
{
	auto system = vr::VRSystem();
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		if (system->GetTrackedDeviceClass(id) !=
			vr::TrackedDeviceClass_Controller)
			continue;
		vr::VRControllerState_t state{};
		if (!system->GetControllerState(id, &state, sizeof state))
			continue;
		if (questcal::ControllerTriggerPressed(state, [&](vr::ETrackedDeviceProperty property)
		{
			vr::ETrackedPropertyError error = vr::TrackedProp_Success;
			const int32_t type = system->GetInt32TrackedDeviceProperty(id, property, &error);
			return error == vr::TrackedProp_Success ? type : vr::k_eControllerAxis_None;
		}))
			return true;
	}
	return false;
}

static void ContinuousTick(CalibrationContext &ctx, double now)
{
	if (!ctx.ContinuousShouldRun())
	{
		if (ContinuousActive)
		{
			ResetContinuousObservations(ctx);
			LegacyReset();
			ContinuousActive = false;
		}
		PoseHub.DiscardBacklog(ContinuousConsumer);
		ctx.continuousCorrectionGate.Clear();
		ctx.continuousState = ctx.continuousMode == ContinuousMode::Legacy
			? questcal::ContinuousAlignment::State::Inactive : Continuous->GetState();
		return;
	}

	if (!ContinuousActive)
	{
		PoseHub.DiscardBacklog(ContinuousConsumer);
		ContinuousActive = true;
	}

	// A method switch starts the other loop from nothing: their windows and
	// baselines mean different things.
	if (ctx.continuousMode != LastContinuousMode)
	{
		ResetContinuousObservations(ctx);
		LegacyReset();
		LastContinuousMode = ctx.continuousMode;
	}
	if (ctx.continuousMode == ContinuousMode::Legacy)
	{
		LegacyContinuousTick(ctx, now);
		return;
	}

	// Cheap unconditional sync: the extrinsic changes only on recalibration
	// or profile load, but re-copying it every tick needs no bookkeeping.
	Continuous->SetExtrinsic(ctx.mountExtrinsic);
	Continuous->SetLatencyReestimation(ctx.continuousLatencyReestimation);

	uint64_t dropped = PoseHub.Drain(ContinuousConsumer, ContinuousScratch);
	if (dropped > 0)
	{
		// A drain hole could fake a discontinuity; baselines across it are unsafe.
		ResetContinuousObservations(ctx);
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
			ctx.transform.rotation, ctx.transform.translationMeters, ctx.transform.scale,
			targetRawPos);
		// Same width SendAlignmentField put on the wire - the expectation has to
		// be the field the driver is actually applying, not a similar one.
		questcal::BlendedFieldCalibration(ctx.ActiveFieldAnchors(), ctx.transform.rotation,
			ctx.transform.translationMeters, basePos, rotationOut, translationOut,
			questcal::FieldBlendSigmaMeters);
	};
	Continuous->Update(ringNow, ctx.transform.rotation, ctx.transform.translationMeters,
		ctx.transform.scale, ctx.transform.timeOffset, expectedAt);

	questcal::ContinuousAlignment::Correction corr;
	bool hadPendingCorrection = ctx.continuousCorrectionGate.HasPending();
	bool receivedCorrection = false;
	while (Continuous->PollCorrection(corr))
		receivedCorrection = true;
	if (!Continuous->CorrectionEligible())
	{
		ctx.continuousCorrectionGate.Clear();
		hadPendingCorrection = false;
		receivedCorrection = false;
	}

	bool triggerPressed = ctx.continuousRequireTrigger &&
		(hadPendingCorrection || receivedCorrection) && AnyControllerTriggerPressed();
	if (receivedCorrection)
		ctx.continuousCorrectionGate.Offer(corr, triggerPressed);

	if (ctx.continuousCorrectionGate.HasPending())
	{
		if (!ctx.continuousCorrectionGate.Take(
			ctx.continuousRequireTrigger, triggerPressed, corr))
		{
			if (!hadPendingCorrection)
				ctx.Tell("Correction ready. Pull a trigger to apply it.\n");
		}
		else
		{
			if (ctx.continuousRequireTrigger)
				ctx.Tell("Trigger pulled; applying the correction.\n");
			if (!questcal::ApplyCalibrationDelta(ctx, corr.rotation, corr.translation,
				/*snap=*/false, now))
			{
				ctx.ReportError("A correction was too large to be safe and was skipped. If this keeps happening, recalibrate.\n");
			}
			else
			{
				ctx.lastAutoCorrectionUnixTime = static_cast<double>(std::time(nullptr));
				ctx.autoCorrectionsApplied++;

				// The evidence these counters accumulated was just acted on (same
				// rationale as the post-jump reset); the monitor windows themselves
				// stay valid because a correction never moves raw poses.
				ctx.driftSlideEvents = 0;
				ctx.driftMaxSlideM = 0.0;
				ctx.discontinuousLossEvents = 0;
				char buf[128];
				snprintf(buf, sizeof buf, "correction applied: yaw %.3f deg, position %.1f mm",
					2.0 * std::asin(std::min(1.0, std::abs(corr.rotation.y()))) * 180.0 / EIGEN_PI,
					corr.translation.norm() * 1000.0);
				ctx.Diag(buf);
			}
		}
	}

	// The loop's decision inputs, once every evaluate interval: what a bug
	// report about "it keeps pausing" needs and the pane does not.
	static double lastDiagTime = -1e9;
	if (ctx.detailedLogging && now - lastDiagTime >= 2.0)
	{
		lastDiagTime = now;
		char buf[256];
		snprintf(buf, sizeof buf,
			"continuous: state %d, deviation yaw %.2f deg tilt %.2f deg pos %.1f cm, scatter %.2f deg / %.1f cm, %u corrections",
			static_cast<int>(ctx.continuousState),
			ctx.continuousDeviation.yawDeg, ctx.continuousDeviation.tiltDeg, ctx.continuousDeviation.posM * 100.0,
			ctx.continuousScatterRotDeg, ctx.continuousScatterPosM * 100.0, ctx.autoCorrectionsApplied);
		ctx.Diag(buf);
	}

	questcal::ContinuousAlignment::Event ev;
	while (Continuous->PollEvent(ev))
	{
		char buf[256];
		switch (ev.type)
		{
		// Both freeze causes keep their evidence in the detail line. What the
		// player reads is the fact (the readings disagree with the calibration
		// by more than the loop will correct on its own), not a diagnosis: a
		// strapped tracker does not move, and blaming it sent people to check
		// hardware that was fine.
		case questcal::ContinuousAlignment::Event::FrozenLargeDeviation:
			snprintf(buf, sizeof buf,
				"Continuous calibration frozen: deviation yaw %.2f deg, tilt %.2f deg, %.1f cm\n",
				ev.deviation.yawDeg, ev.deviation.tiltDeg, ev.deviation.posM * 100.0);
			ctx.Log(buf);
			NotifyOnce(ctx, Monitors.freezeNotified,
				"Continuous calibration paused: readings drifted too far from the calibration to correct safely.",
				CalibrationContext::Tone::Warn,
				"QuestCalibrator: continuous calibration paused; readings drifted too far to correct safely. Recalibrate with the headset tracker to resume.",
				ctx.notifyPoorCalibration);
			break;
		case questcal::ContinuousAlignment::Event::FrozenMountScatter:
			// The event now carries the scatter it froze on, so this no longer
			// re-reads live accessors that have moved on since it was raised.
			snprintf(buf, sizeof buf,
				"Continuous calibration frozen: observation scatter %.2f deg / %.1f cm stays far above the tracking noise -- structured scatter\n",
				ev.scatterRotDeg, ev.scatterPosM * 100.0);
			ctx.Log(buf);
			NotifyOnce(ctx, Monitors.freezeNotified,
				"Continuous calibration paused: readings are inconsistent with the headset tracker measurement.",
				CalibrationContext::Tone::Warn,
				"QuestCalibrator: continuous calibration paused; readings are inconsistent. Recalibrate with the headset tracker to resume.",
				ctx.notifyPoorCalibration);
			break;
		case questcal::ContinuousAlignment::Event::ObservationsUnstable:
			snprintf(buf, sizeof buf,
				"Mounted tracker observations unstable (scatter %.2f deg / %.1f cm) -- alignment updates paused until tracking settles\n",
				Continuous->ScatterRotRmsDeg(), Continuous->ScatterPosRmsM() * 100.0);
			ctx.Log(buf);
			NotifyOnce(ctx, Monitors.unstableNotified,
				"Tracking is noisy here; continuous calibration is waiting and resumes on its own.",
				CalibrationContext::Tone::Warn,
				"QuestCalibrator: tracking is noisy here. Continuous calibration is waiting and resumes on its own.",
				ctx.notifyPoorCalibration);
			break;
		case questcal::ContinuousAlignment::Event::Resumed:
			ctx.Tell("Continuous calibration resumed.\n", CalibrationContext::Tone::Good);
			Monitors.freezeNotified = false;
			break;
		case questcal::ContinuousAlignment::Event::TrackerLost:
			ctx.Tell("Headset tracker not tracking; continuous calibration waiting.\n");
			break;
		case questcal::ContinuousAlignment::Event::TrackerRecovered:
			ctx.Tell("Headset tracker back; continuous calibration warming up.\n");
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
		double target = 0.75 * ctx.transform.timeOffset + 0.25 * measuredOffset;
		double step = target - ctx.transform.timeOffset;
		if (step > 0.002) step = 0.002;
		if (step < -0.002) step = -0.002;
		double updated = ctx.transform.timeOffset + step;
		if (updated > 0.060) updated = 0.060;
		if (updated < -0.060) updated = -0.060;

		double appliedBefore = questcal::ComputeAppliedTimeOffset(ctx.transform.timeOffset);
		double appliedAfter = questcal::ComputeAppliedTimeOffset(updated);
		ctx.transform.timeOffset = updated;
		ctx.persistence.MarkProfile(now);
		if (std::abs(appliedAfter - appliedBefore) > 0.0005)
			SynchronizeCalibrationDriver(ctx);
	}

	ctx.continuousState = Continuous->GetState();
	ctx.continuousDeviation = Continuous->CurrentDeviation();
	ctx.continuousScatterRotDeg = Continuous->ScatterRotRmsDeg();
	ctx.continuousScatterPosM = Continuous->ScatterPosRmsM();
}

static bool EndCalibrationRun(CalibrationContext &ctx)
{
	const bool heldDriver = ctx.run.neutralizationSequence != 0;
	if (heldDriver)
		questcal::ReleaseCalibrationDeviceNeutralization();
	ctx.state = CalibrationState::None;
	ctx.run.Reset();
	return heldDriver;
}

static void AbortCalibration(CalibrationContext &ctx, const StopReason &reason)
{
	ctx.lastRunHint = reason.hint;
	ctx.Outcome("Calibration stopped", reason.body, reason.action, reason.detail,
		CalibrationContext::Tone::Warn);
	if (EndCalibrationRun(ctx) && vr::VRSystem())
		SynchronizeCalibrationDriver(ctx);
}

// The player's own name for the device when they gave it one, else the model
// when it could be read, else the role the pane gave it.
static std::string DeviceName(const CalibrationContext &ctx, const std::string &model,
                              const std::string &serial, bool reference)
{
	auto named = ctx.deviceNames.find(serial);
	if (!serial.empty() && named != ctx.deviceNames.end() && !named->second.empty())
		return named->second;
	if (!model.empty())
		return model;
	return reference ? "The reference device" : "The target device";
}

// Reading of a refused solve for the modal: what went wrong and what to
// change, one line each. The engine's own sentence stays in `message` for the
// log and the details toggle.
static StopReason DescribeSolveFailure(const questcal::EngineResult &result)
{
	using questcal::EngineFailure;
	using Hint = CalibrationContext::GuideHint;
	StopReason reason;
	reason.detail = result.message;
	switch (result.failure)
	{
	case EngineFailure::NotEnoughRotation:
		reason.body = "The devices didn't rotate far enough.";
		reason.action = "Use wider turns while keeping both devices fixed together.";
		reason.hint = Hint::RotateMore;
		break;
	case EngineFailure::SingleAxis:
	case EngineFailure::TranslationUnobservable:
		reason.body = "The devices only rotated in one direction.";
		reason.action = "Turn and tilt both devices together in different directions.";
		reason.hint = Hint::TwoAxes;
		break;
	case EngineFailure::RotationResidual:
		reason.body = "The devices' rotation readings didn't match closely enough.";
		reason.action = "Hold them firmly together and try again.";
		reason.hint = Hint::HoldTogether;
		break;
	case EngineFailure::PositionResidual:
		reason.body = "The devices' position readings didn't match closely enough.";
		reason.action = "Move slowly and keep both devices in clear view, then try again.";
		reason.hint = Hint::SlowDown;
		break;
	case EngineFailure::TimeOffset:
		reason.body = "Couldn't measure the tracking delay between the devices.";
		reason.action = "Keep turning both devices together until the timer finishes.";
		reason.hint = Hint::KeepTracking;
		break;
	case EngineFailure::NotEnoughSamples:
		reason.body = "Almost no tracking data arrived.";
		reason.action = "Check that both devices are tracking, then try again.";
		break;
	case EngineFailure::InvalidSamples:
		reason.body = "Some tracking data was corrupt.";
		reason.action = "Try again; restart SteamVR if it repeats.";
		break;
	case EngineFailure::ScaleNotIdentifiable:
		reason.body = "Playspace scale couldn't be determined from this motion.";
		reason.action = "Cover a larger area, or turn off Solve playspace scale in Settings.";
		break;
	case EngineFailure::Config:
		reason.body = "Internal configuration error.";
		reason.action = "Please report this.";
		break;
	case EngineFailure::NonFinite:
	case EngineFailure::OutOfRange:
	default:
		reason.body = "The solve produced an unusable result.";
		reason.action = "Try again with smooth motion around the room.";
		break;
	}
	return reason;
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
	double rotDeltaDeg = result.rotation.angularDistance(ctx.transform.rotation) * 180.0 / EIGEN_PI;
	Eigen::Vector3d baseHere = ctx.transform.rotation * targetCentroid + ctx.transform.translationMeters;
	double posDeltaM = (centroidRef - baseHere).norm();

	char buf[256];
	if (rotDeltaDeg > 8.0 || posDeltaM > 0.30)
	{
		snprintf(buf, sizeof buf,
			"Anchor rejected: %.1f deg / %.1f cm from the base calibration",
			rotDeltaDeg, posDeltaM * 100.0);
		ctx.Outcome("Anchor not added",
			"This spot is too far off from your calibration for a small correction.",
			"Run a full calibration instead.", buf, CalibrationContext::Tone::Warn);
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
		ctx.Outcome("Anchor not added", "You already have 8 anchors.",
			"Add this one within 1 m of an existing anchor to replace it, or clear the anchors in Settings.",
			"Anchor limit reached (8)", CalibrationContext::Tone::Warn);
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
		ctx.Outcome("Anchor not added", "It couldn't be saved.",
			"Restart QuestCalibrator and try again.",
			"Field anchor was not applied because the updated profile could not be saved",
			CalibrationContext::Tone::Warn);
		return;
	}
	SynchronizeCalibrationDriver(ctx);

	snprintf(buf, sizeof buf, "Field anchor %zu stored at (%.2f, %.2f): %.2f deg / %.1f cm from base\n",
		slot + 1, centroidRef.x(), centroidRef.z(), rotDeltaDeg, posDeltaM * 100.0);
	ctx.Log(buf);
	ctx.Outcome("Anchor added",
		"This spot now has its own correction, blended in as you walk around.",
		"", "", CalibrationContext::Tone::Good);
}

static void FinishCalibration(CalibrationContext &ctx)
{
	auto &run = ctx.run;
	bool asAnchor = run.anchor && ctx.validProfile;

	questcal::EngineConfig config;
	// Scale stays global from the base solve; an anchor solve fits R,T on
	// target samples pre-scaled by it, so its absolute transform composes with
	// the driver's scale-then-transform application unchanged.
	config.solveScale = ctx.solveScale && !asAnchor;
	if (asAnchor && ctx.transform.scale != 1.0)
	{
		for (auto &s : run.targetSamples)
		{
			s.pos *= ctx.transform.scale;
			s.vel *= ctx.transform.scale;
		}
	}

	char buf[512];
	snprintf(buf, sizeof buf, "Collected %zu reference / %zu target samples, solving%s...\n",
		run.referenceSamples.size(), run.targetSamples.size(), asAnchor ? " (field anchor)" : "");
	ctx.Log(buf);

	questcal::EngineResult result = questcal::CalibrationEngine::Solve(
		run.referenceSamples, run.targetSamples, config);

	Eigen::Vector3d targetCentroid = Eigen::Vector3d::Zero();
	if (!run.targetSamples.empty())
	{
		for (const auto &s : run.targetSamples)
			targetCentroid += s.pos;
		targetCentroid /= static_cast<double>(run.targetSamples.size());
	}

	// Mount extrinsic for continuous calibration: a head-referenced base
	// solve's buffers directly measure the tracker's pose in the HMD frame.
	// Must run before the buffers are cleared. The rigidity gate keeps a
	// hand-held calibration from arming continuous mode; on failure any
	// previously learned mount (which did not move just because this solve
	// happened without the tracker) is kept.
	// What the headset-tracker half of the run achieved. It is folded into the
	// run's single outcome below: a line emitted here would sit above the
	// headline, and the result stage only reads from the headline down.
	struct
	{
		bool attempted = false;
		bool measured = false;
		bool tooFast = false;  // the failure was motion, so the modal shows the slow-down picture
		std::string note;
		std::string action;
	} mount;
	if (result.valid && !asAnchor && run.referenceId == vr::k_unTrackedDeviceIndex_Hmd)
	{
		questcal::MountExtrinsic extrinsic;
		if (questcal::ContinuousAlignment::DeriveMountExtrinsic(
			run.referenceSamples, run.targetSamples, result, extrinsic))
		{
			mount.attempted = true;
			std::string serial;
			if (ReadTrackedDeviceString(run.targetId,
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
				mount.measured = true;
				mount.note = ctx.continuousEnabled
					? "Headset tracker measured, so continuous calibration can keep it that way."
					: "Headset tracker measured. Turn on continuous calibration in Settings to use it.";
			}
			else
			{
				mount.note = "The tracker's identity couldn't be verified, so the previous headset tracker measurement is kept.";
				mount.action = "Try again. If it repeats, restart SteamVR.";
			}
		}
		else if (ctx.continuousEnabled || run.targetSerial == ctx.continuousTrackerSerial)
		{
			// Said whenever the pick or the feature says this tracker is meant
			// to be on the headset: staying silent here is how "not set up yet"
			// became a permanent status for users who had done exactly that.
			// A strapped tracker does not move; an inconsistent measurement
			// means the motion was too fast for the two systems' latency.
			mount.attempted = true;
			mount.tooFast = true;
			mount.note = ctx.mountExtrinsic.valid
				? "The headset tracker's position couldn't be measured consistently, so the previous measurement is kept."
				: "The headset tracker's position couldn't be measured consistently.";
			mount.action = "Run the setup again and look around more slowly.";
		}
	}

	ctx.state = CalibrationState::None;
	if (!result.valid)
	{
		const StopReason reason = DescribeSolveFailure(result);
		ctx.lastRunHint = reason.hint;
		ctx.Outcome("Calibration failed", reason.body, reason.action, reason.detail,
			CalibrationContext::Tone::Warn);
		if (EndCalibrationRun(ctx) && vr::VRSystem())
			SynchronizeCalibrationDriver(ctx);
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
			guardNote = !result.scaleIdentifiable
				? " -- motion did not identify scale; held at neutral 1.0"
				: result.motionSmoothingDetected
					? " -- motion attenuation contaminated scale; held at neutral 1.0"
					: " -- frequency bands are physically inconsistent; scale held at neutral 1.0";
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
	if (config.solveScale)
	{
		snprintf(buf, sizeof buf,
			"Scale confidence: %s (condition %.4f, one-sigma %.4f)\n",
			result.scaleIdentifiable ? "identifiable" : "insufficient",
			result.scaleCondition, result.scaleStdDev);
		ctx.Log(buf);
	}

	if (asAnchor)
	{
		EndCalibrationRun(ctx);
		ctx.lastRunHint = CalibrationContext::GuideHint::Success;
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
	ctx.referenceTrackingSystem = run.referenceSystem;
	ctx.targetTrackingSystem = run.targetSystem;
	ctx.SetCalibration(result.rotation, result.translation, result.scale);
	ctx.transform.timeOffset = result.timeOffset;
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
	questcal::RebindCalibrationUniverse(
		ctx, run.hmdSerial, priorUniverseUnsafe);
	ctx.persistence.AdvanceRevision();
	ctx.persistence.MarkProfileAndSettings(ctx.timeLastTick);
	bool saved = SavePendingChanges(ctx);
	EndCalibrationRun(ctx);
	SynchronizeCalibrationDriver(ctx);

	if (result.scale != 1.0)
	{
		snprintf(buf, sizeof buf, "Playspace scale: %.4f\n", result.scale);
		ctx.Log(buf);
	}

	// One plain sentence on the quality band the rating uses; the residuals
	// behind it went to the detail lines above.
	ctx.lastRunHint = CalibrationContext::GuideHint::Success;
	const double rotDeg = result.rotationRmsDeg;
	const double posCm = result.translationRmsMeters * 100.0;
	const bool good = rotDeg <= 3.0 && posCm <= 1.5;
	const bool rough = rotDeg > 6.0 || posCm > 3.0;
	const std::string quality = good ? "Check that the tracker positions line up in VR."
		: rough ? "The alignment is rough." : "The alignment may need another pass.";
	const std::string action = good ? ""
		: rough ? "Try again with slower motion, turning and tilting in different directions."
		: "Check the tracker positions in VR. Try again if they look off.";
	const CalibrationContext::Tone tone = rough ? CalibrationContext::Tone::Warn
		: CalibrationContext::Tone::Good;
	if (mount.attempted)
	{
		// A headset-tracker run is judged on what it was for. Its result is
		// folded into the one outcome the modal shows, instead of a line
		// that would have landed above the headline and gone unread.
		if (mount.measured)
			ctx.Outcome("Calibration complete", quality + " " + mount.note, action, "", tone);
		else
		{
			if (mount.tooFast)
				ctx.lastRunHint = CalibrationContext::GuideHint::SlowDown;
			ctx.Outcome("Done, but the headset tracker wasn't measured",
				quality + " " + mount.note, mount.action, "", CalibrationContext::Tone::Warn);
		}
	}
	else
		ctx.Outcome("Calibration complete", quality, action, "", tone);
	if (!saved)
		ctx.Tell("Applied for this session, but it couldn't be saved; redo it after restarting.",
			CalibrationContext::Tone::Warn);
}

bool StartCalibration()
{
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
	if (CalCtx.pendingReferenceTrackingSystem != CalCtx.referenceTrackingSystem ||
		CalCtx.pendingTargetTrackingSystem != CalCtx.targetTrackingSystem)
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

	RuntimeMonitorTick(ctx, time);
	questcal::CalibrationSpaceTick(ctx, time);
	// After the monitors: an accepted jump must land (and reset the continuous
	// window) before the continuous loop reads the calibration this tick.
	ContinuousTick(ctx, time);

	// Runtime poses are only the compatibility collection source. The normal
	// raw-ring path and all idle monitors already have timestamped samples, so
	// querying all 64 slots at 50 Hz outside this narrow window was pure work.
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

		// Names first: every stop reason below names the hardware the player
		// picked (by their own name for it when they gave one), not "the
		// reference device". A failed read falls back to the pane the pick
		// came from; the serials are re-read strictly below, where a failure
		// is its own stop reason.
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
				"Missing reference device.",
				"Pick a device that's switched on.",
				"Missing reference device",
				CalibrationContext::GuideHint::WrongPick });
			return;
		}
		if (run.targetId >= vr::k_unMaxTrackedDeviceCount)
		{
			AbortCalibration(ctx, {
				"Missing target device.",
				"Pick a device that's switched on.",
				"Missing target device",
				CalibrationContext::GuideHint::WrongPick });
			return;
		}
		if (!trackingOk(run.referenceId))
		{
			AbortCalibration(ctx, {
				DeviceName(ctx, run.referenceModel, run.referenceSerial, true) + " isn't tracking.",
				"Check it's awake and visible to its base stations, then try again.",
				"Reference device is not Running_OK",
				CalibrationContext::GuideHint::TrackingLost });
			return;
		}
		if (!trackingOk(run.targetId))
		{
			AbortCalibration(ctx, {
				DeviceName(ctx, run.targetModel, run.targetSerial, false) + " isn't tracking.",
				"Check it's awake and visible to its base stations, then try again.",
				"Target device is not Running_OK",
				CalibrationContext::GuideHint::TrackingLost });
			return;
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
				"Could not verify that the current HMD owns the selected reference tracking system",
				CalibrationContext::GuideHint::WrongPick });
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
		// pair. Drain once and require recent trusted traffic from both devices.
		PoseHub.Drain(CollectorConsumer, CollectorScratch);
		LARGE_INTEGER qpcNow{};
		bool hasClock = QueryPerformanceCounter(&qpcNow) != FALSE;
		run.usesPoseRing = hasClock && PoseHub.RingOpen() && PreflightPoseRing(
			run, CollectorScratch,
			static_cast<double>(qpcNow.QuadPart) * QpcToSeconds);

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
				: "Reference device stopped reporting Running_OK runtime poses",
			CalibrationContext::GuideHint::TrackingLost });
		return;
	}
	if (time - run.lastTargetSample > 2.0)
	{
		AbortCalibration(ctx, {
			DeviceName(ctx, run.targetModel, run.targetSerial, false) + " stopped tracking mid-run.",
			"Keep it in view of its base stations for the whole countdown.",
			run.usesPoseRing ? "No trusted Running_OK raw poses arrived for the target device"
				: "Target device stopped reporting Running_OK runtime poses",
			CalibrationContext::GuideHint::TrackingLost });
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
	ctx.lastRunHint = CalibrationContext::GuideHint::None;
	ctx.Outcome("Calibration cancelled", "", "", "");
	if (EndCalibrationRun(ctx) && vr::VRSystem())
		SynchronizeCalibrationDriver(ctx);
}
