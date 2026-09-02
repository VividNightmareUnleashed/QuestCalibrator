#include "stdafx.h"
#include "Calibration.h"
#include "CalibrationDriver.h"
#include "CalibrationEngine.h"
#include "CalibrationSpace.h"
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

CalibrationContext CalCtx;

static void AbortCalibration(CalibrationContext &ctx, const std::string &reason);
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
	std::streampos written = SessionLog.tellp();
	SessionLogBytes = written > 0 ? static_cast<size_t>(written) : 0;
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
		AbortCalibration(ctx,
			"Pose stream overran during collection; retry calibration so no samples are missing");
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
				AbortCalibration(ctx,
					"A selected tracking universe changed during collection; restart calibration after tracking stabilizes");
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

static bool AnyControllerTriggerPressed()
{
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		if (vr::VRSystem()->GetTrackedDeviceClass(id) !=
			vr::TrackedDeviceClass_Controller)
			continue;
		vr::VRControllerState_t state{};
		if (!vr::VRSystem()->GetControllerState(id, &state, sizeof state))
			continue;
		if (state.rAxis[vr::k_eControllerAxis_Trigger].x > 0.75f)
			return true;
	}
	return false;
}

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
		ctx.continuousCorrectionGate.Clear();
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
				ctx.Log("Continuous correction ready -- squeeze a controller trigger to apply it\n");
		}
		else
		{
			if (ctx.continuousRequireTrigger)
				ctx.Log("Controller confirmation received -- applying the continuous correction\n");
			if (!questcal::ApplyCalibrationDelta(ctx, corr.rotation, corr.translation,
				/*snap=*/false, now))
			{
				ctx.ReportError("A continuous-calibration correction exceeded the safe transform bounds and was ignored\n");
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
			}
		}
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

static void AbortCalibration(CalibrationContext &ctx, const std::string &reason)
{
	AppendSessionLog("Calibration aborted: " + reason + "\n");
	ctx.Log("Calibration aborted: " + reason + "\n");
	if (EndCalibrationRun(ctx) && vr::VRSystem())
		SynchronizeCalibrationDriver(ctx);
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
	SynchronizeCalibrationDriver(ctx);

	snprintf(buf, sizeof buf, "Field anchor %zu stored at (%.2f, %.2f): %.2f deg / %.1f cm from base\n",
		slot + 1, centroidRef.x(), centroidRef.z(), rotDeltaDeg, posDeltaM * 100.0);
	ctx.Log(buf);
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
	if (result.valid && !asAnchor && run.referenceId == vr::k_unTrackedDeviceIndex_Hmd)
	{
		questcal::MountExtrinsic extrinsic;
		if (questcal::ContinuousAlignment::DeriveMountExtrinsic(
			run.referenceSamples, run.targetSamples, result, extrinsic))
		{
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

	ctx.state = CalibrationState::None;
	if (!result.valid)
	{
		AppendSessionLog("Calibration failed: " + result.message + "\n");
		ctx.Log("Calibration failed: " + result.message + "\n");
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
		CalCtx.ReportError("Select the active profile's tracking systems before collecting a field anchor\n");
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
	ctx.Log("Keep the selected devices rigidly together.\n"
		"Move them through wide, varied rotations around at least two different axes and across the play area.\n");
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
		ctx.wantedUpdateInterval = 1.0;

		if ((time - ctx.timeLastScan) >= 1.0)
		{
			SynchronizeCalibrationDriver(ctx);
			questcal::CheckProtectedChaperone(ctx);
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
			AbortCalibration(ctx,
				"Raw pose traffic is unavailable and the selected device transforms could not be neutralized");
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
		bool ok = true;

		if (run.referenceId >= vr::k_unMaxTrackedDeviceCount)
		{
			ctx.Log("Missing reference device\n");
			ok = false;
		}
		else if (!ctx.devicePoses[run.referenceId].bPoseIsValid ||
			ctx.devicePoses[run.referenceId].eTrackingResult != vr::TrackingResult_Running_OK)
		{
			ctx.Log("Reference device is not tracking\n");
			ok = false;
		}

		if (run.targetId >= vr::k_unMaxTrackedDeviceCount)
		{
			ctx.Log("Missing target device\n");
			ok = false;
		}
		else if (!ctx.devicePoses[run.targetId].bPoseIsValid ||
			ctx.devicePoses[run.targetId].eTrackingResult != vr::TrackingResult_Running_OK)
		{
			ctx.Log("Target device is not tracking\n");
			ok = false;
		}

		if (!ok)
		{
			AbortCalibration(ctx,
				"Both selected devices must report Running_OK before collection starts");
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
			AbortCalibration(ctx,
				"Selected devices no longer belong to the chosen tracking systems");
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
			AbortCalibration(ctx,
				"Could not verify both selected device serials; reconnect them and try again");
			return;
		}
		run.lastIdentityCheck = time;
		std::string hmdSystem;
		if (!ReadCurrentHmdIdentity(hmdSystem, run.hmdSerial) ||
			hmdSystem != run.referenceSystem)
		{
			AbortCalibration(ctx,
				"Could not verify that the current HMD owns the selected reference tracking system");
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
		AbortCalibration(ctx,
			"The driver pose channel closed mid-collection; the remaining samples would carry a different clock");
		return;
	}

	// The frozen pair is an OpenVR index. Re-read the two serials at 1 Hz --
	// never per sample, these are expensive property reads -- so a device that
	// power-cycled into another device's slot cannot have its poses concatenated
	// into one buffer and fitted as a single rigid body. A serial that cannot be
	// read proves nothing; the no-sample timeouts below stay the liveness check.
	if (time - run.lastIdentityCheck >= 1.0)
	{
		run.lastIdentityCheck = time;
		std::string serial;
		auto deviceReplaced = [&](uint32_t id, const std::string &frozen)
		{
			return !ReadTrackedDeviceString(
				id, vr::Prop_SerialNumber_String, serial) || serial != frozen;
		};
		if (deviceReplaced(run.referenceId, run.referenceSerial) ||
			deviceReplaced(run.targetId, run.targetSerial))
		{
			AbortCalibration(ctx,
				"A selected device was replaced in its slot mid-collection");
			return;
		}
	}

	if (run.usesPoseRing)
	{
		if (!CollectFromPoseRing(ctx, time))
			return;
	}
	else
		CollectFromRuntimePoses(ctx, time);

	if (time - run.lastReferenceSample > 2.0)
	{
		AbortCalibration(ctx,
			run.usesPoseRing ? "No trusted Running_OK raw poses arrived for the reference device"
				: "Reference device stopped reporting Running_OK runtime poses");
		return;
	}
	if (time - run.lastTargetSample > 2.0)
	{
		AbortCalibration(ctx,
			run.usesPoseRing ? "No trusted Running_OK raw poses arrived for the target device"
				: "Target device stopped reporting Running_OK runtime poses");
		return;
	}

	double duration = ctx.CollectionSeconds();
	double elapsed = time - run.collectionStart;
	ctx.Progress(static_cast<int>(elapsed * 100.0), static_cast<int>(duration * 100.0));

	if (elapsed >= duration)
		FinishCalibration(ctx);
}
