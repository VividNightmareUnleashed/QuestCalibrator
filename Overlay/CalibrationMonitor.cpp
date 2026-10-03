#include "stdafx.h"
#include "CalibrationInternal.h"
#include "CalibrationDriver.h"
#include "CalibrationSpace.h"
#include "LighthouseLog.h"
#include "PoseMath.h"
#include "QualityBands.h"
#include "RingPoseMath.h"
#include "TrackingStreamDigest.h"
#include "../common/MathConstants.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <memory>
#include <string>
#include <vector>

// The monitors that run beside the calibration every tick: drift staleness
// (detected and notified, never corrected), base station visibility from
// SteamVR's lighthouse log, and the lighthouse frame moves the target
// trackers' corrections follow.

using namespace calibration_internal;
using questcal::ReadTrackedDeviceString;
using questcal::SynchronizeCalibrationDriver;

// Base station visibility from SteamVR's lighthouse log (LighthouseLog.h):
// followed at 4 Hz, folded into ctx.lighthouse, joined to the drift
// monitor's device ids through this serial table.
static std::unique_ptr<lighthouselog::Tailer> LighthouseTail;
static std::string DeviceSerials[vr::k_unMaxTrackedDeviceCount];
// Read in the same scan; the drift feed keeps base stations out by it.
static vr::ETrackedDeviceClass DeviceClasses[vr::k_unMaxTrackedDeviceCount] = {};
static double LastLighthousePoll = -1e9;
static double LastSerialScan = -1e9;
static uint64_t LighthouseRotationsSeen = 0;
static bool LighthouseAnnounced = false;
static VisibilityDigest LighthouseDigest;
static TrackingStreamDigest StreamDigest;
static std::vector<protocol::DevicePoseSample> MonitorScratch;
static bool MonitorActive = false;
static std::string FrameObservedSerials[vr::k_unMaxTrackedDeviceCount];
static uint64_t FrameEpoch = 0;
static uint64_t FrameStreamBoundary = 0;

// A new driver session (the ring's writer restarted or its epoch moved): its
// tracker frame corrections start over, and the driver is asked for any it
// kept for the profile.
static void StartFrameSession(CalibrationContext &ctx, uint64_t boundary)
{
	FrameStreamBoundary = boundary;
	ctx.frameDriverSession = 0;
	ctx.frameRecoveryPending = true;
	ctx.ResetTrackerFrames();
	questcal::SynchronizeCalibrationDriver(ctx);
}

// ---------------------------------------------------------------------------
// Runtime monitoring: drift staleness (detect + notify only, never corrects)

// Supplied by the app shell (see Calibration.h); absent under -uipreview.
std::function<void(const char *)> calibration_internal::ToastSink;

void SetToastSink(std::function<void(const char *)> sink)
{
	ToastSink = std::move(sink);
}

// One-shot player-facing line + optional VR toast; each caller owns its
// re-arm flag. The line goes through Tell so it reaches the main screen's
// activity feed as well as the log.
void calibration_internal::NotifyOnce(CalibrationContext &ctx, bool &notified, const char *line,
	CalibrationContext::Tone tone, const char *toast, bool showToast)
{
	if (notified)
		return;
	notified = true;

	// Always logged, even with toasts off or no sink.
	ctx.Tell(std::string(line) + "\n", tone);

	if (showToast && ToastSink)
		ToastSink(toast);
}

static void NotifyStaleAlignment(CalibrationContext &ctx)
{
	NotifyOnce(ctx, Monitors.staleNotified,
		"Your calibration looks off. Recalibrate.",
		CalibrationContext::Tone::Warn,
		"QuestCalibrator: your calibration looks off. Recalibrate.",
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
		ctx.driftScore >= questcal::DriftStaleScore ? CalibrationContext::AlignmentHealth::Stale :
		ctx.driftScore >= questcal::DriftAgingScore ? CalibrationContext::AlignmentHealth::Aging :
		CalibrationContext::AlignmentHealth::Fresh;

	// A healthy continuous loop re-measures the alignment constantly; the
	// rating already skips staleness for it, and toasting "quality looks poor"
	// while it is visibly being maintained is pure noise.
	bool maintained = ctx.ContinuousArmed() &&
		ctx.continuousState == questcal::ContinuousAlignment::State::Tracking;
	if (ctx.alignment == CalibrationContext::AlignmentHealth::Stale && !maintained)
		NotifyStaleAlignment(ctx);
}

// Follows the lighthouse driver's log and keeps the serial table the drift
// monitor needs to join its events onto it. The log is a convenience: when
// it is missing or unreadable everything downstream behaves as it did
// without it.
void calibration_internal::LighthouseTick(CalibrationContext &ctx, double time)
{
	if (!LighthouseTail)
	{
		LighthouseTail = std::make_unique<lighthouselog::Tailer>(lighthouselog::DefaultLogPath());
		ctx.lighthouseLogPath = LighthouseTail->Path();
	}

	// Serials are what the log names devices by. Property reads are
	// cross-process calls, so a slow scan; the set of devices changes rarely.
	if (time - LastSerialScan >= 2.0)
	{
		LastSerialScan = time;
		auto *system = vr::VRSystem();
		for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
		{
			DeviceClasses[id] = system->GetTrackedDeviceClass(id);
			if (DeviceClasses[id] == vr::TrackedDeviceClass_Invalid)
			{
				DeviceSerials[id].clear();
				continue;
			}
			std::string serial;
			if (ReadTrackedDeviceString(id, vr::Prop_SerialNumber_String, serial))
			{
				DeviceSerials[id] = serial;
				if (ctx.trackerFrames.Bind(id, serial))
				{
					FrameWatch.ForgetDevice(id);
					FrameObservedThrough[id] = 0;
					SynchronizeCalibrationDriver(ctx);
				}
			}
		}
	}

	if (time - LastLighthousePoll < 0.25)
		return;
	LastLighthousePoll = time;

	static std::vector<lighthouselog::Event> events;
	events.clear();
	const bool wasAvailable = ctx.lighthouseLogAvailable;
	LighthouseTail->Poll(events);
	ctx.lighthouseLogAvailable = LighthouseTail->Available();
	if (!LighthouseAnnounced || ctx.lighthouseLogAvailable != wasAvailable)
	{
		LighthouseAnnounced = true;
		ctx.Log(ctx.lighthouseLogAvailable
			? "Reading base station visibility from " + ctx.lighthouseLogPath + "\n"
			: "SteamVR's log is not readable at " + ctx.lighthouseLogPath +
				"; base station visibility is unavailable\n");
	}
	// A rotated log is a new SteamVR session: the old sets and counts are
	// about devices that were since restarted.
	if (LighthouseTail->Rotations() != LighthouseRotationsSeen)
	{
		LighthouseRotationsSeen = LighthouseTail->Rotations();
		ctx.lighthouse.Reset();
		LighthouseDigest.Reset();
	}
	const std::string digest = LighthouseDigest.Flush(time);
	if (!digest.empty())
		ctx.Diag(digest);
	if (events.empty())
		return;

	// The lines carry wall-clock stamps; the monitors run on the ring clock.
	const double ringNow = QpcNowSeconds();
	const double unixNow = std::chrono::duration<double>(
		std::chrono::system_clock::now().time_since_epoch()).count();
	for (const auto &e : events)
	{
		double ringTime = ringNow;
		if (e.timeKnown)
		{
			const double age = unixNow - e.unixTime;
			if (age > -5.0 && age < 3600.0)
				ringTime = ringNow - age;
		}
		std::string note = ctx.lighthouse.Apply(e, ringTime);
		if (e.kind == lighthouselog::Event::Kind::ServerStarted)
			LighthouseDigest.Reset();
		if (!e.historical && (e.kind == lighthouselog::Event::Kind::UniverseChosen ||
			e.kind == lighthouselog::Event::Kind::UniverseStopped))
		{
			const char *what = e.kind == lighthouselog::Event::Kind::UniverseStopped ? "stopped tracking with"
				: e.universeCreated ? "created" : "chose";
			ctx.Log("SteamVR " + std::string(what) + " lighthouse universe " + std::to_string(e.universeId) + "\n");
		}
		if (!note.empty())
			ctx.Log(e.serial + " " + note + "\n");
		else if (!e.historical)
		{
			// The device's count, not the line's: a SECONDARY line names one
			// station and the set it joined is the model's.
			const LighthouseVisibility::Device *seen = ctx.lighthouse.Find(e.serial);
			if (seen && seen->visibleKnown)
				LighthouseDigest.Note(e.serial, seen->InView());
		}
	}
}

// A frame move's yaw and tilt in degrees, and whether it is worth a session
// log line by the frame watch's own bounds (smaller is a station refined in
// place).
static bool DescribeFrameMove(const LighthouseFrameWatch::Move &move, double &yawDeg, double &tiltDeg)
{
	double tilt = 0.0;
	yawDeg = std::abs(questcal::SignedYawRadians(move.rotation, &tilt)) * 180.0 / questcal::Pi;
	tiltDeg = tilt * 180.0 / questcal::Pi;
	return LighthouseFrameWatch::NotableMove(move.shiftM, yawDeg, tiltDeg);
}

// While SteamVR sets up its universe (LighthouseVisibility.h) a frame move
// places a station it guessed at startup: the calibration belongs to the
// universe, not to the guess, and is left where it is (live 2026-09-27: the
// census followed one such move 30.5 deg and 63 cm off). With no word from
// the log, the first seconds after the lighthouse devices start tracking
// stand in for it: the universe was chosen 1.1 and 4.6 s after the first
// new solution on 2026-09-26 and -27, and its stations placed within 5 s.
static constexpr double UniverseSetupFallbackSeconds = 15.0;

static bool DuringUniverseSetup(const CalibrationContext &ctx, double moveTime)
{
	switch (ctx.lighthouse.Universe(moveTime))
	{
	case LighthouseVisibility::UniverseSetup::SettingUp:
		return true;
	case LighthouseVisibility::UniverseSetup::Settled:
		return false;
	case LighthouseVisibility::UniverseSetup::Unknown:
		break;
	}
	return moveTime - FrameWatch.TrackingSince() < UniverseSetupFallbackSeconds;
}

// The shared calibration stays in one target space. A proven frame move is
// cancelled only for the device that reported it: N_i' = N_i o F_i^-1.
// The watch also handles hidden trackers and returns after tracking loss.
static void CompensateTrackerFrameMoves(CalibrationContext &ctx)
{
	const bool overflow = FrameWatch.TakeMoveOverflow();
	auto moves = FrameWatch.TakeMoves();
	if (overflow)
	{
		EndCalibrationRun(ctx);
		ctx.enabled = false;
		ctx.profileUniverseUnsafe = true;
		ctx.frameMovesLost = true;
		ctx.persistence.MarkProfile(ctx.timeLastTick);
		ResetContinuousObservations(ctx, questcal::ContinuousAlignment::ResetReason::StreamGap);
		ctx.Log("Lighthouse frame move queue overflowed; calibration disabled until a new measurement\n");
		ctx.ReportError("SteamVR moved the base stations more often than QuestCalibrator could follow. "
			"The calibration is off until you recalibrate.\n");
		SynchronizeCalibrationDriver(ctx);
		return;
	}
	std::sort(moves.begin(), moves.end(), [](const auto &a, const auto &b)
	{ return a.time < b.time; });
	bool changed = false;
	for (const auto &move : moves)
	{
		if (!ctx.targetDeviceMask[move.id])
			continue;
		double yawDeg = 0.0, tiltDeg = 0.0;
		const bool notable = DescribeFrameMove(move, yawDeg, tiltDeg);
		const bool setup = DuringUniverseSetup(ctx, move.time);
		const bool mounted = move.id == ctx.continuousTrackerId;
		if (setup)
		{
			if (mounted)
				ResetContinuousObservations(ctx, questcal::ContinuousAlignment::ResetReason::TargetFrameMoved);
			++ctx.frameMovesInSetup;
		}
		else if (!ctx.trackerFrames.Follow(move))
		{
			// Logged always, not only in detailed logging: the tracker stays
			// off by the move it reported, and the log has to say why.
			ctx.Log("Lighthouse: rejected an out-of-order or unsafe frame correction for device " +
				std::to_string(move.id) + "\n");
			continue;
		}
		else
		{
			changed = true;
			++ctx.frameMovesFollowed;
			if (mounted)
			{
				// Observations and previous estimates already live in the
				// normalized space. Clear the window without rebasing them.
				ResetContinuousObservations(ctx, questcal::ContinuousAlignment::ResetReason::TargetFrameMoved);
				++ctx.trackerFrameCompensations;
			}
		}
		const auto &frame = ctx.trackerFrames.Snapshot()[move.id];
		char line[1024];
		snprintf(line, sizeof line,
			"lighthouse device %u %s frame moved%s%s: yaw %.2f deg, tilt %.2f deg, %.1f cm; %s; "
			"from q(%.6f,%.6f,%.6f,%.6f) t(%.4f,%.4f,%.4f) "
			"to q(%.6f,%.6f,%.6f,%.6f) t(%.4f,%.4f,%.4f); "
			"normalization q(%.6f,%.6f,%.6f,%.6f) t(%.4f,%.4f,%.4f)",
			move.id, DeviceSerials[move.id].c_str(), move.returned ? " while off" : "",
			move.ownJump ? " with a local pose jump" : "", yawDeg, tiltDeg, move.shiftM * 100.0,
			setup ? "left alone during universe setup" : "compensated for this device",
			move.fromRot.w(), move.fromRot.x(), move.fromRot.y(), move.fromRot.z(),
			move.fromTrans.x(), move.fromTrans.y(), move.fromTrans.z(),
			move.toRot.w(), move.toRot.x(), move.toRot.y(), move.toRot.z(),
			move.toTrans.x(), move.toTrans.y(), move.toTrans.z(),
			frame.rotation.w, frame.rotation.x, frame.rotation.y, frame.rotation.z,
			frame.translation.v[0], frame.translation.v[1], frame.translation.v[2]);
		if (setup) ctx.lastFrameMoveInSetup = line;
		else
		{
			ctx.lastFrameMoveFollowed = line;
			if (mounted) ctx.lastTrackerFrameCompensation = line;
		}
		if (notable && ctx.detailedLogging && !FrameCapturePending)
		{
			FrameCapturePending = true;
			FrameCaptureDue = (std::max)(ctx.timeLastTick + 0.15, LastFrameCapture + 2.0);
		}
		if (notable) ctx.Log(std::string(line) + "\n");
		else ctx.Diag(line);
	}
	if (changed)
	{
		DispatchStreamEvent(ctx, questcal::StreamEvent::FrameMoved);
		SynchronizeCalibrationDriver(ctx);
	}
}

void calibration_internal::RuntimeMonitorTick(CalibrationContext &ctx, double now)
{
	const uint64_t boundary = PoseHub.StreamBoundaries();
	if (boundary != FrameStreamBoundary)
		StartFrameSession(ctx, boundary);
	// Frame corrections also start over from a recalibration or a profile
	// load, and come back from the driver's recovery: the epoch is how those
	// reach the windows kept here.
	if (FrameEpoch != ctx.trackerFrameEpoch)
	{
		FrameEpoch = ctx.trackerFrameEpoch;
		DispatchStreamEvent(ctx, questcal::StreamEvent::TrackerFramesReset);
	}
	// Score even while the monitors are parked so the UI's health readout
	// tracks calibration age from the moment a profile loads. Age alone
	// saturates below the stale threshold, so this can't notify by itself.
	if (ctx.validProfile)
		UpdateDriftScore(ctx);

	const bool idle = ctx.state == CalibrationState::None;
	// Frame normalization belongs to the driver session, including a manual
	// measurement. Drift and universe acceptance still run only while idle.
	bool shouldRun = !ctx.frameRecoveryPending && ctx.enabled && ctx.validProfile && PoseHub.RingOpen();

	if (!shouldRun)
	{
		if (MonitorActive)
		{
			// The frame watch keeps each device's last frame across the pause:
			// a station SteamVR re-solved meanwhile is still found from the
			// samples on either side of it, and corrections are kept for the
			// profile when it comes back. Forgetting them would leave a tracker
			// off by that move until the next fresh calibration.
			DispatchStreamEvent(ctx, questcal::StreamEvent::MonitorsParked);
			MonitorActive = false;
		}
		PoseHub.DiscardBacklog(MonitorConsumer);
		return;
	}

	if (!MonitorActive)
	{
		PoseHub.DiscardBacklog(MonitorConsumer);
		DispatchStreamEvent(ctx, questcal::StreamEvent::MonitorsResumed);
		MonitorActive = true;
	}

	// Judge the whole hole in front of the batch, from the drain itself: a
	// StreamBoundaries read before or after the drain can land on the wrong
	// side of a session boundary, and a hole split across two drains would
	// pass the leash share by share.
	PoseStreamHub::Hole hole;
	uint64_t dropped = PoseHub.Drain(MonitorConsumer, MonitorScratch, &hole);
	if (hole.sessionBoundary && PoseHub.StreamBoundaries() != FrameStreamBoundary)
	{
		StartFrameSession(ctx, PoseHub.StreamBoundaries());
		// The session boundary below drops the frame watch and its progress;
		// the continuous loop hears the boundary from its own drain.
		FrameEpoch = ctx.trackerFrameEpoch;
	}
	if (!ringpose::MonitorGapTolerable(hole.size, hole.sessionBoundary))
	{
		// We lost part of our own observation window; baselines across the
		// hole are unsafe. For the drift monitor a drain hole would read as a
		// tracking loss and could fake a discontinuity event.
		DispatchStreamEvent(ctx, hole.sessionBoundary
			? questcal::StreamEvent::SessionBoundary : questcal::StreamEvent::MonitorHole);
	}
	else if (dropped > 0)
	{
		// The driver's isolated contended-publish drops (one or two poses,
		// every few seconds to minutes). The drift monitor and the watermarks
		// work on sample times and lose nothing to a hole this short; the
		// jump detector must not fit a step across it, and keeps the rest.
		DispatchStreamEvent(ctx, questcal::StreamEvent::MonitorDrop);
	}
	if (!ctx.detailedLogging)
		StreamDigest.Reset();
	else if (dropped > 0)
		StreamDigest.NoteDrops(dropped);

	for (const auto &s : MonitorScratch)
	{
		if (s.deviceId >= vr::k_unMaxTrackedDeviceCount)
			continue;
		FrameObservedThrough[s.deviceId] = (std::max)(FrameObservedThrough[s.deviceId], s.sampleTimeQpc);
		// JumpDetector owns the observation-continuity policy and must see bad
		// frames as well as good ones. Drift/HMD caches below remain valid-only.
		if (ctx.referenceDeviceMask[s.deviceId])
		{
			if (idle) questcal::ObserveUniversePose(s);
			if (ctx.detailedLogging)
				StreamDigest.Note(s, QpcToSeconds);
		}
		else
		{
			const auto &serial = ctx.trackerFrames.Serial(s.deviceId);
			if (FrameObservedSerials[s.deviceId] != serial)
			{
				FrameWatch.ForgetDevice(s.deviceId);
				FrameObservedSerials[s.deviceId] = serial;
			}
			FrameWatch.Note(s, QpcToSeconds,
				DeviceClasses[s.deviceId] == vr::TrackedDeviceClass_TrackingReference);
		}
	}
	const bool jumped = idle && questcal::FinishUniverseObservations(ctx, now);
	CompensateTrackerFrameMoves(ctx);

	if (!idle)
	{
		DispatchStreamEvent(ctx, questcal::StreamEvent::Calibrating);
		return;
	}

	for (const auto &s : MonitorScratch)
	{
		if (s.deviceId >= vr::k_unMaxTrackedDeviceCount)
			continue;
		questcal::PoseSample sample;
		if (!TryComposeRingSample(s, QpcToSeconds, sample))
			continue;
		if (ctx.targetDeviceMask[s.deviceId] && !ctx.trackerFrames.Normalize(s.deviceId, sample))
			continue;
		// Per-device watermark across drains.
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
		candidate.deviceClass = DeviceClasses[s.deviceId];
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

	if (ctx.detailedLogging)
		for (const auto &line : StreamDigest.Flush(now))
			ctx.Diag(line);

	// Lighthouse frame moves are rare and are the evidence a "the whole
	// calibration jumped" report needs, so one is always logged; a pose merely
	// re-expressed in another station's frame, or refined in place, only in
	// detail.
	for (const auto &report : FrameWatch.Flush())
	{
		const std::string line = LighthouseFrameWatch::Describe(report, ctx.continuousTrackerId);
		if (LighthouseFrameWatch::Notable(report))
		{
			ctx.lighthouseFrameMoves++;
			ctx.lastLighthouseFrameMove = line;
			ctx.Log(line + "\n");
		}
		else
		{
			ctx.Diag(line);
		}
	}

	// Jump acceptance lands before a slide window can conclude (~0.25 s vs
	// ~2.5 s), so dropping the drift window also discards any event queued
	// from the same discontinuity.
	if (jumped)
		DispatchStreamEvent(ctx, questcal::StreamEvent::UniverseJump);

	DriftMonitor::Event drift;
	while (Drift->PollEvent(drift))
	{
		char buf[256];
		// A lighthouse device that just lost or regained a base station is
		// reporting its own tracking, not the universes (LighthouseVisibility.h).
		const std::string &serial = DeviceSerials[drift.deviceId];
		if (!serial.empty() && ctx.lighthouse.Disturbed(serial, drift.time))
		{
			ctx.lighthouseAttributedEvents++;
			const LighthouseVisibility::Device *seen = ctx.lighthouse.Find(serial);
			snprintf(buf, sizeof buf,
				"Device %u %s %.1f cm after it %s -- its base stations, not drift\n",
				drift.deviceId,
				drift.type == DriftMonitor::Event::StationarySlide ? "slid" : "recovered",
				drift.magnitude * 100.0,
				seen && !seen->lastDisturbanceText.empty()
					? seen->lastDisturbanceText.c_str() : "changed base stations");
			ctx.Log(buf);
			continue;
		}
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
