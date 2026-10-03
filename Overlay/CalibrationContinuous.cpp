#include "stdafx.h"
#include "CalibrationInternal.h"
#include "CalibrationDriver.h"
#include "CalibrationEngine.h"
#include "CalibrationSpace.h"
#include "ContinuousCorrectionGate.h"
#include "FieldMath.h"
#include "PoseMath.h"
#include "RingPoseMath.h"
#include "../common/MathConstants.h"

#include <cmath>
#include <cstdio>
#include <ctime>
#include <string>
#include <vector>

// Continuous calibration: the loop that keeps the calibration aligned from a
// tracker strapped to the headset, and that tracker's radio link.

using namespace calibration_internal;
using questcal::SynchronizeCalibrationDriver;

static std::vector<protocol::DevicePoseSample> ContinuousScratch;
static bool ContinuousActive = false;
// The headset tracker's live lighthouse disturbance count last acted on, so
// each new one restarts the window exactly once, and its bootstrap count, so
// the loop hears which of them began a solution of its own.
static std::string ContinuousLighthouseSerial;
static uint32_t ContinuousLighthouseDisturbances = 0;
static uint32_t ContinuousLighthouseBootstraps = 0;
// The stream the last TrackerLost named, for the TrackerRecovered after it.
static ringpose::StoppedStream ContinuousLostStream = ringpose::StoppedStream::Tracker;

// Returns whether the drain crossed a hole the loops' windows must not span:
// a stall-sized one or a driver session boundary. The driver's isolated
// contended-publish drops are ridden through: the loop pairs poses by sample
// time, and clearing the window for each drop starved it (simulated, a pose
// lost every 18 s kept a 25 s window from ever filling).
static bool DrainContinuousInput(CalibrationContext &ctx)
{
	// The whole hole in front of the batch (see RuntimeMonitorTick).
	PoseStreamHub::Hole hole;
	const uint64_t dropped = PoseHub.Drain(ContinuousConsumer, ContinuousScratch, &hole);
	auto &diagnostics = ctx.continuousDiagnostics;
	++diagnostics.batches;
	diagnostics.samples += ContinuousScratch.size();
	if (dropped > 0)
	{
		++diagnostics.gapEvents;
		diagnostics.reportedLoss += dropped;
	}
	return !ringpose::MonitorGapTolerable(hole.size, hole.sessionBoundary);
}

// A freeze right after the headset tracker's lighthouse solution restarted.
// Its next restart usually puts it back, and a recalibration now would measure
// the misplaced solution into every lighthouse tracker, so the advice is to
// wait rather than recalibrate.
static void NotifyResolveFreeze(CalibrationContext &ctx)
{
	NotifyOnce(ctx, Monitors.freezeNotified,
		"Continuous calibration paused after the headset tracker restarted. It resumes if tracking recovers; another tracker restart may be needed.",
		CalibrationContext::Tone::Warn,
		"QuestCalibrator: tracking has not recovered after the headset tracker restarted. Wait, or turn that tracker off and on in view of its base stations.",
		ctx.notifyPoorCalibration);
}

// ---------------------------------------------------------------------------
// Continuous calibration (HMD-mounted tracker)

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

// SteamVR's "Turn off controllers after" time in whole minutes, or 0 when it
// is off or unreadable.
static int TurnOffControllersMinutes()
{
	auto *settings = vr::VRSettings();
	if (!settings)
		return 0;
	vr::EVRSettingsError error = vr::VRSettingsError_None;
	const float seconds = settings->GetFloat(vr::k_pch_Power_Section,
		vr::k_pch_Power_TurnOffControllersTimeout_Float, &error);
	if (error != vr::VRSettingsError_None || !(seconds >= 60.0f) || seconds > 86400.0f)
		return 0;
	return static_cast<int>(std::lround(seconds / 60.0f));
}

// The headset tracker's radio link, apart from the loop (see
// CalibrationContext::continuousTrackerConnected). SteamVR switches a tracker
// off once it has sat still for its "Turn off controllers after" time, as a
// headset tracker does whenever the headset is off, and nothing turns it back
// on (live 2026-09-26: headset set down at 23:42:15, tracker off at 23:47:20;
// the loop dropped to "warming up" and the player learned of it only from a
// calibration refused at 23:50:03).
struct TrackerPresenceState
{
	uint32_t id = vr::k_unTrackedDeviceIndexInvalid;
	bool connected = true;
	double lastCheck = -1e9;
	double offSince = -1.0;       // UI clock; -1 while connected
	bool noticed = false;         // this time off's feed line went out
	// The toast is repeated when the player picks the headset up again: the
	// first went out while nobody wore it. Picked up is the headset moved
	// PickUpMeters from where it lay; PickUpDelaySeconds later it is on.
	bool pickUpPending = false;
	Eigen::Vector3d hmdAtNotice{ 0, 0, 0 };
	double pickedUpAt = -1.0;
	// Across times off: a flapping radio link tells the player once a minute.
	bool settingHinted = false;   // once a session
	double lastNoticeAt = -1e9;
	static constexpr double PickUpMeters = 0.3;
	static constexpr double PickUpDelaySeconds = 3.0;
	static constexpr double NoticeSpacingSeconds = 60.0;

	// Everything but what outlives one time off.
	void Restart(uint32_t trackerId)
	{
		const bool hinted = settingHinted;
		const double lastNotice = lastNoticeAt;
		*this = TrackerPresenceState{};
		id = trackerId;
		settingHinted = hinted;
		lastNoticeAt = lastNotice;
	}
};
static TrackerPresenceState TrackerPresence;

static void TrackerPresenceTick(CalibrationContext &ctx, double now)
{
	auto &p = TrackerPresence;
	const uint32_t id = ctx.continuousTrackerId;
	if (!ctx.ContinuousArmed() || id >= vr::k_unMaxTrackedDeviceCount)
	{
		p.Restart(vr::k_unTrackedDeviceIndexInvalid);
		ctx.continuousTrackerConnected = true;
		return;
	}
	if (id != p.id)
		p.Restart(id);
	if (now - p.lastCheck >= 0.25)
	{
		p.lastCheck = now;
		p.connected = vr::VRSystem()->IsTrackedDeviceConnected(id);
	}
	ctx.continuousTrackerConnected = p.connected;

	if (p.connected)
	{
		if (p.offSince >= 0.0)
		{
			char buf[160];
			snprintf(buf, sizeof buf,
				"Headset tracker connected again after %.1f min off; continuous calibration resumes once it tracks\n",
				(now - p.offSince) / 60.0);
			ctx.Log(buf);
			p.Restart(id);
			p.lastCheck = now;
		}
		return;
	}

	if (p.offSince < 0.0)
	{
		p.offSince = now;
		ctx.continuousTrackerOffEpisodes++;
	}
	static const char *const Toast =
		"QuestCalibrator: the headset tracker is off. Turn it back on to resume continuous calibration.";
	const bool toasts = ctx.notifyPoorCalibration && ToastSink;
	// The log line naming the reason precedes OpenVR dropping the device by
	// half a second and is read four times a second; it gets a moment.
	const LighthouseVisibility::Device *seen = ctx.lighthouse.Find(ctx.continuousTrackerSerial);
	const bool standby = seen && seen->off && seen->standbyOff;
	if (!p.noticed && (standby || now - p.offSince >= 3.0))
	{
		p.noticed = true;
		if (now - p.lastNoticeAt < TrackerPresenceState::NoticeSpacingSeconds)
			return;
		p.lastNoticeAt = now;
		std::string line;
		if (!standby)
			line = "The headset tracker switched off or lost its connection. Turn it back on to resume continuous calibration.";
		else if (const int minutes = TurnOffControllersMinutes())
		{
			char buf[256];
			snprintf(buf, sizeof buf,
				"SteamVR switched the headset tracker off after it sat still for %d minutes. Turn it back on to resume continuous calibration.",
				minutes);
			line = buf;
		}
		else
			line = "SteamVR switched the headset tracker off after it sat still. Turn it back on to resume continuous calibration.";
		ctx.Tell(line + "\n", CalibrationContext::Tone::Warn);
		if (standby && !p.settingHinted)
		{
			p.settingHinted = true;
			ctx.Tell("To keep SteamVR from doing this, set \"Turn off controllers after\" to Never in SteamVR's Startup / Shutdown settings.\n");
		}
		if (toasts)
			ToastSink(Toast);
		// Where the headset lay, when it is being tracked at all.
		if (QpcNowSeconds() - Monitors.hmdRawTime <= 1.0)
		{
			p.pickUpPending = true;
			p.hmdAtNotice = Monitors.hmdRawPosition;
		}
	}
	if (p.pickUpPending && QpcNowSeconds() - Monitors.hmdRawTime <= 1.0)
	{
		if (p.pickedUpAt < 0.0 &&
			(Monitors.hmdRawPosition - p.hmdAtNotice).norm() >= TrackerPresenceState::PickUpMeters)
			p.pickedUpAt = now;
		if (p.pickedUpAt >= 0.0 && now - p.pickedUpAt >= TrackerPresenceState::PickUpDelaySeconds)
		{
			p.pickUpPending = false;
			if (toasts)
				ToastSink(Toast);
		}
	}
}

void calibration_internal::ContinuousTick(CalibrationContext &ctx, double now)
{
	TrackerPresenceTick(ctx, now);
	if (!ctx.ContinuousShouldRun())
	{
		if (ContinuousActive)
		{
			ResetContinuousObservations(ctx, questcal::ContinuousAlignment::ResetReason::Suspended);
			ContinuousActive = false;
		}
		PoseHub.DiscardBacklog(ContinuousConsumer);
		ContinuousPending.clear();
		ctx.continuousCorrectionGate.Clear();
		ctx.continuousState = Continuous->GetState();
		// Only the running loop drains the tracker's poses to judge by.
		ctx.continuousTrackerSeen = true;
		ctx.continuousHeadsetSeen = true;
		return;
	}

	if (!ContinuousActive)
	{
		PoseHub.DiscardBacklog(ContinuousConsumer);
		ContinuousPending.clear();
		ContinuousActive = true;
	}

	// Cheap unconditional sync: the extrinsic changes only on recalibration
	// or profile load, but re-copying it every tick needs no bookkeeping.
	Continuous->SetExtrinsic(ctx.mountExtrinsic);
	Continuous->SetLatencyReestimation(ctx.continuousLatencyReestimation);
	Continuous->SetFollowMode(ctx.continuousNoPause);

	auto &diagnostics = ctx.continuousDiagnostics;
	if (DrainContinuousInput(ctx))
	{
		// A stall-sized hole could fake a discontinuity; baselines across it are unsafe.
		ResetContinuousObservations(ctx, questcal::ContinuousAlignment::ResetReason::StreamGap);
		ContinuousPending.clear();
	}
	ContinuousScratch.insert(ContinuousScratch.begin(), ContinuousPending.begin(), ContinuousPending.end());
	ContinuousPending.clear();

	for (const auto &s : ContinuousScratch)
	{
		if (s.deviceId >= vr::k_unMaxTrackedDeviceCount)
			continue;

		if (s.deviceId != vr::k_unTrackedDeviceIndex_Hmd && s.deviceId != ctx.continuousTrackerId)
			continue;
		if (s.sampleTimeQpc > FrameObservedThrough[s.deviceId])
		{
			if (ContinuousPending.size() < PoseStreamHub::HistoryCapacity)
				ContinuousPending.push_back(s);
			continue;
		}
		questcal::PoseSample sample;
		if (!diagnostics.devices[s.deviceId].Compose(s, QpcToSeconds, sample))
		{
			ResetContinuousObservations(ctx, questcal::ContinuousAlignment::ResetReason::StreamGap);
			continue;
		}
		if (s.deviceId == vr::k_unTrackedDeviceIndex_Hmd)
			Continuous->PushReference(sample);
		else if (ctx.trackerFrames.Normalize(s.deviceId, sample))
			Continuous->PushTarget(sample);
	}

	// The engine's clock is the ring's, not the UI clock.
	const double ringNow = QpcNowSeconds();
	// Most tracking losses begin with a rejected pose, which resets the loop
	// to gathering; without this the status line said "warming up" for as
	// long as the tracker stayed away.
	ctx.continuousTrackerSeen = ringNow -
		diagnostics.devices[ctx.continuousTrackerId].lastAcceptedCaptureTime <=
		Continuous->GetConfig().coastGapSeconds;
	ctx.continuousHeadsetSeen = ringNow -
		diagnostics.devices[vr::k_unTrackedDeviceIndex_Hmd].lastAcceptedCaptureTime <=
		Continuous->GetConfig().coastGapSeconds;

	// The headset tracker's own lighthouse tracking (LighthouseVisibility.h).
	// A new solution or a change of stations can move its pose by centimeters
	// and a single-baseline fit swims, so a window must not straddle one and no
	// verdict is drawn while it settles. Without the log nothing changes here.
	const LighthouseVisibility::Device *trackerSeen = ctx.continuousTrackerSerial.empty()
		? nullptr : ctx.lighthouse.Find(ctx.continuousTrackerSerial);
	if (!trackerSeen)
	{
		ContinuousLighthouseSerial.clear();
	}
	else
	{
		if (trackerSeen->serial == ContinuousLighthouseSerial &&
			trackerSeen->liveDisturbances != ContinuousLighthouseDisturbances)
		{
			// A new server session starts the counts over, and its first line
			// for the tracker need not be a bootstrap.
			Continuous->NoteTargetResolved(questcal::RingTime(trackerSeen->lastDisturbance),
				trackerSeen->liveBootstraps != ContinuousLighthouseBootstraps && trackerSeen->liveBootstraps > 0);
			ctx.continuousCorrectionGate.Clear();
			ctx.Diag("continuous: window restarted, headset tracker " + trackerSeen->lastDisturbanceText);
		}
		ContinuousLighthouseSerial = trackerSeen->serial;
		ContinuousLighthouseDisturbances = trackerSeen->liveDisturbances;
		ContinuousLighthouseBootstraps = trackerSeen->liveBootstraps;
	}
	Continuous->SetTargetSettling(trackerSeen &&
		ctx.lighthouse.Settling(ctx.continuousTrackerSerial, ringNow));
	// A freeze re-anchors only where a restart of the tracker would have shown.
	const bool restartsVisible = ctx.lighthouseLogAvailable && trackerSeen;
	Continuous->SetTargetRestartsVisible(restartsVisible);

	// Re-evaluate the current field for each retained observation: comparing a
	// multi-position history with only the latest spot turns healthy anchor
	// gradients into apparent drift. With no active anchors the blend returns
	// the base calibration exactly.
	auto expectedAt = [&](const Eigen::Vector3d &targetRawPos,
		Eigen::Quaterniond &rotationOut, Eigen::Vector3d &translationOut)
	{
		// Looked up at the tracker's base-calibrated position, with the width
		// BuildAlignmentField puts on the wire.
		Eigen::Vector3d basePos = ringpose::BaseCalibratedPosition(
			ctx.transform.rotation, ctx.transform.translationMeters, ctx.transform.scale,
			targetRawPos);
		questcal::BlendedFieldCalibration(ctx.ActiveFieldAnchors(), ctx.transform.rotation,
			ctx.transform.translationMeters, basePos, rotationOut, translationOut,
			questcal::FieldBlendSigmaMeters);
	};
	Continuous->Update(questcal::RingTime(ringNow), ctx.transform.rotation, ctx.transform.translationMeters,
		ctx.transform.scale, ctx.transform.timeOffset, expectedAt);
	diagnostics.lastUpdateTime = ringNow;
	diagnostics.engine = Continuous->GetDiagnostics();

	// A re-anchor and a correction both move only the lighthouse side, not the
	// headset's space, so the protected chaperone stays where it is.
	auto applyDelta = [&](const questcal::ContinuousAlignment::Correction &delta, bool snap)
	{
		if (!snap && ctx.continuousRequireTrigger)
			ctx.Tell("Trigger pulled; applying the correction.\n");
		if (!questcal::ApplyCalibrationDelta(ctx, delta.rotation, delta.translation,
			snap, now, /*moveChaperone=*/false))
		{
			ctx.ReportError("A correction was too large to be safe and was skipped. If this keeps happening, recalibrate.\n");
			return false;
		}
		ctx.lastAutoCorrectionUnixTime = static_cast<double>(std::time(nullptr));
		// The drift evidence was just acted on; the monitor windows stay
		// valid because a correction never moves raw poses.
		ctx.ClearDriftEvidence();
		if (!snap)
		{
			ctx.autoCorrectionsApplied++;
			char buf[128];
			snprintf(buf, sizeof buf, "correction applied: yaw %.3f deg, position %.1f mm",
				std::abs(questcal::SignedYawRadians(delta.rotation)) * 180.0 / questcal::Pi,
				delta.translation.norm() * 1000.0);
			ctx.Diag(buf);
		}
		return true;
	};
	const auto handled = questcal::HandleContinuousOutput(*Continuous,
		ctx.continuousCorrectionGate, ctx.continuousRequireTrigger, applyDelta,
		AnyControllerTriggerPressed);
	const bool reanchorApplied =
		handled.reanchor == questcal::ContinuousOutputHandling::Outcome::Applied;
	if (handled.correctionReady)
		ctx.Tell("Correction ready. Pull a trigger to apply it.\n");

	ctx.continuousState = Continuous->GetState();
	if (ctx.continuousState != questcal::ContinuousAlignment::State::Frozen)
		ctx.continuousFreezeFromRestart = false;
	ctx.continuousDeviation = Continuous->CurrentDeviation();
	ctx.continuousScatterRotDeg = Continuous->ScatterRotRmsDeg();
	ctx.continuousScatterPosM = Continuous->ScatterPosRmsM();

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
		// The freeze keeps its evidence in the detail line. The player reads
		// the fact, not a diagnosis: a mounted tracker does not move, so
		// blaming it sends people to check hardware that is fine.
		case questcal::ContinuousAlignment::Event::FrozenLargeDeviation:
			ctx.continuousFreezeFromRestart = ev.afterTargetResolve;
			snprintf(buf, sizeof buf,
				"Continuous calibration frozen: deviation yaw %.2f deg, tilt %.2f deg, %.1f cm%s\n",
				ev.deviation.yawDeg, ev.deviation.tiltDeg, ev.deviation.posM * 100.0,
				ev.afterTargetResolve ? " -- after the headset tracker's base station tracking restarted" : "");
			ctx.Log(buf);
			if (ev.afterTargetResolve)
				NotifyResolveFreeze(ctx);
			else if (restartsVisible)
				NotifyOnce(ctx, Monitors.freezeNotified,
					"Continuous calibration paused: readings moved too far from the calibration. It re-aligns on its own if they hold steady.",
					CalibrationContext::Tone::Warn,
					"QuestCalibrator: continuous calibration paused; readings moved too far from the calibration. It re-aligns on its own if they hold steady, or recalibrate with the headset tracker.",
					ctx.notifyPoorCalibration);
			else
				// Without the tracker's restarts in view nothing re-aligns on
				// its own; only the readings coming back resume it.
				NotifyOnce(ctx, Monitors.freezeNotified,
					"Continuous calibration paused: readings drifted too far from the calibration to correct safely.",
					CalibrationContext::Tone::Warn,
					"QuestCalibrator: continuous calibration paused; readings drifted too far to correct safely. Recalibrate with the headset tracker to resume.",
					ctx.notifyPoorCalibration);
			break;
		case questcal::ContinuousAlignment::Event::Reanchored:
			if (!reanchorApplied)
				break;
			// Legacy does this at every evaluation that exceeds the freeze
			// thresholds, so its line says it followed rather than re-anchored.
			snprintf(buf, sizeof buf,
				"Continuous calibration %s: deviation yaw %.2f deg, tilt %.2f deg, %.1f cm%s%s\n",
				ctx.continuousNoPause ? "followed" : "re-anchored",
				ev.deviation.yawDeg, ev.deviation.tiltDeg, ev.deviation.posM * 100.0,
				ev.deviation.tiltDeg >= Continuous->GetConfig().holdTiltDeg ? ", tilt included" : "",
				ev.afterTargetResolve ? " -- after the headset tracker's base station tracking restarted"
				: ev.acrossSolutions ? " -- the headset tracker's next solution read it too, so its restart did not explain it"
				: "");
			ctx.Log(buf);
			ctx.continuousReanchors++;
			if (ev.acrossSolutions)
				ctx.continuousReanchorsAcrossSolutions++;
			Monitors.freezeNotified = false;
			if (!ctx.continuousNoPause)
				ctx.Tell("Continuous calibration re-aligned your trackers after the tracking spaces moved apart.\n",
					CalibrationContext::Tone::Good);
			break;
		case questcal::ContinuousAlignment::Event::ReanchorUndone:
			if (!reanchorApplied)
				break;
			snprintf(buf, sizeof buf,
				"Continuous calibration undid its last re-anchor: the readings fit the calibration before it again "
				"(against the current one: yaw %.2f deg, tilt %.2f deg, %.1f cm)\n",
				ev.deviation.yawDeg, ev.deviation.tiltDeg, ev.deviation.posM * 100.0);
			ctx.Log(buf);
			ctx.continuousReanchorsUndone++;
			Monitors.freezeNotified = false;
			if (!ctx.continuousNoPause)
				ctx.Tell("Continuous calibration went back to the alignment it had before it last re-aligned your trackers.\n",
					CalibrationContext::Tone::Good);
			break;
		case questcal::ContinuousAlignment::Event::ObservationsUnstable:
			snprintf(buf, sizeof buf,
				"Mounted tracker observations unstable (scatter %.2f deg / %.1f cm) -- alignment updates paused until tracking settles\n",
				ev.scatterRotDeg, ev.scatterPosM * 100.0);
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
			// Named after the stream that stopped, and the recovery after the
			// same one.
			ContinuousLostStream = ringpose::WhichStreamStopped(ringNow,
				diagnostics.devices[vr::k_unTrackedDeviceIndex_Hmd].lastAcceptedCaptureTime,
				diagnostics.devices[ctx.continuousTrackerId].lastAcceptedCaptureTime);
			switch (ContinuousLostStream)
			{
			case ringpose::StoppedStream::Headset:
				ctx.Tell("Headset not tracking; continuous calibration waiting.\n");
				break;
			case ringpose::StoppedStream::Both:
				ctx.Tell("Headset and headset tracker not tracking; continuous calibration waiting.\n");
				break;
			case ringpose::StoppedStream::Neither:
				ctx.Tell("No usable readings from the headset and its tracker; continuous calibration waiting.\n");
				break;
			default:
				ctx.Tell("Headset tracker not tracking; continuous calibration waiting.\n");
				break;
			}
			break;
		case questcal::ContinuousAlignment::Event::TrackerRecovered:
			switch (ContinuousLostStream)
			{
			case ringpose::StoppedStream::Headset:
				ctx.Tell("Headset back; continuous calibration warming up.\n");
				break;
			case ringpose::StoppedStream::Both:
				ctx.Tell("Headset and headset tracker back; continuous calibration warming up.\n");
				break;
			case ringpose::StoppedStream::Neither:
				ctx.Tell("Readings back; continuous calibration warming up.\n");
				break;
			default:
				ctx.Tell("Headset tracker back; continuous calibration warming up.\n");
				break;
			}
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
		// Never past the range a calibration searches.
		const double range = questcal::EngineConfig().timeOffsetRange;
		if (updated > range) updated = range;
		if (updated < -range) updated = -range;

		double appliedBefore = questcal::ComputeAppliedTimeOffset(ctx.transform.timeOffset);
		double appliedAfter = questcal::ComputeAppliedTimeOffset(updated);
		ctx.transform.timeOffset = updated;
		ctx.persistence.MarkProfile(now);
		if (std::abs(appliedAfter - appliedBefore) > 0.0005)
			SynchronizeCalibrationDriver(ctx);
	}
}
