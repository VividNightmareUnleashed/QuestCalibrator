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
#include "../common/PoseChannel.h"

#include <Eigen/Dense>

#include <ctime>
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

static std::unique_ptr<questcal::ContinuousAlignment> Continuous;
static int ContinuousConsumer = -1;
static std::vector<protocol::DevicePoseSample> ContinuousScratch;
static bool ContinuousActive = false;

CalibrationContext CalCtx;

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

	Continuous = std::make_unique<questcal::ContinuousAlignment>();
	ContinuousConsumer = PoseHub.CreateConsumer();
}

void ShutdownCalibrator()
{
	// A quit inside the save-debounce window must not lose a runtime
	// compensation update.
	if (CalCtx.profileSaveDirty && CalCtx.validProfile)
	{
		SaveProfile(CalCtx);
		CalCtx.profileSaveDirty = false;
	}
	PoseHub.Stop();
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

static void ResetAndDisableOffsets(uint32_t id)
{
	protocol::Request req(protocol::RequestSetDeviceTransform);
	req.setDeviceTransform = protocol::SetDeviceTransform(id, false);
	Driver.SendBlocking(req);
}

static_assert(vr::k_unTrackedDeviceIndex_Hmd == 0, "HMD index expected to be 0");

// Ship the spatial correction field. Deltas are derived here against the
// current base calibration: delta_i = anchor_i o base^-1 is the correction
// that, applied after the base calibration, reproduces the absolute solve at
// that anchor's spot.
static void SendAlignmentField(CalibrationContext &ctx)
{
	protocol::Request req(protocol::RequestSetAlignmentField);
	auto &f = req.setAlignmentField;

	f.enabled = ctx.enabled && ctx.fieldEnabled && !ctx.fieldAnchors.empty();
	f.generation = ctx.fieldGeneration;
	f.anchorCount = static_cast<uint32_t>(std::min(
		ctx.fieldAnchors.size(), static_cast<size_t>(protocol::SetAlignmentField::MaxAnchors)));

	Eigen::Quaterniond baseInv = ctx.calibratedRotationQ.conjugate();
	Eigen::Vector3d baseT = ctx.TranslationMeters();
	for (uint32_t i = 0; i < f.anchorCount; ++i)
	{
		const auto &a = ctx.fieldAnchors[i];
		Eigen::Quaterniond dR = (a.rotation * baseInv).normalized();
		Eigen::Vector3d dT = a.translationMeters - dR * baseT;

		for (int k = 0; k < 3; ++k)
		{
			f.anchors[i].position[k] = a.position(k);
			f.anchors[i].translationDelta[k] = dT(k);
		}
		f.anchors[i].rotationDelta = VRQuat(dR);
	}

	Driver.SendBlocking(req);
}

static void ScanAndApplyProfile(CalibrationContext &ctx)
{
	char buffer[vr::k_unMaxPropertyStringSize];
	ctx.enabled = ctx.validProfile;

	// Timeline shift for target devices (runtime latency re-prediction). The
	// manual override bypasses the solved value; it exists to pin the sign
	// convention against a live SteamVR session.
	double timeShift = 0.0;
	if (ctx.useManualTimeOffset)
		timeShift = ctx.manualTimeOffsetMs / 1000.0;
	else if (ctx.applyTimeOffset)
		timeShift = questcal::ComputeAppliedTimeOffset(ctx.calibratedTimeOffset);
	ctx.appliedTimeOffset = timeShift;

	ctx.continuousTrackerId = vr::k_unTrackedDeviceIndexInvalid;

	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		ctx.referenceDeviceMask[id] = false;
		ctx.targetDeviceMask[id] = false;

		auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid)
			continue;

		if (!ctx.enabled)
		{
			ResetAndDisableOffsets(id);
			continue;
		}

		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer, vr::k_unMaxPropertyStringSize, &err);

		if (err != vr::TrackedProp_Success)
		{
			ResetAndDisableOffsets(id);
			continue;
		}

		std::string trackingSystem(buffer);
		ctx.referenceDeviceMask[id] = (trackingSystem == ctx.referenceTrackingSystem);
		ctx.targetDeviceMask[id] = (trackingSystem == ctx.targetTrackingSystem);

		if (id == vr::k_unTrackedDeviceIndex_Hmd)
		{
			if (trackingSystem != ctx.referenceTrackingSystem)
			{
				// Currently using an HMD with a different tracking system than the calibration.
				ctx.enabled = false;
			}

			ResetAndDisableOffsets(id);
			continue;
		}

		if (trackingSystem != ctx.targetTrackingSystem)
		{
			ResetAndDisableOffsets(id);
			continue;
		}

		// The continuous-calibration tracker is identified by serial (ids are
		// not stable across sessions).
		if (!ctx.continuousTrackerSerial.empty())
		{
			vr::ETrackedPropertyError serialErr = vr::TrackedProp_Success;
			vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String,
				buffer, vr::k_unMaxPropertyStringSize, &serialErr);
			if (serialErr == vr::TrackedProp_Success && ctx.continuousTrackerSerial == buffer)
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
		req.setDeviceTransform.hidden = ctx.continuousEnabled && ctx.hideMountedTracker &&
			id == ctx.continuousTrackerId;
		Driver.SendBlocking(req);
	}

	// Re-asserted alongside the per-device transforms so a restarted driver
	// converges without special casing. On a universe jump the base transforms
	// land first and the field one pipe round-trip later; the mixed window is
	// bounded by the (small) delta magnitudes and the generation bump snaps
	// driver-side smoothing when it arrives.
	SendAlignmentField(ctx);

	// Auto-restore of the protected chaperone snapshot. The trigger compares
	// wall GEOMETRY content only: quads are standing-frame, so both universe
	// jumps and play-space movers (which move the standing center, not the
	// walls) leave them untouched — deliberate adjustments never trip this.
	// A count-only check would miss a reset that lands on the same number of
	// walls (two 4-wall rectangles), hence the corner-wise comparison.
	// An empty snapshot never auto-restores: it would stomp any bounds the
	// runtime imports later (e.g. a Guardian arriving after session start).
	if (ctx.enabled && ctx.chaperone.valid && ctx.chaperone.autoApply &&
		!ctx.chaperone.geometry.empty())
	{
		uint32_t quadCount = 0;
		vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(nullptr, &quadCount);

		bool differs = quadCount != ctx.chaperone.geometry.size();
		if (!differs)
		{
			std::vector<vr::HmdQuad_t> live(quadCount);
			if (vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(live.data(), &quadCount))
				differs = !questcal::QuadsMatch(live, ctx.chaperone.geometry, 0.002f);
		}

		// Cooldown so a runtime that keeps reasserting its own bounds is
		// contested every few seconds at worst, not at the 1 Hz scan rate.
		if (differs && ctx.timeLastTick - ctx.chaperone.lastRestoreTime >= 5.0)
		{
			ApplyChaperoneBounds();
			ctx.chaperone.lastRestoreTime = ctx.timeLastTick;

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

// Drop the collector's backlog so a new collection starts fresh.
static void DiscardPoseRingBacklog()
{
	PoseHub.DiscardBacklog(CollectorConsumer);
}

static void CollectFromPoseRing(CalibrationContext &ctx, double now)
{
	PoseHub.Drain(CollectorConsumer, CollectorScratch);
	for (const auto &s : CollectorScratch)
	{
		// Strict validity: a pose the driver flagged invalid never enters the
		// solve, even if the tracking result still claims "running OK".
		if (!s.poseIsValid || s.trackingResult != static_cast<uint32_t>(vr::TrackingResult_Running_OK))
			continue;

		if (s.deviceId == ctx.referenceID)
		{
			ctx.refSamples.push_back(EngineSampleFromRing(s));
			ctx.lastRefSampleTime = now;
		}
		else if (s.deviceId == ctx.targetID)
		{
			ctx.targetSamples.push_back(EngineSampleFromRing(s));
			ctx.lastTargetSampleTime = now;
		}
	}
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
		into.push_back(s);
		lastTime = now;
	};

	push(ctx.referenceID, ctx.refSamples, ctx.lastRefSampleTime);
	push(ctx.targetID, ctx.targetSamples, ctx.lastTargetSampleTime);
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

	// The chaperone snapshot's standing center maps the standing frame into
	// the (just re-based) raw frame, so it re-anchors by D like everything
	// else raw-frame; the wall quads are standing-frame and stay put. Without
	// this, the ScanAndApplyProfile below could restore bounds displaced by
	// exactly the delta we just compensated.
	if (ctx.chaperone.valid)
		ctx.chaperone.standingCenter =
			questcal::DeltaTimesPose(dR, dT, ctx.chaperone.standingCenter);

	ScanAndApplyProfile(ctx);

	ctx.profileSaveDirty = true;
	ctx.profileSaveDirtyTime = now;
}

// ---------------------------------------------------------------------------
// Runtime monitoring: universe-jump compensation

// Fold an accepted universe delta into the calibration: the reference universe
// moved by D in one frame, so target devices must follow to stay aligned.
static void ApplyUniverseDelta(CalibrationContext &ctx, const JumpDetector::UniverseDelta &d, double now)
{
	ApplyAlignmentDelta(ctx, d.rotation, d.translation, /*snap=*/true, now);

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
			MonitorActive = false;
		}
		PoseHub.DiscardBacklog(MonitorConsumer);
		return;
	}

	if (!MonitorActive)
	{
		PoseHub.DiscardBacklog(MonitorConsumer);
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
	}

	// The HMD's latest raw position, for the worn-device heuristic below.
	// Persisted across ticks; only rough currency is needed.
	static Eigen::Vector3d lastHmdRawPos;
	static double lastHmdRawTime = -1e9;

	for (const auto &s : MonitorScratch)
	{
		if (s.deviceId >= vr::k_unMaxTrackedDeviceCount)
			continue;
		if (ctx.referenceDeviceMask[s.deviceId])
			Jumps->Push(s);

		bool sampleValid = s.poseIsValid &&
			s.trackingResult == static_cast<uint32_t>(vr::TrackingResult_Running_OK);
		if (sampleValid && s.deviceId == vr::k_unTrackedDeviceIndex_Hmd &&
			ctx.referenceDeviceMask[s.deviceId])
		{
			RingSampleParts p = UnpackRingSample(s);
			lastHmdRawPos = p.wfdRot * p.drvPos + p.wfdTrans;
			lastHmdRawTime = static_cast<double>(s.sampleTimeQpc) * QpcToSeconds;
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

		if (anchorsUniverse && sampleValid && s.deviceId != vr::k_unTrackedDeviceIndex_Hmd &&
			static_cast<double>(s.sampleTimeQpc) * QpcToSeconds - lastHmdRawTime < 3.0)
		{
			RingSampleParts p = UnpackRingSample(s);
			Eigen::Vector3d rawPos = p.wfdRot * p.drvPos + p.wfdTrans;
			Eigen::Vector3d refPos = ctx.calibratedRotationQ
				* (ctx.calibratedScale * rawPos) + ctx.TranslationMeters();
			if ((refPos - lastHmdRawPos).norm() < 1.2)
				anchorsUniverse = false;
		}

		if (anchorsUniverse)
			Drift->Push(s);
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

	// Debounced persistence of compensation updates.
	if (ctx.profileSaveDirty && now - ctx.profileSaveDirtyTime > 5.0)
	{
		SaveProfile(ctx);
		ctx.profileSaveDirty = false;
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
		if (!s.poseIsValid || s.trackingResult != static_cast<uint32_t>(vr::TrackingResult_Running_OK))
			continue;

		if (s.deviceId == vr::k_unTrackedDeviceIndex_Hmd)
			Continuous->PushReference(EngineSampleFromRing(s));
		else if (s.deviceId == ctx.continuousTrackerId)
		{
			questcal::PoseSample sample = EngineSampleFromRing(s);
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
		ctx.profileSaveDirty = true;
		ctx.profileSaveDirtyTime = now;
		if (std::abs(appliedAfter - appliedBefore) > 0.0005)
			ScanAndApplyProfile(ctx);
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
	SaveProfile(ctx);
	ScanAndApplyProfile(ctx);

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
	if (result.valid && !asAnchor && ctx.referenceID == vr::k_unTrackedDeviceIndex_Hmd)
	{
		questcal::ContinuousAlignment::Config contCfg;
		questcal::MountExtrinsic extrinsic;
		if (questcal::ContinuousAlignment::DeriveMountExtrinsic(
			ctx.refSamples, ctx.targetSamples, result, contCfg, extrinsic))
		{
			ctx.mountExtrinsic = extrinsic;

			char serial[256] = { 0 };
			vr::VRSystem()->GetStringTrackedDeviceProperty(
				ctx.targetID, vr::Prop_SerialNumber_String, serial, sizeof serial);
			if (serial[0])
				ctx.continuousTrackerSerial = serial;

			snprintf(buf, sizeof buf,
				"Mount offset learned for continuous calibration (%.2f deg / %.1f mm spread, %zu pairs)\n",
				extrinsic.rotRmsDeg, extrinsic.posRmsM * 1000.0, extrinsic.pairs);
			ctx.Log(buf);
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

	SaveProfile(ctx);
	ScanAndApplyProfile(ctx);
	ctx.Log("Finished calibration, profile saved\n");

	if (result.scale != 1.0)
	{
		snprintf(buf, sizeof buf, "Playspace scale: %.4f\n", result.scale);
		ctx.Log(buf);
	}
}

void StartCalibration()
{
	CalCtx.collectAsAnchor = false;
	CalCtx.state = CalibrationState::Begin;
	CalCtx.wantedUpdateInterval = 0.0;
	CalCtx.messages.clear();
}

void StartAnchorCalibration()
{
	StartCalibration();
	CalCtx.collectAsAnchor = true;
}

void CalibrationTick(double time)
{
	if (!vr::VRSystem())
		return;

	auto &ctx = CalCtx;
	if ((time - ctx.timeLastTick) < 0.02)
		return;

	ctx.timeLastTick = time;

	// RawAndUncalibrated is deliberate: the solve must see pre-calibration
	// poses. "Fixing" this to Standing breaks calibration entirely.
	vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0.0f, ctx.devicePoses, vr::k_unMaxTrackedDeviceCount);

	RuntimeMonitorTick(ctx, time);
	// After the monitors: an accepted jump must land (and reset the continuous
	// window) before the continuous loop reads the calibration this tick.
	ContinuousTick(ctx, time);

	if (ctx.state == CalibrationState::None)
	{
		ctx.wantedUpdateInterval = 1.0;

		if ((time - ctx.timeLastScan) >= 1.0)
		{
			ScanAndApplyProfile(ctx);
			ctx.timeLastScan = time;
		}
		return;
	}

	if (ctx.state == CalibrationState::Editing)
	{
		ctx.wantedUpdateInterval = 0.1;

		if ((time - ctx.timeLastScan) >= 0.1)
		{
			// Live edits are intentional: every re-apply snaps rather than
			// letting the base slew smear a user's drag over seconds.
			ctx.baseGeneration++;
			ScanAndApplyProfile(ctx);
			ctx.timeLastScan = time;
		}
		return;
	}

	if (ctx.state == CalibrationState::Begin)
	{
		bool ok = true;

		if (ctx.referenceID >= vr::k_unMaxTrackedDeviceCount)
		{
			ctx.Log("Missing reference device\n");
			ok = false;
		}
		else if (!ctx.devicePoses[ctx.referenceID].bPoseIsValid)
		{
			ctx.Log("Reference device is not tracking\n");
			ok = false;
		}

		if (ctx.targetID >= vr::k_unMaxTrackedDeviceCount)
		{
			ctx.Log("Missing target device\n");
			ok = false;
		}
		else if (!ctx.devicePoses[ctx.targetID].bPoseIsValid)
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

		char referenceSerial[256] = { 0 }, targetSerial[256] = { 0 };
		vr::VRSystem()->GetStringTrackedDeviceProperty(ctx.referenceID, vr::Prop_SerialNumber_String, referenceSerial, 256);
		vr::VRSystem()->GetStringTrackedDeviceProperty(ctx.targetID, vr::Prop_SerialNumber_String, targetSerial, 256);

		char buf[256];
		snprintf(buf, sizeof buf, "Reference device ID: %u, serial: %s\n", ctx.referenceID, referenceSerial);
		ctx.Log(buf);
		snprintf(buf, sizeof buf, "Target device ID: %u, serial: %s\n", ctx.targetID, targetSerial);
		ctx.Log(buf);

		ResetAndDisableOffsets(ctx.targetID);

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

		if (ctx.referenceID == vr::k_unTrackedDeviceIndex_Hmd)
			ctx.Log("Wear the headset with the tracker firmly mounted.\nSlowly turn and tilt your head in wide arcs, and lean side to side.\n");
		else
			ctx.Log("Hold the devices firmly together.\nSlowly move them in wide circles, both side to side and up and down.\n");
		return;
	}

	// ---- Collecting ----
	if (PoseHub.RingOpen())
		CollectFromPoseRing(ctx, time);
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

void LoadChaperoneBounds()
{
	vr::VRChaperoneSetup()->RevertWorkingCopy();

	uint32_t quadCount = 0;
	vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(nullptr, &quadCount);

	CalCtx.chaperone.geometry.resize(quadCount);

	if (quadCount != 0)
	{
		vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(&CalCtx.chaperone.geometry[0], &quadCount);
	}

	vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose(&CalCtx.chaperone.standingCenter);
	vr::VRChaperoneSetup()->GetWorkingPlayAreaSize(&CalCtx.chaperone.playSpaceSize.v[0], &CalCtx.chaperone.playSpaceSize.v[1]);
	CalCtx.chaperone.valid = true;
	CalCtx.chaperone.copyUnixTime = static_cast<double>(std::time(nullptr));

	char buf[128];
	if (quadCount == 0)
		snprintf(buf, sizeof buf,
			"Chaperone snapshot saved: play area center only (no chaperone walls yet) -- auto-restore stays inactive\n");
	else
		snprintf(buf, sizeof buf,
			"Chaperone snapshot saved: %u wall segment(s), %.1f x %.1f m play area\n",
			quadCount, CalCtx.chaperone.playSpaceSize.v[0], CalCtx.chaperone.playSpaceSize.v[1]);
	CalCtx.Log(buf);
}

void ApplyChaperoneBounds()
{
	vr::VRChaperoneSetup()->RevertWorkingCopy();
	if (!CalCtx.chaperone.geometry.empty())
	{
		vr::VRChaperoneSetup()->SetWorkingCollisionBoundsInfo(&CalCtx.chaperone.geometry[0], (uint32_t) CalCtx.chaperone.geometry.size());
	}
	vr::VRChaperoneSetup()->SetWorkingStandingZeroPoseToRawTrackingPose(&CalCtx.chaperone.standingCenter);
	vr::VRChaperoneSetup()->SetWorkingPlayAreaSize(CalCtx.chaperone.playSpaceSize.v[0], CalCtx.chaperone.playSpaceSize.v[1]);
	vr::VRChaperoneSetup()->CommitWorkingCopy(vr::EChaperoneConfigFile_Live);
}
