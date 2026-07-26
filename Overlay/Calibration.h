#pragma once

#include "CalibrationEngine.h"
#include "ContinuousAlignment.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <openvr.h>

#include <iostream>
#include <string>
#include <vector>

enum class CalibrationState
{
	None,
	Begin,
	Collecting,   // both device streams recorded; solve runs at the end
	Editing,
};

// Session log file (%LOCALAPPDATA%\QuestCalibrator\QuestCalibrator.log): the
// in-app message pane persisted for bug reports — the Release build is a GUI
// binary, so stderr goes nowhere. One fresh file per session; the previous
// session survives as QuestCalibrator.prev.log (one generation only) and a
// hard size cap bounds a runaway session, so disk use can never grow.
void InitSessionLog();
void AppendSessionLog(const std::string &msg);

struct CalibrationContext
{
	CalibrationState state = CalibrationState::None;
	uint32_t referenceID = 0xFFFFFFFF, targetID = 0xFFFFFFFF;

	// The quaternion is the source of truth for the calibrated rotation.
	// calibratedRotation (Euler, degrees, [roll, yaw, pitch] to match the
	// (2,1,0) decomposition) and calibratedTranslation (centimeters) are
	// display/editing copies — never round-trip state through them except at
	// the explicit editor rebuild below.
	Eigen::Quaterniond calibratedRotationQ{ 1, 0, 0, 0 };
	Eigen::Vector3d calibratedRotation{ 0, 0, 0 };
	Eigen::Vector3d calibratedTranslation{ 0, 0, 0 };
	double calibratedScale = 1.0;
	double calibratedTimeOffset = 0.0;   // seconds; solved inter-system offset

	// Runtime latency re-prediction: when enabled, the solved time offset is
	// applied to target devices via ComputeAppliedTimeOffset. The manual
	// override injects an explicit poseTimeOffset shift instead (milliseconds,
	// bypassing the solved value) — the live sign-pinning spike tool.
	bool applyTimeOffset = true;
	bool useManualTimeOffset = false;
	double manualTimeOffsetMs = 0.0;

	// The shift (seconds) most recently sent to the driver for target devices.
	double appliedTimeOffset = 0.0;

	// Playspace scale solving is opt-in: streamed reference poses are motion-
	// smoothed, which under-reports calibration motion and biases the solved
	// scale low by several percent (varying with motion speed) — far larger
	// than the sub-percent genuine metric difference it could correct.
	bool solveScale = false;

	// UI preference (persisted): show raw solver/drift stats instead of the
	// plain-language calibration rating.
	bool uiAdvanced = false;

	// One-time drift warning shown before the first chaperone protect.
	// Top-level (not in Chaperone) so it survives Clear() and persists even
	// after the snapshot is discarded.
	bool chaperoneWarningAck = false;

	// Spatial correction field: per-spot absolute solves. The
	// per-anchor deltas the driver blends are derived against the base
	// calibration at send time, so a universe jump only has to transform the
	// absolute anchors by D — the conjugated deltas (D delta D^-1) fall out on
	// the next send.
	struct FieldAnchor
	{
		Eigen::Vector3d position{ 0, 0, 0 };        // target-path centroid, reference space
		Eigen::Quaterniond rotation{ 1, 0, 0, 0 };  // absolute calibration solved at this spot
		Eigen::Vector3d translationMeters{ 0, 0, 0 };
	};
	std::vector<FieldAnchor> fieldAnchors;
	bool fieldEnabled = true;
	uint32_t fieldGeneration = 0;      // driver-side smoothing snaps when this changes
	bool collectAsAnchor = false;      // next solve stores an anchor instead of replacing base

	// Base-transform snap/slew discriminator (protocol v5, runtime only).
	// Bumped by SetCalibration so every intentional change snaps by default;
	// continuous-calibration corrections go through SetCalibrationContinuous,
	// the one path that deliberately leaves it unchanged so the driver slews.
	uint32_t baseGeneration = 0;

	// Continuous calibration (HMD-mounted tracker). The tracker's serial is
	// the persisted identity; the id is re-resolved on every profile scan.
	bool continuousEnabled = false;              // persisted
	std::string continuousTrackerSerial;         // persisted
	bool continuousLatencyReestimation = false;  // persisted; opt-in, default off
	bool hideMountedTracker = true;              // persisted; displace from games
	questcal::MountExtrinsic mountExtrinsic;     // persisted while valid
	uint32_t continuousTrackerId = 0xFFFFFFFF;   // runtime
	double lastAutoCorrectionUnixTime = 0.0;     // runtime; feeds the drift age
	uint32_t autoCorrectionsApplied = 0;         // runtime
	// UI status mirror, refreshed every continuous tick.
	int continuousState = 0;                     // questcal::ContinuousAlignment::State
	questcal::ContinuousAlignment::Deviation continuousDeviation;
	double continuousScatterRotDeg = 0.0;
	double continuousScatterPosM = 0.0;

	// Runtime alignment monitoring. The device masks mark reference/target
	// system devices (refreshed by the profile scan); the counters feed drift
	// staleness and the UI.
	bool referenceDeviceMask[vr::k_unMaxTrackedDeviceCount] = {};
	bool targetDeviceMask[vr::k_unMaxTrackedDeviceCount] = {};
	uint32_t jumpsCompensated = 0;
	double lastJumpUiTime = 0.0;
	double jumpTiltResidualDeg = 0.0;    // non-rigid rotation discarded by yaw constraint
	double jumpSpreadResidualM = 0.0;    // device disagreement at accepted jumps
	uint32_t referenceGapEvents = 0;     // hard reference-stream gaps (no compensation possible)

	// Drift staleness (detect + notify only; never auto-corrects). Counters
	// reset on every successful calibration.
	double calibrationUnixTime = 0.0;    // persisted; 0 = unknown (old profile)
	uint32_t driftSlideEvents = 0;
	double driftMaxSlideM = 0.0;
	uint32_t discontinuousLossEvents = 0;
	double driftScore = 0.0;             // 0..1
	enum class AlignmentHealth { Fresh, Aging, Stale };
	AlignmentHealth alignment = AlignmentHealth::Fresh;

	// Debounced profile persistence for runtime compensation updates: dirty
	// profiles save after a quiet period and always on shutdown.
	bool profileSaveDirty = false;
	double profileSaveDirtyTime = 0.0;

	std::string referenceTrackingSystem;
	std::string targetTrackingSystem;

	bool enabled = false;
	bool validProfile = false;
	double timeLastTick = 0, timeLastScan = 0;
	double wantedUpdateInterval = 1.0;

	enum Speed
	{
		FAST = 0,
		SLOW = 1,
		VERY_SLOW = 2
	};
	Speed calibrationSpeed = FAST;

	// Collection buffers, owned here and cleared on every start/abort so an
	// aborted run can never leak samples into the next one.
	std::vector<questcal::PoseSample> refSamples;
	std::vector<questcal::PoseSample> targetSamples;
	double collectionStart = 0.0;
	double lastRefSampleTime = 0.0;
	double lastTargetSampleTime = 0.0;

	questcal::EngineResult lastResult;

	vr::TrackedDevicePose_t devicePoses[vr::k_unMaxTrackedDeviceCount];

	struct Chaperone
	{
		bool valid = false;
		bool autoApply = true;
		// Quads are relative to the standing/play-area center (see
		// ChaperoneMath.h) — jump-invariant; only standingCenter re-anchors.
		std::vector<vr::HmdQuad_t> geometry;
		vr::HmdMatrix34_t standingCenter;
		vr::HmdVector2_t playSpaceSize;
		double copyUnixTime = 0.0;     // when the snapshot was taken (persisted)
		double lastRestoreTime = 0.0;  // auto-restore cooldown (runtime only)
	} chaperone;

	void SetCalibration(const Eigen::Quaterniond &rotation, const Eigen::Vector3d &translationMeters, double scale)
	{
		SetCalibrationContinuous(rotation, translationMeters, scale);
		// Fail-safe direction: a site that forgets to use the continuous setter
		// only makes a small correction snap (invisible at mm scale); the
		// dangerous case — a big intentional change smeared over seconds — can
		// never happen by omission.
		baseGeneration++;
	}

	// Identical to SetCalibration but leaves baseGeneration untouched, so the
	// driver slews toward the new transform instead of snapping. Only for the
	// small auto-applied continuous-calibration corrections.
	void SetCalibrationContinuous(const Eigen::Quaterniond &rotation, const Eigen::Vector3d &translationMeters, double scale)
	{
		calibratedRotationQ = rotation.normalized();
		calibratedRotation = calibratedRotationQ.toRotationMatrix().eulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;
		calibratedTranslation = translationMeters * 100.0;
		calibratedScale = scale;
	}

	// The one sanctioned Euler -> quaternion conversion: an explicit user edit
	// in the profile editor.
	void RebuildRotationFromEuler()
	{
		Eigen::Vector3d e = calibratedRotation * EIGEN_PI / 180.0;
		calibratedRotationQ =
			Eigen::AngleAxisd(e(0), Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(e(1), Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(e(2), Eigen::Vector3d::UnitX());
	}

	Eigen::Vector3d TranslationMeters() const
	{
		return calibratedTranslation * 0.01;
	}

	void ClearSampleBuffers()
	{
		refSamples.clear();
		refSamples.shrink_to_fit();
		targetSamples.clear();
		targetSamples.shrink_to_fit();
	}

	void Clear()
	{
		chaperone.geometry.clear();
		chaperone.standingCenter = vr::HmdMatrix34_t();
		chaperone.playSpaceSize = vr::HmdVector2_t();
		chaperone.valid = false;
		chaperone.copyUnixTime = 0.0;
		chaperone.lastRestoreTime = 0.0;

		calibratedRotationQ = Eigen::Quaterniond(1, 0, 0, 0);
		calibratedRotation = Eigen::Vector3d();
		calibratedTranslation = Eigen::Vector3d();
		calibratedScale = 1.0;
		calibratedTimeOffset = 0.0;
		fieldAnchors.clear();
		fieldGeneration++;
		collectAsAnchor = false;
		// The mount extrinsic was derived from the calibration being cleared;
		// the enable/hide preferences and tracker pick survive like uiAdvanced.
		mountExtrinsic = questcal::MountExtrinsic();
		lastAutoCorrectionUnixTime = 0.0;
		autoCorrectionsApplied = 0;
		continuousState = 0;
		continuousDeviation = questcal::ContinuousAlignment::Deviation();
		continuousScatterRotDeg = 0.0;
		continuousScatterPosM = 0.0;
		calibrationUnixTime = 0.0;
		driftSlideEvents = 0;
		driftMaxSlideM = 0.0;
		discontinuousLossEvents = 0;
		driftScore = 0.0;
		alignment = AlignmentHealth::Fresh;
		lastResult = questcal::EngineResult();
		referenceTrackingSystem = "";
		targetTrackingSystem = "";
		enabled = false;
		validProfile = false;
		ClearSampleBuffers();
	}

	double CollectionSeconds()
	{
		switch (calibrationSpeed)
		{
		case FAST:
			return 10.0;
		case SLOW:
			return 20.0;
		case VERY_SLOW:
			return 35.0;
		}
		return 10.0;
	}

	struct Message
	{
		enum Type
		{
			String,
			Progress
		} type = String;

		Message(Type type) : type(type) { }

		std::string str;
		int progress, target;
	};

	std::vector<Message> messages;

	void Log(const std::string &msg)
	{
		if (messages.empty() || messages.back().type == Message::Progress)
			messages.push_back(Message(Message::String));

		messages.back().str += msg;
		AppendSessionLog(msg);
		std::cerr << msg;
	}

	void Progress(int current, int target)
	{
		if (messages.empty() || messages.back().type == Message::String)
			messages.push_back(Message(Message::Progress));

		messages.back().progress = current;
		messages.back().target = target;
	}
};

extern CalibrationContext CalCtx;

class PoseStreamHub;

void InitCalibrator();
void ShutdownCalibrator();
void CalibrationTick(double time);
void StartCalibration();
void StartAnchorCalibration();     // same collection; result becomes a field anchor
void LoadChaperoneBounds();
void ApplyChaperoneBounds();

// Shared pose-stream access for runtime monitors (jump detection, drift).
PoseStreamHub &GetPoseHub();
double GetQpcToSeconds();

// Dashboard overlay handle (0 until created); owned by QuestCalibrator.cpp.
vr::VROverlayHandle_t GetMainOverlayHandle();
