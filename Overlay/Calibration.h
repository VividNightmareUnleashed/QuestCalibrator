#pragma once

#include "CalibrationEngine.h"
#include "ContinuousAlignment.h"
#include "ProfileValidation.h"

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
	// Frozen with the tracking-system names when a collection starts. The UI's
	// live device panes may refresh/reselect while the modal is open, but an
	// in-flight solve must continue consuming the exact pair the user started.
	uint32_t calibrationReferenceID = 0xFFFFFFFF;
	uint32_t calibrationTargetID = 0xFFFFFFFF;

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
	bool notifyPoorCalibration = true;

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

	// The feature is armed only when the tracker pick and the mount offset
	// learned for that tracker are both present — picking a tracker in the
	// combo persists a serial and deliberately clears the extrinsic, so the two
	// halves are routinely out of step. Every consumer must ask the same
	// question: gating the driver-side hide on the weaker half displaced the
	// tracker out of every game while nothing maintained the alignment.
	bool ContinuousArmed() const
	{
		return continuousEnabled && mountExtrinsic.valid;
	}

	// Driver pose-channel health, refreshed every tick. Losing the ring parks
	// universe-jump compensation, drift staleness, continuous calibration and
	// chaperone universe verification, and drops collection back to tick-rate
	// runtime poses — all of it silently, so the UI reports it as a first-class
	// status instead of the overlay looking healthy with its monitors off.
	bool poseRingOpen = false;

	// Runtime alignment monitoring. The device masks mark reference/target
	// system devices (refreshed by the profile scan); the counters feed drift
	// staleness and the UI.
	bool referenceDeviceMask[vr::k_unMaxTrackedDeviceCount] = {};
	bool targetDeviceMask[vr::k_unMaxTrackedDeviceCount] = {};
	uint32_t jumpsCompensated = 0;
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
	// Universe rebases update both the calibration and the protected standing
	// center.  Keep the two registry records on one revision and retry the
	// Settings half if a partial write occurs.
	bool settingsSaveDirty = false;
	// Both records share one quiet-period clock: any persistent mutation restarts
	// the debounce, while the independent dirty bits retain partial-write state.
	double persistenceDirtyTime = 0.0;
	// When the current dirty streak began. A session that keeps correcting
	// never reaches a quiet period, so the debounce alone would defer both
	// records until shutdown and lose everything to a crash or a SteamVR kill.
	double persistenceFirstDirtyTime = 0.0;
	uint32_t persistenceRevision = 0;
	// A universe rebase writes the calibration and the protected standing
	// center as one revision: the Settings half must not land without the
	// Config half. Independent dirty bits carry no such ordering requirement.
	bool persistenceCoupled = false;
	// Older releases embedded global settings in Config.  Until their first
	// Settings write succeeds, SaveProfile must not replace that only copy.
	bool legacySettingsMigrationPending = false;
	// Missing, successfully loaded, and unreadable are deliberately closed
	// states. In particular, unreadable is not absence: automatic migration must
	// preserve that record for diagnosis instead of overwriting it with defaults.
	questcal::RecordLoadState profileLoadState = questcal::RecordLoadState::Missing;
	questcal::RecordLoadState settingsLoadState = questcal::RecordLoadState::Missing;

	std::string referenceTrackingSystem;
	std::string targetTrackingSystem;
	// Device-pane choices are candidates for the next base calibration.  They
	// deliberately stay separate from the active profile identity above so
	// browsing another tracking system cannot apply the old transform to it.
	std::string pendingReferenceTrackingSystem;
	std::string pendingTargetTrackingSystem;
	std::string calibrationReferenceTrackingSystem;
	std::string calibrationTargetTrackingSystem;

	bool enabled = false;
	bool validProfile = false;
	// Fail-closed latch (persisted with the profile): the profile's HMD universe
	// baseline changed while normal multi-device monitoring had no continuity.
	// Only a fresh base calibration can safely re-enable this profile, so the
	// latch must survive a restart — a session-only latch simply hands the
	// corrupted profile back at the next launch.
	bool profileUniverseUnsafe = false;
	// The reference universe the calibration was solved in: the headset that
	// owns it plus that headset's raw worldFromDriver, persisted with the
	// profile. The protected chaperone snapshot used to be the only carrier of
	// this, so a user who never protected a room got no continuity check at all.
	bool profileUniverseValid = false;
	std::string profileHmdSerial;
	Eigen::Quaterniond profileWorldFromDriverRotation{ 1, 0, 0, 0 };
	Eigen::Vector3d profileWorldFromDriverTranslation{ 0, 0, 0 };
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
		// A raw-space snapshot belongs to the HMD/runtime universe which
		// produced it.  Unknown ownership is deliberately not restorable.
		std::string ownerTrackingSystem;
		std::string ownerHmdSerial;
		// Raw-universe transform observed on the owning HMD when the snapshot
		// was last anchored. Persisting it lets the first pose after an app or
		// ring outage reveal a same-headset worldFromDriver rebase.
		bool worldFromDriverValid = false;
		Eigen::Quaterniond worldFromDriverRotation{ 1, 0, 0, 0 };
		Eigen::Vector3d worldFromDriverTranslation{ 0, 0, 0 };
		bool baselineVerifiedThisSession = false; // runtime only; never serialized
		// Quads are relative to the standing/play-area center (see
		// ChaperoneMath.h) — jump-invariant; only standingCenter re-anchors.
		std::vector<vr::HmdQuad_t> geometry;
		vr::HmdMatrix34_t standingCenter;
		vr::HmdVector2_t playSpaceSize;
		double copyUnixTime = 0.0;     // when the snapshot was taken (persisted)
		double lastRestoreTime = 0.0;  // last auto-restore attempt (runtime cooldown)
	} chaperone;

	void AdvancePersistenceRevision()
	{
		if (++persistenceRevision == 0)
			persistenceRevision = 1;
		persistenceCoupled = true;
	}

	bool HasDirtyPersistence() const
	{
		return profileSaveDirty || settingsSaveDirty;
	}

	// Starts the maximum-age clock only on the clean -> dirty transition, so a
	// stream of corrections cannot push the forced flush out indefinitely.
	void StartPersistenceDebounce(double now)
	{
		if (!HasDirtyPersistence())
			persistenceFirstDirtyTime = now;
		persistenceDirtyTime = now;
	}

	void MarkProfileDirty(double now)
	{
		StartPersistenceDebounce(now);
		profileSaveDirty = true;
	}

	void MarkSettingsDirty(double now)
	{
		StartPersistenceDebounce(now);
		settingsSaveDirty = true;
	}

	void MarkProfileAndSettingsDirty(double now)
	{
		StartPersistenceDebounce(now);
		profileSaveDirty = true;
		settingsSaveDirty = true;
	}

	void DelayPersistenceRetry(double now)
	{
		if (!HasDirtyPersistence())
			return;
		// Restart both clocks: a record that keeps refusing the write must retry
		// on the 5 s cadence, not once per tick because the ceiling has passed.
		persistenceDirtyTime = now;
		persistenceFirstDirtyTime = now;
	}

	void DisarmChaperone()
	{
		chaperone.valid = false;
		chaperone.autoApply = false;
		chaperone.ownerTrackingSystem.clear();
		chaperone.ownerHmdSerial.clear();
		chaperone.worldFromDriverValid = false;
		chaperone.baselineVerifiedThisSession = false;
		chaperone.worldFromDriverRotation = Eigen::Quaterniond::Identity();
		chaperone.worldFromDriverTranslation = Eigen::Vector3d::Zero();
		chaperone.geometry.clear();
	}

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
	// in the profile editor. The editor converts its own draft before
	// validating it, so this takes the angles and returns the quaternion
	// instead of touching the live members.
	static Eigen::Quaterniond RebuildRotationFromEuler(const Eigen::Vector3d &eulerDegrees)
	{
		Eigen::Vector3d e = eulerDegrees * EIGEN_PI / 180.0;
		return
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
		// Chaperone protection and global preferences are independent settings;
		// clearing a calibration must not silently disarm the room boundary.
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
		profileUniverseUnsafe = false;
		profileUniverseValid = false;
		profileHmdSerial.clear();
		profileWorldFromDriverRotation = Eigen::Quaterniond(1, 0, 0, 0);
		profileWorldFromDriverTranslation = Eigen::Vector3d::Zero();
		profileSaveDirty = false;
		timeLastScan = -1e9;
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
	size_t messageBytes = 0;
	// The pane is the bug-report surface for a GUI binary with no stderr, so it
	// keeps recent history — but the runtime monitors log for the whole session
	// and a driver that rebases at pose rate grows it at MB/minute. Bound it the
	// way AppendSessionLog bounds the file. Entries are capped too: without that
	// the whole session is one std::string, which nothing can trim and which
	// ImGui::TextWrapped re-wraps every frame while the modal is open.
	// StartCalibration clears the pane, so trimming only ever drops backlog the
	// calibration modal does not render.
	static constexpr size_t MessageEntryMaxBytes = 8 * 1024;
	static constexpr size_t MessagePaneMaxBytes = 256 * 1024;
	// Persistent banner for failures that occur outside the calibration modal
	// (registry/chaperone operations in the settings screen).
	enum class ErrorSource
	{
		None,
		General,
		Driver,
		ProfilePersistence,
		SettingsPersistence,
		Chaperone,
		ChaperoneMonitor
	};
	std::string uiError;
	ErrorSource uiErrorSource = ErrorSource::None;

	void ClearMessages()
	{
		messages.clear();
		messageBytes = 0;
	}

	void Log(const std::string &msg)
	{
		if (messages.empty() || messages.back().type == Message::Progress ||
			messages.back().str.size() >= MessageEntryMaxBytes)
			messages.push_back(Message(Message::String));

		messages.back().str += msg;
		messageBytes += msg.size();
		// Never drop the entry being appended to: the newest lines are the ones
		// a bug report is about.
		while (messageBytes > MessagePaneMaxBytes && messages.size() > 1)
		{
			messageBytes -= messages.front().str.size();
			messages.erase(messages.begin());
		}

		AppendSessionLog(msg);
		std::cerr << msg;
	}

	void ReportError(const std::string &msg, ErrorSource source = ErrorSource::General)
	{
		uiError = msg;
		uiErrorSource = source;
		while (!uiError.empty() && (uiError.back() == '\n' || uiError.back() == '\r'))
			uiError.pop_back();
		Log(msg);
	}

	void ClearError(ErrorSource source)
	{
		if (uiErrorSource != source)
			return;
		uiError.clear();
		uiErrorSource = ErrorSource::None;
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

void InitCalibrator();
void ShutdownCalibrator(bool cleanExit = true);
void CalibrationTick(double time);
bool StartCalibration();
bool StartAnchorCalibration();     // same collection; result becomes a field anchor
bool LoadChaperoneBounds();
bool ApplyChaperoneBounds(bool logSuccess = true);

// Push the live profile to the driver immediately. A UI action that changes
// what the driver should be applying must call this once its save succeeds:
// the runtime monitors read the local flags on the very next tick, so waiting
// for the periodic scan leaves them measuring against a calibration the driver
// is not applying yet.
void ResyncDriverState();

// Dashboard overlay handle (0 until created); owned by QuestCalibrator.cpp.
vr::VROverlayHandle_t GetMainOverlayHandle();
