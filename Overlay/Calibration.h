#pragma once

#include "CalibrationEngine.h"
#include "CalibrationRun.h"
#include "ContinuousAlignment.h"
#include "ContinuousCorrectionGate.h"
#include "PersistenceState.h"
#include "ProfileValidation.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <openvr.h>

#include <ctime>
#include <deque>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <utility>
#include <vector>

enum class CalibrationState
{
	None,
	Begin,
	Neutralizing,
	Collecting,   // both device streams recorded; solve runs at the end
	Editing,
};

// Canonical live transform. Runtime, persistence, field math, and continuous
// calibration all consume this value directly; centimeters and Euler angles
// exist only at the UI boundary.
struct CalibrationTransform
{
	Eigen::Quaterniond rotation{ 1, 0, 0, 0 };
	Eigen::Vector3d translationMeters{ 0, 0, 0 };
	double scale = 1.0;
	double timeOffset = 0.0;

	Eigen::Vector3d RotationEulerDegrees() const
	{
		return rotation.toRotationMatrix().canonicalEulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;
	}
};

// Everything invalidated when the active calibration profile is discarded.
// CalibrationContext keeps preferences, device choices, chaperone protection,
// persistence scheduling, and monotonic driver generations outside this value.
struct CalibrationProfileState
{
	CalibrationTransform transform;
	// The target-device timeline shift most recently requested from the driver.
	double appliedTimeOffset = 0.0;

	// Absolute local solves; driver deltas are derived against `transform`.
	struct FieldAnchor
	{
		Eigen::Vector3d position{ 0, 0, 0 };        // target centroid in reference space
		Eigen::Quaterniond rotation{ 1, 0, 0, 0 };  // absolute solve at this position
		Eigen::Vector3d translationMeters{ 0, 0, 0 };
	};
	std::vector<FieldAnchor> fieldAnchors;

	questcal::MountExtrinsic mountExtrinsic;
	// Runtime mirrors and evidence derived from this profile.
	questcal::ContinuousCorrectionGate continuousCorrectionGate;
	uint32_t continuousTrackerId = 0xFFFFFFFF;
	double lastAutoCorrectionUnixTime = 0.0;
	uint32_t autoCorrectionsApplied = 0;
	questcal::ContinuousAlignment::State continuousState =
		questcal::ContinuousAlignment::State::Inactive;
	questcal::ContinuousAlignment::Deviation continuousDeviation;
	double continuousScatterRotDeg = 0.0;
	double continuousScatterPosM = 0.0;

	uint32_t jumpsCompensated = 0;
	uint32_t referenceGapEvents = 0;
	double calibrationUnixTime = 0.0;
	uint32_t driftSlideEvents = 0;
	double driftMaxSlideM = 0.0;
	uint32_t discontinuousLossEvents = 0;
	double driftScore = 0.0;
	enum class AlignmentHealth { Fresh, Aging, Stale };
	AlignmentHealth alignment = AlignmentHealth::Fresh;

	std::string referenceTrackingSystem;
	std::string targetTrackingSystem;
	bool enabled = false;
	enum class DisableReason
	{
		None,
		InvalidIdentity,
		InvalidTransform,
		HmdMismatch,
		DriverUnreachable,
		UniverseUnsafe,
	};
	DisableReason disableReason = DisableReason::None;
	bool validProfile = false;
	// Physical HMD and raw universe in which `transform` was solved.
	bool profileUniverseUnsafe = false;
	bool profileUniverseValid = false;
	std::string profileHmdSerial;
	Eigen::Quaterniond profileWorldFromDriverRotation{ 1, 0, 0, 0 };
	Eigen::Vector3d profileWorldFromDriverTranslation{ 0, 0, 0 };
	questcal::EngineResult lastResult;

	void ResetProfile() { *this = CalibrationProfileState{}; }
};

// Session log file (%LOCALAPPDATA%\QuestCalibrator\QuestCalibrator.log): the
// in-app message pane persisted for bug reports — the Release build is a GUI
// binary, so stderr goes nowhere. One fresh file per session; the previous
// session survives as QuestCalibrator.prev.log (one generation only) and a
// hard size cap bounds a runaway session, so disk use can never grow.
void InitSessionLog();
void AppendSessionLog(const std::string &msg);

// Which loop keeps the calibration true during play. Quest is
// QuestCalibrator's own model (a measured mount offset, corrections from
// every pose); Legacy is the verbatim port of OpenVR-SpaceCalibrator's, which
// re-solves from motion. Persisted with the profile.
enum class ContinuousMode { Quest = 0, Legacy = 1 };

struct CalibrationContext : CalibrationProfileState
{
	CalibrationState state = CalibrationState::None;
	uint32_t referenceID = 0xFFFFFFFF, targetID = 0xFFFFFFFF;
	// Every transient input and temporary mutation belongs to one run. Keeping
	// these together makes abort, solve failure, and shutdown share one cleanup.
	questcal::CalibrationRun run;

	// Runtime latency re-prediction: when enabled, the solved time offset is
	// applied to target devices via ComputeAppliedTimeOffset. The manual
	// override injects an explicit poseTimeOffset shift instead (milliseconds,
	// bypassing the solved value) — the live sign-pinning spike tool.
	bool applyTimeOffset = true;
	bool useManualTimeOffset = false;
	double manualTimeOffsetMs = 0.0;

	// Playspace scale solving is opt-in: streamed reference poses are motion-
	// smoothed, which under-reports calibration motion and biases the solved
	// scale low by several percent (varying with motion speed) — far larger
	// than the sub-percent genuine metric difference it could correct.
	bool solveScale = false;

	// UI preference (persisted): show raw solver/drift stats instead of the
	// plain-language calibration rating.
	bool uiAdvanced = false;
	bool notifyPoorCalibration = true;

	// Extra detail for bug reports (continuous-loop decisions, solve numbers),
	// written to the session log only while on. Off by default: the log is
	// bounded and the detail is per second.
	bool detailedLogging = false;  // persisted setting

	// Network access and package downloads are opt-in. Installation remains an
	// explicit action because Steam must be closed and Windows must approve the
	// elevated package installer.
	bool automaticUpdates = false;  // persisted setting

	void Diag(const std::string &msg)
	{
		if (detailedLogging)
			AppendSessionLog("diag: " + msg);
	}

	// Player-given device names keyed by serial (persisted in Settings): six
	// identical "VIVE Tracker 3.0" rows are told apart by hex serial otherwise.
	static constexpr size_t DeviceNameMaxBytes = 32;
	static constexpr size_t DeviceNameMaxCount = 64;
	std::map<std::string, std::string> deviceNames;

	// One-time drift warning shown before the first chaperone protect.
	// Top-level (not in Chaperone) so it survives Clear() and persists even
	// after the snapshot is discarded.
	bool chaperoneWarningAck = false;

	bool fieldEnabled = true;
	uint32_t fieldGeneration = 0;      // driver-side smoothing snaps when this changes

	const std::vector<FieldAnchor> &ActiveFieldAnchors() const
	{
		static const std::vector<FieldAnchor> none;
		return fieldEnabled ? fieldAnchors : none;
	}

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
	bool continuousRequireTrigger = false;       // persisted; confirm corrections manually
	bool hideMountedTracker = true;              // persisted; displace from games
	ContinuousMode continuousMode = ContinuousMode::Quest;  // persisted; which loop runs

	// The feature is armed only when the tracker pick and the mount offset
	// learned for that tracker are both present — picking a tracker in the
	// combo persists a serial and deliberately clears the extrinsic, so the two
	// halves are routinely out of step. Every consumer must ask the same
	// question: gating the driver-side hide on the weaker half displaced the
	// tracker out of every game while nothing maintained the alignment.
	// The legacy loop measures its own tracker offset, so for it the pick
	// alone arms the feature.
	bool ContinuousArmed() const
	{
		if (!continuousEnabled)
			return false;
		if (continuousMode == ContinuousMode::Legacy)
			return !continuousTrackerSerial.empty();
		return mountExtrinsic.valid;
	}

	// Driver pose-channel health, refreshed every tick. Losing the ring parks
	// universe-jump compensation, drift staleness, continuous calibration and
	// chaperone universe verification, and drops collection back to tick-rate
	// runtime poses — all of it silently, so the UI reports it as a first-class
	// status instead of the overlay looking healthy with its monitors off.
	bool poseRingOpen = false;
	uint32_t driverPoseHookMask = 0;

	// Runtime alignment monitoring. The device masks mark reference/target
	// system devices (refreshed by the profile scan); the counters feed drift
	// staleness and the UI.
	bool referenceDeviceMask[vr::k_unMaxTrackedDeviceCount] = {};
	bool targetDeviceMask[vr::k_unMaxTrackedDeviceCount] = {};
	// Debounced persistence for runtime compensation updates: dirty records save
	// after a quiet period and always on shutdown. See PersistenceState.h — the
	// rules live with the data rather than as loose fields here.
	questcal::PersistenceState persistence;
	// Missing, successfully loaded, and unreadable are deliberately closed
	// states. In particular, unreadable is not absence: automatic migration must
	// preserve that record for diagnosis instead of overwriting it with defaults.
	questcal::RecordLoadState profileLoadState = questcal::RecordLoadState::Missing;
	questcal::RecordLoadState settingsLoadState = questcal::RecordLoadState::Missing;

	// Device-pane choices are candidates for the next base calibration.  They
	// deliberately stay separate from the active profile identity above so
	// browsing another tracking system cannot apply the old transform to it.
	std::string pendingReferenceTrackingSystem;
	std::string pendingTargetTrackingSystem;

	double timeLastTick = 0, timeLastScan = 0;
	double wantedUpdateInterval = 1.0;
	static constexpr double ContinuousInputInterval = 0.05;

	bool ContinuousShouldRun() const
	{
		return state == CalibrationState::None && enabled && validProfile &&
			poseRingOpen && ContinuousArmed() &&
			continuousTrackerId < vr::k_unMaxTrackedDeviceCount &&
			referenceDeviceMask[vr::k_unTrackedDeviceIndex_Hmd];
	}

	double IdleUpdateInterval() const
	{
		// The dashboard's visibility must not throttle jump/drift detection or
		// protected-boundary rebasing. Device scans retain their one-second gate.
		return (enabled && validProfile) || (chaperone.valid && chaperone.autoApply)
			? ContinuousInputInterval : 1.0;
	}

	enum Speed
	{
		FAST = 0,
		SLOW = 1,
		VERY_SLOW = 2
	};
	Speed calibrationSpeed = FAST;

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

	// See PersistenceState.h for the revision rule, the debounce and the retry.

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
		// Queued deltas were measured against the previous transform.
		continuousCorrectionGate.Clear();
		transform.rotation = rotation.normalized();
		transform.translationMeters = translationMeters;
		transform.scale = scale;
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

	// Persisted profile mutations are transactions owned by Configuration.cpp
	// (SaveProfileFieldEdit / SaveProfileTransformEdit), never by this struct:
	// they persist a candidate record and only then apply it, so a refused write
	// leaves live state untouched. A mutate-then-save helper here would have to
	// roll back on failure, and no rollback held by a caller can undo what a
	// failed write leaves behind in the persistence layer — which is why there
	// is deliberately no such helper to add the next toggle to.

	void Clear()
	{
		ResetProfile();
		fieldGeneration++;
		persistence.OnProfileDiscarded();
		timeLastScan = -1e9;
		run.Reset();
	}

	double CollectionSeconds() const
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

	// One entry in the calibration pane. The kind separates what a player must
	// read from what a bug report needs: the modal always renders Instruction,
	// Info, Headline and Action, shows Detail only behind its details toggle,
	// and the session log receives every kind.
	struct Message
	{
		enum Kind
		{
			Detail,       // engineer text: ids, serials, residuals; log-first
			Info,         // a sentence written for the player
			Instruction,  // what to do right now, rendered large
			Headline,     // outcome headline ("Calibration stopped.")
			Action,       // what to do next, in the action colour
			Progress
		} kind = Detail;

		explicit Message(Kind kind) : kind(kind) { }

		std::string str;
		int progress = 0, target = 0;
	};

	std::vector<Message> messages;
	size_t messageBytes = 0;

	// The main screen's activity feed: the last few player-facing lines from
	// the runtime monitors, so a freeze, a resume or a pending correction can
	// reach someone who is not inside the calibration modal (which is the only
	// place the pane itself renders).
	enum class Tone { Neutral, Good, Warn, Bad };
	struct ActivityEntry
	{
		double unixTime = 0.0;
		Tone tone = Tone::Neutral;
		std::string text;
	};
	static constexpr size_t ActivityMax = 6;
	std::deque<ActivityEntry> activity;

	// What the modal should show a picture of after a run: the user error the
	// refused solve or the stop reason points at, or success. None for
	// environmental stops and cancellations, which get no picture.
	enum class GuideHint
	{
		None,
		Success,
		RotateMore,       // not enough rotation
		TwoAxes,          // rotation about a single axis
		HoldTogether,     // the pair did not move as one
		SlowDown,         // motion too fast for the sample rate
		KeepTracking,     // latency could not be measured
		TrackingLost,     // a device stopped tracking
		WrongPick,        // the reference must be on the headset's system
		WaitForTracking,  // the headset re-centred mid-run
	};
	GuideHint lastRunHint = GuideHint::None;
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
	static constexpr size_t UiErrorMaxBytes = 8 * 1024;
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

	// Technical line: ids, serials, residuals, monitor evidence. Coalesces into
	// the pane's current Detail entry and always reaches the session log.
	void Log(const std::string &msg)
	{
		size_t offset = 0;
		while (offset < msg.size())
		{
			if (messages.empty() || messages.back().kind != Message::Detail ||
				messages.back().str.size() >= MessageEntryMaxBytes)
				messages.push_back(Message(Message::Detail));

			size_t count = (std::min)(MessageEntryMaxBytes - messages.back().str.size(),
				msg.size() - offset);
			messages.back().str.append(msg, offset, count);
			messageBytes += count;
			offset += count;
		}
		TrimPane();

		AppendSessionLog(msg);
		std::cerr << msg;
	}

	// A sentence for the player: its own pane entry, the activity feed on the
	// main screen, and the session log.
	void Tell(const std::string &msg, Tone tone = Tone::Neutral)
	{
		PushEntry(Message::Info, msg);
		PushActivity(msg, tone);
		AppendSessionLog(msg);
		std::cerr << msg;
	}

	// What to do right now (rendered large in the modal).
	void Instruct(const std::string &msg)
	{
		PushEntry(Message::Instruction, msg);
		AppendSessionLog(msg);
	}

	// A player-facing sentence for the pane only: guidance that belongs next
	// to the instruction, not in the main screen's activity feed.
	void Note(const std::string &msg)
	{
		PushEntry(Message::Info, msg);
		AppendSessionLog(msg);
	}

	// How a run ended: a headline everyone reads, what happened in the
	// player's words, an action line when there is something to do, and the
	// engineer's reason behind the details toggle.
	void Outcome(const std::string &headline, const std::string &body,
		const std::string &action, const std::string &detail, Tone tone = Tone::Neutral)
	{
		PushEntry(Message::Headline, headline);
		if (!body.empty())
			PushEntry(Message::Info, body);
		if (!action.empty())
			PushEntry(Message::Action, action);
		if (!detail.empty())
			PushEntry(Message::Detail, detail);
		PushActivity(body.empty() ? headline : headline + ": " + body, tone);
		std::string line = headline;
		if (!body.empty())
			line += ": " + body;
		if (!action.empty())
			line += " " + action;
		if (!detail.empty())
			line += " [" + detail + "]";
		AppendSessionLog(line);
		std::cerr << line << "\n";
	}

	void PushActivity(const std::string &text, Tone tone)
	{
		ActivityEntry entry;
		entry.unixTime = static_cast<double>(std::time(nullptr));
		entry.tone = tone;
		entry.text = text;
		while (!entry.text.empty() && (entry.text.back() == '\n' || entry.text.back() == '\r'))
			entry.text.pop_back();
		activity.push_back(std::move(entry));
		while (activity.size() > ActivityMax)
			activity.pop_front();
	}

	void PushEntry(Message::Kind kind, const std::string &text)
	{
		Message entry(kind);
		entry.str.assign(text, 0, (std::min)(text.size(), MessageEntryMaxBytes));
		while (!entry.str.empty() && (entry.str.back() == '\n' || entry.str.back() == '\r'))
			entry.str.pop_back();
		messageBytes += entry.str.size();
		messages.push_back(std::move(entry));
		TrimPane();
	}

	// Never drop the entry being appended to: the newest lines are the ones a
	// bug report is about.
	void TrimPane()
	{
		while (messageBytes > MessagePaneMaxBytes && messages.size() > 1)
		{
			messageBytes -= messages.front().str.size();
			messages.erase(messages.begin());
		}
	}

	void ReportError(const std::string &msg, ErrorSource source = ErrorSource::General)
	{
		uiError.assign(msg, 0, (std::min)(msg.size(), UiErrorMaxBytes));
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
		if (messages.empty() || messages.back().kind != Message::Progress)
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
// Stops a run the user no longer wants. Not a failure: the outcome says so.
void CancelCalibration();
bool LoadChaperoneBounds();
bool ApplyChaperoneBounds(bool logSuccess = true);

// Push the live profile to the driver immediately. A UI action that changes
// what the driver should be applying must call this once its save succeeds:
// the runtime monitors read the local flags on the very next tick, so waiting
// for the periodic scan leaves them measuring against a calibration the driver
// is not applying yet.
void ResyncDriverState();

// How this layer raises a VR toast. The shell installs it at startup: the
// notification target is a presentation-layer resource (the dashboard overlay)
// created and owned by QuestCalibrator.cpp, and reaching up for its handle was
// the calibration domain's only dependency on the app shell. A sink that was
// never installed degrades to log-only, exactly as a not-yet-created overlay
// handle did — the log line runs first and unconditionally either way.
void SetToastSink(std::function<void(const char *)> sink);
