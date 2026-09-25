#pragma once

// Continuous calibration from an HMD-mounted tracker.
//
// After a manual calibration performed with the tracker mounted (HMD as the
// reference device), the rigid mount extrinsic E — the tracker's pose in the
// HMD body frame — is derived from the same sample buffers. During play each
// time-aligned pose pair (HMD in the reference universe, tracker in the target
// universe) then yields a DIRECT observation of the universe calibration with
// no motion requirement:
//
//   C_obs = H o E o T_s^-1        (T_s = tracker pose with positions pre-scaled)
//
// A rolling window of observations is robust-averaged (hemisphere-safe
// eigenvector quaternion mean + trimmed translation mean) and compared to the
// current calibration. With a spatial field, each observation is first rebased
// through the field calibration expected at that observation's own target-raw
// position, so movement across anchor gradients never reads as drift. Small deviations are
// emitted as yaw+translation corrections for the caller to auto-apply (the
// driver slews them); large or tilted sustained deviations mean a tracking
// fault and freeze auto-apply instead. Noisy windows only hold corrections
// until tracking settles; scatter never freezes. Tilt is never applied: both
// runtimes are gravity-aligned, so a tilt deviation is tracker orientation
// bias or noise, not universe drift. The yaw-only correction pivots at the
// head, so the position there is still corrected in full: dropping the tilt
// about any other point leaves tilt x lever arm at the head uncorrected.
//
// Pure Eigen + CalibrationEngine reuse; no OpenVR or UI dependencies, so the
// synthetic test harness compiles exactly the code the overlay ships. All side
// effects are polled out by the caller (same pattern as JumpDetector /
// DriftMonitor).

#include "CalibrationEngine.h"

#include <array>
#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <vector>

namespace questcal
{

// Rigid mount extrinsic: the HMD-mounted tracker's pose expressed in the HMD
// body frame. A property of the physical mount only — independent of the
// calibration, so it survives recalibration as long as the mount holds.
struct MountExtrinsic
{
	bool valid = false;
	Eigen::Quaterniond rot{ 1, 0, 0, 0 };   // tracker frame -> HMD frame
	Eigen::Vector3d pos{ 0, 0, 0 };         // meters, in the HMD frame
	double rotRmsDeg = 0.0;                 // per-pair spread of the derivation
	double posRmsM = 0.0;                   // (doubles as the rigidity verdict)
	size_t pairs = 0;
};

class ContinuousAlignment
{
public:
	struct Config
	{
		// --- observation forming ---
		double maxInterpolationGap = 0.06; // s; engine convention
		double maxLinearSpeed = 1.5;       // m/s; residual timing error scales
		double maxAngularSpeed = 2.0;      // rad/s; with speed through the mount
		double obsMinSpacing = 0.10;       // s; thin observations to ~10 Hz
		double windowSeconds = 10.0;       // rolling estimate window
		size_t minObsForEstimate = 40;

		// --- discontinuity guard: an obs-to-obs step this large this fast is a
		// possible universe jump (JumpDetector's business). Verify an adjacent
		// raw-pose step before dropping the window. Confirmation comes at least
		// jumpConfirmSpacing later: a glitched reference sample corrupts every
		// observation interpolated across it with correlated errors, and only
		// temporal separation makes the confirming observation independent ---
		double jumpGuardRotDeg = 2.0;
		double jumpGuardPosM = 0.05;
		double jumpGuardWindow = 0.5;      // s
		double jumpConfirmSpacing = 0.1;   // s

		// --- estimate sanity: window scatter (after trimming) above these
		// means the pair is untrustworthy right now — hold, don't correct, and
		// resume once it settles. Scatter never freezes: headset trackers are
		// glued or bolted on, and what scatters in practice is lighthouse
		// tracking (grazing geometry while lying down, partial occlusion) ---
		double maxScatterRotDeg = 0.8;
		double maxScatterPosM = 0.02;
		double scatterNotifySeconds = 10.0;         // sustained scatter -> one info event

		// --- apply policy ---
		double evaluateInterval = 2.0;     // s between decisions
		double deadbandYawDeg = 0.1;       // below both: noise floor, do nothing
		double deadbandPosM = 0.003;
		double maxStepYawDeg = 0.5;        // per-correction clamp
		double maxStepPosM = 0.01;
		double freezeYawDeg = 2.0;         // at/above (sustained): freeze + event
		double freezeTiltDeg = 1.5;        // tilt alone also freezes
		double freezePosM = 0.05;
		double freezeConfirmSeconds = 6.0;
		double resumeFactor = 0.5;         // unfreeze below freeze*factor ...
		double resumeConfirmSeconds = 5.0; // ... sustained this long
		// ... or anywhere inside the freeze band, sustained this long. Tracking
		// corrects any deviation inside the band without freezing, so a freeze
		// whose cause went away must not demand more than that: without this,
		// drift that accrued while frozen (nothing corrects it then) could sit
		// between the two thresholds and hold the freeze for the whole session.
		double resumeInBandSeconds = 30.0;
		// A freeze confirmed this soon after the target's own tracking restarted
		// (NoteTargetResolved) is reported as that, not as universe drift.
		double resolveAttributionSeconds = 30.0;
		double coastGapSeconds = 2.0;      // no fresh obs -> coasting
		double maxStreamGapSeconds = 0.2; // never estimate across a tracking hiatus

		// --- extrinsic derivation (rigidity gate) ---
		size_t extrinsicMinPairs = 100;
		double extrinsicMaxRotRmsDeg = 0.5;
		double extrinsicMaxPosRmsM = 0.010;

		// --- opt-in online time-offset re-estimation: a rigid pair's two
		// angular-speed profiles are identical up to latency, so the engine's
		// cross-correlator applies unmodified. Off by default; when off the
		// buffers stay short and nothing is measured ---
		bool latencyReestimation = false;
		double latencyWindowSeconds = 20.0;   // correlation window
		double latencyIntervalSeconds = 60.0; // cadence while Tracking
	};

	enum class State
	{
		Inactive,   // no valid extrinsic or not enough observations yet
		Tracking,   // fresh estimate available, corrections flowing
		Coasting,   // tracker occluded/off; calibration holds, resumes cleanly
		Frozen,     // sustained large deviation; nothing applied until resolved
		Holding,    // observations too noisy, or the target's tracking unsettled; resumes when they settle
	};

	// Left delta over the current calibration: newCal = D o oldCal. Rotation is
	// yaw-only by construction; already clamped to the per-correction step.
	struct Correction
	{
		Eigen::Quaterniond rotation{ 1, 0, 0, 0 };
		Eigen::Vector3d translation{ 0, 0, 0 };
	};

	// Windowed-estimate deviation vs the current calibration. posM is the
	// effective displacement at the user's head — a yaw delta far from the
	// origin has a huge raw translation but a small effect at the user.
	struct Deviation
	{
		bool valid = false;
		double yawDeg = 0.0;
		double tiltDeg = 0.0;
		double posM = 0.0;
	};

	struct Event
	{
		enum Type
		{
			FrozenLargeDeviation,   // sustained deviation vs the calibration
			Resumed,
			TrackerLost,
			TrackerRecovered,
			ObservationsUnstable,   // sustained scatter; informational
		} type = FrozenLargeDeviation;
		// Each event carries its own evidence, so one message never needs a
		// second data source: the deviation for FrozenLargeDeviation, the
		// window scatter for ObservationsUnstable.
		Deviation deviation;
		double scatterRotDeg = 0.0;
		double scatterPosM = 0.0;
		// FrozenLargeDeviation: confirmed within resolveAttributionSeconds of a
		// NoteTargetResolved, so the target's tracking restart is the likely
		// cause and the next restart the likely cure.
		bool afterTargetResolve = false;
	};

	using ExpectedCalibrationAt = std::function<void(
		const Eigen::Vector3d &targetRawPos,
		Eigen::Quaterniond &rotationOut,
		Eigen::Vector3d &translationOut)>;

	enum class ResetReason { Requested, StreamGap, UniverseJump, Suspended, ModeChanged, TargetResolved, Count };

	// Counters last for this engine's lifetime, including across Reset(). Window
	// sizes and timestamps in GetDiagnostics() describe the current window only.
	struct Diagnostics
	{
		uint64_t referenceOutOfOrder = 0, targetOutOfOrder = 0;
		// The part of each out-of-order count that repeated the previous
		// timestamp exactly, as opposed to stepping back past it.
		uint64_t referenceSameTime = 0, targetSameTime = 0;
		uint64_t referenceSpeedRejected = 0, targetSpeedRejected = 0;
		uint64_t referenceTooOld = 0, interpolationRejected = 0;
		uint64_t referenceWaitUpdates = 0;
		uint64_t jumpGuardRejected = 0, jumpGuardResets = 0;
		uint64_t observationsFormed = 0, observationsKept = 0;
		std::array<uint64_t, static_cast<size_t>(ResetReason::Count)> resets{};
		size_t referenceSamples = 0, targetSamples = 0, pendingTargets = 0;
		size_t observations = 0, requiredObservations = 0;
		double lastObservationTime = 0.0;
	};
	Diagnostics GetDiagnostics() const;

	void SetConfig(const Config &c) { config = c; }
	void SetExtrinsic(const MountExtrinsic &e) { extrinsic = e; }
	const MountExtrinsic &Extrinsic() const { return extrinsic; }
	void SetLatencyReestimation(bool on) { config.latencyReestimation = on; }

	// Feed raw-universe poses (already composed from ring samples and validity
	// gated by the caller). Reference = the HMD, target = the mounted tracker.
	void PushReference(const PoseSample &s);
	void PushTarget(const PoseSample &s);

	// Consume pushed samples, form observations, and (at the configured
	// cadence) estimate + decide. `now` is on the samples' clock. The current
	// calibration is passed in every call so deviation is always measured
	// against the caller-owned truth, including our own applied corrections.
	// When supplied, expectedAt evaluates the local field calibration for every
	// retained observation rather than only for the latest tracker position.
	void Update(double now, const Eigen::Quaterniond &calRotation,
	            const Eigen::Vector3d &calTranslationMeters,
	            double calScale, double calTimeOffset,
	            const ExpectedCalibrationAt &expectedAt = ExpectedCalibrationAt());

	bool PollCorrection(Correction &out);
	// The most recent decision still permits a correction. Unlike the one-shot
	// output, this remains true between evaluations and revokes queued approval
	// immediately when a later decision settles or starts confirming a fault.
	bool CorrectionEligible() const { return correctionEligible; }
	bool PollEvent(Event &out);

	// Freshly measured inter-system latency (seconds, positive = target lags
	// reference), produced at most once per latencyIntervalSeconds while
	// Tracking with the opt-in on. The caller owns filtering/clamping.
	bool PollTimeOffset(double &out);

	State GetState() const { return state; }
	Deviation CurrentDeviation() const { return deviation; }
	double ScatterRotRmsDeg() const { return scatterRotRmsDeg; }
	double ScatterPosRmsM() const { return scatterPosRmsM; }
	size_t ObservationCount() const { return observations.size(); }

	// Drop all windows and pending output (ring gap, accepted universe jump,
	// suspension). Keeps the extrinsic; a persisting deviation re-freezes
	// after fresh evidence. StreamGap preserves Frozen and Coasting states.
	void Reset(ResetReason reason = ResetReason::Requested);

	// The target's own tracking restarted at `time` (a lighthouse device began
	// a new solution, or regained or lost a base station): its pose before and
	// after can differ by centimeters, so no window may straddle the restart.
	// A Reset that keeps Frozen and Coasting like a stream gap, plus the time
	// for the freeze attribution.
	void NoteTargetResolved(double time);

	// True while the target's tracking says its pose is not settled (fewer
	// than two base stations in view, or moments after a restart). No verdict
	// is drawn from such a pose: no correction, no freeze, no resume; the
	// state shows Holding unless it is already Frozen.
	void SetTargetSettling(bool settling) { targetSettling = settling; }

	// Derive the mount extrinsic from a manual calibration's sample buffers
	// and its solved (valid) result. The per-pair spread doubles as the
	// rigidity gate: a tracker that was not rigid on the HMD fails it and `out`
	// is left untouched (the caller keeps any previous extrinsic). The gates
	// are fixed policy, so this takes no config.
	static bool DeriveMountExtrinsic(const std::vector<PoseSample> &refStream,
	                                 const std::vector<PoseSample> &targetStream,
	                                 const EngineResult &calibration,
	                                 MountExtrinsic &out);

private:
	struct Observation
	{
		double time = 0.0;
		Eigen::Quaterniond rot{ 1, 0, 0, 0 };
		Eigen::Vector3d trans{ 0, 0, 0 };
		Eigen::Vector3d targetRawPos{ 0, 0, 0 };
	};

	// One window's estimate: the robust average plus its scatter. Decide
	// publishes it to the members only on success.
	struct WindowEstimate
	{
		Eigen::Quaterniond rot{ 1, 0, 0, 0 };
		Eigen::Vector3d trans{ 0, 0, 0 };
		double scatterRotDeg = 0.0;
		double scatterPosM = 0.0;
	};

	void FormObservations(double calScale, double calTimeOffset);
	void TrimWindows(double now);
	void CompactWindows();
	bool EstimateWindow(const Eigen::Quaterniond &calRotation,
	                    const Eigen::Vector3d &calTranslationMeters,
	                    const ExpectedCalibrationAt &expectedAt,
	                    WindowEstimate &out) const;
	void Decide(double now, const Eigen::Quaterniond &calRotation,
	            const Eigen::Vector3d &calTranslationMeters,
	            const ExpectedCalibrationAt &expectedAt);
	// The estimate's deviation from the calibration at the head, and the yaw
	// angle and head displacement a correction toward it would apply.
	Deviation MeasureDeviation(const WindowEstimate &est,
	                           const Eigen::Quaterniond &calRotation,
	                           const Eigen::Vector3d &calTranslationMeters,
	                           double &yawAngleOut, Eigen::Vector3d &headStepOut,
	                           Eigen::Vector3d &headPosOut) const;
	void EnterState(State s);
	void ClearConfirmMarks();

	Config config;
	Diagnostics diagnostics;
	MountExtrinsic extrinsic;
	State state = State::Inactive;

	std::vector<PoseSample> refWindow;
	std::vector<PoseSample> targetWindow;
	// Expired-prefix cursors: a tick retires a handful of samples out of
	// thousands, so consumers work off the live range [head, size) and the
	// dead prefix is compacted away only when it is worth one move.
	size_t refHead = 0;
	size_t targetHead = 0;
	std::deque<Observation> observations;
	size_t targetProcessed = 0;        // targetWindow prefix already turned into obs
	double lastObsTime = 0.0;          // sample clock
	double lastKeptObsTime = 0.0;      // thinning watermark
	double lastEvaluateTime = 0.0;

	Deviation deviation;
	double scatterRotRmsDeg = 0.0;
	double scatterPosRmsM = 0.0;

	double freezeExceededSince = -1.0;
	double resumeBelowSince = -1.0;
	double resumeInBandSince = -1.0;

	bool targetSettling = false;
	double lastTargetResolveTime = -1e9;

	// How long the degraded episode has been running. It survives a coast:
	// the degraded tracking it reports is exactly what produces the gaps.
	// Only the settle path and Reset clear it.
	double scatterEpisodeSince = -1.0;
	bool unstableNotified = false;

	// Jump-guard candidate: a discontinuous observation awaiting confirmation
	// by a second one before the window is dropped (vs a one-off glitch).
	std::optional<Observation> pendingObs;

	std::optional<Correction> pendingCorrection;
	bool correctionEligible = false;
	std::deque<Event> events;

	double lastLatencyEstimateTime = 0.0;
	std::optional<double> pendingTimeOffset;
};

} // namespace questcal
