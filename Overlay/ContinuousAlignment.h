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
// driver slews them); large or tilted sustained deviations mean a bumped
// mount or a tracking fault and freeze auto-apply instead. Noisy windows only
// hold corrections until tracking settles — unless the scatter is structured
// (constant between neighboring observations yet large across the window),
// which is the slipped-mount signature and freezes. Tilt is never applied:
// both runtimes are gravity-aligned, so a growing tilt deviation is mount
// creep or noise, not universe drift.
//
// Pure Eigen + CalibrationEngine reuse; no OpenVR or UI dependencies, so the
// synthetic test harness compiles exactly the code the overlay ships. All side
// effects are polled out by the caller (same pattern as JumpDetector /
// DriftMonitor).

#include "CalibrationEngine.h"

#include <deque>
#include <functional>
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
		// universe jump (JumpDetector's business) — drop the window, never
		// "correct" it. Confirmation must come from an observation at least
		// jumpConfirmSpacing later: a glitched reference sample corrupts every
		// observation interpolated across it with correlated errors, and only
		// temporal separation makes the confirming observation independent ---
		double jumpGuardRotDeg = 2.0;
		double jumpGuardPosM = 0.05;
		double jumpGuardWindow = 0.5;      // s
		double jumpConfirmSpacing = 0.1;   // s

		// --- estimate sanity: window scatter (after trimming) above these
		// means the pair itself is untrustworthy — hold, don't correct. What
		// happens next depends on the scatter's structure. A slipped mount's
		// error rotates with the head: large across the window, but nearly
		// constant between consecutive observations — so window scatter far
		// above the short-term (consecutive-obs) noise estimate is a mount
		// fault and freezes, on a longer confirm than a stable deviation.
		// Unstructured scatter — window RMS explained by per-sample noise —
		// is degraded tracking (grazing lighthouse geometry while lying down,
		// partial occlusion) and only holds corrections until it settles;
		// freezing on it spammed "check the mount" at users whose mount was
		// fine every time they lay down ---
		double maxScatterRotDeg = 0.8;
		double maxScatterPosM = 0.02;
		double scatterFreezeConfirmSeconds = 8.0;   // structured scatter -> freeze
		double scatterNotifySeconds = 10.0;         // unstructured -> one info event
		double structuredScatterFactor = 1.6;       // window RMS vs noise estimate
		int    scatterVoteWindow = 8;               // sliding votes (evaluateInterval apart) behind the majority

		// --- apply policy ---
		double evaluateInterval = 2.0;     // s between decisions
		double deadbandYawDeg = 0.1;       // below both: noise floor, do nothing
		double deadbandPosM = 0.003;
		double maxStepYawDeg = 0.5;        // per-correction clamp
		double maxStepPosM = 0.01;
		double freezeYawDeg = 2.0;         // at/above (sustained): freeze + event
		double freezeTiltDeg = 1.5;        // tilt alone also freezes (mount slip)
		double freezePosM = 0.05;
		double freezeConfirmSeconds = 6.0;
		double resumeFactor = 0.5;         // unfreeze below freeze*factor ...
		double resumeConfirmSeconds = 5.0; // ... sustained this long
		double coastGapSeconds = 2.0;      // no fresh obs -> coasting

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
		Holding,    // observations too noisy to act on; resumes when they settle
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
			FrozenLargeDeviation,
			Resumed,
			TrackerLost,
			TrackerRecovered,
			ObservationsUnstable,   // sustained unstructured scatter; informational
		} type = FrozenLargeDeviation;
		Deviation deviation;   // populated for a deviation-path FrozenLargeDeviation
	};

	using ExpectedCalibrationAt = std::function<void(
		const Eigen::Vector3d &targetRawPos,
		Eigen::Quaterniond &rotationOut,
		Eigen::Vector3d &translationOut)>;

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
	double SecondsSinceObservation(double now) const
	{
		return lastObsTime > 0.0 ? now - lastObsTime : -1.0;
	}

	// Drop all windows and pending output (ring gap, accepted universe jump,
	// suspension). Keeps the extrinsic; a persisting deviation re-freezes
	// within freezeConfirmSeconds of resuming.
	void Reset();

	// Derive the mount extrinsic from a manual calibration's sample buffers
	// and its solved result. The per-pair spread doubles as the rigidity gate:
	// a tracker that was not rigid on the HMD fails it and `out` is left
	// invalid (the caller keeps any previous extrinsic).
	static bool DeriveMountExtrinsic(const std::vector<PoseSample> &refStream,
	                                 const std::vector<PoseSample> &targetStream,
	                                 const EngineResult &calibration,
	                                 const Config &config,
	                                 MountExtrinsic &out);

private:
	struct Observation
	{
		double time = 0.0;
		Eigen::Quaterniond rot{ 1, 0, 0, 0 };
		Eigen::Vector3d trans{ 0, 0, 0 };
		Eigen::Vector3d targetRawPos{ 0, 0, 0 };
	};

	void FormObservations(double calScale, double calTimeOffset);
	void TrimWindows(double now);
	bool EstimateWindow(const Eigen::Quaterniond &calRotation,
	                    const Eigen::Vector3d &calTranslationMeters,
	                    const ExpectedCalibrationAt &expectedAt,
	                    Eigen::Quaterniond &rotOut, Eigen::Vector3d &transOut);
	void Decide(double now, const Eigen::Quaterniond &calRotation,
	            const Eigen::Vector3d &calTranslationMeters,
	            const ExpectedCalibrationAt &expectedAt);
	void EnterState(State s);

	Config config;
	MountExtrinsic extrinsic;
	State state = State::Inactive;

	std::vector<PoseSample> refWindow;
	std::vector<PoseSample> targetWindow;
	std::deque<Observation> observations;
	size_t targetProcessed = 0;        // targetWindow prefix already turned into obs
	double lastObsTime = 0.0;          // sample clock
	double lastKeptObsTime = 0.0;      // thinning watermark
	double lastEvaluateTime = 0.0;

	Deviation deviation;
	double scatterRotRmsDeg = 0.0;
	double scatterPosRmsM = 0.0;
	// Short-term noise estimate (robust sigma from consecutive-obs deltas):
	// the unstructured part of the window scatter.
	double noiseRotDeg = 0.0;
	double noisePosM = 0.0;

	double freezeExceededSince = -1.0;
	double resumeBelowSince = -1.0;

	// Scatter episode bookkeeping: one timer, plus a SLIDING window of
	// structured/noise classification votes so a flickering classification
	// converges to its majority — sliding rather than whole-episode counters
	// so a slip that starts deep into a long degraded-tracking episode still
	// freezes within ~scatterVoteWindow evaluations instead of having to
	// outvote the entire episode's history.
	double scatterSince = -1.0;
	std::deque<char> scatterStructuredVotes;
	bool unstableNotified = false;

	// Jump-guard candidate: a discontinuous observation awaiting confirmation
	// by a second one before the window is dropped (vs a one-off glitch).
	bool pendingDiscontinuity = false;
	Observation pendingObs;

	bool hasPendingCorrection = false;
	Correction pendingCorrection;
	std::deque<Event> events;

	double lastLatencyEstimateTime = 0.0;
	bool hasPendingTimeOffset = false;
	double pendingTimeOffset = 0.0;
};

} // namespace questcal
