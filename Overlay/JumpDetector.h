#pragma once

#include "RingPoseMath.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

// Universe-jump detection over the reference system's pose stream.
//
// When the Quest runtime recenters or re-localizes, its whole universe shifts
// in one frame. Two detection paths:
//
//  - Exact: the reference driver re-bases worldFromDriver while the
//    driver-local pose stays continuous. The delta is then exactly
//    D = new_wfd ∘ old_wfd⁻¹, per device, no heuristics.
//  - Heuristic fallback: the composed world pose jumps inconsistently with
//    reported velocity. D is estimated from short velocity-compensated
//    windows on both sides of the discontinuity, never a single frame pair.
//
// Accepted deltas are constrained to yaw + translation (a recenter preserves
// gravity). On the exact path the discarded tilt magnitude is reported as the
// non-rigid residual; on the heuristic path only a scalar heading is
// regressed, so residualTiltRad stays at its default there. Nothing consumes
// either residual today - they are reported for diagnostics, and drift
// staleness is scored from age, slide events and loss events alone. Times are
// ring-sample seconds (QPC * QpcToSeconds), not UI time.
class JumpDetector
{
public:
	struct Config
	{
		double wfdRotEpsRad = 1e-5;        // wfd rotation change that counts as a rebase
		double wfdTransEps = 1e-4;         // meters
		double discontinuityPos = 0.20;    // meters of unexplained motion in one frame pair
		double discontinuityYawRad = 5.0 * 3.14159265358979 / 180.0;
		double maxFrameGap = ringpose::MaxAdjacentFrameSeconds;
		double localContinuityPos = ringpose::MaxLocalPositionErrorMeters;
		double localContinuityRotRad = ringpose::MaxLocalRotationErrorRadians;
		double window = 0.2;               // seconds of history on each side of a candidate
		double agreeWindow = 0.25;         // seconds within which devices must agree
		double agreePosTol = 0.10;         // meters between per-device delta translations
		double agreeYawTolRad = 3.0 * 3.14159265358979 / 180.0;
		double soloPosThreshold = 0.30;    // single-device heuristic acceptance floor
		double soloYawThresholdRad = 10.0 * 3.14159265358979 / 180.0;
		double retriggerHold = 1.0;        // seconds; gates re-triggering, never application
		double gapSeconds = 2.0;           // reference stream gap => no compensation, event only
	};

	// A universe delta to fold into the calibration: newCal = D ∘ oldCal.
	struct UniverseDelta
	{
		Eigen::Quaterniond rotation{ 1, 0, 0, 0 };   // yaw-only
		Eigen::Vector3d translation{ 0, 0, 0 };
		double time = 0.0;                 // ring seconds at the jump
		bool exact = false;                // wfd-rebase path vs heuristic
		int devicesAgreeing = 0;
		double residualTiltRad = 0.0;      // non-rigid part discarded by the yaw constraint
		double residualSpread = 0.0;       // meters; disagreement between devices (heuristic)
		// Absolute HMD worldFromDriver endpoint captured by this exact sample.
		// `exact` is the validity discriminator; heuristic deltas leave identity.
		Eigen::Quaterniond worldFromDriverRotation{ 1, 0, 0, 0 };
		Eigen::Vector3d worldFromDriverTranslation{ 0, 0, 0 };
	};

	struct GapEvent
	{
		uint32_t deviceId = 0;
		double duration = 0.0;             // seconds without valid reference samples
		double time = 0.0;
	};

	explicit JumpDetector(double qpcToSeconds) : qpcToSeconds(qpcToSeconds) { }

	void SetConfig(const Config &c) { config = c; }

	// Feed one reference-system sample (any device of the reference system,
	// in ring order). Non-reference devices must not be pushed.
	void Push(const protocol::DevicePoseSample &sample);

	// An accepted jump, if one is ready. Call until it returns false.
	bool PollDelta(UniverseDelta &out);

	// A reference-stream gap, if one occurred. Call until it returns false.
	bool PollGap(GapEvent &out);

	// Human-readable notes (wfd rebases seen, candidates expired, ...) for the
	// calibration log; drained by the caller.
	bool PollNote(std::string &out);

	// Drop all per-device state (calibration started, monitors disabled, ...).
	void Reset();

private:
	struct Hist
	{
		double t;
		Eigen::Vector3d pos;               // composed world position
		Eigen::Vector3d vel;               // world-rotated velocity
		double yaw;                        // composed world heading
		double yawRate;                    // rad/s around world +Y
	};

	struct Candidate
	{
		uint32_t deviceId = 0;
		double t = 0.0;
		bool exact = false;
		bool ready = false;
		bool dead = false;
		Eigen::Quaterniond rot{ 1, 0, 0, 0 };
		Eigen::Vector3d trans{ 0, 0, 0 };
		double residualTiltRad = 0.0;
		Eigen::Quaterniond worldFromDriverRotation{ 1, 0, 0, 0 };
		Eigen::Vector3d worldFromDriverTranslation{ 0, 0, 0 };
		// Heuristic candidates: pre-jump window snapshot, evaluated once the
		// post-jump window has filled.
		std::vector<Hist> preWindow;
	};

	struct DeviceState
	{
		bool wfdValid = false;
		Eigen::Quaterniond wfdRot{ 1, 0, 0, 0 };
		Eigen::Vector3d wfdTrans{ 0, 0, 0 };
		Eigen::Quaterniond drvRot{ 1, 0, 0, 0 };
		Eigen::Vector3d drvPos{ 0, 0, 0 };
		Eigen::Vector3d drvVel{ 0, 0, 0 };
		Eigen::Vector3d drvAngVel{ 0, 0, 0 };
		double lastValidTime = -1.0;
		std::deque<Hist> hist;
	};

	void DetectWfdRebase(uint32_t id, DeviceState &dev, double t,
	                     const Eigen::Quaterniond &newRot, const Eigen::Vector3d &newTrans);
	void DetectDiscontinuity(uint32_t id, DeviceState &dev, const Hist &incoming);
	void EvaluatePendingCandidates(double now);
	void TryAccept(double now);
	int ActiveDeviceCount(double now) const;

	Config config;
	double qpcToSeconds;

	DeviceState devices[64];               // vr::k_unMaxTrackedDeviceCount
	std::vector<Candidate> candidates;
	std::deque<UniverseDelta> accepted;
	std::deque<GapEvent> gaps;
	std::deque<std::string> notes;
	double lastAcceptTime = -1e9;
};
