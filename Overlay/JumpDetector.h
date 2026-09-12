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
	// Fixed detection policy, not a caller knob: no code outside this class
	// ever varied it, so the values live here as a named-constant block with
	// their rationale rather than behind a setter nobody called.
	struct Config
	{
		double wfdRotEpsRad = 1e-5;        // wfd rotation change that counts as a rebase
		double wfdTransEps = 1e-4;         // meters
		double discontinuityPos = 0.20;    // meters of unexplained motion in one frame pair
		double discontinuityYawRad = 5.0 * 3.14159265358979 / 180.0;
		// Smaller steps need a clean fit and tighter HMD/controller agreement.
		// They never qualify for the single-device fallback.
		double corroboratedPos = 0.05;
		double corroboratedYawRad = 2.0 * 3.14159265358979 / 180.0;
		double corroboratedPosTol = 0.02;
		double corroboratedYawTolRad = 1.0 * 3.14159265358979 / 180.0;
		double fitPositionRms = 0.01;
		double fitYawRmsRad = 0.5 * 3.14159265358979 / 180.0;
		double maxFrameGap = ringpose::MaxAdjacentFrameSeconds;
		double localContinuityPos = ringpose::MaxLocalPositionErrorMeters;
		double localContinuityRotRad = ringpose::MaxLocalRotationErrorRadians;
		double window = 0.2;               // seconds of history on each side of a candidate
		double agreeWindow = 0.25;         // seconds within which devices must agree
		double agreePosTol = 0.10;         // meters between per-device delta translations
		double agreeYawTolRad = 3.0 * 3.14159265358979 / 180.0;
		// Each Quest Pro controller is its own tracking frontend on the shared
		// map. After the headset switches map, the engine lets a controller
		// keep its previous 6DoF frame for up to 30 s before it follows, so
		// the HMD steps alone and the matching controller step arrives
		// seconds later. An HMD candidate no
		// device confirmed inside agreeWindow is therefore held this long for
		// that follow-up instead of being discarded. A step the HMD sees
		// alone and no controller ever follows - a 3DoF-to-6DoF catch-up
		// after a wake, a stream hiccup - still expires unapplied.
		double controllerFollowSeconds = 30.0;
		double soloPosThreshold = 0.30;    // single-device heuristic acceptance floor
		double soloYawThresholdRad = 10.0 * 3.14159265358979 / 180.0;
		double gapSeconds = 2.0;           // reference stream gap => no compensation, event only
		// A discontinuity this soon after the device's stream (re)started is
		// annotated with its resume age. Observed on a Quest Pro through
		// Virtual Desktop: after the headset wakes, poses resume while the
		// tracking engine is still in 3DoF, and the snap to the relocalized
		// 6DoF pose a few seconds later looks exactly like a universe jump.
		// The age lets a log reader tell the two apart.
		double recentResumeSeconds = 60.0;
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
		// Seconds between the reporting device's most recent stream (re)start
		// (its first valid sample ever, or the first after a gap) and the
		// jump; < 0 when unknown. Reported, never used to gate acceptance.
		double secondsSinceStreamResume = -1.0;
		// Heuristic path: seconds between the HMD's step and the confirming
		// device's step when the confirmation arrived after agreeWindow
		// (a controller following a map switch); 0 when devices agreed at
		// once or the delta was accepted solo. `time` stays the HMD's jump
		// instant either way.
		double confirmationLagSeconds = 0.0;
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

	// Which detection path produced a candidate, and where it is in its life.
	// Kept as two enums rather than three booleans: of the eight boolean
	// combinations only four ever meant anything, and every consumer had to
	// re-derive which four by writing the same three-term filter again.
	enum class Kind { Exact, Heuristic };
	enum class Life
	{
		Pending,   // heuristic: post-jump window still filling
		Ready,     // has a usable delta (exact candidates are born Ready)
		Dead,      // expired unconfirmed, or its observation continuity broke
	};

	// The absolute worldFromDriver endpoint the exact path captured. One
	// sub-struct rather than two loose fields, so "these two are meaningful
	// only for Kind::Exact" is a property of the record: the heuristic path
	// never writes it and leaves the identity it was born with.
	struct ExactEndpoint
	{
		Eigen::Quaterniond rotation{ 1, 0, 0, 0 };
		Eigen::Vector3d translation{ 0, 0, 0 };
	};

	struct Candidate
	{
		uint32_t deviceId = 0;
		double t = 0.0;
		Kind kind = Kind::Heuristic;
		Life life = Life::Pending;
		Eigen::Quaterniond rot{ 1, 0, 0, 0 };
		Eigen::Vector3d trans{ 0, 0, 0 };
		double residualTiltRad = 0.0;
		ExactEndpoint endpoint;   // Kind::Exact only
		// Heuristic candidates: pre-jump window snapshot, evaluated once the
		// post-jump window has filled.
		std::vector<Hist> preWindow;
		bool needsCorroboration = false;
		bool held = false;   // HMD heuristic: past agreeWindow, awaiting a controller follow-up

		// The filters every consumer needs, once each.
		bool LiveExact() const { return kind == Kind::Exact && life != Life::Dead; }
		bool ReadyHeuristic() const { return kind == Kind::Heuristic && life == Life::Ready; }
		bool PendingHeuristic() const { return kind == Kind::Heuristic && life == Life::Pending; }
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
		double streamResumeTime = -1.0;    // first valid sample ever, or first after a gap
		std::deque<Hist> hist;
	};

	static double ResumeAge(const DeviceState &dev, double t)
	{
		return dev.streamResumeTime >= 0.0 ? t - dev.streamResumeTime : -1.0;
	}

	void DetectWfdRebase(uint32_t id, DeviceState &dev, double t,
	                     const Eigen::Quaterniond &newRot, const Eigen::Vector3d &newTrans);
	void DetectDiscontinuity(uint32_t id, DeviceState &dev, const Hist &incoming);
	void EvaluatePendingCandidates();
	void TryAccept();
	int ActiveDeviceCount(double now) const;

	Config config;
	double qpcToSeconds;

	DeviceState devices[vr::k_unMaxTrackedDeviceCount];
	std::vector<Candidate> candidates;
	std::deque<UniverseDelta> accepted;
	std::deque<GapEvent> gaps;
	std::deque<std::string> notes;
	double lastAcceptTime = -1e9;
};
