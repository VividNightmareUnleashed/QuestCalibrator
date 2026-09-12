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
		// The clock pauses while every other reference device is locked
		// still: the engine's static prior freezes a controller that has
		// been stationary for a second (its position then repeats bit for
		// bit, which a tracked 6DoF position never does), and a frozen
		// controller cannot step until the hand moves again. Hands resting
		// through a headset reset would otherwise expire the step and leave
		// the whole error in place; instead the candidate waits for the
		// hands and is confirmed by their first movement, or expires 30 s
		// after they move without stepping.
		double controllerFollowSeconds = 30.0;
		// The controller frontend's anchor follower has no grace period and
		// snaps a delta above 5 cm at once, while the headset smoother slides
		// for the rest of its 5 s grace after a reset before it resets
		// again. The second of two close map moves therefore reaches the
		// controllers up to 5 s before the headset. A controller step this
		// far ahead of the HMD's counts as confirmation; the deltas still
		// have to match.
		double controllerLeadSeconds = 5.0;
		double soloPosThreshold = 0.30;    // single-device heuristic acceptance floor
		double soloYawThresholdRad = 10.0 * 3.14159265358979 / 180.0;
		// Headset-only setups (the primary population: a Quest headset with
		// lighthouse trackers and controllers, no Quest controllers) never
		// have a second reference device to confirm a step, so every SLAM
		// correction under the solo floor used to be fitted, found
		// persistent, and discarded. One session logged three same-sign
		// steps of 1 to 4 cm and 0.5 to 3 deg inside three minutes: the
		// drift a recalibration later removes. A clean persistent HMD step
		// with no other device tracking is therefore accepted alone down to
		// the corroborated floor, once the two ways a false step arises are
		// excluded: the stream must have been continuous for
		// soloSettledSeconds (a wake, a stream restart and Virtual Desktop's
		// reconnect re-zeroing all fall inside the first minute), and the
		// pre-window must carry no held-position signature. The engine's
		// 3DoF fallback holds the last tracked position bit-for-bit while the
		// IMU orientation keeps moving, and the snap back to 6DoF is a clean
		// step; in 6DoF the SLAM position never repeats between frames
		// (0 of 8,515 streamed frames on 2026-09-11).
		double soloSettledPos = 0.05;
		double soloSettledYawRad = 2.0 * 3.14159265358979 / 180.0;
		double soloSettledSeconds = 60.0;
		// The engine removes odometry drift through its correction smoother:
		// it slides toward the map target while the head moves and, once the
		// remaining offset exceeds its reset threshold (10 cm or 10 deg on
		// the examined build, checked every frame after a 5 s grace), snaps
		// the rest in one step. Drift grows by a fraction of a millimetre or
		// a thousandth of a degree per frame, so that snap is a step of
		// almost exactly the threshold. Such a snap restores the alignment
		// (it cancels error accrued since the calibration) and compensating
		// it would put the drift back; a frame change lands on the threshold
		// only by coincidence. A headset step whose translation or yaw sits
		// inside this band is therefore never applied alone; a matching
		// controller step still confirms it, and controllers never step on
		// drift because their follower slews it continuously. A frame change
		// of exactly the threshold is the cost: it is logged and left for
		// the next correction. The thresholds are compiled defaults read
		// from the library, not from the device; solo steps clustering at
		// some other size in the log would show a different build's values.
		double driftCatchUpPos = 0.10;
		double driftCatchUpPosBand = 0.005;
		double driftCatchUpYawRad = 10.0 * 3.14159265358979 / 180.0;
		double driftCatchUpYawBandRad = 0.3 * 3.14159265358979 / 180.0;
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
		// device's step when the confirmation arrived outside agreeWindow:
		// positive for a controller following a map switch, negative for a
		// controller that stepped ahead of the headset (the smoother's
		// grace); 0 when devices agreed at once or the delta was accepted
		// solo. `time` stays the HMD's jump instant either way.
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
		// HMD heuristic: when the follow-up wait ends. Starts at
		// t + window + controllerFollowSeconds and moves out by every second
		// the other reference devices spend locked still (see
		// controllerFollowSeconds); `heldLocked` marks that the extension
		// was logged.
		double followDeadline = 0.0;
		double lastHoldCheck = -1.0;
		bool heldLocked = false;
		// HMD heuristic, decided once when the fit completes. `heldPosition`:
		// the pre-window carried the 3DoF fallback's signature, so this step
		// is a catch-up onto tracking that resumed, never a frame change;
		// no solo path applies it. `driftCatchUp`: the step is the size of
		// the engine's reset threshold, so it is most likely the smoother
		// cancelling accrued drift; no solo path applies it (see
		// driftCatchUp* in Config). `settledSolo`: the step may be applied
		// with no other reference device tracking (see soloSettled*).
		bool heldPosition = false;
		bool driftCatchUp = false;
		bool settledSolo = false;

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
		// Consecutive samples whose composed position repeated the previous
		// one bit for bit: the static prior's lock on a still controller.
		int repeatedPositions = 0;
		std::deque<Hist> hist;
	};

	static double ResumeAge(const DeviceState &dev, double t)
	{
		return dev.streamResumeTime >= 0.0 ? t - dev.streamResumeTime : -1.0;
	}

	// The engine's 3DoF fallback: position held bit-for-bit between frames
	// while the orientation keeps integrating. Never seen in 6DoF.
	static bool HeldPositionSignature(const std::vector<Hist> &window);

	void DetectWfdRebase(uint32_t id, DeviceState &dev, double t,
	                     const Eigen::Quaterniond &newRot, const Eigen::Vector3d &newTrans);
	void DetectDiscontinuity(uint32_t id, DeviceState &dev, const Hist &incoming);
	void EvaluatePendingCandidates();
	void TryAccept();
	int ActiveDeviceCount(double now) const;
	// True when at least one other reference device is tracking and every
	// one of them is locked still (position repeated over the last samples).
	bool OthersLockedStill(double now) const;

	Config config;
	double qpcToSeconds;

	DeviceState devices[vr::k_unMaxTrackedDeviceCount];
	std::vector<Candidate> candidates;
	std::deque<UniverseDelta> accepted;
	std::deque<GapEvent> gaps;
	std::deque<std::string> notes;
	double lastAcceptTime = -1e9;
	// HMD steps that expired unapplied since the last Reset, summed as one
	// transform so the log shows whether they add up to the drift a
	// recalibration later removes.
	int ignoredSteps = 0;
	int driftCatchUps = 0;
	double ignoredYaw = 0.0;
	Eigen::Vector3d ignoredTranslation{ 0, 0, 0 };
};
