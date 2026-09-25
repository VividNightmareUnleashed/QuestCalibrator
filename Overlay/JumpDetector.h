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
// gravity). The exact path reports the discarded tilt as residualTiltRad; the
// heuristic path regresses heading only and leaves it 0. Both residuals are
// diagnostics only. Times are ring-sample seconds (QPC * QpcToSeconds), not
// UI time.
class JumpDetector
{
public:
	// Fixed detection policy.
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
		// map and may keep its previous frame for up to 30 s after the headset
		// switches map, so an HMD candidate no device confirmed inside
		// agreeWindow is held this long for the controller's step; one no
		// controller follows (a 3DoF catch-up after a wake) still expires. The
		// clock pauses while every other reference device is locked still: the
		// engine's static prior freezes a resting controller (its position
		// repeats bit for bit), and it cannot step until the hand moves again.
		double controllerFollowSeconds = 30.0;
		// The controller follower snaps a delta above 5 cm at once while the
		// headset smoother slides for its 5 s grace, so the second of two close
		// map moves can reach the controllers up to 5 s before the headset. A
		// controller step that far ahead still confirms if the deltas match.
		double controllerLeadSeconds = 5.0;
		double soloPosThreshold = 0.30;    // single-device heuristic acceptance floor
		double soloYawThresholdRad = 10.0 * 3.14159265358979 / 180.0;
		// Headset-only setups (the primary population) never have a second
		// reference device to confirm a step; one session logged three 1-4 cm
		// same-sign steps in three minutes that a recalibration later removed.
		// A clean persistent HMD step is therefore accepted alone down to the
		// corroborated floor once the stream has been continuous for
		// soloSettledSeconds (a wake, a stream restart and Virtual Desktop's
		// reconnect re-zeroing all fall inside the first minute) and the
		// pre-window shows no held position: the 3DoF fallback holds position
		// bit for bit, which 6DoF never did (0 of 4,331 frames, 2026-09-11).
		double soloSettledPos = 0.05;
		double soloSettledYawRad = 2.0 * 3.14159265358979 / 180.0;
		double soloSettledSeconds = 60.0;
		// The engine's correction smoother slides toward the map target while
		// the head moves and, once the remaining offset exceeds its reset
		// threshold (10 cm or 10 deg, compiled defaults of the examined build,
		// after a 5 s grace), snaps the rest in one step of almost exactly the
		// threshold. That snap restores the alignment; compensating it would put
		// the drift back. A headset step inside this band is never applied alone
		// unless continuous alignment has followed the drift (SetDriftFollowed);
		// a matching controller step still confirms it. A real frame change of
		// exactly the threshold is logged and left for the next correction.
		double driftCatchUpPos = 0.10;
		double driftCatchUpPosBand = 0.005;
		double driftCatchUpYawRad = 10.0 * 3.14159265358979 / 180.0;
		double driftCatchUpYawBandRad = 0.3 * 3.14159265358979 / 180.0;
		// The translation band only holds for a step without yaw: the smoother
		// measures about the map origin, which the stream does not show. Heading
		// drift accrues where the user turns slowly in place, so its catch-up
		// turns the frame about the head, while a frame change moves the head by
		// its distance from the pivot times the angle. A headset step within the
		// yaw threshold that moves the head no more than this is a catch-up too;
		// a frame change pivoting within ~30 cm of the head is the cost.
		double driftCatchUpHeadShift = 0.02;
		double gapSeconds = 2.0;           // reference stream gap => no compensation, event only
		// A discontinuity this soon after the device's stream (re)started is
		// annotated with its resume age: after a Quest Pro wakes (seen through
		// Virtual Desktop) poses resume in 3DoF, and the snap to 6DoF a few
		// seconds later looks exactly like a universe jump.
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
		// jump. Reported, never used to gate acceptance.
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
		// The HMD worldFromDriver the exact rebase started from.
		Eigen::Quaterniond previousWorldFromDriverRotation{ 1, 0, 0, 0 };
		Eigen::Vector3d previousWorldFromDriverTranslation{ 0, 0, 0 };
	};

	struct GapEvent
	{
		uint32_t deviceId = 0;
		double duration = 0.0;             // seconds without valid reference samples
		double time = 0.0;
	};

	explicit JumpDetector(double qpcToSeconds) : qpcToSeconds(qpcToSeconds) { }

	// Feed one reference-system sample, in ring order and valid or not (a bad
	// frame breaks continuity). The caller bounds deviceId; non-reference
	// devices must not be pushed.
	void Push(const protocol::DevicePoseSample &sample);

	// Queued output, each drained by the caller until it returns false: an
	// accepted jump, a reference-stream gap, and human-readable notes (wfd
	// rebases seen, candidates expired, ...) for the calibration log.
	bool PollDelta(UniverseDelta &out) { return PopFront(accepted, out); }
	bool PollGap(GapEvent &out) { return PopFront(gaps, out); }
	bool PollNote(std::string &out) { return PopFront(notes, out); }

	// The caller's copy of the stream is missing a few samples (the driver
	// drops an isolated pose when two device threads contend for a queue
	// claim). Which device lost one is unknown, so every device's continuity
	// breaks as for an observed bad frame and a candidate whose fit window was
	// still filling dies. The resume clock, Ready candidates and the session
	// totals stay: such drops come every few seconds to minutes, and treating
	// each as a Reset would keep soloSettledSeconds from ever being reached.
	void NoteStreamHole();
	// Whether continuous alignment keeps the calibration on an independent
	// reference (a mounted tracker). The
	// calibration has then followed the drift, so a drift catch-up moves the
	// frame away from it and is compensated like any other step. The caller
	// refreshes this every tick; Reset leaves it alone.
	void SetDriftFollowed(bool followed) { driftFollowed = followed; }
	// Detailed logging: each candidate's evidence (the frame jump that raised
	// it, both fitted states and their residuals, the samples around the
	// step) and the reason any candidate is dropped, which otherwise happens
	// silently. Off, nothing is gathered. The caller refreshes it every tick.
	void SetDetailed(bool on)
	{
		detailed = on;
		if (!on)
			details.clear();
	}
	bool PollDetail(std::string &out) { return PopFront(details, out); }

	// Drop all per-device state (calibration started, monitors disabled, a
	// stall-sized hole or a driver session boundary in the stream, ...).
	void Reset();

	// Whether a headset heuristic candidate raised at sample time `time` is
	// still alive: its fit window is filling, or it waits for a device to
	// confirm it. The profile-universe verdict does not give up on a
	// worldFromDriver change such a candidate may yet explain.
	bool HasLiveHeadsetCandidate(double time) const;

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
	enum class Kind { Exact, Heuristic };
	enum class Life
	{
		Pending,   // heuristic: post-jump window still filling
		Ready,     // has a usable delta (exact candidates are born Ready)
		Dead,      // expired unconfirmed, or its observation continuity broke
	};

	// The worldFromDriver endpoints the exact path captured.
	struct ExactEndpoint
	{
		Eigen::Quaterniond rotation{ 1, 0, 0, 0 };
		Eigen::Vector3d translation{ 0, 0, 0 };
		Eigen::Quaterniond previousRotation{ 1, 0, 0, 0 };
		Eigen::Vector3d previousTranslation{ 0, 0, 0 };
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
		// The velocity-compensated jump between the two frames that raised
		// the candidate. Against the fitted step it shows whether the pose
		// overshot within a frame and settled back.
		double frameJumpPos = 0.0, frameJumpYawRad = 0.0;
		bool needsCorroboration = false;
		bool held = false;   // HMD heuristic: past agreeWindow, awaiting a controller follow-up
		// HMD heuristic: when the follow-up wait ends, pushed out while the
		// other reference devices are locked still; `heldLocked` marks that
		// the extension was logged.
		double followDeadline = 0.0;
		double lastHoldCheck = -1.0;
		bool heldLocked = false;
		// HMD heuristic, decided once when the fit completes. `heldPosition`:
		// the pre-window shows the 3DoF fallback, so this is a catch-up onto
		// resumed tracking. `driftCatchUp`: shaped like the smoother's reset
		// (see driftCatchUp* in Config). No solo path applies either.
		// `settledSolo`: may be applied with no other reference device
		// tracking (see soloSettled*).
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

	// A candidate's device has always had a valid sample since the last
	// Reset, and a gap after the candidate kills it, so this is never negative.
	static double ResumeAge(const DeviceState &dev, double t)
	{
		return t - dev.streamResumeTime;
	}

	template <typename T>
	static bool PopFront(std::deque<T> &queue, T &out)
	{
		if (queue.empty())
			return false;
		out = queue.front();
		queue.pop_front();
		return true;
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
	bool driftFollowed = false;
	bool detailed = false;
	std::deque<std::string> details;
	// Marks a live candidate dead and, in detailed mode, says why.
	void Drop(Candidate &c, const std::string &reason);
	// HMD steps that expired unapplied since the last Reset, summed as one
	// transform so the log shows whether they add up to the drift a
	// recalibration later removes.
	int ignoredSteps = 0;
	int driftCatchUps = 0;
	double ignoredYaw = 0.0;
	Eigen::Vector3d ignoredTranslation{ 0, 0, 0 };
};
