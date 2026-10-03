#pragma once

#include "CalibrationEngine.h"
#include "LocalPoseContinuity.h"
#include "ProfileRecord.h"
#include "RingSampleGate.h"

#include "../common/NumericValidation.h"
#include "../common/Protocol.h"
#include "../common/TransformLimits.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>

// Eigen views of a ring sample's transform fields, shared by everything that
// composes world poses from the shared-memory stream, plus the pure ingestion
// policy that governs them: the trust boundary a sample must clear, the
// composition itself, and which composed samples may feed the drift monitor.
// Kept here, not in Calibration.cpp, so the harness can drive it.
struct RingSampleParts
{
	Eigen::Quaterniond wfdRot;
	Eigen::Vector3d wfdTrans;
	Eigen::Quaterniond drvRot;
	Eigen::Vector3d drvPos;
};

namespace ringpose
{

// Freshness must be measured from the producer's raw capture QPC. UI
// observation time makes an arbitrarily old backlog look new when it is
// finally drained after a stall, while poseTimeOffset can legitimately move
// a freshly captured pose's validity time into the future.
inline bool IsFreshCaptureTime(
	double sampleTime, double qpcNow, double maxAgeSeconds)
{
	double age = qpcNow - sampleTime;   // NaN fails both comparisons
	return age >= 0.0 && age <= maxAgeSeconds;
}

// Raw collection rides through the isolated poses the driver drops when two
// device threads contend for a queue claim: one or two at a time, every few
// seconds to minutes with a dozen devices on the ring. The solver already
// refuses to interpolate across a dropout, so a short hole costs a pair or
// two. A gap past this size (about 40 ms of a dozen devices' traffic) means
// the overlay stalled, and a session boundary means the samples on either
// side may not share a clock or a universe; either stops the collection.
constexpr uint64_t MaxToleratedCollectionGap = 256;

inline bool CollectionGapTolerable(uint64_t largestGap, bool crossedSessionBoundary)
{
	return !crossedSessionBoundary && largestGap <= MaxToleratedCollectionGap;
}

// The runtime monitors and both continuous loops ride through the same
// contended-publish drops, on a much shorter leash. All of them judge
// continuity by sample time, so a lost pose costs them a pair, not a window.
// The leash is set by the drift monitor, which tells a tracking loss from a
// hole in its own input by time alone, so a tolerated hole must never span
// its 0.3 s lossGap: eight poses of a lone 72 Hz headset, the sparsest stream
// there is, last 110 ms (DriftMonitor.cpp asserts it). Anything larger, and
// any session boundary, still resets them.
constexpr uint64_t MaxToleratedMonitorGap = 8;
constexpr double SparsestPoseRateHz = 72.0;

inline bool MonitorGapTolerable(uint64_t gap, bool crossedSessionBoundary)
{
	return !crossedSessionBoundary && gap <= MaxToleratedMonitorGap;
}

} // namespace ringpose

// IsUsableRingSample and IsTrustedRingSample, the accept-or-drop gate, are in
// RingSampleGate.h.

inline double RingSampleTime(const protocol::DevicePoseSample &s, double qpcToSeconds)
{
	return static_cast<double>(s.sampleTimeQpc) * qpcToSeconds + s.poseTimeOffset;
}

inline double RingCaptureTime(const protocol::DevicePoseSample &s, double qpcToSeconds)
{
	return static_cast<double>(s.sampleTimeQpc) * qpcToSeconds;
}

inline RingSampleParts UnpackRingSample(const protocol::DevicePoseSample &s)
{
	return {
		Eigen::Quaterniond(s.worldFromDriverRotation.w, s.worldFromDriverRotation.x,
			s.worldFromDriverRotation.y, s.worldFromDriverRotation.z).normalized(),
		Eigen::Vector3d(s.worldFromDriverTranslation[0], s.worldFromDriverTranslation[1],
			s.worldFromDriverTranslation[2]),
		Eigen::Quaterniond(s.rotation.w, s.rotation.x, s.rotation.y, s.rotation.z).normalized(),
		Eigen::Vector3d(s.position[0], s.position[1], s.position[2])
	};
}

// A ring sample's pose as the driver reported it, before worldFromDriver.
inline ringpose::DriverLocalPoseSample DriverLocalPose(const protocol::DevicePoseSample &s,
	const RingSampleParts &p, double time)
{
	return { time, p.drvRot, p.drvPos,
		Eigen::Vector3d(s.velocity[0], s.velocity[1], s.velocity[2]),
		Eigen::Vector3d(s.angularVelocity[0], s.angularVelocity[1], s.angularVelocity[2]) };
}

// The same pose in the raw universe (worldFromDriver o driver pose), for the
// readers that take its parts without TryComposeRingSample's trust checks.
inline ringpose::DriverLocalPoseSample RingWorldPose(const protocol::DevicePoseSample &s,
	const RingSampleParts &p, double time)
{
	return ringpose::ComposeWithWorldFromDriver(DriverLocalPose(s, p, time), p.wfdRot, p.wfdTrans);
}

// A misbehaving driver can publish poseIsValid=true with field values that pass
// the per-field ring checks yet compose into something unusable; gate the
// composed pose too so no consumer (solver, continuous alignment, drift
// monitor) ever sees one. Also used by the runtime-pose fallback, whose samples
// never come off the ring at all.
inline bool IsUsableComposedSample(const questcal::PoseSample &s)
{
	return std::isfinite(s.time) && questcal::IsValidRotation(s.rot) &&
		questcal::IsBoundedVector(s.pos,
			protocol::limits::MaxAbsPosePositionMeters) &&
		questcal::IsBoundedVector(s.vel,
			protocol::limits::MaxAbsLinearVelocityMetersPerSecond) &&
		questcal::IsBoundedVector(s.angVel,
			protocol::limits::MaxAbsAngularVelocityRadiansPerSecond);
}

// Compose the raw-world pose (worldFromDriver * driver pose) from a ring sample
// and convert its capture timestamp to seconds on the QPC clock — the single
// ingestion path from the shared-memory stream into solver/monitor space.
// Returns false for anything the trust boundary rejects and leaves `out`
// untouched: an invalid sample is tracking absence, never a defaulted pose.
inline bool TryComposeRingSample(const protocol::DevicePoseSample &s,
	double qpcToSeconds, questcal::PoseSample &out)
{
	if (!IsTrustedRingSample(s, qpcToSeconds))
		return false;

	// poseTimeOffset is the driver's own estimate of how far the pose's validity
	// time differs from the submit time; folding it in tightens the alignment.
	const ringpose::DriverLocalPoseSample world =
		RingWorldPose(s, UnpackRingSample(s), RingSampleTime(s, qpcToSeconds));
	questcal::PoseSample composed;
	composed.time = world.time;
	composed.rot = world.rotation;
	composed.pos = world.position;
	composed.vel = world.velocity;
	composed.angVel = world.angularVelocity;

	if (!IsUsableComposedSample(composed))
		return false;
	out = composed;
	return true;
}

namespace ringpose
{

// Which of the continuous loop's two streams stopped when it runs out of
// fresh observations. Both run at hundreds of hertz, so one whose last
// accepted pose is staleSeconds old has stopped; neither, when both still
// arrive and only their pairing fails. The loss is not always the tracker's:
// a streamed headset pauses on its own (live 2026-09-26 22:04: a Steam Link
// stream reset left the headset without poses for 2.3 s, and the feed blamed
// the headset tracker).
enum class StoppedStream { Neither, Headset, Tracker, Both };

inline StoppedStream WhichStreamStopped(double now, double headsetLastAccepted,
	double trackerLastAccepted, double staleSeconds = 0.5)
{
	const bool headset = !(now - headsetLastAccepted <= staleSeconds);
	const bool tracker = !(now - trackerLastAccepted <= staleSeconds);
	return headset && tracker ? StoppedStream::Both
		: headset ? StoppedStream::Headset
		: tracker ? StoppedStream::Tracker
		: StoppedStream::Neither;
}

} // namespace ringpose

struct RingInputDiagnostics
{
	uint64_t received = 0, accepted = 0;
	uint64_t trackingRejected = 0, numericRejected = 0;
	double lastCaptureTime = 0.0, lastAcceptedCaptureTime = 0.0;

	bool Compose(const protocol::DevicePoseSample &s, double qpcToSeconds,
		questcal::PoseSample &out)
	{
		++received;
		lastCaptureTime = RingCaptureTime(s, qpcToSeconds);
		if (TryComposeRingSample(s, qpcToSeconds, out))
		{
			++accepted;
			lastAcceptedCaptureTime = lastCaptureTime;
			return true;
		}
		if (!s.deviceIsConnected || !s.poseIsValid ||
			s.trackingResult != static_cast<uint32_t>(vr::TrackingResult_Running_OK))
			++trackingRejected;
		else
			++numericRejected;
		return false;
	}
};

namespace ringpose
{

// Apply the base calibration to a target-universe raw position. The scale
// multiplies the RAW position, before the rotation and translation — the order
// the driver applies (PoseScale then PoseTransform). Everything that has to
// predict where the driver puts a target device reads it from here: at scale
// 1.0 a misplaced factor is invisible, and at 1.03 it shifts the answer by 9 cm
// three metres from the origin.
inline Eigen::Vector3d BaseCalibratedPosition(
	const Eigen::Quaterniond &rotation,
	const Eigen::Vector3d &translationMeters, double scale,
	const Eigen::Vector3d &targetRawPos)
{
	return rotation * (scale * targetRawPos) + translationMeters;
}

// Drift evidence must come from devices that anchor a universe. On the
// reference side that is the HMD alone: its SLAM map IS the reference universe,
// while reference-side peripherals (e.g. set-down Touch controllers) IMU-coast
// with slowly sliding poses that still report Running_OK -- centimeters of
// "slide" that say nothing about alignment. Every lighthouse device is rigid to
// the target universe, so target-system devices stay eligible -- except the
// HMD-mounted tracker (rides a human head), and any lighthouse device within
// arm's reach of the headset: that is a worn tracker (hips, feet) or a device
// the user is right next to, and a supported resting body passes the
// stationarity gates then "slides" with slow posture creep.
//
// A base station is on the target side too, but it is the universe, not a
// device in it: its early poses move by metres while SteamVR settles where it
// stands (one session read two as losses recovered 2.3 m away). A device whose
// class has not been read yet (the scan runs every 2 s) is held back likewise.
//
// The mounted-tracker exclusion is the only thing keeping a resting head out of
// the drift feed, which would otherwise call an actively maintained
// calibration stale.
struct DriftFeedCandidate
{
	uint32_t deviceId = vr::k_unTrackedDeviceIndexInvalid;
	// Invalid until the property scan has read it.
	vr::ETrackedDeviceClass deviceClass = vr::TrackedDeviceClass_Invalid;
	bool referenceSide = false;   // device is in the reference tracking system
	bool targetSide = false;      // ... the target tracking system
	// The HMD-mounted continuous-calibration tracker, or invalid when the
	// feature is off / the id has not been resolved this scan.
	uint32_t mountedTrackerId = vr::k_unTrackedDeviceIndexInvalid;

	Eigen::Vector3d rawPosition{ 0, 0, 0 };   // this sample's composed raw pose
	double composedTime = 0.0;
	// Latest HMD raw observation, for the proximity heuristic.
	Eigen::Vector3d hmdRawPosition{ 0, 0, 0 };
	double hmdRawTime = -1e9;

	// Live base calibration: a target-raw position has to be brought into
	// reference space before it can be compared with the HMD's.
	Eigen::Quaterniond calibratedRotation{ 1, 0, 0, 0 };
	Eigen::Vector3d calibratedTranslationMeters{ 0, 0, 0 };
	double calibratedScale = 1.0;
};

// How stale the HMD observation may be for the proximity test to apply, and
// what counts as "within arm's reach".
constexpr double DriftHmdProximityWindowSeconds = 3.0;
constexpr double DriftHmdProximityMeters = 1.2;
// The sphere misses the lower body: knee and foot trackers sit 1.2 to 1.7 m
// below the head of a standing user, pass the stationarity gates while the
// user stands still, and "slide" a centimetre with each weight shift. One
// session built a stale verdict out of five such events. Anything below the
// headset within this horizontal radius is worn, or set down at the user's
// feet, and says nothing about either universe.
constexpr double DriftHmdBodyRadiusMeters = 0.8;

inline bool AnchorsUniverse(const DriftFeedCandidate &c)
{
	if (c.deviceClass == vr::TrackedDeviceClass_Invalid ||
		c.deviceClass == vr::TrackedDeviceClass_TrackingReference)
		return false;

	bool anchors =
		(c.deviceId == vr::k_unTrackedDeviceIndex_Hmd && c.referenceSide) ||
		(c.targetSide && c.deviceId != c.mountedTrackerId);

	if (anchors && c.deviceId != vr::k_unTrackedDeviceIndex_Hmd &&
		c.composedTime - c.hmdRawTime < DriftHmdProximityWindowSeconds)
	{
		Eigen::Vector3d refPos = BaseCalibratedPosition(c.calibratedRotation,
			c.calibratedTranslationMeters, c.calibratedScale, c.rawPosition);
		Eigen::Vector3d fromHead = refPos - c.hmdRawPosition;
		if (fromHead.norm() < DriftHmdProximityMeters)
			anchors = false;
		else if (fromHead.y() < 0.0 &&
			std::hypot(fromHead.x(), fromHead.z()) < DriftHmdBodyRadiusMeters)
			anchors = false;
	}
	return anchors;
}

} // namespace ringpose
