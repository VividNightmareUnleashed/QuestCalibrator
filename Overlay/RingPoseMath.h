#pragma once

#include "CalibrationEngine.h"
#include "ProfileValidation.h"

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
// The overlay used to spell those policies out at each consumer inside
// Calibration.cpp, which no test compiles; they live here so there is one copy
// and the harness can drive it.
struct RingSampleParts
{
	Eigen::Quaterniond wfdRot;
	Eigen::Vector3d wfdTrans;
	Eigen::Quaterniond drvRot;
	Eigen::Vector3d drvPos;
};

namespace ringpose
{

constexpr double MaxAdjacentFrameSeconds = 0.1;
constexpr double MaxLocalPositionErrorMeters = 0.005;
constexpr double MaxLocalRotationErrorRadians =
	1.0 * 3.14159265358979323846 / 180.0;

struct DriverLocalPoseSample
{
	double time = 0.0;
	Eigen::Quaterniond rotation{ 1, 0, 0, 0 };
	Eigen::Vector3d position{ 0, 0, 0 };
	Eigen::Vector3d velocity{ 0, 0, 0 };
	Eigen::Vector3d angularVelocity{ 0, 0, 0 };
};

// A worldFromDriver transition describes a real raw-universe rebase only if
// adjacent driver-local poses remain on their reported trajectory. An inverse
// local-pose rewrite is bookkeeping that leaves the composed raw pose still.
inline bool IsDriverLocalPoseContinuous(
	const DriverLocalPoseSample &previous,
	const DriverLocalPoseSample &current,
	double maxFrameSeconds = MaxAdjacentFrameSeconds,
	double maxPositionErrorMeters = MaxLocalPositionErrorMeters,
	double maxRotationErrorRadians = MaxLocalRotationErrorRadians)
{
	double dt = current.time - previous.time;
	auto validRotation = [](const Eigen::Quaterniond &rotation) {
		double normSquared = rotation.squaredNorm();
		return rotation.coeffs().allFinite() && std::isfinite(normSquared) &&
			normSquared > 1e-12;
	};
	if (!std::isfinite(dt) || dt <= 0.0 || dt > maxFrameSeconds ||
		!validRotation(previous.rotation) || !validRotation(current.rotation) ||
		!previous.position.allFinite() || !current.position.allFinite() ||
		!previous.velocity.allFinite() || !current.velocity.allFinite() ||
		!previous.angularVelocity.allFinite() ||
		!current.angularVelocity.allFinite())
		return false;

	Eigen::Vector3d predictedPosition = previous.position +
		0.5 * (previous.velocity + current.velocity) * dt;
	Eigen::Quaterniond predictedRotation = previous.rotation.normalized();
	Eigen::Vector3d meanAngularVelocity =
		0.5 * (previous.angularVelocity + current.angularVelocity);
	double predictedAngle = meanAngularVelocity.norm() * dt;
	if (!std::isfinite(predictedAngle))
		return false;
	if (predictedAngle > 1e-12)
	{
		predictedRotation = Eigen::Quaterniond(Eigen::AngleAxisd(
			predictedAngle, meanAngularVelocity.normalized())) * predictedRotation;
	}

	return (current.position - predictedPosition).norm() <=
			maxPositionErrorMeters &&
		current.rotation.normalized().angularDistance(predictedRotation) <=
			maxRotationErrorRadians;
}

// Freshness must be measured from the producer's raw capture QPC. UI
// observation time makes an arbitrarily old backlog look new when it is
// finally drained after a stall, while poseTimeOffset can legitimately move
// a freshly captured pose's validity time into the future.
inline bool IsFreshCaptureTime(
	double sampleTime, double qpcNow, double maxAgeSeconds)
{
	if (!std::isfinite(sampleTime) || !std::isfinite(qpcNow) ||
		!std::isfinite(maxAgeSeconds) || maxAgeSeconds < 0.0)
		return false;
	double age = qpcNow - sampleTime;
	return std::isfinite(age) && age >= 0.0 && age <= maxAgeSeconds;
}

} // namespace ringpose

inline bool IsFiniteRingVector(const double (&v)[3])
{
	return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

inline bool IsBoundedRingVector(const double (&v)[3], double maxAbs)
{
	return IsFiniteRingVector(v) &&
		std::abs(v[0]) <= maxAbs && std::abs(v[1]) <= maxAbs &&
		std::abs(v[2]) <= maxAbs;
}

inline bool IsUsableRingQuaternion(const vr::HmdQuaternion_t &q)
{
	if (!std::isfinite(q.w) || !std::isfinite(q.x) ||
		!std::isfinite(q.y) || !std::isfinite(q.z))
		return false;
	double normSq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
	return std::isfinite(normSq) && normSq > 1e-12;
}

// A driver can mark a pose Running_OK while still supplying malformed numeric
// fields. Validate the complete raw sample before any solver or runtime monitor
// composes it; invalid samples are treated as tracking absence.
inline bool IsUsableRingSample(const protocol::DevicePoseSample &s, double qpcToSeconds)
{
	if (!std::isfinite(qpcToSeconds) || qpcToSeconds <= 0.0 ||
		!std::isfinite(s.poseTimeOffset) ||
		std::abs(s.poseTimeOffset) > protocol::limits::MaxAbsTimeOffsetSeconds ||
		!IsUsableRingQuaternion(s.worldFromDriverRotation) ||
		!IsUsableRingQuaternion(s.rotation) ||
		!IsBoundedRingVector(s.worldFromDriverTranslation,
			protocol::limits::MaxAbsTranslationMeters) ||
		!IsBoundedRingVector(s.position, protocol::limits::MaxAbsPosePositionMeters) ||
		!IsBoundedRingVector(s.velocity,
			protocol::limits::MaxAbsLinearVelocityMetersPerSecond) ||
		!IsBoundedRingVector(s.angularVelocity,
			protocol::limits::MaxAbsAngularVelocityRadiansPerSecond))
		return false;

	double composedTime = static_cast<double>(s.sampleTimeQpc) * qpcToSeconds + s.poseTimeOffset;
	return std::isfinite(composedTime) &&
		std::abs(composedTime) <= protocol::limits::MaxAbsPoseTimestampSeconds;
}

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

// The complete overlay-side trust boundary for one ring sample: the driver must
// claim the pose is valid AND that the device is actually tracking, and the
// numeric fields must survive validation. Spelled once because every consumer
// (chaperone baseline, collector, runtime monitor, continuous loop) must ask the
// same question — a bound added to the numeric half has to reach all four, and
// they used to carry four hand-written copies of this conjunction.
inline bool IsTrustedRingSample(
	const protocol::DevicePoseSample &s, double qpcToSeconds)
{
	return s.poseIsValid &&
		s.trackingResult == static_cast<uint32_t>(vr::TrackingResult_Running_OK) &&
		IsUsableRingSample(s, qpcToSeconds);
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
//
// Returns false for anything the trust boundary rejects, and leaves `out`
// untouched so a rejection can never be mistaken for data: an invalid sample is
// tracking absence, never a defaulted pose. `qpcToSeconds` is a parameter rather
// than a file-static so this stays pure and the harness can drive it.
inline bool TryComposeRingSample(const protocol::DevicePoseSample &s,
	double qpcToSeconds, questcal::PoseSample &out)
{
	if (!IsTrustedRingSample(s, qpcToSeconds))
		return false;

	RingSampleParts p = UnpackRingSample(s);
	questcal::PoseSample composed;
	// poseTimeOffset is the driver's own estimate of how far the pose's validity
	// time differs from the submit time; folding it in tightens the alignment.
	composed.time = RingSampleTime(s, qpcToSeconds);
	composed.rot = (p.wfdRot * p.drvRot).normalized();
	composed.pos = p.wfdRot * p.drvPos + p.wfdTrans;
	composed.vel = p.wfdRot * Eigen::Vector3d(s.velocity[0], s.velocity[1], s.velocity[2]);
	composed.angVel = p.wfdRot * Eigen::Vector3d(
		s.angularVelocity[0], s.angularVelocity[1], s.angularVelocity[2]);

	if (!IsUsableComposedSample(composed))
		return false;
	out = composed;
	return true;
}

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
// All three rules are load-bearing and are one predicate on purpose. The
// mounted-tracker exclusion in particular is the only thing keeping a resting
// head out of the drift feed: without it the monitor logs StationarySlide,
// UpdateDriftScore crosses the stale threshold, and the user is told a
// calibration that is being actively maintained looks poor.
struct DriftFeedCandidate
{
	uint32_t deviceId = vr::k_unTrackedDeviceIndexInvalid;
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

inline bool AnchorsUniverse(const DriftFeedCandidate &c)
{
	bool anchors =
		(c.deviceId == vr::k_unTrackedDeviceIndex_Hmd && c.referenceSide) ||
		(c.targetSide && c.deviceId != c.mountedTrackerId);

	if (anchors && c.deviceId != vr::k_unTrackedDeviceIndex_Hmd &&
		c.composedTime - c.hmdRawTime < DriftHmdProximityWindowSeconds)
	{
		Eigen::Vector3d refPos = BaseCalibratedPosition(c.calibratedRotation,
			c.calibratedTranslationMeters, c.calibratedScale, c.rawPosition);
		if ((refPos - c.hmdRawPosition).norm() < DriftHmdProximityMeters)
			anchors = false;
	}
	return anchors;
}

} // namespace ringpose
