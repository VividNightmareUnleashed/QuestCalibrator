#pragma once

#include "../common/Protocol.h"
#include "../common/TransformLimits.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

// Eigen views of a ring sample's transform fields, shared by everything that
// composes world poses from the shared-memory stream.
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
