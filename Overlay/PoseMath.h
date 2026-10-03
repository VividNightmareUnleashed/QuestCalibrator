#pragma once

// Pose primitives the solver, the monitors and the overlay share. Eigen only,
// with no OpenVR header, so any translation unit can include it.

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <cmath>

namespace questcal
{

// The single yaw projection behind every raw-universe delta in the overlay, so
// the jump path's applied delta and the chaperone re-anchor are the same
// transform. A recenter preserves gravity, so only the twist around +Y may be
// applied; worldFromDriver's tiny tilt residual would move gravity itself.
// `unitRotation` must be normalized; the optional residual (the discarded tilt)
// is diagnostic only. Normalizing the twist directly keeps x and z exactly
// zero, and a rotation of almost exactly 180 degrees about a horizontal axis,
// which has no recoverable heading, answers identity.
inline Eigen::Quaterniond YawOnlyRotation(
	const Eigen::Quaterniond &unitRotation, double *residualTiltRadians = nullptr)
{
	Eigen::Quaterniond yaw(unitRotation.w(), 0.0, unitRotation.y(), 0.0);
	if (yaw.squaredNorm() <= 1e-12)
		yaw = Eigen::Quaterniond::Identity();
	else
		yaw.normalize();

	if (residualTiltRadians)
		*residualTiltRadians = yaw.angularDistance(unitRotation);
	return yaw;
}

// The angle of YawOnlyRotation's twist, in [-pi, pi]. q and -q are one
// rotation, so the angle is read on the w >= 0 side: a small turn reads small
// whichever sign the quaternion arrived with, never close to 360 degrees.
inline double SignedYawRadians(
	const Eigen::Quaterniond &unitRotation, double *residualTiltRadians = nullptr)
{
	const Eigen::Quaterniond yaw = YawOnlyRotation(unitRotation, residualTiltRadians);
	const double sign = yaw.w() < 0.0 ? -1.0 : 1.0;
	return 2.0 * std::atan2(sign * yaw.y(), sign * yaw.w());
}

// The smallest worldFromDriver change that counts as one. The universe
// verdict pairs the transitions it sees with the jump detector's by sample
// time, so both judge with these.
constexpr double WorldFromDriverChangeRadians = 1e-5;
constexpr double WorldFromDriverChangeMeters = 1e-4;

// Inputs come from trusted ring samples or values validated when loaded.
inline bool WorldFromDriverChanged(
	const Eigen::Quaterniond &oldRotation, const Eigen::Vector3d &oldTranslation,
	const Eigen::Quaterniond &newRotation, const Eigen::Vector3d &newTranslation,
	double rotationEpsilonRadians = WorldFromDriverChangeRadians,
	double translationEpsilonMeters = WorldFromDriverChangeMeters)
{
	return oldRotation.normalized().angularDistance(newRotation.normalized()) >
			rotationEpsilonRadians ||
		(newTranslation - oldTranslation).norm() > translationEpsilonMeters;
}

// Where a position moving at its reported velocity is `dt` later: the
// trapezoid of the velocities reported at both ends.
inline Eigen::Vector3d PredictPosition(const Eigen::Vector3d &position,
	const Eigen::Vector3d &velocity, const Eigen::Vector3d &nextVelocity, double dt)
{
	return position + 0.5 * (velocity + nextVelocity) * dt;
}

// The same for a rotation and its world-frame angular velocity.
inline Eigen::Quaterniond PredictRotation(const Eigen::Quaterniond &rotation,
	const Eigen::Vector3d &angularVelocity, const Eigen::Vector3d &nextAngularVelocity, double dt)
{
	const Eigen::Vector3d meanAngularVelocity = 0.5 * (angularVelocity + nextAngularVelocity);
	const double angle = meanAngularVelocity.norm() * dt;
	if (angle <= 1e-12)
		return rotation;
	return Eigen::Quaterniond(Eigen::AngleAxisd(angle, meanAngularVelocity.normalized())) * rotation;
}

// Hemisphere-safe quaternion mean (Markley's eigenvector method), given the
// sum of q q^T over the quaternions' (x, y, z, w) coefficients: its largest
// eigenvector. The outer product is invariant under q -> -q, so the double
// cover needs no bookkeeping.
inline Eigen::Quaterniond QuaternionMean(const Eigen::Matrix4d &outerProductSum)
{
	const Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigen(outerProductSum);
	const Eigen::Vector4d v = eigen.eigenvectors().col(3);   // eigenvalues ascend
	return Eigen::Quaterniond(v(3), v(0), v(1), v(2)).normalized();
}

} // namespace questcal
