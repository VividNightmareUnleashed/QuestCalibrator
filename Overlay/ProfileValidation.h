#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

namespace questcal
{

inline bool IsFinite(const Eigen::Vector3d &v)
{
	return v.allFinite();
}

inline bool IsFiniteQuaternion(const Eigen::Quaterniond &q)
{
	return std::isfinite(q.w()) && std::isfinite(q.x()) &&
		std::isfinite(q.y()) && std::isfinite(q.z());
}

inline bool IsValidRotation(const Eigen::Quaterniond &q)
{
	return IsFiniteQuaternion(q) && q.squaredNorm() > 1e-12;
}

inline bool IsValidScale(double scale)
{
	// The solver normally stays within [0.85, 1.15]. This wider range
	// preserves intentional manual edits while rejecting destructive input.
	return std::isfinite(scale) && scale >= 0.25 && scale <= 4.0;
}

inline bool IsValidCalibrationTransform(const Eigen::Quaterniond &rotation,
	const Eigen::Vector3d &translation, double scale)
{
	return IsValidRotation(rotation) && IsFinite(translation) && IsValidScale(scale);
}

inline bool IsValidResidual(double value)
{
	return std::isfinite(value) && value >= 0.0;
}

} // namespace questcal
