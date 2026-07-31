#pragma once

#include "../common/TransformLimits.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <string>

namespace questcal
{

enum class RecordLoadState
{
	Missing,
	Loaded,
	Unreadable,
};

// A separately parsed Settings record is already authoritative and remains
// usable when Config is damaged. Falling back to embedded/default settings is
// safe only when Config was absent or parsed successfully.
inline bool CanUseRecoveredSettings(
	RecordLoadState settings, RecordLoadState config)
{
	return settings == RecordLoadState::Loaded ||
		config != RecordLoadState::Unreadable;
}

// Creating or automatically rewriting Settings can discard the only legacy
// settings copy hidden inside Config, so it requires Config to be absent or to
// have parsed successfully even when a separate Settings record was readable.
inline bool CanMaterializeSettings(RecordLoadState config)
{
	return config != RecordLoadState::Unreadable;
}

inline bool CanPersistConfig(RecordLoadState config)
{
	return config != RecordLoadState::Unreadable;
}

inline bool CanPersistSettings(
	RecordLoadState config, RecordLoadState settings)
{
	return config != RecordLoadState::Unreadable ||
		settings == RecordLoadState::Loaded;
}

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
	if (!IsFiniteQuaternion(q))
		return false;
	double normSquared = q.squaredNorm();
	// Finite components can still overflow while forming the norm. Eigen's
	// normalized() would then produce a degenerate/non-finite quaternion.
	return std::isfinite(normSquared) && normSquared > 1e-12;
}

inline bool IsValidTrackingSystemPair(
	const std::string &reference, const std::string &target)
{
	return !reference.empty() && !target.empty() && reference != target;
}

inline bool IsValidScale(double scale)
{
	// The solver normally stays within [0.85, 1.15]. This wider range
	// preserves intentional manual edits while rejecting destructive input.
	return std::isfinite(scale) && scale >= protocol::limits::MinScale &&
		scale <= protocol::limits::MaxScale;
}

inline bool IsBoundedVector(const Eigen::Vector3d &value, double maxAbs)
{
	return IsFinite(value) && value.cwiseAbs().maxCoeff() <= maxAbs;
}

inline bool IsValidCalibrationTransform(const Eigen::Quaterniond &rotation,
	const Eigen::Vector3d &translation, double scale)
{
	return IsValidRotation(rotation) &&
		IsBoundedVector(translation, protocol::limits::MaxAbsTranslationMeters) &&
		IsValidScale(scale);
}

inline bool IsValidFieldAnchor(const Eigen::Vector3d &position,
	const Eigen::Quaterniond &rotation, const Eigen::Vector3d &translation,
	const Eigen::Quaterniond &baseRotation, const Eigen::Vector3d &baseTranslation)
{
	if (!IsBoundedVector(position, protocol::limits::MaxAbsAnchorPositionMeters) ||
		!IsValidRotation(rotation) ||
		!IsBoundedVector(translation, protocol::limits::MaxAbsTranslationMeters) ||
		!IsValidRotation(baseRotation) ||
		!IsBoundedVector(baseTranslation, protocol::limits::MaxAbsTranslationMeters))
		return false;

	Eigen::Quaterniond deltaRotation =
		(rotation.normalized() * baseRotation.normalized().conjugate()).normalized();
	Eigen::Vector3d deltaTranslation = translation - deltaRotation * baseTranslation;
	return IsBoundedVector(deltaTranslation, protocol::limits::MaxAbsAnchorDeltaMeters);
}

inline bool IsValidResidual(double value)
{
	return std::isfinite(value) && value >= 0.0;
}

} // namespace questcal
