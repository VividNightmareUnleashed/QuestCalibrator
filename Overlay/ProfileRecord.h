#pragma once

// The saved records' shapes and what makes a value well-formed in them,
// shared by both codecs and the load plan.

#include "../common/TransformLimits.h"
#include "ProfileScalarValidation.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

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

// IsValidScale, IsValidResidual, IsValidTimeOffset and IsValidRecordUnixTime
// are in ProfileScalarValidation.h.

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

// A raw worldFromDriver baseline: the headset universe a calibration or a
// protected room was captured in. Both records store the same shape, so both
// must accept exactly the same values or one of them becomes unwritable.
inline bool IsValidUniverseBaseline(
	const Eigen::Quaterniond &rotation, const Eigen::Vector3d &translation)
{
	return IsValidRotation(rotation) &&
		IsBoundedVector(translation, protocol::limits::MaxAbsTranslationMeters);
}

inline bool ProfileHmdIdentityMatches(
	const std::string &profileSerial, const std::string &currentSerial)
{
	return !profileSerial.empty() && profileSerial == currentSerial;
}

// A raw-space snapshot belongs to the headset/universe that produced it.
inline bool IsCompleteChaperoneOwner(const std::string &ownerTrackingSystem,
	const std::string &ownerHmdSerial, bool worldFromDriverValid)
{
	return !ownerTrackingSystem.empty() && !ownerHmdSerial.empty() &&
		worldFromDriverValid;
}

// Mirrors CalibrationContext::Speed. This header cannot include Calibration.h:
// it pulls in openvr.h, which conflicts with the openvr_driver.h the test
// harness already carries. Configuration.cpp static_asserts that they agree.
enum class PersistedCalibrationSpeed
{
	Fast = 0,
	Slow = 1,
	VerySlow = 2,
};

inline bool IsValidCalibrationSpeed(double value)
{
	return std::isfinite(value) &&
		value >= static_cast<int>(PersistedCalibrationSpeed::Fast) &&
		value <= static_cast<int>(PersistedCalibrationSpeed::VerySlow) &&
		std::floor(value) == value;
}

// Both persisted records carry the same revision when a universe rebase writes
// them as one transaction. Absent is a legitimate state (a record written
// before revisions existed), which is why presence is tracked separately from
// the value instead of overloading 0.
struct PersistedRevision
{
	bool present = false;
	uint32_t value = 0;
};

// The profile record as it exists on disk. The anchor mirrors
// CalibrationContext::FieldAnchor for the reason given at
// PersistedCalibrationSpeed.
struct PersistedFieldAnchor
{
	Eigen::Vector3d position{ 0, 0, 0 };
	Eigen::Quaterniond rotation{ 1, 0, 0, 0 };
	Eigen::Vector3d translationMeters{ 0, 0, 0 };
};

struct MountExtrinsicRecord
{
	bool valid = false;
	Eigen::Quaterniond rotation{ 1, 0, 0, 0 };
	Eigen::Vector3d translationMeters{ 0, 0, 0 };
	double rotationRmsDeg = 0.0;
	double translationRmsM = 0.0;
};

// The profile's switches, each listed once as the Settings record's are:
// its member (named as in CalibrationContext), its key and its default.
// The record's members, the Config writer and parser, Configuration.cpp's
// capture and apply and the tests' record comparisons expand from this
// list. continuousNoPause is not one; it is saved as the continuous_mode
// name.
#define QUESTCAL_PROFILE_SWITCHES(X) \
	X(fieldEnabled, "field_enabled", true) \
	X(continuousEnabled, "continuous_enabled", false) \
	X(continuousLatencyReestimation, "continuous_latency_reestimation", false) \
	X(continuousRequireTrigger, "continuous_require_trigger", false) \
	X(hideMountedTracker, "hide_mounted_tracker", true)

struct ProfileRecord
{
	bool valid = false;
	std::string referenceTrackingSystem;
	std::string targetTrackingSystem;
	Eigen::Quaterniond rotation{ 1, 0, 0, 0 };
	Eigen::Vector3d translationMeters{ 0, 0, 0 };
	double scale = 1.0;
	double timeOffset = 0.0;
	double calibrationUnixTime = 0.0;
	// Reference-universe identity and the fail-closed latch keyed to it. All
	// optional on read, so a profile written before they existed loads
	// unchanged and adopts a baseline from the first fresh observation.
	bool universeUnsafe = false;
	bool universeValid = false;
	std::string universeHmdSerial;
	Eigen::Quaterniond universeRotation{ 1, 0, 0, 0 };
	Eigen::Vector3d universeTranslation{ 0, 0, 0 };
#define QUESTCAL_SWITCH_MEMBER(member, key, value) bool member = value;
	QUESTCAL_PROFILE_SWITCHES(QUESTCAL_SWITCH_MEMBER)
#undef QUESTCAL_SWITCH_MEMBER
	std::vector<PersistedFieldAnchor> fieldAnchors;
	std::string continuousTrackerSerial;
	bool continuousNoPause = false;
	MountExtrinsicRecord mountExtrinsic;
};

// The one definition of a well-formed profile, shared by the parser and the
// writer so a record that loads can be written back and vice versa.
// `maxAnchors` is a parameter because Protocol.h pulls in openvr_driver.h (see
// PersistedCalibrationSpeed); callers pass SetAlignmentField::MaxAnchors.
inline bool ValidateProfileRecord(
	const ProfileRecord &record, size_t maxAnchors, std::string &why)
{
	if (record.valid)
	{
		if (!IsValidTrackingSystemPair(
			record.referenceTrackingSystem, record.targetTrackingSystem))
		{
			why = "the tracking systems must be non-empty and different";
			return false;
		}
		if (!IsValidCalibrationTransform(
			record.rotation, record.translationMeters, record.scale))
		{
			why = "the calibration transform is invalid";
			return false;
		}
		if (!IsValidTimeOffset(record.timeOffset) ||
			!IsValidRecordUnixTime(record.calibrationUnixTime))
		{
			why = "the calibration timing values are invalid";
			return false;
		}
		if (record.universeValid &&
			(record.universeHmdSerial.empty() || !IsValidUniverseBaseline(
				record.universeRotation, record.universeTranslation)))
		{
			why = "the reference-universe baseline is invalid";
			return false;
		}
		if (record.mountExtrinsic.valid &&
			(!IsValidRotation(record.mountExtrinsic.rotation) ||
				!IsBoundedVector(record.mountExtrinsic.translationMeters,
					protocol::limits::MaxAbsAnchorDeltaMeters) ||
				!IsValidResidual(record.mountExtrinsic.rotationRmsDeg) ||
				!IsValidResidual(record.mountExtrinsic.translationRmsM)))
		{
			why = "the mount extrinsic is invalid";
			return false;
		}
		if (record.fieldAnchors.size() > maxAnchors)
		{
			why = "there are too many field anchors";
			return false;
		}
	}
	// Outside the valid gate: anchors are checked against whatever base
	// transform the record carries, even when the record claims nothing.
	for (const auto &anchor : record.fieldAnchors)
	{
		if (!IsValidFieldAnchor(anchor.position, anchor.rotation,
			anchor.translationMeters, record.rotation, record.translationMeters))
		{
			why = "a field anchor is invalid";
			return false;
		}
	}
	return true;
}

} // namespace questcal
