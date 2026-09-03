#pragma once

#include "../common/TransformLimits.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

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

inline bool IsValidTimeOffset(double seconds)
{
	return std::isfinite(seconds) &&
		std::abs(seconds) <= protocol::limits::MaxAbsTimeOffsetSeconds;
}

// Unix seconds of the last successful solve, or of the room copy; 0 means
// "unknown", which older records legitimately carry.
inline bool IsValidRecordUnixTime(double seconds)
{
	return std::isfinite(seconds) && seconds >= 0.0 &&
		seconds <= protocol::limits::MaxPlausibleUnixTimeSeconds;
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
// Asked in three places - the load path disarms a snapshot without it, the
// writer refuses to persist one, and the restore path will not apply one - so
// it is one predicate rather than three copies of the same conjunction.
inline bool IsCompleteChaperoneOwner(const std::string &ownerTrackingSystem,
	const std::string &ownerHmdSerial, bool worldFromDriverValid)
{
	return !ownerTrackingSystem.empty() && !ownerHmdSerial.empty() &&
		worldFromDriverValid;
}

// Mirrors CalibrationContext::Speed (FAST=0, SLOW=1, VERY_SLOW=2). The record
// layer deliberately does not include Calibration.h: that header pulls in
// openvr.h, while the test harness translation unit already carries the
// incompatible openvr_driver.h, so depending on it is what made this whole
// state machine unreachable from a test. Configuration.cpp static_asserts that
// the two spellings still agree.
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

// The profile record as it exists on disk. It intentionally duplicates
// CalibrationContext::FieldAnchor rather than reusing it: see
// PersistedCalibrationSpeed above for why nothing here may name that header.
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
	bool fieldEnabled = true;
	std::vector<PersistedFieldAnchor> fieldAnchors;
	bool continuousEnabled = false;
	std::string continuousTrackerSerial;
	bool continuousLatencyReestimation = false;
	bool continuousRequireTrigger = false;
	bool hideMountedTracker = true;
	int continuousMode = 0;   // 0 = QuestCalibrator's loop, 1 = legacy port
	MountExtrinsicRecord mountExtrinsic;
};

// The parser and the writer have to agree on what a well-formed profile is.
// They used to hold two copies of these six checks, seven hundred lines apart
// and free to drift: tighten one and you get a record that loads but can never
// be written back (every later edit silently lost), or one that writes but
// will not load (the profile is gone at the next launch). One definition, one
// reason string; the caller decides whether to throw it or show it.
//
// `maxAnchors` is a parameter rather than a direct use of
// protocol::SetAlignmentField::MaxAnchors because Protocol.h reaches for
// openvr_driver.h - see PersistedCalibrationSpeed. Both call sites in
// Configuration.cpp pass that constant.
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
	// Deliberately outside the valid gate, as in the writer this replaces: an
	// anchor set is checked against whatever base transform the record carries
	// even when the record itself claims nothing.
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

// ---------------------------------------------------------------------------
// Write gates
// ---------------------------------------------------------------------------

enum class PersistenceWriteGate
{
	Allowed,
	// -uipreview drives the entire UI from fabricated state (and -frames N runs
	// that as a CI smoke test), so it must never touch HKCU. It is the only
	// outcome that reports success without writing, and that combination is
	// exactly what would turn a mis-set preview flag on a normal launch into
	// total silent persistence loss - the user recalibrates, the UI says
	// "saved", and the profile is gone on restart. It is a named outcome here,
	// not an early `return true`, so the guard can be asserted rather than
	// assumed.
	SkippedPreview,
	RefusedConfigUnreadable,
	RefusedSettingsUnreadable,
};

inline PersistenceWriteGate GateProfileWrite(bool previewMode,
	RecordLoadState config, RecordLoadState settings,
	bool legacySettingsMigrationPending)
{
	if (previewMode)
		return PersistenceWriteGate::SkippedPreview;
	if (!CanPersistConfig(config))
		return PersistenceWriteGate::RefusedConfigUnreadable;
	// The one-time migration materializes Settings as part of the profile
	// write, so an unreadable Settings record blocks the profile write too:
	// completing it would strip the legacy copy out of Config while the only
	// other copy is unreadable.
	if (legacySettingsMigrationPending && settings == RecordLoadState::Unreadable)
		return PersistenceWriteGate::RefusedSettingsUnreadable;
	return PersistenceWriteGate::Allowed;
}

inline PersistenceWriteGate GateSettingsWrite(
	bool previewMode, RecordLoadState config, RecordLoadState settings)
{
	if (previewMode)
		return PersistenceWriteGate::SkippedPreview;
	if (!CanPersistSettings(config, settings))
		return PersistenceWriteGate::RefusedConfigUnreadable;
	if (settings == RecordLoadState::Unreadable)
		return PersistenceWriteGate::RefusedSettingsUnreadable;
	return PersistenceWriteGate::Allowed;
}

// "Did this touch the registry?" and "did the caller see success?" are
// different questions for exactly one gate. Keeping them as two functions is
// what makes the divergence visible instead of implied by a bare `return true`.
inline bool GateWritesRecord(PersistenceWriteGate gate)
{
	return gate == PersistenceWriteGate::Allowed;
}

inline bool GateReportsSuccess(PersistenceWriteGate gate)
{
	return gate == PersistenceWriteGate::Allowed ||
		gate == PersistenceWriteGate::SkippedPreview;
}

// ---------------------------------------------------------------------------
// Load decision matrix
// ---------------------------------------------------------------------------

// Why the protected chaperone ended the load disarmed. Each carries its own
// message because the user's next action differs: two of them are recoverable
// by fixing a registry value, two require capturing the room again.
enum class ChaperoneLoadGate
{
	Armed,
	// Config could not be read and there is no separate Settings record, so
	// Config may hold the only copy of the room. Disarm in memory, persist
	// nothing.
	ProfileUnreadable,
	// A present-but-unreadable Settings record is authoritative over any legacy
	// snapshot embedded in a Config that happened to parse.
	SettingsUnreadable,
	IncompleteOwner,
	ForeignTrackingSystem,
};

struct PersistenceLoadFacts
{
	RecordLoadState profile = RecordLoadState::Missing;
	RecordLoadState settings = RecordLoadState::Missing;
	PersistedRevision profileRevision;
	PersistedRevision settingsRevision;
	// State of the chaperone snapshot after both records have been parsed.
	bool chaperoneArmed = false;
	bool chaperoneOwnerComplete = false;
	bool chaperoneOwnerMatchesReference = true;
	bool profileValid = false;
};

struct PersistenceLoadPlan
{
	uint32_t persistenceRevision = 1;
	// The two records were written at different revisions (or the Settings half
	// never landed). A chaperone captured before a universe rebase must not be
	// restored onto the rebased playspace, so this fails closed.
	bool revisionMismatch = false;
	// ... and there was actually a snapshot to lose. The mismatch is only worth
	// telling the user about when it cost them something.
	bool reportRevisionMismatch = false;
	bool disarmChaperone = false;
	ChaperoneLoadGate gate = ChaperoneLoadGate::Armed;
	bool settingsRewriteNeeded = false;
	bool legacySettingsMigrationPending = false;
	// If that rewrite fails, this is the value the migration latch must fall
	// back to. Getting it wrong lets one later profile save destroy a legacy
	// user's only copy of their global settings and protected room.
	bool legacySettingsMigrationPendingIfRewriteFails = false;
};

// The whole load-time state machine over {profile} x {settings} x {revision},
// as a function of facts rather than of CalibrationContext. Nothing here reads
// the registry, so every cell of the matrix is reachable from a test.
inline PersistenceLoadPlan PlanPersistenceLoad(const PersistenceLoadFacts &facts)
{
	PersistenceLoadPlan plan;
	const bool profileLoaded = facts.profile == RecordLoadState::Loaded;
	const bool settingsLoaded = facts.settings == RecordLoadState::Loaded;
	const bool settingsMissing = facts.settings == RecordLoadState::Missing;
	// An unreadable record is preserved, never migrated over: rewriting
	// Settings is allowed only when Settings is absent or parsed AND Config was
	// not the unreadable one holding the only legacy copy.
	const bool settingsCanRewrite = (settingsMissing || settingsLoaded) &&
		CanMaterializeSettings(facts.profile);
	bool armed = facts.chaperoneArmed;

	plan.legacySettingsMigrationPendingIfRewriteFails =
		profileLoaded && !facts.profileRevision.present;

	// Config is written before Settings for a coupled universe rebase. If the
	// process dies between those writes, their revisions differ (or Settings is
	// absent). Never restore a raw-space boundary from that mixed state.
	if (profileLoaded && facts.profileRevision.present)
	{
		plan.persistenceRevision = facts.profileRevision.value;
		if (!settingsLoaded || !facts.settingsRevision.present ||
			facts.settingsRevision.value != facts.profileRevision.value)
		{
			plan.revisionMismatch = true;
			plan.reportRevisionMismatch = armed;
			if (armed)
			{
				plan.disarmChaperone = true;
				armed = false;
			}
			plan.settingsRewriteNeeded = settingsCanRewrite;
		}
	}
	else if (profileLoaded)
	{
		// Config-only releases kept the sole copy of global settings and the
		// chaperone inside Config. Materialize Settings before any new-format
		// profile save is allowed to strip those embedded fields.
		plan.persistenceRevision =
			facts.settingsRevision.present ? facts.settingsRevision.value : 1;
		if (!settingsLoaded || !facts.settingsRevision.present)
		{
			plan.legacySettingsMigrationPending = true;
			plan.settingsRewriteNeeded = settingsCanRewrite;
		}
	}
	else
	{
		plan.persistenceRevision =
			facts.settingsRevision.present ? facts.settingsRevision.value : 1;
		if (settingsMissing || (settingsLoaded && !facts.settingsRevision.present))
			plan.settingsRewriteNeeded = settingsCanRewrite;
	}

	// Assignment, not |=, in the two gates below: they are the same question
	// the revision branch already answered, and a gate that fires while
	// rewriting is forbidden must not leave an earlier `true` standing.
	if (!CanUseRecoveredSettings(facts.settings, facts.profile))
	{
		plan.gate = ChaperoneLoadGate::ProfileUnreadable;
		if (armed)
		{
			plan.disarmChaperone = true;
			armed = false;
		}
	}
	else if (facts.settings == RecordLoadState::Unreadable && armed)
	{
		plan.gate = ChaperoneLoadGate::SettingsUnreadable;
		plan.disarmChaperone = true;
		armed = false;
	}
	else if (armed && !facts.chaperoneOwnerComplete)
	{
		plan.gate = ChaperoneLoadGate::IncompleteOwner;
		plan.disarmChaperone = true;
		armed = false;
		plan.settingsRewriteNeeded = settingsCanRewrite;
	}
	else if (armed && facts.profileValid && !facts.chaperoneOwnerMatchesReference)
	{
		plan.gate = ChaperoneLoadGate::ForeignTrackingSystem;
		plan.disarmChaperone = true;
		armed = false;
		plan.settingsRewriteNeeded = settingsCanRewrite;
	}

	return plan;
}

} // namespace questcal
