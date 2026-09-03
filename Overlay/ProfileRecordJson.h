#pragma once

// The Config record's JSON codec: envelope, profile fields, and the legacy
// global settings that Config-only releases embedded alongside them.
//
// Split out of Configuration.cpp so the write -> read identity is reachable
// from the test harness. A field the writer emits and the parser no longer
// reads (or the reverse) is silent per-restart data loss - the mount extrinsic
// or the field anchors quietly reverting on every launch - and nothing above
// this line can detect it.
//
// Deliberately free of CalibrationContext and of the registry: this header is
// included by Configuration.cpp and by the harness, nothing else. The chaperone
// snapshot is NOT here, because its record needs the OpenVR geometry types and
// ChaperoneMath.h, which already includes ProfileValidation.h.

#include "ProfileValidation.h"

#include <picojson.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <istream>
#include <limits>
#include <ostream>
#include <stdexcept>
#include <string>

namespace questcal
{

// picojson's get<T>() is guarded only by assert() (compiled out in Release), so
// every read of untrusted profile JSON must type-check first to reach the
// intended runtime_error path instead of reading the wrong union member.
inline double GetDouble(const picojson::value &v)
{
	if (!v.is<double>())
		throw std::runtime_error("expected number");
	double value = v.get<double>();
	if (!std::isfinite(value))
		throw std::runtime_error("expected finite number");
	return value;
}

template<typename T>
inline bool HasTypedValue(const picojson::object &obj, const char *name)
{
	auto it = obj.find(name);
	if (it == obj.end())
		return false;
	if (!it->second.is<T>())
		throw std::runtime_error(std::string("invalid type for ") + name);
	return true;
}

inline picojson::array FloatArray(const float *buf, size_t numFloats)
{
	picojson::array arr;
	arr.reserve(numFloats);

	for (size_t i = 0; i < numFloats; i++)
		arr.push_back(picojson::value(double(buf[i])));

	return arr;
}

inline void LoadFloatArray(const picojson::value &obj, float *buf, size_t numFloats)
{
	if (!obj.is<picojson::array>())
		throw std::runtime_error("expected array");

	auto &arr = obj.get<picojson::array>();
	if (arr.size() != numFloats)
		throw std::runtime_error("wrong buffer size");

	for (size_t i = 0; i < numFloats; i++)
	{
		double value = GetDouble(arr[i]);
		if (value < -std::numeric_limits<float>::max() ||
			value > std::numeric_limits<float>::max())
			throw std::runtime_error("number is outside float range");
		buf[i] = static_cast<float>(value);
	}
}

inline PersistedRevision ReadPersistenceRevision(const picojson::object &obj)
{
	PersistedRevision revision;
	if (!HasTypedValue<double>(obj, "persistence_revision"))
		return revision;

	double value = GetDouble(obj.at("persistence_revision"));
	if (value < 1.0 || value > std::numeric_limits<uint32_t>::max() ||
		std::floor(value) != value)
		throw std::runtime_error("invalid persistence_revision");
	revision.present = true;
	revision.value = static_cast<uint32_t>(value);
	return revision;
}

// picojson parses by recursive descent with no depth limit, and the registry
// admits values far larger than any real record. A stack overflow on Windows
// is an SEH exception, so neither the runtime_error catch around the parse nor
// wWinMain's catch (...) can see it: a deeply nested value crashes at startup
// with no window, no dialog and no session log, and the only recovery is
// deleting the registry value by hand. Reject that shape before parsing so it
// lands in Unreadable like any other malformed record. Both persisted schemas
// nest at most three levels; this bound is far above them and far below what
// exhausts the stack. Nesting inside strings does not count, so a value whose
// text merely contains brackets still round-trips.
inline void RejectExcessiveJsonNesting(const std::string &text)
{
	constexpr int maxDepth = 16;
	int depth = 0;
	bool inString = false;
	bool escaped = false;
	for (char c : text)
	{
		if (inString)
		{
			if (escaped)
				escaped = false;
			else if (c == '\\')
				escaped = true;
			else if (c == '"')
				inString = false;
			continue;
		}

		if (c == '"')
			inString = true;
		else if (c == '[' || c == '{')
		{
			if (++depth > maxDepth)
				throw std::runtime_error("record nesting is too deep");
		}
		else if (c == ']' || c == '}')
			--depth;
	}
}

// Global preferences that Config-only releases stored inside the profile
// record. They are settings, not profile state, so the codec only reports what
// it found and Configuration.cpp decides what to do with it - it is the one
// place that knows whether a separate Settings record already exists.
struct LegacyProfileSettings
{
	int settingsVersion = 1;
	bool hasApplyTimeOffset = false;
	bool applyTimeOffset = true;
	bool hasSolveScale = false;
	bool solveScale = false;
	bool hasUiAdvanced = false;
	bool uiAdvanced = false;
	bool hasChaperoneWarningAck = false;
	bool chaperoneWarningAck = false;
	bool hasCalibrationSpeed = false;
	int calibrationSpeed = static_cast<int>(PersistedCalibrationSpeed::Fast);
};

struct ProfileParseResult
{
	PersistedRevision revision;
	bool migratedScaleSetting = false;
	bool suspiciousLegacyScale = false;
};

// The Config envelope: a one-element array of profile objects. Returned by
// value so the caller can keep reading the same object (the chaperone snapshot
// is parsed on top of it by Configuration.cpp).
inline picojson::value ParseProfileEnvelope(std::istream &stream)
{
	picojson::value v;
	std::string err = picojson::parse(v, stream);
	if (!err.empty())
		throw std::runtime_error(err);

	if (!v.is<picojson::array>())
		throw std::runtime_error("profile file is not an array");
	const auto &arr = v.get<picojson::array>();
	if (arr.size() < 1)
		throw std::runtime_error("no profiles in file");

	if (!arr[0].is<picojson::object>())
		throw std::runtime_error("profile entry is not an object");
	return arr[0];
}

inline ProfileParseResult ParseProfileObject(ProfileRecord &profile,
	LegacyProfileSettings &legacy, const picojson::object &obj, size_t maxAnchors)
{
	ProfileParseResult result;
	result.revision = ReadPersistenceRevision(obj);

	if (!HasTypedValue<std::string>(obj, "reference_tracking_system") ||
		!HasTypedValue<std::string>(obj, "target_tracking_system"))
		throw std::runtime_error("profile is missing the tracking system names");
	profile.referenceTrackingSystem = obj.at("reference_tracking_system").get<std::string>();
	profile.targetTrackingSystem = obj.at("target_tracking_system").get<std::string>();
	if (!IsValidTrackingSystemPair(
		profile.referenceTrackingSystem, profile.targetTrackingSystem))
		throw std::runtime_error("tracking system names must be non-empty and different");

	// The quaternion is the stored truth; Euler display values are derived
	// from it by SetCalibration when the record is applied.
	if (!HasTypedValue<picojson::array>(obj, "rotation_quat") ||
		!HasTypedValue<picojson::array>(obj, "translation_meters"))
		throw std::runtime_error("profile is missing rotation_quat/translation_meters");

	const auto &quatArr = obj.at("rotation_quat").get<picojson::array>();
	const auto &transArr = obj.at("translation_meters").get<picojson::array>();
	if (quatArr.size() != 4 || transArr.size() != 3)
		throw std::runtime_error("malformed rotation_quat/translation_meters");

	Eigen::Quaterniond rotation(
		GetDouble(quatArr[0]),   // w
		GetDouble(quatArr[1]),   // x
		GetDouble(quatArr[2]),   // y
		GetDouble(quatArr[3]));  // z
	Eigen::Vector3d translationMeters(
		GetDouble(transArr[0]),
		GetDouble(transArr[1]),
		GetDouble(transArr[2]));

	double scale = HasTypedValue<double>(obj, "scale") ? GetDouble(obj.at("scale")) : 1.0;
	// Checked here as well as in ValidateProfileRecord below: normalized() on a
	// degenerate quaternion produces NaNs, so the guard has to precede the
	// normalize rather than only judge the finished record.
	if (!IsValidCalibrationTransform(rotation, translationMeters, scale))
		throw std::runtime_error("invalid calibration transform");
	profile.rotation = rotation.normalized();
	profile.translationMeters = translationMeters;
	profile.scale = scale;

	if (HasTypedValue<double>(obj, "time_offset"))
	{
		profile.timeOffset = GetDouble(obj.at("time_offset"));
		if (!IsValidTimeOffset(profile.timeOffset))
			throw std::runtime_error("invalid time_offset");
	}

	// Unix seconds of the last successful solve; 0 = unknown (older profile).
	if (HasTypedValue<double>(obj, "calibration_time"))
	{
		profile.calibrationUnixTime = GetDouble(obj.at("calibration_time"));
		if (!IsValidRecordUnixTime(profile.calibrationUnixTime))
			throw std::runtime_error("invalid calibration_time");
	}

	// Reference-universe identity: which headset owned the universe this
	// calibration was solved in, and that headset's raw worldFromDriver. All
	// optional — a profile written before these existed is not evidence that
	// the universe moved — but a partial record is malformed, not permissive:
	// half a baseline can neither detect a rebase nor prove there was none.
	profile.universeUnsafe = false;
	profile.universeValid = false;
	profile.universeHmdSerial.clear();
	if (HasTypedValue<bool>(obj, "universe_unsafe"))
		profile.universeUnsafe = obj.at("universe_unsafe").get<bool>();
	if (HasTypedValue<std::string>(obj, "hmd_serial"))
	{
		profile.universeHmdSerial = obj.at("hmd_serial").get<std::string>();
		if (profile.universeHmdSerial.empty())
			throw std::runtime_error("empty profile HMD serial");
	}

	bool hasUniverseSerial = HasTypedValue<std::string>(obj, "universe_hmd_serial");
	bool hasUniverseRotation = HasTypedValue<picojson::array>(
		obj, "universe_world_from_driver_rotation_quat");
	bool hasUniverseTranslation = HasTypedValue<picojson::array>(
		obj, "universe_world_from_driver_translation_meters");
	if (hasUniverseSerial != hasUniverseRotation ||
		hasUniverseSerial != hasUniverseTranslation)
		throw std::runtime_error("incomplete profile reference-universe baseline");
	if (hasUniverseSerial)
	{
		const std::string &baselineSerial =
			obj.at("universe_hmd_serial").get<std::string>();
		if (!profile.universeHmdSerial.empty() &&
			profile.universeHmdSerial != baselineSerial)
			throw std::runtime_error("conflicting profile HMD serials");
		profile.universeHmdSerial = baselineSerial;
		const auto &universeRotation = obj.at(
			"universe_world_from_driver_rotation_quat").get<picojson::array>();
		const auto &universeTranslation = obj.at(
			"universe_world_from_driver_translation_meters").get<picojson::array>();
		if (profile.universeHmdSerial.empty() || universeRotation.size() != 4 ||
			universeTranslation.size() != 3)
			throw std::runtime_error("malformed profile reference-universe baseline");

		Eigen::Quaterniond baselineRotation(
			GetDouble(universeRotation[0]), GetDouble(universeRotation[1]),
			GetDouble(universeRotation[2]), GetDouble(universeRotation[3]));
		Eigen::Vector3d baselineTranslation(
			GetDouble(universeTranslation[0]), GetDouble(universeTranslation[1]),
			GetDouble(universeTranslation[2]));
		if (!IsValidUniverseBaseline(baselineRotation, baselineTranslation))
			throw std::runtime_error("invalid profile reference-universe baseline");

		profile.universeRotation = baselineRotation.normalized();
		profile.universeTranslation = baselineTranslation;
		profile.universeValid = true;
	}

	if (HasTypedValue<bool>(obj, "apply_time_offset"))
	{
		legacy.hasApplyTimeOffset = true;
		legacy.applyTimeOffset = obj.at("apply_time_offset").get<bool>();
	}

	// One-time migration (settings_version < 2): scale solving used to default
	// on, but streamed reference poses are motion-smoothed and the solved
	// scale absorbs the attenuation (several percent, varying with motion
	// speed) — so it is opt-in now, including for profiles saved before the
	// change. The already-applied scale is deliberately kept: it was solved
	// jointly with the translation, and clearing it without re-solving would
	// visibly misalign the space. The next recalibration replaces it.
	double settingsVersionValue = HasTypedValue<double>(obj, "settings_version")
		? GetDouble(obj.at("settings_version")) : 1.0;
	if (settingsVersionValue < 1.0 || settingsVersionValue > 100.0 ||
		std::floor(settingsVersionValue) != settingsVersionValue)
		throw std::runtime_error("invalid settings_version");
	legacy.settingsVersion = static_cast<int>(settingsVersionValue);
	bool hasSolveScale = HasTypedValue<bool>(obj, "solve_scale");
	if (legacy.settingsVersion >= 2 && hasSolveScale)
	{
		legacy.hasSolveScale = true;
		legacy.solveScale = obj.at("solve_scale").get<bool>();
	}

	if (HasTypedValue<bool>(obj, "ui_advanced"))
	{
		legacy.hasUiAdvanced = true;
		legacy.uiAdvanced = obj.at("ui_advanced").get<bool>();
	}

	if (HasTypedValue<bool>(obj, "chaperone_warning_ack"))
	{
		legacy.hasChaperoneWarningAck = true;
		legacy.chaperoneWarningAck = obj.at("chaperone_warning_ack").get<bool>();
	}

	if (HasTypedValue<double>(obj, "calibration_speed"))
	{
		double speed = GetDouble(obj.at("calibration_speed"));
		if (!IsValidCalibrationSpeed(speed))
			throw std::runtime_error("invalid calibration_speed");
		legacy.hasCalibrationSpeed = true;
		legacy.calibrationSpeed = static_cast<int>(speed);
	}

	if (HasTypedValue<bool>(obj, "field_enabled"))
		profile.fieldEnabled = obj.at("field_enabled").get<bool>();

	// Continuous calibration (all optional: older profiles load unchanged).
	if (HasTypedValue<bool>(obj, "continuous_enabled"))
		profile.continuousEnabled = obj.at("continuous_enabled").get<bool>();

	if (HasTypedValue<std::string>(obj, "continuous_tracker_serial"))
		profile.continuousTrackerSerial =
			obj.at("continuous_tracker_serial").get<std::string>();

	if (HasTypedValue<bool>(obj, "continuous_latency_reestimation"))
		profile.continuousLatencyReestimation =
			obj.at("continuous_latency_reestimation").get<bool>();
	if (HasTypedValue<bool>(obj, "continuous_require_trigger"))
		profile.continuousRequireTrigger =
			obj.at("continuous_require_trigger").get<bool>();

	if (HasTypedValue<bool>(obj, "hide_mounted_tracker"))
		profile.hideMountedTracker = obj.at("hide_mounted_tracker").get<bool>();
	// A name rather than a number, so a hand-edited profile reads, and an
	// unknown one falls back to the default loop instead of an out-of-range
	// enum.
	if (HasTypedValue<std::string>(obj, "continuous_mode"))
		profile.continuousMode = obj.at("continuous_mode").get<std::string>() == "legacy" ? 1 : 0;

	// Presence makes the mount extrinsic part of the profile contract. Reject
	// malformed data instead of normalizing a degenerate quaternion and
	// silently arming continuous calibration with NaNs.
	profile.mountExtrinsic = MountExtrinsicRecord();
	if (HasTypedValue<picojson::object>(obj, "mount_extrinsic"))
	{
		const auto &extrinsic = obj.at("mount_extrinsic").get<picojson::object>();
		if (!HasTypedValue<picojson::array>(extrinsic, "rotation_quat") ||
			!HasTypedValue<picojson::array>(extrinsic, "translation_meters"))
			throw std::runtime_error("malformed mount_extrinsic");
		const auto &rotArr = extrinsic.at("rotation_quat").get<picojson::array>();
		const auto &traArr = extrinsic.at("translation_meters").get<picojson::array>();
		if (rotArr.size() != 4 || traArr.size() != 3)
			throw std::runtime_error("malformed mount_extrinsic");

		Eigen::Quaterniond mountRotation(
			GetDouble(rotArr[0]), GetDouble(rotArr[1]),
			GetDouble(rotArr[2]), GetDouble(rotArr[3]));
		Eigen::Vector3d mountPosition(
			GetDouble(traArr[0]), GetDouble(traArr[1]), GetDouble(traArr[2]));
		if (!IsValidRotation(mountRotation) ||
			!IsBoundedVector(mountPosition, protocol::limits::MaxAbsAnchorDeltaMeters))
			throw std::runtime_error("invalid mount_extrinsic");

		profile.mountExtrinsic.rotation = mountRotation.normalized();
		profile.mountExtrinsic.translationMeters = mountPosition;
		if (HasTypedValue<double>(extrinsic, "rot_rms_deg"))
			profile.mountExtrinsic.rotationRmsDeg = GetDouble(extrinsic.at("rot_rms_deg"));
		if (HasTypedValue<double>(extrinsic, "pos_rms_m"))
			profile.mountExtrinsic.translationRmsM = GetDouble(extrinsic.at("pos_rms_m"));
		if (!IsValidResidual(profile.mountExtrinsic.rotationRmsDeg) ||
			!IsValidResidual(profile.mountExtrinsic.translationRmsM))
			throw std::runtime_error("invalid mount_extrinsic residual");
		profile.mountExtrinsic.valid = true;
	}

	profile.fieldAnchors.clear();
	if (HasTypedValue<picojson::array>(obj, "field_anchors"))
	{
		const auto &fieldAnchors = obj.at("field_anchors").get<picojson::array>();
		// Before the reserve, not only in ValidateProfileRecord: an absurd
		// count would otherwise allocate for it before being rejected.
		if (fieldAnchors.size() > maxAnchors)
			throw std::runtime_error("too many field anchors");
		profile.fieldAnchors.reserve(fieldAnchors.size());
		for (const auto &anchorV : fieldAnchors)
		{
			if (!anchorV.is<picojson::object>())
				throw std::runtime_error("malformed field anchor");
			const auto &anchorObj = anchorV.get<picojson::object>();

			if (!HasTypedValue<picojson::array>(anchorObj, "position") ||
				!HasTypedValue<picojson::array>(anchorObj, "rotation_quat") ||
				!HasTypedValue<picojson::array>(anchorObj, "translation_meters"))
				throw std::runtime_error("malformed field anchor");

			const auto &posArr = anchorObj.at("position").get<picojson::array>();
			const auto &rotArr = anchorObj.at("rotation_quat").get<picojson::array>();
			const auto &traArr = anchorObj.at("translation_meters").get<picojson::array>();
			if (posArr.size() != 3 || rotArr.size() != 4 || traArr.size() != 3)
				throw std::runtime_error("malformed field anchor");

			PersistedFieldAnchor anchor;
			anchor.position = Eigen::Vector3d(GetDouble(posArr[0]), GetDouble(posArr[1]), GetDouble(posArr[2]));
			Eigen::Quaterniond anchorRotation(GetDouble(rotArr[0]), GetDouble(rotArr[1]),
				GetDouble(rotArr[2]), GetDouble(rotArr[3]));
			anchor.translationMeters = Eigen::Vector3d(GetDouble(traArr[0]), GetDouble(traArr[1]), GetDouble(traArr[2]));
			if (!IsValidFieldAnchor(anchor.position, anchorRotation,
				anchor.translationMeters, profile.rotation, profile.translationMeters))
				throw std::runtime_error("invalid field anchor");
			anchor.rotation = anchorRotation.normalized();

			profile.fieldAnchors.push_back(anchor);
		}
	}

	if (legacy.settingsVersion < 2)
	{
		legacy.hasSolveScale = true;
		legacy.solveScale = false;
		result.migratedScaleSetting = true;
		result.suspiciousLegacyScale = profile.scale < 0.98 || profile.scale > 1.02;
	}

	profile.valid = true;

	// The record the parser produces must be one the writer would accept.
	// Anything else is a profile that loads and then silently refuses every
	// later save, or one the user will find gone at the next launch.
	std::string why;
	if (!ValidateProfileRecord(profile, maxAnchors, why))
		throw std::runtime_error(why);
	return result;
}

inline void WriteProfile(const ProfileRecord &record,
	uint32_t persistenceRevisionValue, std::ostream &out)
{
	if (!record.valid)
		return;

	picojson::object profile;
	double persistenceRevision = static_cast<double>(persistenceRevisionValue);
	profile["persistence_revision"].set<double>(persistenceRevision);
	profile["reference_tracking_system"].set<std::string>(record.referenceTrackingSystem);
	profile["target_tracking_system"].set<std::string>(record.targetTrackingSystem);

	picojson::array quat;
	quat.reserve(4);
	quat.push_back(picojson::value(record.rotation.w()));
	quat.push_back(picojson::value(record.rotation.x()));
	quat.push_back(picojson::value(record.rotation.y()));
	quat.push_back(picojson::value(record.rotation.z()));
	profile["rotation_quat"].set<picojson::array>(std::move(quat));

	picojson::array trans;
	trans.reserve(3);
	trans.push_back(picojson::value(record.translationMeters(0)));
	trans.push_back(picojson::value(record.translationMeters(1)));
	trans.push_back(picojson::value(record.translationMeters(2)));
	profile["translation_meters"].set<picojson::array>(std::move(trans));

	profile["scale"].set<double>(record.scale);
	profile["time_offset"].set<double>(record.timeOffset);
	profile["calibration_time"].set<double>(record.calibrationUnixTime);
	profile["universe_unsafe"].set<bool>(record.universeUnsafe);
	if (!record.universeHmdSerial.empty())
		profile["hmd_serial"].set<std::string>(record.universeHmdSerial);
	if (record.universeValid)
	{
		profile["universe_hmd_serial"].set<std::string>(record.universeHmdSerial);

		picojson::array universeRotation;
		universeRotation.reserve(4);
		universeRotation.push_back(picojson::value(record.universeRotation.w()));
		universeRotation.push_back(picojson::value(record.universeRotation.x()));
		universeRotation.push_back(picojson::value(record.universeRotation.y()));
		universeRotation.push_back(picojson::value(record.universeRotation.z()));
		profile["universe_world_from_driver_rotation_quat"].set<picojson::array>(
			std::move(universeRotation));

		picojson::array universeTranslation;
		universeTranslation.reserve(3);
		for (int axis = 0; axis < 3; ++axis)
			universeTranslation.push_back(picojson::value(record.universeTranslation(axis)));
		profile["universe_world_from_driver_translation_meters"].set<picojson::array>(
			std::move(universeTranslation));
	}
	// Bumped when a load-time migration must not re-run (see ParseProfileObject).
	double settingsVersion = 2.0;
	profile["settings_version"].set<double>(settingsVersion);
	profile["continuous_enabled"].set<bool>(record.continuousEnabled);
	if (!record.continuousTrackerSerial.empty())
		profile["continuous_tracker_serial"].set<std::string>(record.continuousTrackerSerial);
	profile["continuous_latency_reestimation"].set<bool>(record.continuousLatencyReestimation);
	profile["continuous_require_trigger"].set<bool>(record.continuousRequireTrigger);
	profile["hide_mounted_tracker"].set<bool>(record.hideMountedTracker);
	profile["continuous_mode"].set<std::string>(record.continuousMode == 1 ? "legacy" : "questcalibrator");

	if (record.mountExtrinsic.valid)
	{
		picojson::object extrinsic;

		picojson::array rot, tra;
		rot.reserve(4);
		tra.reserve(3);
		rot.push_back(picojson::value(record.mountExtrinsic.rotation.w()));
		rot.push_back(picojson::value(record.mountExtrinsic.rotation.x()));
		rot.push_back(picojson::value(record.mountExtrinsic.rotation.y()));
		rot.push_back(picojson::value(record.mountExtrinsic.rotation.z()));
		for (int k = 0; k < 3; ++k)
			tra.push_back(picojson::value(record.mountExtrinsic.translationMeters(k)));

		extrinsic["rotation_quat"].set<picojson::array>(std::move(rot));
		extrinsic["translation_meters"].set<picojson::array>(std::move(tra));
		extrinsic["rot_rms_deg"].set<double>(record.mountExtrinsic.rotationRmsDeg);
		extrinsic["pos_rms_m"].set<double>(record.mountExtrinsic.translationRmsM);

		profile["mount_extrinsic"].set<picojson::object>(std::move(extrinsic));
	}

	profile["field_enabled"].set<bool>(record.fieldEnabled);
	if (!record.fieldAnchors.empty())
	{
		picojson::array anchors;
		anchors.reserve(record.fieldAnchors.size());
		for (const auto &a : record.fieldAnchors)
		{
			picojson::object anchorObj;

			picojson::array pos, rot, tra;
			pos.reserve(3);
			rot.reserve(4);
			tra.reserve(3);
			for (int k = 0; k < 3; ++k)
			{
				pos.push_back(picojson::value(a.position(k)));
				tra.push_back(picojson::value(a.translationMeters(k)));
			}
			rot.push_back(picojson::value(a.rotation.w()));
			rot.push_back(picojson::value(a.rotation.x()));
			rot.push_back(picojson::value(a.rotation.y()));
			rot.push_back(picojson::value(a.rotation.z()));

			anchorObj["position"].set<picojson::array>(std::move(pos));
			anchorObj["rotation_quat"].set<picojson::array>(std::move(rot));
			anchorObj["translation_meters"].set<picojson::array>(std::move(tra));

			picojson::value anchorV;
			anchorV.set<picojson::object>(std::move(anchorObj));
			anchors.push_back(std::move(anchorV));
		}
		profile["field_anchors"].set<picojson::array>(std::move(anchors));
	}

	picojson::value profileV;
	profileV.set<picojson::object>(std::move(profile));

	picojson::array profiles;
	profiles.reserve(1);
	profiles.push_back(std::move(profileV));

	picojson::value profilesV;
	profilesV.set<picojson::array>(std::move(profiles));

	out << profilesV.serialize(true);
}

} // namespace questcal
