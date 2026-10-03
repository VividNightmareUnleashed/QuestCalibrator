#pragma once

// The Config record's JSON codec: envelope, profile fields, and the legacy
// global settings that Config-only releases embedded alongside them. Free of
// CalibrationContext and the registry so the harness can pin the write -> read
// identity. SettingsRecordJson.h shares these guarded primitives with the
// settings and chaperone codecs.

#include "JsonNesting.h"
#include "ProfileValidation.h"
#include "RecordBounds.h"

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
// Parsed numbers are always finite: picojson's parser throws on anything else.
inline double GetDouble(const picojson::value &v)
{
	if (!v.is<double>())
		throw std::runtime_error("expected number");
	const double number = v.get<double>();
	if (!std::isfinite(number)) throw std::runtime_error("expected finite number");
	return number;
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

// `obj` must already be type-checked as an array.
inline void LoadFloatArray(const picojson::value &obj, float *buf, size_t numFloats)
{
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

// A quaternion is stored as [w, x, y, z], a vector as [x, y, z].
inline picojson::array QuatArray(const Eigen::Quaterniond &q)
{
	return { picojson::value(q.w()), picojson::value(q.x()),
		picojson::value(q.y()), picojson::value(q.z()) };
}

inline picojson::array Vec3Array(const Eigen::Vector3d &v)
{
	return { picojson::value(v.x()), picojson::value(v.y()), picojson::value(v.z()) };
}

// Callers check the lengths first, to say which field is malformed; the
// check here only keeps a read inside the array.
inline Eigen::Quaterniond QuatFromArray(const picojson::array &arr)
{
	if (arr.size() != 4)
		throw std::runtime_error("wrong buffer size");
	return Eigen::Quaterniond(GetDouble(arr[0]), GetDouble(arr[1]),
		GetDouble(arr[2]), GetDouble(arr[3]));
}

inline Eigen::Vector3d Vec3FromArray(const picojson::array &arr)
{
	if (arr.size() != 3)
		throw std::runtime_error("wrong buffer size");
	return Eigen::Vector3d(GetDouble(arr[0]), GetDouble(arr[1]), GetDouble(arr[2]));
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

// Global preferences that Config-only releases stored inside the profile
// record. The codec only reports what it found; Configuration.cpp decides what
// to do with it.
struct LegacyProfileSettings
{
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

// The Config record's schema. It is stored under "settings_version", the key
// every earlier build reads, and older builds accept any whole number from 1
// to 100 there (IsValidSettingsVersion), so a record a newer build wrote still
// loads in an older one. A change an older record has to be brought through
// bumps it and adds its step to ConfigMigrations.
constexpr int ConfigSchema = 2;

struct ProfileParseResult
{
	PersistedRevision revision;
	int schema = ConfigSchema;   // the schema the record was written at
	bool migratedScaleSetting = false;
	bool suspiciousLegacyScale = false;
};

// picojson::parse reports malformed text as an error string, but a number past
// a double's range (1e999) throws std::overflow_error with an empty message.
// Both records parse through here so that value gets a reason like any other.
inline std::string ParseRecordJson(picojson::value &v, std::istream &stream)
{
	try
	{
        std::string text;
        char chunk[4096];
        while (stream) {
            stream.read(chunk, sizeof chunk);
            text.append(chunk, static_cast<size_t>(stream.gcount()));
            if (!IsValidRecordByteCount(text.size())) return "record is too large";
        }
        RejectExcessiveJsonNesting(text);
        return picojson::parse(v, text);
	}
	catch (const std::overflow_error &)
	{
		return "a number is out of range";
	}
}

// The Config envelope: a one-element array of profile objects. Returned by
// value so the caller can keep reading the same object (the chaperone snapshot
// is parsed on top of it by Configuration.cpp).
inline picojson::value ParseProfileEnvelope(std::istream &stream)
{
	picojson::value v;
	std::string err = ParseRecordJson(v, stream);
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

// One step of an older Config record towards ConfigSchema, from `from` to
// the next, on what the parser read.
struct ConfigMigration
{
	int from;
	void (*apply)(const ProfileRecord &profile, LegacyProfileSettings &legacy,
		ProfileParseResult &result);
};

// 1 to 2: scale solving goes off, whatever the record held. Streamed
// reference poses are motion-smoothed and the solved scale absorbs the
// attenuation. The applied scale is kept; it was solved jointly with the
// translation, and clearing it without re-solving would misalign the space.
inline void MigrateConfigScaleSolvingOff(const ProfileRecord &profile,
	LegacyProfileSettings &legacy, ProfileParseResult &result)
{
	legacy.hasSolveScale = true;
	legacy.solveScale = false;
	result.migratedScaleSetting = true;
	result.suspiciousLegacyScale = profile.scale < 0.98 || profile.scale > 1.02;
}

// In order, one per older schema: a record at schema s runs every step from
// s on.
inline constexpr ConfigMigration ConfigMigrations[] = {
	{ 1, MigrateConfigScaleSolvingOff },
};
static_assert(sizeof ConfigMigrations / sizeof ConfigMigrations[0] == ConfigSchema - 1,
	"one Config migration step for each older schema");

inline ProfileParseResult ParseProfileObjectUnchecked(ProfileRecord &profile,
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

	Eigen::Quaterniond rotation = QuatFromArray(quatArr);
	Eigen::Vector3d translationMeters = Vec3FromArray(transArr);

	double scale = HasTypedValue<double>(obj, "scale") ? GetDouble(obj.at("scale")) : 1.0;
	// Before normalizing: normalized() of a degenerate quaternion is NaN.
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

		Eigen::Quaterniond baselineRotation = QuatFromArray(universeRotation);
		Eigen::Vector3d baselineTranslation = Vec3FromArray(universeTranslation);
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

	// The schema the record was written at; one from before the key existed
	// is schema 1. ConfigMigrations brings it up to date below.
	double schemaValue = HasTypedValue<double>(obj, "settings_version")
		? GetDouble(obj.at("settings_version")) : 1.0;
	if (!IsValidSettingsVersion(schemaValue))
		throw std::runtime_error("invalid settings_version");
	result.schema = static_cast<int>(schemaValue);
	if (HasTypedValue<bool>(obj, "solve_scale"))
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

	// The switches and the continuous pick are optional: older profiles load
	// unchanged.
#define QUESTCAL_PARSE_SWITCH(member, key, value) \
	if (HasTypedValue<bool>(obj, key)) \
		profile.member = obj.at(key).get<bool>();
	QUESTCAL_PROFILE_SWITCHES(QUESTCAL_PARSE_SWITCH)
#undef QUESTCAL_PARSE_SWITCH

	if (HasTypedValue<std::string>(obj, "continuous_tracker_serial"))
		profile.continuousTrackerSerial =
			obj.at("continuous_tracker_serial").get<std::string>();
	// A name rather than a flag, so a hand-edited profile reads and an unknown
	// value is the default. "no_pause" is the Legacy method; "legacy" is what
	// the old Legacy solver saved, and players picked it to stop the pausing.
	if (HasTypedValue<std::string>(obj, "continuous_mode"))
	{
		const std::string mode = obj.at("continuous_mode").get<std::string>();
		profile.continuousNoPause = mode == "no_pause" || mode == "legacy";
	}

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

		Eigen::Quaterniond mountRotation = QuatFromArray(rotArr);
		Eigen::Vector3d mountPosition = Vec3FromArray(traArr);
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
		// Before the reserve, so an absurd count cannot allocate.
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
			anchor.position = Vec3FromArray(posArr);
			Eigen::Quaterniond anchorRotation = QuatFromArray(rotArr);
			anchor.translationMeters = Vec3FromArray(traArr);
			if (!IsValidFieldAnchor(anchor.position, anchorRotation,
				anchor.translationMeters, profile.rotation, profile.translationMeters))
				throw std::runtime_error("invalid field anchor");
			anchor.rotation = anchorRotation.normalized();

			profile.fieldAnchors.push_back(anchor);
		}
	}

	for (const auto &step : ConfigMigrations)
		if (step.from >= result.schema)
			step.apply(profile, legacy, result);

	profile.valid = true;

	// The record the parser produces must be one the writer accepts, exactly:
	// the inline checks judged anchors against the unnormalized rotation, and
	// absent timing fields keep the caller's values.
	std::string why;
	if (!ValidateProfileRecord(profile, maxAnchors, why))
		throw std::runtime_error(why);
	return result;
}

inline ProfileParseResult ParseProfileObject(ProfileRecord &destination,
    LegacyProfileSettings &legacyDestination, const picojson::object &obj, size_t maxAnchors)
{
    ProfileRecord parsed = destination;
    LegacyProfileSettings legacy = legacyDestination;
    const auto result = ParseProfileObjectUnchecked(parsed, legacy, obj, maxAnchors);
    destination = std::move(parsed);
    legacyDestination = legacy;
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

	profile["rotation_quat"].set<picojson::array>(QuatArray(record.rotation));
	profile["translation_meters"].set<picojson::array>(Vec3Array(record.translationMeters));
	profile["scale"].set<double>(record.scale);
	profile["time_offset"].set<double>(record.timeOffset);
	profile["calibration_time"].set<double>(record.calibrationUnixTime);
	profile["universe_unsafe"].set<bool>(record.universeUnsafe);
	if (!record.universeHmdSerial.empty())
		profile["hmd_serial"].set<std::string>(record.universeHmdSerial);
	if (record.universeValid)
	{
		profile["universe_hmd_serial"].set<std::string>(record.universeHmdSerial);
		profile["universe_world_from_driver_rotation_quat"].set<picojson::array>(
			QuatArray(record.universeRotation));
		profile["universe_world_from_driver_translation_meters"].set<picojson::array>(
			Vec3Array(record.universeTranslation));
	}
	// What ConfigMigrations brings every record to, so none re-runs.
	double schema = ConfigSchema;
	profile["settings_version"].set<double>(schema);
#define QUESTCAL_WRITE_SWITCH(member, key, value) profile[key].set<bool>(record.member);
	QUESTCAL_PROFILE_SWITCHES(QUESTCAL_WRITE_SWITCH)
#undef QUESTCAL_WRITE_SWITCH
	if (!record.continuousTrackerSerial.empty())
		profile["continuous_tracker_serial"].set<std::string>(record.continuousTrackerSerial);
	profile["continuous_mode"].set<std::string>(record.continuousNoPause ? "no_pause" : "questcalibrator");

	if (record.mountExtrinsic.valid)
	{
		picojson::object extrinsic;
		extrinsic["rotation_quat"].set<picojson::array>(
			QuatArray(record.mountExtrinsic.rotation));
		extrinsic["translation_meters"].set<picojson::array>(
			Vec3Array(record.mountExtrinsic.translationMeters));
		extrinsic["rot_rms_deg"].set<double>(record.mountExtrinsic.rotationRmsDeg);
		extrinsic["pos_rms_m"].set<double>(record.mountExtrinsic.translationRmsM);

		profile["mount_extrinsic"].set<picojson::object>(std::move(extrinsic));
	}

	if (!record.fieldAnchors.empty())
	{
		picojson::array anchors;
		anchors.reserve(record.fieldAnchors.size());
		for (const auto &a : record.fieldAnchors)
		{
			picojson::object anchorObj;
			anchorObj["position"].set<picojson::array>(Vec3Array(a.position));
			anchorObj["rotation_quat"].set<picojson::array>(QuatArray(a.rotation));
			anchorObj["translation_meters"].set<picojson::array>(Vec3Array(a.translationMeters));

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
