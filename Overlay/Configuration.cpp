#include "stdafx.h"
#include "Configuration.h"
#include "ChaperoneMath.h"
#include "ProfileValidation.h"
#include "UserInterface.h"
#include "../common/Protocol.h"

#include <picojson.h>

#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <limits>
#include <cmath>

static constexpr DWORD MaxRegistryValueBytes = 16u * 1024u * 1024u;

struct PersistedRevision
{
	bool present = false;
	uint32_t value = 0;
};

// Serialization owns narrow records rather than cloning CalibrationContext.
// These contain exactly the values represented in Config/Settings; live poses,
// solver buffers, monitor state, UI messages and retry metadata never cross the
// persistence boundary.
struct ChaperoneRecord
{
	bool valid = false;
	bool autoApply = true;
	std::string ownerTrackingSystem;
	std::string ownerHmdSerial;
	bool worldFromDriverValid = false;
	Eigen::Quaterniond worldFromDriverRotation{ 1, 0, 0, 0 };
	Eigen::Vector3d worldFromDriverTranslation{ 0, 0, 0 };
	std::vector<vr::HmdQuad_t> geometry;
	vr::HmdMatrix34_t standingCenter{};
	vr::HmdVector2_t playSpaceSize{};
	double copyUnixTime = 0.0;
};

struct SettingsRecord
{
	bool uiAdvanced = false;
	bool notifyPoorCalibration = true;
	bool chaperoneWarningAck = false;
	CalibrationContext::Speed calibrationSpeed = CalibrationContext::FAST;
	bool solveScale = false;
	bool applyTimeOffset = true;
	ChaperoneRecord chaperone;
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
	bool fieldEnabled = true;
	std::vector<CalibrationContext::FieldAnchor> fieldAnchors;
	bool continuousEnabled = false;
	std::string continuousTrackerSerial;
	bool continuousLatencyReestimation = false;
	bool hideMountedTracker = true;
	MountExtrinsicRecord mountExtrinsic;
};

struct ProfileParseResult
{
	PersistedRevision revision;
	bool migratedScaleSetting = false;
	bool suspiciousLegacyScale = false;
};

static ChaperoneRecord CaptureChaperoneRecord(
	const CalibrationContext::Chaperone &source)
{
	ChaperoneRecord record;
	if (!source.valid)
		return record;
	record.valid = source.valid;
	record.autoApply = source.autoApply;
	record.ownerTrackingSystem = source.ownerTrackingSystem;
	record.ownerHmdSerial = source.ownerHmdSerial;
	record.worldFromDriverValid = source.worldFromDriverValid;
	record.worldFromDriverRotation = source.worldFromDriverRotation;
	record.worldFromDriverTranslation = source.worldFromDriverTranslation;
	record.geometry = source.geometry;
	record.standingCenter = source.standingCenter;
	record.playSpaceSize = source.playSpaceSize;
	record.copyUnixTime = source.copyUnixTime;
	return record;
}

static SettingsRecord CaptureSettingsRecord(const CalibrationContext &ctx)
{
	SettingsRecord record;
	record.uiAdvanced = ctx.uiAdvanced;
	record.notifyPoorCalibration = ctx.notifyPoorCalibration;
	record.chaperoneWarningAck = ctx.chaperoneWarningAck;
	record.calibrationSpeed = ctx.calibrationSpeed;
	record.solveScale = ctx.solveScale;
	record.applyTimeOffset = ctx.applyTimeOffset;
	record.chaperone = CaptureChaperoneRecord(ctx.chaperone);
	return record;
}

static ProfileRecord CaptureProfileRecord(const CalibrationContext &ctx)
{
	ProfileRecord record;
	record.valid = ctx.validProfile;
	record.referenceTrackingSystem = ctx.referenceTrackingSystem;
	record.targetTrackingSystem = ctx.targetTrackingSystem;
	record.rotation = ctx.calibratedRotationQ;
	record.translationMeters = ctx.TranslationMeters();
	record.scale = ctx.calibratedScale;
	record.timeOffset = ctx.calibratedTimeOffset;
	record.calibrationUnixTime = ctx.calibrationUnixTime;
	record.fieldEnabled = ctx.fieldEnabled;
	record.fieldAnchors = ctx.fieldAnchors;
	record.continuousEnabled = ctx.continuousEnabled;
	record.continuousTrackerSerial = ctx.continuousTrackerSerial;
	record.continuousLatencyReestimation = ctx.continuousLatencyReestimation;
	record.hideMountedTracker = ctx.hideMountedTracker;
	record.mountExtrinsic.valid = ctx.mountExtrinsic.valid;
	record.mountExtrinsic.rotation = ctx.mountExtrinsic.rot;
	record.mountExtrinsic.translationMeters = ctx.mountExtrinsic.pos;
	record.mountExtrinsic.rotationRmsDeg = ctx.mountExtrinsic.rotRmsDeg;
	record.mountExtrinsic.translationRmsM = ctx.mountExtrinsic.posRmsM;
	return record;
}

static void ApplyChaperoneRecord(CalibrationContext &ctx, ChaperoneRecord record)
{
	CalibrationContext::Chaperone applied;
	applied.valid = record.valid;
	applied.autoApply = record.autoApply;
	applied.ownerTrackingSystem = std::move(record.ownerTrackingSystem);
	applied.ownerHmdSerial = std::move(record.ownerHmdSerial);
	applied.worldFromDriverValid = record.worldFromDriverValid;
	applied.worldFromDriverRotation = record.worldFromDriverRotation;
	applied.worldFromDriverTranslation = record.worldFromDriverTranslation;
	applied.geometry = std::move(record.geometry);
	applied.standingCenter = record.standingCenter;
	applied.playSpaceSize = record.playSpaceSize;
	applied.copyUnixTime = record.copyUnixTime;
	// Runtime verification/cooldown state deliberately starts fresh after load.
	applied.baselineVerifiedThisSession = false;
	applied.lastRestoreTime = 0.0;
	ctx.chaperone = std::move(applied);
}

static void ApplySettingsRecord(CalibrationContext &ctx, SettingsRecord record)
{
	ctx.uiAdvanced = record.uiAdvanced;
	ctx.notifyPoorCalibration = record.notifyPoorCalibration;
	ctx.chaperoneWarningAck = record.chaperoneWarningAck;
	ctx.calibrationSpeed = record.calibrationSpeed;
	ctx.solveScale = record.solveScale;
	ctx.applyTimeOffset = record.applyTimeOffset;
	ApplyChaperoneRecord(ctx, std::move(record.chaperone));
}

static void ApplyProfileRecord(CalibrationContext &ctx, ProfileRecord record)
{
	ctx.referenceTrackingSystem = std::move(record.referenceTrackingSystem);
	ctx.targetTrackingSystem = std::move(record.targetTrackingSystem);
	ctx.SetCalibration(record.rotation, record.translationMeters, record.scale);
	ctx.calibratedTimeOffset = record.timeOffset;
	ctx.calibrationUnixTime = record.calibrationUnixTime;
	ctx.fieldEnabled = record.fieldEnabled;
	ctx.fieldAnchors = std::move(record.fieldAnchors);
	ctx.continuousEnabled = record.continuousEnabled;
	ctx.continuousTrackerSerial = std::move(record.continuousTrackerSerial);
	ctx.continuousLatencyReestimation = record.continuousLatencyReestimation;
	ctx.hideMountedTracker = record.hideMountedTracker;
	ctx.mountExtrinsic = questcal::MountExtrinsic();
	ctx.mountExtrinsic.valid = record.mountExtrinsic.valid;
	ctx.mountExtrinsic.rot = record.mountExtrinsic.rotation;
	ctx.mountExtrinsic.pos = record.mountExtrinsic.translationMeters;
	ctx.mountExtrinsic.rotRmsDeg = record.mountExtrinsic.rotationRmsDeg;
	ctx.mountExtrinsic.posRmsM = record.mountExtrinsic.translationRmsM;
	ctx.validProfile = record.valid;
}

static picojson::array FloatArray(const float *buf, size_t numFloats)
{
	picojson::array arr;
	arr.reserve(numFloats);

	for (size_t i = 0; i < numFloats; i++)
		arr.push_back(picojson::value(double(buf[i])));

	return arr;
}

// picojson's get<T>() is guarded only by assert() (compiled out in Release), so
// every read of untrusted profile JSON must type-check first to reach the
// intended runtime_error path instead of reading the wrong union member.
static double GetDouble(const picojson::value &v)
{
	if (!v.is<double>())
		throw std::runtime_error("expected number, got " + v.to_str());
	double value = v.get<double>();
	if (!std::isfinite(value))
		throw std::runtime_error("expected finite number");
	return value;
}

template<typename T>
static bool HasTypedValue(const picojson::object &obj, const char *name)
{
	auto it = obj.find(name);
	if (it == obj.end())
		return false;
	if (!it->second.is<T>())
		throw std::runtime_error(std::string("invalid type for ") + name);
	return true;
}

static PersistedRevision ReadPersistenceRevision(const picojson::object &obj)
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

static void LoadFloatArray(const picojson::value &obj, float *buf, size_t numFloats)
{
	if (!obj.is<picojson::array>())
		throw std::runtime_error("expected array, got " + obj.to_str());

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

static void ParseChaperone(SettingsRecord &settings, const picojson::object &obj)
{
	if (!HasTypedValue<picojson::object>(obj, "chaperone"))
		return;

	const auto &chaperone = obj.at("chaperone").get<picojson::object>();
	ChaperoneRecord parsed;
	if (HasTypedValue<bool>(chaperone, "auto_apply"))
		parsed.autoApply = chaperone.at("auto_apply").get<bool>();
	if (HasTypedValue<std::string>(chaperone, "owner_tracking_system"))
		parsed.ownerTrackingSystem = chaperone.at("owner_tracking_system").get<std::string>();
	if (HasTypedValue<std::string>(chaperone, "owner_hmd_serial"))
		parsed.ownerHmdSerial = chaperone.at("owner_hmd_serial").get<std::string>();

	bool hasWorldRotation = HasTypedValue<picojson::array>(
		chaperone, "world_from_driver_rotation_quat");
	bool hasWorldTranslation = HasTypedValue<picojson::array>(
		chaperone, "world_from_driver_translation_meters");
	if (hasWorldRotation != hasWorldTranslation)
		throw std::runtime_error("incomplete chaperone worldFromDriver baseline");
	if (hasWorldRotation)
	{
		const auto &rotation = chaperone.at(
			"world_from_driver_rotation_quat").get<picojson::array>();
		const auto &translation = chaperone.at(
			"world_from_driver_translation_meters").get<picojson::array>();
		if (rotation.size() != 4 || translation.size() != 3)
			throw std::runtime_error("malformed chaperone worldFromDriver baseline");

		Eigen::Quaterniond baselineRotation(
			GetDouble(rotation[0]), GetDouble(rotation[1]),
			GetDouble(rotation[2]), GetDouble(rotation[3]));
		Eigen::Vector3d baselineTranslation(
			GetDouble(translation[0]), GetDouble(translation[1]),
			GetDouble(translation[2]));
		if (!questcal::IsValidRotation(baselineRotation) ||
			!questcal::IsBoundedVector(baselineTranslation,
				protocol::limits::MaxAbsTranslationMeters))
			throw std::runtime_error("invalid chaperone worldFromDriver baseline");

		parsed.worldFromDriverRotation = baselineRotation.normalized();
		parsed.worldFromDriverTranslation = baselineTranslation;
		parsed.worldFromDriverValid = true;
	}

	// Required, unlike every optional field above. These were read with a bare
	// .at(), so a missing key threw out_of_range carrying picojson's own
	// "invalid map<K, T> key" - naming neither the field nor the record - and
	// failed the WHOLE record: on the Config path a damaged room snapshot
	// discarded a good calibration, on the Settings path it locked every
	// settings write permanently. Same inputs rejected, but say what broke.
	for (const char *required : { "play_space_size", "standing_center", "geometry" })
		if (!HasTypedValue<picojson::array>(chaperone, required))
			throw std::runtime_error(
				std::string("chaperone is missing the array ") + required);

	LoadFloatArray(chaperone.at("play_space_size"), parsed.playSpaceSize.v, 2);
	LoadFloatArray(chaperone.at("standing_center"),
		reinterpret_cast<float *>(parsed.standingCenter.m),
		sizeof(parsed.standingCenter.m) / sizeof(float));

	auto &geometry = chaperone.at("geometry").get<picojson::array>();

	// HmdQuad_t is twelve packed floats. Reject partial or absurd payloads
	// before allocating or filling the destination buffer.
	constexpr size_t floatsPerQuad = sizeof(vr::HmdQuad_t) / sizeof(float);
	if (geometry.size() % floatsPerQuad != 0 || geometry.size() > 16384 * floatsPerQuad)
		throw std::runtime_error("chaperone geometry has invalid length");

	parsed.geometry.resize(geometry.size() / floatsPerQuad);
	if (!geometry.empty())
		LoadFloatArray(chaperone.at("geometry"),
			reinterpret_cast<float *>(parsed.geometry.data()), geometry.size());

	if (HasTypedValue<double>(chaperone, "copy_time"))
	{
		parsed.copyUnixTime = GetDouble(chaperone.at("copy_time"));
		if (!std::isfinite(parsed.copyUnixTime) || parsed.copyUnixTime < 0.0 ||
			parsed.copyUnixTime > protocol::limits::MaxPlausibleUnixTimeSeconds)
			throw std::runtime_error("invalid chaperone copy_time");
	}
	if (!questcal::IsPlausibleChaperone(parsed.geometry, parsed.standingCenter,
		parsed.playSpaceSize))
		throw std::runtime_error("invalid chaperone geometry or standing transform");

	parsed.valid = true;
	settings.chaperone = std::move(parsed);
}

static void WriteChaperone(const SettingsRecord &settings, picojson::object &obj)
{
	const auto &snapshot = settings.chaperone;
	if (!snapshot.valid)
		return;

	picojson::object chaperone;
	chaperone["auto_apply"].set<bool>(snapshot.autoApply);
	chaperone["owner_tracking_system"].set<std::string>(snapshot.ownerTrackingSystem);
	chaperone["owner_hmd_serial"].set<std::string>(snapshot.ownerHmdSerial);
	if (snapshot.worldFromDriverValid)
	{
		picojson::array rotation;
		rotation.reserve(4);
		rotation.push_back(picojson::value(snapshot.worldFromDriverRotation.w()));
		rotation.push_back(picojson::value(snapshot.worldFromDriverRotation.x()));
		rotation.push_back(picojson::value(snapshot.worldFromDriverRotation.y()));
		rotation.push_back(picojson::value(snapshot.worldFromDriverRotation.z()));
		chaperone["world_from_driver_rotation_quat"].set<picojson::array>(std::move(rotation));

		picojson::array translation;
		translation.reserve(3);
		for (int axis = 0; axis < 3; ++axis)
			translation.push_back(picojson::value(
				snapshot.worldFromDriverTranslation(axis)));
		chaperone["world_from_driver_translation_meters"].set<picojson::array>(std::move(translation));
	}
	chaperone["play_space_size"].set<picojson::array>(FloatArray(snapshot.playSpaceSize.v, 2));
	chaperone["standing_center"].set<picojson::array>(FloatArray(
		reinterpret_cast<const float *>(snapshot.standingCenter.m),
		sizeof(snapshot.standingCenter.m) / sizeof(float)));
	chaperone["geometry"].set<picojson::array>(FloatArray(
		reinterpret_cast<const float *>(snapshot.geometry.data()),
		(sizeof(vr::HmdQuad_t) / sizeof(float)) * snapshot.geometry.size()));
	chaperone["copy_time"].set<double>(snapshot.copyUnixTime);
	obj["chaperone"].set<picojson::object>(std::move(chaperone));
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
static void RejectExcessiveJsonNesting(const std::string &text)
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

static ProfileParseResult ParseProfile(ProfileRecord &profile,
	SettingsRecord &legacySettings, std::istream &stream)
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
	const auto &obj = arr[0].get<picojson::object>();
	ProfileParseResult result;
	result.revision = ReadPersistenceRevision(obj);

	if (!HasTypedValue<std::string>(obj, "reference_tracking_system") ||
		!HasTypedValue<std::string>(obj, "target_tracking_system"))
		throw std::runtime_error("profile is missing the tracking system names");
	profile.referenceTrackingSystem = obj.at("reference_tracking_system").get<std::string>();
	profile.targetTrackingSystem = obj.at("target_tracking_system").get<std::string>();
	if (!questcal::IsValidTrackingSystemPair(
		profile.referenceTrackingSystem, profile.targetTrackingSystem))
		throw std::runtime_error("tracking system names must be non-empty and different");

	// The quaternion is the stored truth; Euler display values are derived
	// from it by SetCalibration below.
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
	if (!questcal::IsValidCalibrationTransform(rotation, translationMeters, scale))
		throw std::runtime_error("invalid calibration transform");
	profile.rotation = rotation.normalized();
	profile.translationMeters = translationMeters;
	profile.scale = scale;

	if (HasTypedValue<double>(obj, "time_offset"))
	{
		profile.timeOffset = GetDouble(obj.at("time_offset"));
		if (std::abs(profile.timeOffset) >
				protocol::limits::MaxAbsTimeOffsetSeconds)
			throw std::runtime_error("invalid time_offset");
	}

	// Unix seconds of the last successful solve; 0 = unknown (older profile).
	if (HasTypedValue<double>(obj, "calibration_time"))
	{
		profile.calibrationUnixTime = GetDouble(obj.at("calibration_time"));
		if (profile.calibrationUnixTime < 0.0 ||
			profile.calibrationUnixTime >
				protocol::limits::MaxPlausibleUnixTimeSeconds)
			throw std::runtime_error("invalid calibration_time");
	}

	if (HasTypedValue<bool>(obj, "apply_time_offset"))
		legacySettings.applyTimeOffset = obj.at("apply_time_offset").get<bool>();

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
	int settingsVersion = static_cast<int>(settingsVersionValue);
	bool hasSolveScale = HasTypedValue<bool>(obj, "solve_scale");
	if (settingsVersion >= 2 && hasSolveScale)
		legacySettings.solveScale = obj.at("solve_scale").get<bool>();

	if (HasTypedValue<bool>(obj, "ui_advanced"))
		legacySettings.uiAdvanced = obj.at("ui_advanced").get<bool>();

	if (HasTypedValue<bool>(obj, "chaperone_warning_ack"))
		legacySettings.chaperoneWarningAck = obj.at("chaperone_warning_ack").get<bool>();

	if (HasTypedValue<double>(obj, "calibration_speed"))
	{
		double speed = GetDouble(obj.at("calibration_speed"));
		if (speed < CalibrationContext::FAST || speed > CalibrationContext::VERY_SLOW ||
			std::floor(speed) != speed)
			throw std::runtime_error("invalid calibration_speed");
		legacySettings.calibrationSpeed =
			static_cast<CalibrationContext::Speed>(static_cast<int>(speed));
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

	if (HasTypedValue<bool>(obj, "hide_mounted_tracker"))
		profile.hideMountedTracker = obj.at("hide_mounted_tracker").get<bool>();

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
		if (!questcal::IsValidRotation(mountRotation) ||
			!questcal::IsBoundedVector(mountPosition,
				protocol::limits::MaxAbsAnchorDeltaMeters))
			throw std::runtime_error("invalid mount_extrinsic");

		profile.mountExtrinsic.rotation = mountRotation.normalized();
		profile.mountExtrinsic.translationMeters = mountPosition;
		if (HasTypedValue<double>(extrinsic, "rot_rms_deg"))
			profile.mountExtrinsic.rotationRmsDeg = GetDouble(extrinsic.at("rot_rms_deg"));
		if (HasTypedValue<double>(extrinsic, "pos_rms_m"))
			profile.mountExtrinsic.translationRmsM = GetDouble(extrinsic.at("pos_rms_m"));
		if (!questcal::IsValidResidual(profile.mountExtrinsic.rotationRmsDeg) ||
			!questcal::IsValidResidual(profile.mountExtrinsic.translationRmsM))
			throw std::runtime_error("invalid mount_extrinsic residual");
		profile.mountExtrinsic.valid = true;
	}

	profile.fieldAnchors.clear();
	if (HasTypedValue<picojson::array>(obj, "field_anchors"))
	{
		const auto &fieldAnchors = obj.at("field_anchors").get<picojson::array>();
		if (fieldAnchors.size() > protocol::SetAlignmentField::MaxAnchors)
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

			CalibrationContext::FieldAnchor anchor;
			anchor.position = Eigen::Vector3d(GetDouble(posArr[0]), GetDouble(posArr[1]), GetDouble(posArr[2]));
			Eigen::Quaterniond anchorRotation(GetDouble(rotArr[0]), GetDouble(rotArr[1]),
				GetDouble(rotArr[2]), GetDouble(rotArr[3]));
			anchor.translationMeters = Eigen::Vector3d(GetDouble(traArr[0]), GetDouble(traArr[1]), GetDouble(traArr[2]));
			if (!questcal::IsValidFieldAnchor(anchor.position, anchorRotation,
				anchor.translationMeters, profile.rotation, profile.translationMeters))
				throw std::runtime_error("invalid field anchor");
			anchor.rotation = anchorRotation.normalized();

			profile.fieldAnchors.push_back(anchor);
		}
	}

	ParseChaperone(legacySettings, obj);

	if (settingsVersion < 2)
	{
		legacySettings.solveScale = false;
		result.migratedScaleSetting = true;
		result.suspiciousLegacyScale = profile.scale < 0.98 || profile.scale > 1.02;
	}

	profile.valid = true;
	return result;
}

static void WriteProfile(const ProfileRecord &record,
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
	// Bumped when a load-time migration must not re-run (see ParseProfile).
	double settingsVersion = 2.0;
	profile["settings_version"].set<double>(settingsVersion);
	profile["continuous_enabled"].set<bool>(record.continuousEnabled);
	if (!record.continuousTrackerSerial.empty())
		profile["continuous_tracker_serial"].set<std::string>(record.continuousTrackerSerial);
	profile["continuous_latency_reestimation"].set<bool>(record.continuousLatencyReestimation);
	profile["hide_mounted_tracker"].set<bool>(record.hideMountedTracker);

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

static void WriteSettings(const SettingsRecord &record,
	uint32_t persistenceRevisionValue, std::ostream &out)
{
	picojson::object settings;
	double settingsVersion = 1.0;
	double calibrationSpeed = static_cast<double>(record.calibrationSpeed);
	double persistenceRevision = static_cast<double>(persistenceRevisionValue);
	settings["persistence_revision"].set<double>(persistenceRevision);
	settings["settings_version"].set<double>(settingsVersion);
	settings["ui_advanced"].set<bool>(record.uiAdvanced);
	settings["notify_poor_calibration"].set<bool>(record.notifyPoorCalibration);
	settings["chaperone_warning_ack"].set<bool>(record.chaperoneWarningAck);
	settings["calibration_speed"].set<double>(calibrationSpeed);
	settings["solve_scale"].set<bool>(record.solveScale);
	settings["apply_time_offset"].set<bool>(record.applyTimeOffset);
	WriteChaperone(record, settings);

	picojson::value value;
	value.set<picojson::object>(std::move(settings));
	out << value.serialize(true);
}

static PersistedRevision ParseSettings(SettingsRecord &settings, std::istream &stream)
{
	picojson::value value;
	std::string err = picojson::parse(value, stream);
	if (!err.empty())
		throw std::runtime_error(err);
	if (!value.is<picojson::object>())
		throw std::runtime_error("settings are not an object");
	const auto &obj = value.get<picojson::object>();
	PersistedRevision revision = ReadPersistenceRevision(obj);

	if (HasTypedValue<double>(obj, "settings_version"))
	{
		double version = GetDouble(obj.at("settings_version"));
		if (version < 1.0 || version > 100.0 || std::floor(version) != version)
			throw std::runtime_error("invalid settings_version");
	}
	if (HasTypedValue<bool>(obj, "ui_advanced"))
		settings.uiAdvanced = obj.at("ui_advanced").get<bool>();
	if (HasTypedValue<bool>(obj, "notify_poor_calibration"))
		settings.notifyPoorCalibration = obj.at("notify_poor_calibration").get<bool>();
	if (HasTypedValue<bool>(obj, "chaperone_warning_ack"))
		settings.chaperoneWarningAck = obj.at("chaperone_warning_ack").get<bool>();
	if (HasTypedValue<bool>(obj, "solve_scale"))
		settings.solveScale = obj.at("solve_scale").get<bool>();
	if (HasTypedValue<bool>(obj, "apply_time_offset"))
		settings.applyTimeOffset = obj.at("apply_time_offset").get<bool>();
	if (HasTypedValue<double>(obj, "calibration_speed"))
	{
		double speed = GetDouble(obj.at("calibration_speed"));
		if (speed < CalibrationContext::FAST || speed > CalibrationContext::VERY_SLOW ||
			std::floor(speed) != speed)
			throw std::runtime_error("invalid calibration_speed");
		settings.calibrationSpeed =
			static_cast<CalibrationContext::Speed>(static_cast<int>(speed));
	}
	// Once Settings exists it is authoritative for chaperone state.  Absence
	// means deliberately unarmed, not "fall back to an old embedded snapshot".
	settings.chaperone = ChaperoneRecord();
	ParseChaperone(settings, obj);
	return revision;
}

static std::string RegistryError(LSTATUS result)
{
	char *message = nullptr;
	DWORD chars = FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_ALLOCATE_BUFFER |
		FORMAT_MESSAGE_IGNORE_INSERTS, nullptr, result, LANG_USER_DEFAULT,
		reinterpret_cast<LPSTR>(&message), 0, nullptr);
	std::string text = chars != 0 && message ? message : "Windows error " + std::to_string(result);
	if (message)
		LocalFree(message);
	while (!text.empty() && (text.back() == '\r' || text.back() == '\n' || text.back() == ' '))
		text.pop_back();
	return text;
}

static const char *RegistryKey = "Software\\QuestCalibrator";

// HKEY_CURRENT_USER_LOCAL_SETTINGS is a predefined handle, not a location
// regedit displays: it resolves under HKCU\Software\Classes\Local Settings.
// An unreadable record is fail-closed by design (it must never be silently
// overwritten), so clearing it by hand is the only escape - and the error text
// is the only place the user can learn where "it" actually is.
static const char *RegistryKeyDisplayPath =
	"HKEY_CURRENT_USER\\Software\\Classes\\Local Settings\\Software\\QuestCalibrator";

enum class RegistryReadStatus
{
	Missing,
	Present,
	Error,
};

struct RegistryReadResult
{
	RegistryReadStatus status = RegistryReadStatus::Missing;
	std::string value;
	std::string error;
};

static RegistryReadResult ReadRegistryValue(const char *valueName)
{
	DWORD size = 0;
	auto result = RegGetValueA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, valueName, RRF_RT_REG_SZ, 0, 0, &size);
	if (result != ERROR_SUCCESS)
	{
		if (result == ERROR_FILE_NOT_FOUND)
			return {};
		return { RegistryReadStatus::Error, {},
			std::string("reading ") + valueName + ": " + RegistryError(result) };
	}

	// size counts the trailing NUL; zero would underflow the resize below.
	if (size == 0)
		return { RegistryReadStatus::Present, {}, {} };
	if (size > MaxRegistryValueBytes)
	{
		return { RegistryReadStatus::Error, {},
			std::string(valueName) + " exceeds the 16 MiB safety limit" };
	}

	std::string str;
	str.resize(size);

	result = RegGetValueA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, valueName, RRF_RT_REG_SZ, 0, &str[0], &size);
	if (result != ERROR_SUCCESS)
	{
		return { RegistryReadStatus::Error, {},
			std::string("reading ") + valueName + ": " + RegistryError(result) };
	}

	if (size == 0)
		return { RegistryReadStatus::Present, {}, {} };

	str.resize(size - 1);
	return { RegistryReadStatus::Present, std::move(str), {} };
}

static bool WriteRegistryValue(const char *valueName, const std::string &str, std::string &error)
{
	if (str.size() >= MaxRegistryValueBytes)
	{
		error = std::string(valueName) + " exceeds the 16 MiB safety limit";
		return false;
	}

	HKEY hkey = nullptr;
	auto result = RegCreateKeyExA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, 0, REG_NONE, 0,
		KEY_SET_VALUE, nullptr, &hkey, nullptr);
	if (result != ERROR_SUCCESS)
	{
		error = "opening the registry key: " + RegistryError(result);
		return false;
	}

	DWORD size = static_cast<DWORD>(str.size() + 1);

	result = RegSetValueExA(hkey, valueName, 0, REG_SZ,
		reinterpret_cast<const BYTE *>(str.c_str()), size);
	RegCloseKey(hkey);
	if (result != ERROR_SUCCESS)
	{
		error = std::string("writing ") + valueName + ": " + RegistryError(result);
		return false;
	}
	return true;
}

void LoadProfile(CalibrationContext &ctx)
{
	ctx.profileUniverseUnsafe = false;
	ctx.chaperone.baselineVerifiedThisSession = false;
	ctx.profileLoadState = questcal::RecordLoadState::Missing;
	ctx.settingsLoadState = questcal::RecordLoadState::Missing;
	PersistedRevision profileRevision;
	PersistedRevision settingsRevision;
	bool settingsRewriteNeeded = false;

	auto profileRead = ReadRegistryValue("Config");
	if (profileRead.status == RegistryReadStatus::Error)
	{
		ctx.profileLoadState = questcal::RecordLoadState::Unreadable;
		ctx.ReportError("Could not read the calibration profile: " + profileRead.error + "\n",
			CalibrationContext::ErrorSource::ProfilePersistence);
	}
	else if (profileRead.status == RegistryReadStatus::Missing || profileRead.value.empty())
	{
		std::cout << "Profile is empty" << std::endl;
		ctx.Clear();
	}
	else
	{
		try
		{
			RejectExcessiveJsonNesting(profileRead.value);
			std::stringstream io(profileRead.value);
			// Parse transactionally. A malformed late field must not leave an
			// earlier transform active after the overall profile load failed.
			ProfileRecord profile = CaptureProfileRecord(ctx);
			SettingsRecord legacySettings = CaptureSettingsRecord(ctx);
			profile.valid = false;
			ProfileParseResult parsed = ParseProfile(profile, legacySettings, io);
			profileRevision = parsed.revision;
			ApplyProfileRecord(ctx, std::move(profile));
			ApplySettingsRecord(ctx, std::move(legacySettings));
			ctx.profileLoadState = questcal::RecordLoadState::Loaded;
			if (parsed.migratedScaleSetting)
			{
				ctx.Log("Playspace scale solving is now opt-in and has been turned off for this profile (re-enable it in settings if you need it)\n");
				if (parsed.suspiciousLegacyScale)
				{
					ctx.Log("The stored playspace scale (" +
						std::to_string(ctx.calibratedScale).substr(0, 5) +
						"x) likely came from streamed-pose smoothing -- recalibrate to clear it\n");
				}
			}
			ctx.ClearError(CalibrationContext::ErrorSource::ProfilePersistence);
			std::cout << "Loaded profile" << std::endl;
		}
		catch (const std::exception &e)
		{
			ctx.profileLoadState = questcal::RecordLoadState::Unreadable;
			ctx.ReportError(std::string("Error loading calibration profile: ") + e.what() +
					"\nThe profile is preserved, not overwritten. To start over, delete the"
					" Config value under " + RegistryKeyDisplayPath + "\n",
				CalibrationContext::ErrorSource::ProfilePersistence);
		}
	}

	// Independent settings keep chaperone protection and global preferences
	// alive even when there is no calibration profile. Existing Config-only
	// installations continue to load their embedded copies unchanged.
	auto settingsRead = ReadRegistryValue("Settings");
	if (settingsRead.status == RegistryReadStatus::Error)
	{
		ctx.settingsLoadState = questcal::RecordLoadState::Unreadable;
		ctx.ReportError("Could not read application settings: " + settingsRead.error + "\n",
			CalibrationContext::ErrorSource::SettingsPersistence);
	}
	else if (settingsRead.status == RegistryReadStatus::Missing || settingsRead.value.empty())
	{
	}
	else if (settingsRead.status == RegistryReadStatus::Present && !settingsRead.value.empty())
	{
		try
		{
			SettingsRecord parsed = CaptureSettingsRecord(ctx);
			RejectExcessiveJsonNesting(settingsRead.value);
			std::stringstream io(settingsRead.value);
			settingsRevision = ParseSettings(parsed, io);
			ApplySettingsRecord(ctx, std::move(parsed));
			ctx.settingsLoadState = questcal::RecordLoadState::Loaded;
			ctx.ClearError(CalibrationContext::ErrorSource::SettingsPersistence);
		}
		catch (const std::exception &e)
		{
			ctx.settingsLoadState = questcal::RecordLoadState::Unreadable;
			// Until this record is cleared every settings write is refused, so
			// the toggles in the UI will silently roll back. Say where it is.
			ctx.ReportError(std::string("Error loading application settings: ") + e.what() +
					"\nSettings changes will not save until this is fixed. To start over,"
					" delete the Settings value under " + RegistryKeyDisplayPath + "\n",
				CalibrationContext::ErrorSource::SettingsPersistence);
		}
	}
	bool profileLoaded = ctx.profileLoadState == questcal::RecordLoadState::Loaded;
	bool settingsLoaded = ctx.settingsLoadState == questcal::RecordLoadState::Loaded;
	bool settingsMissing = ctx.settingsLoadState == questcal::RecordLoadState::Missing;
	bool settingsRecoveryAllowed = questcal::CanUseRecoveredSettings(
		ctx.settingsLoadState, ctx.profileLoadState);
	bool settingsMaterializationAllowed = questcal::CanMaterializeSettings(
		ctx.profileLoadState);
	bool settingsCanRewrite = (settingsMissing || settingsLoaded) &&
		settingsMaterializationAllowed;

	// Config is written before Settings for a coupled universe rebase.  If the
	// process dies between those writes, their revisions differ (or Settings
	// is absent).  Never restore a raw-space boundary from that mixed state.
	if (profileLoaded && profileRevision.present)
	{
		ctx.persistenceRevision = profileRevision.value;
		if (!settingsLoaded || !settingsRevision.present ||
			settingsRevision.value != profileRevision.value)
		{
			bool hadSnapshot = ctx.chaperone.valid;
			if (hadSnapshot)
				ctx.DisarmChaperone();
			settingsRewriteNeeded = settingsCanRewrite;
			if (hadSnapshot)
			{
				ctx.ReportError(
					"The calibration profile and protected chaperone were saved at different revisions. "
					"Chaperone auto-restore is disabled until you capture it again.\n",
					CalibrationContext::ErrorSource::Chaperone);
			}
		}
	}
	else if (profileLoaded)
	{
		// Config-only releases kept the sole copy of global settings and the
		// chaperone inside Config.  Materialize Settings before any new-format
		// profile save is allowed to strip those embedded fields.
		ctx.persistenceRevision = settingsRevision.present ? settingsRevision.value : 1;
		if (!settingsLoaded || !settingsRevision.present)
		{
			ctx.legacySettingsMigrationPending = true;
			settingsRewriteNeeded = settingsCanRewrite;
		}
	}
	else
	{
		ctx.persistenceRevision = settingsRevision.present ? settingsRevision.value : 1;
		if (settingsMissing || (settingsLoaded && !settingsRevision.present))
		{
			settingsRewriteNeeded = settingsCanRewrite;
		}
	}

	if (!settingsRecoveryAllowed)
	{
		// Config may be a legacy Config-only record containing the sole copy of
		// the protected room and global preferences. With no separately parsed
		// Settings record, do not persist this conservative in-memory disarm over
		// recoverable data.
		if (ctx.chaperone.valid)
			ctx.DisarmChaperone();
		ctx.Log("Protected chaperone left disarmed because the calibration profile could not be read\n");
	}
	else if (ctx.settingsLoadState == questcal::RecordLoadState::Unreadable &&
		ctx.chaperone.valid)
	{
		// A successfully parsed Config may contain a legacy fallback snapshot,
		// but a present-yet-unreadable Settings record is authoritative.  Keep
		// the parse error visible and never arm the fallback implicitly.
		ctx.DisarmChaperone();
		ctx.Log("Protected chaperone left disarmed because application settings could not be read\n");
	}
	else if (ctx.chaperone.valid &&
		(ctx.chaperone.ownerTrackingSystem.empty() || ctx.chaperone.ownerHmdSerial.empty() ||
			!ctx.chaperone.worldFromDriverValid))
	{
		ctx.DisarmChaperone();
		settingsRewriteNeeded = settingsCanRewrite;
		ctx.ReportError(
			"The protected chaperone has no complete headset/universe baseline. "
			"It has been disarmed; capture it again before enabling auto-restore.\n",
			CalibrationContext::ErrorSource::Chaperone);
	}
	else if (ctx.chaperone.valid && ctx.validProfile &&
		ctx.chaperone.ownerTrackingSystem != ctx.referenceTrackingSystem)
	{
		ctx.DisarmChaperone();
		settingsRewriteNeeded = settingsCanRewrite;
		ctx.ReportError(
			"The protected chaperone belongs to a different reference tracking system. "
			"It has been disarmed; capture it again for this profile.\n",
			CalibrationContext::ErrorSource::Chaperone);
	}

	if (settingsRewriteNeeded)
	{
		ctx.MarkSettingsDirty(ctx.timeLastTick);
		if (!SaveSettings(ctx))
			ctx.legacySettingsMigrationPending = profileLoaded && !profileRevision.present;
	}

	ctx.pendingReferenceTrackingSystem = ctx.referenceTrackingSystem;
	ctx.pendingTargetTrackingSystem = ctx.targetTrackingSystem;
}

static bool SaveSettingsRecord(CalibrationContext &ctx);

static bool SaveProfileRecord(CalibrationContext &ctx, const ProfileRecord &record)
{
	// UI preview runs on fake state; never let it clobber the real profile.
	if (g_uiPreviewMode)
		return true;
	if (!questcal::CanPersistConfig(ctx.profileLoadState))
	{
		ctx.ReportError(
			"Could not save the calibration profile because the existing Config record "
			"could not be read. It was left untouched for recovery.\n",
			CalibrationContext::ErrorSource::ProfilePersistence);
		return false;
	}
	if (ctx.legacySettingsMigrationPending &&
		ctx.settingsLoadState == questcal::RecordLoadState::Unreadable)
	{
		ctx.ReportError(
			"Could not save the calibration profile because the existing Settings record "
			"could not be read. It was left untouched for recovery.\n",
			CalibrationContext::ErrorSource::ProfilePersistence);
		return false;
	}
	if (ctx.legacySettingsMigrationPending && !SaveSettingsRecord(ctx))
	{
		ctx.ReportError(
			"Could not save the calibration profile because the legacy settings copy "
			"has not yet been migrated safely.\n",
			CalibrationContext::ErrorSource::ProfilePersistence);
		return false;
	}
	if (ctx.persistenceRevision == 0)
		ctx.persistenceRevision = 1;
	if (record.valid && !questcal::IsValidTrackingSystemPair(
		record.referenceTrackingSystem, record.targetTrackingSystem))
	{
		ctx.ReportError(
			"Could not save the calibration profile: tracking systems must be non-empty and different\n",
			CalibrationContext::ErrorSource::ProfilePersistence);
		return false;
	}
	if (record.valid && !questcal::IsValidCalibrationTransform(
		record.rotation, record.translationMeters, record.scale))
	{
		ctx.ReportError("Could not save the calibration profile: the live transform is invalid\n",
			CalibrationContext::ErrorSource::ProfilePersistence);
		return false;
	}
	if (record.valid &&
		(!std::isfinite(record.timeOffset) ||
			std::abs(record.timeOffset) >
				protocol::limits::MaxAbsTimeOffsetSeconds ||
			!std::isfinite(record.calibrationUnixTime) ||
			record.calibrationUnixTime < 0.0 ||
			record.calibrationUnixTime >
				protocol::limits::MaxPlausibleUnixTimeSeconds))
	{
		ctx.ReportError(
			"Could not save the calibration profile: calibration timing values are invalid\n",
			CalibrationContext::ErrorSource::ProfilePersistence);
		return false;
	}
	if (record.valid && record.mountExtrinsic.valid &&
		(!questcal::IsValidRotation(record.mountExtrinsic.rotation) ||
			!questcal::IsBoundedVector(record.mountExtrinsic.translationMeters,
				protocol::limits::MaxAbsAnchorDeltaMeters) ||
			!questcal::IsValidResidual(record.mountExtrinsic.rotationRmsDeg) ||
			!questcal::IsValidResidual(record.mountExtrinsic.translationRmsM)))
	{
		ctx.ReportError(
			"Could not save the calibration profile: the mount extrinsic is invalid\n",
			CalibrationContext::ErrorSource::ProfilePersistence);
		return false;
	}
	if (record.valid &&
		record.fieldAnchors.size() > protocol::SetAlignmentField::MaxAnchors)
	{
		ctx.ReportError(
			"Could not save the calibration profile: there are too many field anchors\n",
			CalibrationContext::ErrorSource::ProfilePersistence);
		return false;
	}
	for (const auto &anchor : record.fieldAnchors)
	{
		if (!questcal::IsValidFieldAnchor(anchor.position, anchor.rotation,
			anchor.translationMeters, record.rotation, record.translationMeters))
		{
			ctx.ReportError("Could not save the calibration profile: a field anchor is invalid\n",
				CalibrationContext::ErrorSource::ProfilePersistence);
			return false;
		}
	}

	std::cout << "Saving profile to registry" << std::endl;

	std::stringstream profile;
	WriteProfile(record, ctx.persistenceRevision, profile);
	std::string error;
	if (!WriteRegistryValue("Config", profile.str(), error))
	{
		ctx.ReportError("Could not save the calibration profile: " + error + "\n",
			CalibrationContext::ErrorSource::ProfilePersistence);
		return false;
	}
	ctx.profileSaveDirty = false;
	ctx.profileLoadState = record.valid
		? questcal::RecordLoadState::Loaded
		: questcal::RecordLoadState::Missing;
	ctx.ClearError(CalibrationContext::ErrorSource::ProfilePersistence);
	return true;
}

bool SaveProfile(CalibrationContext &ctx)
{
	return SaveProfileRecord(ctx, CaptureProfileRecord(ctx));
}

bool ClearSavedProfile(CalibrationContext &ctx)
{
	// An empty Config carries no revision. Flush newer Settings first so a
	// crash after clearing Config cannot hide a coupled-write mismatch and make
	// the previous chaperone snapshot look authoritative on the next launch.
	// SaveSettings also commits a dirty Config first when both halves are pending.
	if (ctx.settingsSaveDirty && !SaveSettings(ctx))
		return false;

	ProfileRecord cleared;
	if (!SaveProfileRecord(ctx, cleared))
		return false;
	ctx.Clear();
	return true;
}

bool SaveProfileTransformEdit(CalibrationContext &ctx,
	const Eigen::Quaterniond &rotation,
	const Eigen::Vector3d &translationMeters, double scale,
	bool rotationEdited)
{
	ProfileRecord candidate = CaptureProfileRecord(ctx);
	if (rotationEdited)
	{
		if (!questcal::IsValidRotation(rotation))
		{
			ctx.ReportError("Could not save the calibration profile: the edited rotation is invalid\n",
				CalibrationContext::ErrorSource::ProfilePersistence);
			return false;
		}
		candidate.rotation = rotation.normalized();
	}
	candidate.translationMeters = translationMeters;
	candidate.scale = scale;

	if (!SaveProfileRecord(ctx, candidate))
		return false;

	if (rotationEdited)
		ctx.SetCalibration(candidate.rotation, candidate.translationMeters, candidate.scale);
	else
	{
		// Do not normalize or otherwise rewrite the quaternion when only the
		// translation/scale changed: its persisted bits remain the source truth.
		ctx.calibratedTranslation = candidate.translationMeters * 100.0;
		ctx.calibratedScale = candidate.scale;
		ctx.baseGeneration++;
	}
	if (!ctx.fieldAnchors.empty())
		ctx.fieldGeneration++;
	ctx.state = CalibrationState::None;
	ctx.timeLastScan = -1e9;
	return true;
}

static bool SaveSettingsRecord(CalibrationContext &ctx)
{
	if (g_uiPreviewMode)
		return true;
	if (!questcal::CanPersistSettings(
		ctx.profileLoadState, ctx.settingsLoadState))
	{
		ctx.ReportError(
			"Could not save QuestCalibrator settings because the existing Config record "
			"could not be read. Neither record was changed so legacy settings remain recoverable.\n",
			CalibrationContext::ErrorSource::SettingsPersistence);
		return false;
	}
	if (ctx.settingsLoadState == questcal::RecordLoadState::Unreadable)
	{
		ctx.ReportError(
			"Could not save QuestCalibrator settings because the existing Settings record "
			"could not be read. It was left untouched for recovery.\n",
			CalibrationContext::ErrorSource::SettingsPersistence);
		return false;
	}
	if (ctx.persistenceRevision == 0)
		ctx.persistenceRevision = 1;
	SettingsRecord record = CaptureSettingsRecord(ctx);
	if (record.chaperone.valid &&
		(record.chaperone.ownerTrackingSystem.empty() ||
			record.chaperone.ownerHmdSerial.empty() ||
			!std::isfinite(record.chaperone.copyUnixTime) ||
			record.chaperone.copyUnixTime < 0.0 ||
			record.chaperone.copyUnixTime >
				protocol::limits::MaxPlausibleUnixTimeSeconds ||
			!record.chaperone.worldFromDriverValid ||
			!questcal::IsValidRotation(record.chaperone.worldFromDriverRotation) ||
			!questcal::IsBoundedVector(record.chaperone.worldFromDriverTranslation,
				protocol::limits::MaxAbsTranslationMeters) ||
			!questcal::IsPlausibleChaperone(record.chaperone.geometry,
				record.chaperone.standingCenter, record.chaperone.playSpaceSize)))
	{
		ctx.ReportError(
			"Could not save QuestCalibrator settings because the protected chaperone snapshot is invalid\n",
			CalibrationContext::ErrorSource::SettingsPersistence);
		return false;
	}

	std::stringstream settings;
	WriteSettings(record, ctx.persistenceRevision, settings);
	std::string error;
	if (!WriteRegistryValue("Settings", settings.str(), error))
	{
		ctx.ReportError("Could not save QuestCalibrator settings: " + error + "\n",
			CalibrationContext::ErrorSource::SettingsPersistence);
		return false;
	}
	ctx.legacySettingsMigrationPending = false;
	ctx.settingsLoadState = questcal::RecordLoadState::Loaded;
	ctx.settingsSaveDirty = false;
	ctx.ClearError(CalibrationContext::ErrorSource::SettingsPersistence);
	return true;
}

bool SaveSettings(CalibrationContext &ctx)
{
	// A caller may directly save a setting while Config is waiting to be
	// persisted (including after a universe-revision bump). Never let Settings
	// overtake it: commit Config first, then leave only unfinished stages dirty.
	if (ctx.profileSaveDirty)
	{
		if (!ctx.validProfile)
		{
			ctx.ReportError(
				"Could not save Settings ahead of an unfinished Config update\n",
				CalibrationContext::ErrorSource::ProfilePersistence);
			return false;
		}
		if (!SaveProfile(ctx))
			return false;
		ctx.profileSaveDirty = false;
	}
	return SaveSettingsRecord(ctx);
}
