#include "stdafx.h"
#include "Configuration.h"
#include "ChaperoneMath.h"
#include "ProfileValidation.h"
#include "ProfileRecordJson.h"
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

// PersistedCalibrationSpeed mirrors this enum (see ProfileValidation.h); pin
// the two together where both are visible.
static_assert(static_cast<int>(CalibrationContext::FAST) ==
	static_cast<int>(questcal::PersistedCalibrationSpeed::Fast) &&
	static_cast<int>(CalibrationContext::SLOW) ==
	static_cast<int>(questcal::PersistedCalibrationSpeed::Slow) &&
	static_cast<int>(CalibrationContext::VERY_SLOW) ==
	static_cast<int>(questcal::PersistedCalibrationSpeed::VerySlow),
	"CalibrationContext::Speed and PersistedCalibrationSpeed must agree");

using questcal::MountExtrinsicRecord;
using questcal::PersistedFieldAnchor;
using questcal::PersistedRevision;
using questcal::ProfileParseResult;
using questcal::ProfileRecord;

// Narrow records holding exactly what Config and Settings store. The profile
// half lives in ProfileValidation.h / ProfileRecordJson.h so tests can reach
// it; the chaperone half needs the OpenVR geometry types and stays here.
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
	bool detailedLogging = false;
	bool automaticUpdates = false;
	std::string language;
	std::map<std::string, std::string> deviceNames;
	ChaperoneRecord chaperone;
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
	record.detailedLogging = ctx.detailedLogging;
	record.automaticUpdates = ctx.automaticUpdates;
	record.language = ctx.language;
	record.deviceNames = ctx.deviceNames;
	record.chaperone = CaptureChaperoneRecord(ctx.chaperone);
	return record;
}

questcal::PersistedFieldAnchor PersistedAnchor(
	const CalibrationContext::FieldAnchor &anchor)
{
	PersistedFieldAnchor persisted;
	persisted.position = anchor.position;
	persisted.rotation = anchor.rotation;
	persisted.translationMeters = anchor.translationMeters;
	return persisted;
}

static ProfileRecord CaptureProfileRecord(const CalibrationContext &ctx)
{
	ProfileRecord record;
	record.valid = ctx.validProfile;
	record.referenceTrackingSystem = ctx.referenceTrackingSystem;
	record.targetTrackingSystem = ctx.targetTrackingSystem;
	record.rotation = ctx.transform.rotation;
	record.translationMeters = ctx.transform.translationMeters;
	record.scale = ctx.transform.scale;
	record.timeOffset = ctx.transform.timeOffset;
	record.calibrationUnixTime = ctx.calibrationUnixTime;
	record.universeUnsafe = ctx.profileUniverseUnsafe;
	record.universeValid = ctx.profileUniverseValid;
	record.universeHmdSerial = ctx.profileHmdSerial;
	record.universeRotation = ctx.profileWorldFromDriverRotation;
	record.universeTranslation = ctx.profileWorldFromDriverTranslation;
	record.fieldEnabled = ctx.fieldEnabled;
	record.fieldAnchors.reserve(ctx.fieldAnchors.size());
	for (const auto &anchor : ctx.fieldAnchors)
		record.fieldAnchors.push_back(PersistedAnchor(anchor));
	record.continuousEnabled = ctx.continuousEnabled;
	record.continuousTrackerSerial = ctx.continuousTrackerSerial;
	record.continuousLatencyReestimation = ctx.continuousLatencyReestimation;
	record.continuousRequireTrigger = ctx.continuousRequireTrigger;
	record.continuousMode = static_cast<int>(ctx.continuousMode);
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
	ctx.detailedLogging = record.detailedLogging;
	ctx.automaticUpdates = record.automaticUpdates;
	ctx.language = record.language;
	ctx.deviceNames = record.deviceNames;
	ApplyChaperoneRecord(ctx, std::move(record.chaperone));
}

// The preference half of a profile record (spatial field, continuous pick and
// mount), shared by the load path and SaveProfileFieldEdit. The edit path must
// not re-apply the base transform, which would bump baseGeneration per toggle.
static void ApplyProfilePreferences(
	CalibrationContext &ctx, const ProfileRecord &record)
{
	ctx.fieldEnabled = record.fieldEnabled;
	ctx.fieldAnchors.clear();
	ctx.fieldAnchors.reserve(record.fieldAnchors.size());
	for (const auto &persisted : record.fieldAnchors)
	{
		CalibrationContext::FieldAnchor anchor;
		anchor.position = persisted.position;
		anchor.rotation = persisted.rotation;
		anchor.translationMeters = persisted.translationMeters;
		ctx.fieldAnchors.push_back(anchor);
	}
	ctx.continuousEnabled = record.continuousEnabled;
	ctx.continuousTrackerSerial = record.continuousTrackerSerial;
	ctx.continuousLatencyReestimation = record.continuousLatencyReestimation;
	ctx.continuousRequireTrigger = record.continuousRequireTrigger;
	ctx.continuousMode = record.continuousMode == 1 ? ContinuousMode::Legacy : ContinuousMode::Quest;
	ctx.hideMountedTracker = record.hideMountedTracker;
	// Only the persisted members: MountExtrinsic::pairs is a runtime statistic
	// that the load path resets and a preference edit must keep.
	ctx.mountExtrinsic.valid = record.mountExtrinsic.valid;
	ctx.mountExtrinsic.rot = record.mountExtrinsic.rotation;
	ctx.mountExtrinsic.pos = record.mountExtrinsic.translationMeters;
	ctx.mountExtrinsic.rotRmsDeg = record.mountExtrinsic.rotationRmsDeg;
	ctx.mountExtrinsic.posRmsM = record.mountExtrinsic.translationRmsM;
}

static void ApplyProfileRecord(CalibrationContext &ctx, ProfileRecord record)
{
	ctx.referenceTrackingSystem = std::move(record.referenceTrackingSystem);
	ctx.targetTrackingSystem = std::move(record.targetTrackingSystem);
	ctx.SetCalibration(record.rotation, record.translationMeters, record.scale);
	ctx.transform.timeOffset = record.timeOffset;
	ctx.calibrationUnixTime = record.calibrationUnixTime;
	ctx.profileUniverseUnsafe = record.universeUnsafe;
	ctx.profileUniverseValid = record.universeValid;
	ctx.profileHmdSerial = std::move(record.universeHmdSerial);
	ctx.profileWorldFromDriverRotation = record.universeRotation;
	ctx.profileWorldFromDriverTranslation = record.universeTranslation;
	ctx.mountExtrinsic = questcal::MountExtrinsic();
	ApplyProfilePreferences(ctx, record);
	ctx.validProfile = record.valid;
}

using questcal::FloatArray;
using questcal::GetDouble;
using questcal::HasTypedValue;
using questcal::LoadFloatArray;
using questcal::ReadPersistenceRevision;
using questcal::RejectExcessiveJsonNesting;

// One definition of a well-formed chaperone snapshot for parser and writer.
// Only the writer requires a complete owner: the load path disarms an ownerless
// snapshot instead of failing the whole record (and a good calibration with it).
static bool ValidateChaperoneRecord(const ChaperoneRecord &record,
	bool requireCompleteOwner, std::string &why)
{
	if (!record.valid)
		return true;
	if (requireCompleteOwner && !questcal::IsCompleteChaperoneOwner(
		record.ownerTrackingSystem, record.ownerHmdSerial,
		record.worldFromDriverValid))
	{
		why = "the protected chaperone has no complete headset/universe baseline";
		return false;
	}
	if (record.worldFromDriverValid && !questcal::IsValidUniverseBaseline(
		record.worldFromDriverRotation, record.worldFromDriverTranslation))
	{
		why = "the chaperone worldFromDriver baseline is invalid";
		return false;
	}
	if (!questcal::IsValidRecordUnixTime(record.copyUnixTime))
	{
		why = "the chaperone copy_time is invalid";
		return false;
	}
	if (!questcal::IsPlausibleChaperone(record.geometry, record.standingCenter,
		record.playSpaceSize))
	{
		why = "the chaperone geometry or standing transform is invalid";
		return false;
	}
	return true;
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
		// Before normalizing: normalized() of a degenerate quaternion is NaN.
		if (!questcal::IsValidUniverseBaseline(baselineRotation, baselineTranslation))
			throw std::runtime_error("invalid chaperone worldFromDriver baseline");

		parsed.worldFromDriverRotation = baselineRotation.normalized();
		parsed.worldFromDriverTranslation = baselineTranslation;
		parsed.worldFromDriverValid = true;
	}

	// Required, unlike the optional fields above.
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
	LoadFloatArray(chaperone.at("geometry"),
		reinterpret_cast<float *>(parsed.geometry.data()), geometry.size());

	if (HasTypedValue<double>(chaperone, "copy_time"))
		parsed.copyUnixTime = GetDouble(chaperone.at("copy_time"));

	parsed.valid = true;
	std::string why;
	if (!ValidateChaperoneRecord(parsed, false, why))
		throw std::runtime_error(why);
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

// The Settings-owned half of a legacy Config record: older releases embedded
// the global settings alongside the profile. An absent key leaves the caller's
// already-loaded value alone rather than resetting it to a default.
static void ParseLegacyEmbeddedSettings(const questcal::LegacyProfileSettings &legacy,
	const picojson::object &obj, SettingsRecord &settings)
{
	if (legacy.hasApplyTimeOffset)
		settings.applyTimeOffset = legacy.applyTimeOffset;
	if (legacy.hasSolveScale)
		settings.solveScale = legacy.solveScale;
	if (legacy.hasUiAdvanced)
		settings.uiAdvanced = legacy.uiAdvanced;
	if (legacy.hasChaperoneWarningAck)
		settings.chaperoneWarningAck = legacy.chaperoneWarningAck;
	if (legacy.hasCalibrationSpeed)
		settings.calibrationSpeed =
			static_cast<CalibrationContext::Speed>(legacy.calibrationSpeed);

	ParseChaperone(settings, obj);
}

// The shared codec reads the profile; this adds what it cannot see: the legacy
// embedded settings and the chaperone snapshot.
static ProfileParseResult ParseProfile(ProfileRecord &profile,
	SettingsRecord &legacySettings, std::istream &stream)
{
	picojson::value profileValue = questcal::ParseProfileEnvelope(stream);
	const auto &obj = profileValue.get<picojson::object>();

	questcal::LegacyProfileSettings legacy;
	ProfileParseResult result = questcal::ParseProfileObject(
		profile, legacy, obj, protocol::SetAlignmentField::MaxAnchors);
	ParseLegacyEmbeddedSettings(legacy, obj, legacySettings);
	return result;
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
	settings["detailed_logging"].set<bool>(record.detailedLogging);
	settings["automatic_updates"].set<bool>(record.automaticUpdates);
	if (!record.language.empty())
		settings["language"].set<std::string>(record.language);
	if (!record.deviceNames.empty())
	{
		picojson::object names;
		for (const auto &entry : record.deviceNames)
			names[entry.first].set<std::string>(entry.second);
		settings["device_names"].set<picojson::object>(std::move(names));
	}
	WriteChaperone(record, settings);

	picojson::value value;
	value.set<picojson::object>(std::move(settings));
	out << value.serialize(true);
}

static PersistedRevision ParseSettings(SettingsRecord &settings, std::istream &stream)
{
	picojson::value value;
	std::string err = questcal::ParseRecordJson(value, stream);
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
	if (HasTypedValue<bool>(obj, "detailed_logging"))
		settings.detailedLogging = obj.at("detailed_logging").get<bool>();
	if (HasTypedValue<bool>(obj, "automatic_updates"))
		settings.automaticUpdates = obj.at("automatic_updates").get<bool>();
	// A code this build does not know is dropped, not refused: it reads as
	// "follow Windows", and the rest of the record still loads.
	if (HasTypedValue<std::string>(obj, "language"))
	{
		const std::string code = obj.at("language").get<std::string>();
		if (code == "en" || code == "ja" || code == "it")
			settings.language = code;
	}
	if (HasTypedValue<picojson::object>(obj, "device_names"))
	{
		// Bounded on read as on write: a hand-edited record cannot grow the
		// map or a name past what the UI is built for.
		for (const auto &entry : obj.at("device_names").get<picojson::object>())
		{
			if (!entry.second.is<std::string>() || entry.first.empty())
				continue;
			std::string name = entry.second.get<std::string>();
			if (name.empty() || name.size() > CalibrationContext::DeviceNameMaxBytes)
				continue;
			if (settings.deviceNames.size() >= CalibrationContext::DeviceNameMaxCount)
				break;
			settings.deviceNames[entry.first] = name;
		}
	}
	if (HasTypedValue<double>(obj, "calibration_speed"))
	{
		double speed = GetDouble(obj.at("calibration_speed"));
		if (!questcal::IsValidCalibrationSpeed(speed))
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

// HKEY_CURRENT_USER_LOCAL_SETTINGS resolves under HKCU\Software\Classes\Local
// Settings, which regedit does not label. An unreadable record is never
// overwritten, so the error text must say where to delete it by hand.
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

// Runs once, at startup, on a context whose load states are still Missing.
void LoadProfile(CalibrationContext &ctx)
{
	PersistedRevision profileRevision;
	PersistedRevision settingsRevision;
	bool settingsRewriteNeeded = false;

	auto profileRead = ReadRegistryValue("Config");
	if (profileRead.status == RegistryReadStatus::Error)
	{
		ctx.profileLoadState = questcal::RecordLoadState::Unreadable;
		ctx.Log("Calibration profile read failed: " + profileRead.error + "\n");
		ctx.ReportError("Couldn't read the saved calibration, so it was left as it is. "
			"Restart QuestCalibrator. If this repeats, save a diagnostics file in Settings and report it.\n",
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
						std::to_string(ctx.transform.scale).substr(0, 5) +
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

	// Settings keeps chaperone protection and global preferences alive even
	// with no calibration profile.
	auto settingsRead = ReadRegistryValue("Settings");
	if (settingsRead.status == RegistryReadStatus::Error)
	{
		ctx.settingsLoadState = questcal::RecordLoadState::Unreadable;
		ctx.Log("Settings read failed: " + settingsRead.error + "\n");
		ctx.ReportError("Couldn't read QuestCalibrator's settings, so they were left as they are. "
			"Restart QuestCalibrator. If this repeats, save a diagnostics file in Settings and report it.\n",
			CalibrationContext::ErrorSource::SettingsPersistence);
	}
	// Missing or empty keeps whatever the (possibly legacy) Config load produced;
	// PlanPersistenceLoad decides whether to materialize Settings from it.
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
	// PlanPersistenceLoad decides everything from these facts; this block only
	// gathers them and applies the verdict.
	questcal::PersistenceLoadFacts facts;
	facts.profile = ctx.profileLoadState;
	facts.settings = ctx.settingsLoadState;
	facts.profileRevision = profileRevision;
	facts.settingsRevision = settingsRevision;
	facts.chaperoneArmed = ctx.chaperone.valid;
	facts.chaperoneOwnerComplete = questcal::IsCompleteChaperoneOwner(
		ctx.chaperone.ownerTrackingSystem, ctx.chaperone.ownerHmdSerial,
		ctx.chaperone.worldFromDriverValid);
	facts.chaperoneOwnerMatchesReference =
		ctx.chaperone.ownerTrackingSystem == ctx.referenceTrackingSystem;
	facts.profileValid = ctx.validProfile;

	questcal::PersistenceLoadPlan plan = questcal::PlanPersistenceLoad(facts);
	settingsRewriteNeeded = plan.settingsRewriteNeeded;
	ctx.persistence.SetRevision(plan.persistenceRevision);
	if (plan.legacySettingsMigrationPending)
		ctx.persistence.legacySettingsMigrationPending = true;
	if (plan.disarmChaperone)
		ctx.DisarmChaperone();
	if (plan.reportRevisionMismatch)
	{
		ctx.ReportError(
			"The calibration profile and protected chaperone were saved at different revisions. "
			"Chaperone auto-restore is disabled until you capture it again.\n",
			CalibrationContext::ErrorSource::Chaperone);
	}

	switch (plan.gate)
	{
	case questcal::ChaperoneLoadGate::Armed:
		break;
	case questcal::ChaperoneLoadGate::ProfileUnreadable:
		ctx.Log("Protected chaperone left disarmed because the calibration profile could not be read\n");
		break;
	case questcal::ChaperoneLoadGate::SettingsUnreadable:
		ctx.Log("Protected chaperone left disarmed because application settings could not be read\n");
		break;
	case questcal::ChaperoneLoadGate::IncompleteOwner:
		ctx.ReportError(
			"The protected chaperone has no complete headset/universe baseline. "
			"It has been disarmed; capture it again before enabling auto-restore.\n",
			CalibrationContext::ErrorSource::Chaperone);
		break;
	case questcal::ChaperoneLoadGate::ForeignTrackingSystem:
		ctx.ReportError(
			"The protected chaperone belongs to a different reference tracking system. "
			"It has been disarmed; capture it again for this profile.\n",
			CalibrationContext::ErrorSource::Chaperone);
		break;
	}

	// A new coupled revision, not the stranded one: if only Config lands, the
	// two records still differ and the next load still fails closed.
	if (plan.profileRewriteNeeded)
	{
		ctx.persistence.AdvanceRevision();
		ctx.persistence.MarkProfile(ctx.timeLastTick);
	}

	if (settingsRewriteNeeded)
	{
		ctx.persistence.MarkSettings(ctx.timeLastTick);
		if (!SaveSettings(ctx))
			ctx.persistence.legacySettingsMigrationPending =
				plan.legacySettingsMigrationPendingIfRewriteFails;
	}

	ctx.pendingReferenceTrackingSystem = ctx.referenceTrackingSystem;
	ctx.pendingTargetTrackingSystem = ctx.targetTrackingSystem;
}

// SaveProfile/SaveSettings coordinate the two records: either may commit the
// other first to keep the Config-before-Settings order and shared revision.
// Write*Record writes exactly one record and never calls a coordinator, which
// is what terminates SaveSettings -> SaveProfile -> WriteConfigRecord ->
// WriteSettingsRecord.
static bool WriteSettingsRecord(CalibrationContext &ctx);

// Logged once per record per session, so a preview never silently "saves".
static void NotePreviewWriteSkipped(CalibrationContext &ctx, const char *what,
	bool &announced)
{
	if (announced)
		return;
	announced = true;
	ctx.Log(std::string("UI preview mode: ") + what +
		" was not written to the registry\n");
}

static bool WriteConfigRecord(CalibrationContext &ctx, const ProfileRecord &record)
{
	questcal::PersistenceWriteGate gate = questcal::GateProfileWrite(
		g_uiPreviewMode, ctx.profileLoadState, ctx.settingsLoadState,
		ctx.persistence.legacySettingsMigrationPending);
	switch (gate)
	{
	case questcal::PersistenceWriteGate::Allowed:
		break;
	case questcal::PersistenceWriteGate::SkippedPreview:
		{
			static bool announced = false;
			NotePreviewWriteSkipped(ctx, "the calibration profile", announced);
		}
		return true;
	case questcal::PersistenceWriteGate::RefusedConfigUnreadable:
		ctx.ReportError(
			"Couldn't save the calibration because the saved one couldn't be read. "
			"It was left untouched so it can be recovered.\n",
			CalibrationContext::ErrorSource::ProfilePersistence);
		return false;
	case questcal::PersistenceWriteGate::RefusedSettingsUnreadable:
		ctx.ReportError(
			"Couldn't save the calibration because QuestCalibrator's settings couldn't be read. "
			"They were left untouched so they can be recovered.\n",
			CalibrationContext::ErrorSource::ProfilePersistence);
		return false;
	}
	if (ctx.persistence.legacySettingsMigrationPending && !WriteSettingsRecord(ctx))
	{
		ctx.ReportError(
			"Couldn't save the calibration because settings from an older version haven't been "
			"moved over safely yet. Restart QuestCalibrator and try again.\n",
			CalibrationContext::ErrorSource::ProfilePersistence);
		return false;
	}
	// The same definition the parser enforces, so a record that saves is a
	// record that will load again.
	std::string why;
	if (!questcal::ValidateProfileRecord(
		record, protocol::SetAlignmentField::MaxAnchors, why))
	{
		ctx.Log("Calibration profile rejected: " + why + "\n");
		ctx.ReportError("Couldn't save the calibration because it failed a safety check. Recalibrate.\n",
			CalibrationContext::ErrorSource::ProfilePersistence);
		return false;
	}

	std::cout << "Saving profile to registry" << std::endl;

	std::stringstream profile;
	questcal::WriteProfile(record, ctx.persistence.revision, profile);
	std::string error;
	if (!WriteRegistryValue("Config", profile.str(), error))
	{
		ctx.Log("Calibration profile write failed: " + error + "\n");
		ctx.ReportError("Couldn't save the calibration. It will be lost when QuestCalibrator closes.\n",
			CalibrationContext::ErrorSource::ProfilePersistence);
		return false;
	}
	ctx.persistence.profileDirty = false;
	ctx.profileLoadState = record.valid
		? questcal::RecordLoadState::Loaded
		: questcal::RecordLoadState::Missing;
	ctx.ClearError(CalibrationContext::ErrorSource::ProfilePersistence);
	return true;
}

bool SaveProfile(CalibrationContext &ctx)
{
	return WriteConfigRecord(ctx, CaptureProfileRecord(ctx));
}

bool ClearSavedProfile(CalibrationContext &ctx)
{
	// An empty Config carries no revision. Flush newer Settings first so a
	// crash after clearing Config cannot hide a coupled-write mismatch and make
	// the previous chaperone snapshot look authoritative on the next launch.
	// SaveSettings also commits a dirty Config first when both halves are pending.
	if (ctx.persistence.settingsDirty && !SaveSettings(ctx))
		return false;

	ProfileRecord cleared;
	if (!WriteConfigRecord(ctx, cleared))
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
		candidate.rotation = rotation.normalized();
	candidate.translationMeters = translationMeters;
	candidate.scale = scale;

	if (!WriteConfigRecord(ctx, candidate))
		return false;

	if (rotationEdited)
		ctx.SetCalibration(candidate.rotation, candidate.translationMeters, candidate.scale);
	else
	{
		// Do not normalize or otherwise rewrite the quaternion when only the
		// translation/scale changed: its persisted bits remain the source truth.
		ctx.transform.translationMeters = candidate.translationMeters;
		ctx.transform.scale = candidate.scale;
		ctx.baseGeneration++;
	}
	if (!ctx.fieldAnchors.empty())
		ctx.fieldGeneration++;
	ctx.state = CalibrationState::None;
	ctx.timeLastScan = -1e9;
	return true;
}

bool SaveProfileFieldEdit(CalibrationContext &ctx,
	const std::function<void(questcal::ProfileRecord &)> &mutate,
	bool bumpFieldGeneration)
{
	ProfileRecord candidate = CaptureProfileRecord(ctx);
	mutate(candidate);
	if (!WriteConfigRecord(ctx, candidate))
		return false;

	ApplyProfilePreferences(ctx, candidate);
	if (bumpFieldGeneration)
		ctx.fieldGeneration++;
	return true;
}

static bool WriteSettingsRecord(CalibrationContext &ctx)
{
	questcal::PersistenceWriteGate gate = questcal::GateSettingsWrite(
		g_uiPreviewMode, ctx.profileLoadState, ctx.settingsLoadState);
	switch (gate)
	{
	case questcal::PersistenceWriteGate::Allowed:
		break;
	case questcal::PersistenceWriteGate::SkippedPreview:
		{
			static bool announced = false;
			NotePreviewWriteSkipped(ctx, "the application settings", announced);
		}
		return true;
	case questcal::PersistenceWriteGate::RefusedConfigUnreadable:
		ctx.ReportError(
			"Couldn't save QuestCalibrator's settings because the saved calibration couldn't be read. "
			"Nothing was changed, so both can still be recovered.\n",
			CalibrationContext::ErrorSource::SettingsPersistence);
		return false;
	case questcal::PersistenceWriteGate::RefusedSettingsUnreadable:
		ctx.ReportError(
			"Couldn't save QuestCalibrator's settings because the saved ones couldn't be read. "
			"They were left untouched so they can be recovered.\n",
			CalibrationContext::ErrorSource::SettingsPersistence);
		return false;
	}
	SettingsRecord record = CaptureSettingsRecord(ctx);
	// An unowned room cannot be restored, so refuse to write one.
	std::string why;
	if (!ValidateChaperoneRecord(record.chaperone, true, why))
	{
		ctx.Log("Settings rejected: " + why + "\n");
		ctx.ReportError(
			"Couldn't save QuestCalibrator's settings because the protected chaperone failed a safety check. "
			"Protect it again.\n",
			CalibrationContext::ErrorSource::SettingsPersistence);
		return false;
	}

	std::stringstream settings;
	WriteSettings(record, ctx.persistence.revision, settings);
	std::string error;
	if (!WriteRegistryValue("Settings", settings.str(), error))
	{
		ctx.Log("Settings write failed: " + error + "\n");
		ctx.ReportError("Couldn't save QuestCalibrator's settings. Changes will be lost when it closes.\n",
			CalibrationContext::ErrorSource::SettingsPersistence);
		return false;
	}
	ctx.persistence.legacySettingsMigrationPending = false;
	ctx.settingsLoadState = questcal::RecordLoadState::Loaded;
	ctx.persistence.settingsDirty = false;
	ctx.ClearError(CalibrationContext::ErrorSource::SettingsPersistence);
	return true;
}

bool SaveSettings(CalibrationContext &ctx)
{
	return SaveSettingsWithResult(ctx).AllSaved();
}

// profileDirty implies validProfile: every MarkProfile site requires a valid
// profile, and Clear() (the only way to lose one) drops the pending write.
questcal::SettingsSaveResult SaveSettingsWithResult(CalibrationContext &ctx)
{
	return ctx.persistence.SaveSettings(
		[&]() { return SaveProfile(ctx); },
		[&]() { return WriteSettingsRecord(ctx); });
}

bool SavePendingChanges(CalibrationContext &ctx)
{
	if (ctx.persistence.settingsDirty)
		return SaveSettings(ctx);
	if (ctx.persistence.profileDirty && !SaveProfile(ctx))
		return false;
	ctx.persistence.coupled = false;
	return true;
}
