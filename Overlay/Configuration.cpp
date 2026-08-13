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

// The persisted speed range is spelled in ProfileValidation.h because the
// record layer cannot include Calibration.h (openvr.h vs the test harness's
// openvr_driver.h). Pin the two spellings together here, where both are
// visible, so a reordered enum cannot silently widen what a record may carry.
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

// Serialization owns narrow records rather than cloning CalibrationContext.
// These contain exactly the values represented in Config/Settings; live poses,
// solver buffers, monitor state, UI messages and retry metadata never cross the
// persistence boundary. The profile half lives in ProfileValidation.h /
// ProfileRecordJson.h so its write -> read identity and its validation are
// reachable from a test; the chaperone half stays here because it needs the
// OpenVR geometry types and ChaperoneMath.h.
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
	record.universeUnsafe = ctx.profileUniverseUnsafe;
	record.universeValid = ctx.profileUniverseValid;
	record.universeHmdSerial = ctx.profileHmdSerial;
	record.universeRotation = ctx.profileWorldFromDriverRotation;
	record.universeTranslation = ctx.profileWorldFromDriverTranslation;
	record.fieldEnabled = ctx.fieldEnabled;
	// The record's anchor type is deliberately not CalibrationContext's — see
	// ProfileValidation.h. The two are field-identical; this is the only place
	// that has to know that.
	record.fieldAnchors.reserve(ctx.fieldAnchors.size());
	for (const auto &anchor : ctx.fieldAnchors)
	{
		PersistedFieldAnchor persisted;
		persisted.position = anchor.position;
		persisted.rotation = anchor.rotation;
		persisted.translationMeters = anchor.translationMeters;
		record.fieldAnchors.push_back(persisted);
	}
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
	ctx.profileUniverseUnsafe = record.universeUnsafe;
	ctx.profileUniverseValid = record.universeValid;
	ctx.profileHmdSerial = std::move(record.universeHmdSerial);
	ctx.profileWorldFromDriverRotation = record.universeRotation;
	ctx.profileWorldFromDriverTranslation = record.universeTranslation;
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

using questcal::FloatArray;
using questcal::GetDouble;
using questcal::HasTypedValue;
using questcal::LoadFloatArray;
using questcal::ReadPersistenceRevision;
using questcal::RejectExcessiveJsonNesting;

// One definition of a well-formed chaperone snapshot. The parser and the
// writer used to hold separate copies of these bounds, and had already
// drifted: the writer additionally demanded a complete owner baseline, so a
// legacy Config-embedded room could parse and arm but could never be written
// back. That asymmetry is preserved deliberately — the load path disarms an
// ownerless snapshot rather than rejecting the whole record, so the parser must
// not reject it — but it is now one flag on one function instead of two
// unrelated expressions seven hundred lines apart.
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
		// Checked here as well as in ValidateChaperoneRecord below: normalized()
		// on a degenerate quaternion produces NaNs, so the guard has to precede
		// the normalize rather than only judge the finished record.
		if (!questcal::IsValidUniverseBaseline(baselineRotation, baselineTranslation))
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
		parsed.copyUnixTime = GetDouble(chaperone.at("copy_time"));

	parsed.valid = true;
	// The parser deliberately does not require a complete owner baseline: the
	// load path disarms such a snapshot with its own message rather than
	// failing the whole record, which on the Config path would discard a good
	// calibration over a damaged room.
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

// Reads the profile-owned fields through the shared codec, then layers on the
// two things this record carries that the codec cannot see: the global
// settings Config-only releases embedded alongside the profile, and the
// chaperone snapshot (whose record needs the OpenVR geometry types).
static ProfileParseResult ParseProfile(ProfileRecord &profile,
	SettingsRecord &legacySettings, std::istream &stream)
{
	picojson::value profileValue = questcal::ParseProfileEnvelope(stream);
	const auto &obj = profileValue.get<picojson::object>();

	questcal::LegacyProfileSettings legacy;
	ProfileParseResult result = questcal::ParseProfileObject(
		profile, legacy, obj, protocol::SetAlignmentField::MaxAnchors);

	// Only what the record actually carried: an absent legacy key must leave
	// the caller's already-loaded value alone rather than reset it to a default.
	if (legacy.hasApplyTimeOffset)
		legacySettings.applyTimeOffset = legacy.applyTimeOffset;
	if (legacy.hasSolveScale)
		legacySettings.solveScale = legacy.solveScale;
	if (legacy.hasUiAdvanced)
		legacySettings.uiAdvanced = legacy.uiAdvanced;
	if (legacy.hasChaperoneWarningAck)
		legacySettings.chaperoneWarningAck = legacy.chaperoneWarningAck;
	if (legacy.hasCalibrationSpeed)
		legacySettings.calibrationSpeed =
			static_cast<CalibrationContext::Speed>(legacy.calibrationSpeed);

	ParseChaperone(legacySettings, obj);
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
	// profileUniverseUnsafe is deliberately not cleared here: the latch is part
	// of the record now, so a profile that lost raw-universe continuity stays
	// disabled across the restart instead of being handed back enabled.
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
	// The whole load-time state machine - which record wins, which revision is
	// adopted, what gets disarmed and whether a rewrite is even permitted - is
	// one pure function over the facts gathered above. Keeping it out of here is
	// what makes every cell of that matrix reachable from a test; this block only
	// collects facts and applies the verdict.
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
	ctx.persistenceRevision = plan.persistenceRevision;
	if (plan.legacySettingsMigrationPending)
		ctx.legacySettingsMigrationPending = true;
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
		// Config may be a legacy Config-only record containing the sole copy of
		// the protected room and global preferences. With no separately parsed
		// Settings record, do not persist this conservative in-memory disarm over
		// recoverable data.
		ctx.Log("Protected chaperone left disarmed because the calibration profile could not be read\n");
		break;
	case questcal::ChaperoneLoadGate::SettingsUnreadable:
		// A successfully parsed Config may contain a legacy fallback snapshot,
		// but a present-yet-unreadable Settings record is authoritative.  Keep
		// the parse error visible and never arm the fallback implicitly.
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

	if (settingsRewriteNeeded)
	{
		ctx.MarkSettingsDirty(ctx.timeLastTick);
		if (!SaveSettings(ctx))
			ctx.legacySettingsMigrationPending =
				plan.legacySettingsMigrationPendingIfRewriteFails;
	}

	ctx.pendingReferenceTrackingSystem = ctx.referenceTrackingSystem;
	ctx.pendingTargetTrackingSystem = ctx.targetTrackingSystem;
}

static bool SaveSettingsRecord(CalibrationContext &ctx);

// The preview no-op is the only write outcome that reports success without
// writing, so it gets a line in the session log. Once per record per session:
// this is a "the mode you are in is not persisting" notice, not a per-write
// event, and preview sessions save often.
static void NotePreviewWriteSkipped(CalibrationContext &ctx, const char *what,
	bool &announced)
{
	if (announced)
		return;
	announced = true;
	ctx.Log(std::string("UI preview mode: ") + what +
		" was not written to the registry\n");
}

static bool SaveProfileRecord(CalibrationContext &ctx, const ProfileRecord &record)
{
	questcal::PersistenceWriteGate gate = questcal::GateProfileWrite(
		g_uiPreviewMode, ctx.profileLoadState, ctx.settingsLoadState,
		ctx.legacySettingsMigrationPending);
	switch (gate)
	{
	case questcal::PersistenceWriteGate::Allowed:
		break;
	case questcal::PersistenceWriteGate::SkippedPreview:
		// UI preview runs on fake state; never let it clobber the real profile.
		// It still reports success so every caller path stays exercised by the
		// -frames smoke run, which is precisely why the skip has to be audible:
		// this is the one place the overlay says "saved" without saving.
		{
			static bool announced = false;
			NotePreviewWriteSkipped(ctx, "the calibration profile", announced);
		}
		return true;
	case questcal::PersistenceWriteGate::RefusedConfigUnreadable:
		ctx.ReportError(
			"Could not save the calibration profile because the existing Config record "
			"could not be read. It was left untouched for recovery.\n",
			CalibrationContext::ErrorSource::ProfilePersistence);
		return false;
	case questcal::PersistenceWriteGate::RefusedSettingsUnreadable:
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
	// The same definition the parser enforces, so a record that saves is a
	// record that will load again.
	std::string why;
	if (!questcal::ValidateProfileRecord(
		record, protocol::SetAlignmentField::MaxAnchors, why))
	{
		ctx.ReportError("Could not save the calibration profile: " + why + "\n",
			CalibrationContext::ErrorSource::ProfilePersistence);
		return false;
	}

	std::cout << "Saving profile to registry" << std::endl;

	std::stringstream profile;
	questcal::WriteProfile(record, ctx.persistenceRevision, profile);
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
			"Could not save QuestCalibrator settings because the existing Config record "
			"could not be read. Neither record was changed so legacy settings remain recoverable.\n",
			CalibrationContext::ErrorSource::SettingsPersistence);
		return false;
	case questcal::PersistenceWriteGate::RefusedSettingsUnreadable:
		ctx.ReportError(
			"Could not save QuestCalibrator settings because the existing Settings record "
			"could not be read. It was left untouched for recovery.\n",
			CalibrationContext::ErrorSource::SettingsPersistence);
		return false;
	}
	if (ctx.persistenceRevision == 0)
		ctx.persistenceRevision = 1;
	SettingsRecord record = CaptureSettingsRecord(ctx);
	// A snapshot with no owner is refused rather than written: an unowned room
	// cannot be safely restored, so persisting one only produces a record the
	// restore path will reject later, with nothing said at the time.
	std::string why;
	if (!ValidateChaperoneRecord(record.chaperone, true, why))
	{
		ctx.ReportError(
			"Could not save QuestCalibrator settings because " + why + "\n",
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
	bool profileSaved = true;
	if (ctx.profileSaveDirty)
	{
		if (!ctx.validProfile)
		{
			ctx.ReportError(PendingProfileWithoutValidProfileMessage,
				CalibrationContext::ErrorSource::ProfilePersistence);
			profileSaved = false;
		}
		else
			profileSaved = SaveProfile(ctx);
	}
	// Only a coupled rebase — both records carrying the same revision bump —
	// makes the Settings half wait for the Config half. Otherwise one Config
	// failure would also swallow the fail-closed chaperone disarms, which live
	// in Settings, and the next launch would auto-restore a stale armed
	// snapshot. An uncoupled mismatch is detected and failed closed at load.
	if (!profileSaved && ctx.persistenceCoupled)
		return false;
	bool settingsSaved = SaveSettingsRecord(ctx);
	if (!ctx.HasDirtyPersistence())
		ctx.persistenceCoupled = false;
	return profileSaved && settingsSaved;
}
