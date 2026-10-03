#pragma once
#include "ProfileRecordJson.h"
#include "ChaperoneMath.h"
#include "LanguageCodes.h"
#include <map>
#include <utility>
namespace questcal {
constexpr size_t DeviceNameMaxBytes = 32;
constexpr size_t DeviceNameMaxCount = 64;
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

// The Settings record's schema, stored under "settings_version" as the
// Config record's is (see ConfigSchema). No change has needed a migration
// yet; the first one bumps it and adds a step table as ConfigMigrations.
constexpr int SettingsSchema = 1;

struct SettingsRecord
{
	bool uiAdvanced = false;
	bool notifyPoorCalibration = true;
	bool chaperoneWarningAck = false;
	PersistedCalibrationSpeed calibrationSpeed = PersistedCalibrationSpeed::Fast;
	bool solveScale = false;
	bool applyTimeOffset = true;
	bool detailedLogging = false;
	bool automaticUpdates = false;
	std::string language;
	std::map<std::string, std::string> deviceNames;
	ChaperoneRecord chaperone;
};

inline bool ValidateChaperoneRecord(const ChaperoneRecord &record,
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

inline void ParseChaperone(SettingsRecord &settings, const picojson::object &obj)
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

		Eigen::Quaterniond baselineRotation = QuatFromArray(rotation);
		Eigen::Vector3d baselineTranslation = Vec3FromArray(translation);
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
	static_assert(sizeof(vr::HmdQuad_t) == 12 * sizeof(float), "quad representation changed");
	if (!IsValidChaperoneGeometryLength(geometry.size()))
		throw std::runtime_error("chaperone geometry has invalid length");

	parsed.geometry.resize(geometry.size() / 12);
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

inline void WriteChaperone(const SettingsRecord &settings, picojson::object &obj)
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
		chaperone["world_from_driver_rotation_quat"].set<picojson::array>(
			QuatArray(snapshot.worldFromDriverRotation));
		chaperone["world_from_driver_translation_meters"].set<picojson::array>(
			Vec3Array(snapshot.worldFromDriverTranslation));
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

inline void WriteSettings(const SettingsRecord &record,
	uint32_t persistenceRevisionValue, std::ostream &out)
{
	picojson::object settings;
	double schema = SettingsSchema;
	double calibrationSpeed = static_cast<double>(record.calibrationSpeed);
	double persistenceRevision = static_cast<double>(persistenceRevisionValue);
	settings["persistence_revision"].set<double>(persistenceRevision);
	settings["settings_version"].set<double>(schema);
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

inline PersistedRevision ParseSettingsUnchecked(SettingsRecord &settings, std::istream &stream)
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
		if (!IsValidSettingsVersion(version))
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
		if (i18n::LanguageCodeIndex(code) >= 0)
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
			if (name.empty() || name.size() > DeviceNameMaxBytes)
				continue;
			if (settings.deviceNames.size() >= DeviceNameMaxCount)
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
			static_cast<PersistedCalibrationSpeed>(static_cast<int>(speed));
	}
	// Once Settings exists it is authoritative for chaperone state.  Absence
	// means deliberately unarmed, not "fall back to an old embedded snapshot".
	settings.chaperone = ChaperoneRecord();
	ParseChaperone(settings, obj);
	return revision;
}

inline PersistedRevision ParseSettings(SettingsRecord &destination, std::istream &stream)
{
    SettingsRecord parsed = destination;
    const auto revision = ParseSettingsUnchecked(parsed, stream);
    destination = std::move(parsed);
    return revision;
}

} // namespace questcal
