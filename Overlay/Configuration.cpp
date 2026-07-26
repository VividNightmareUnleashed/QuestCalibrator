#include "stdafx.h"
#include "Configuration.h"
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

static picojson::array FloatArray(const float *buf, size_t numFloats)
{
	picojson::array arr;

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

static void ParseProfile(CalibrationContext &ctx, std::istream &stream)
{
	picojson::value v;
	std::string err = picojson::parse(v, stream);
	if (!err.empty())
		throw std::runtime_error(err);

	if (!v.is<picojson::array>())
		throw std::runtime_error("profile file is not an array");
	auto arr = v.get<picojson::array>();
	if (arr.size() < 1)
		throw std::runtime_error("no profiles in file");

	if (!arr[0].is<picojson::object>())
		throw std::runtime_error("profile entry is not an object");
	auto obj = arr[0].get<picojson::object>();

	if (!obj["reference_tracking_system"].is<std::string>() ||
		!obj["target_tracking_system"].is<std::string>())
		throw std::runtime_error("profile is missing the tracking system names");
	ctx.referenceTrackingSystem = obj["reference_tracking_system"].get<std::string>();
	ctx.targetTrackingSystem = obj["target_tracking_system"].get<std::string>();
	if (ctx.referenceTrackingSystem.empty() || ctx.targetTrackingSystem.empty())
		throw std::runtime_error("tracking system names cannot be empty");

	// The quaternion is the stored truth; Euler display values are derived
	// from it by SetCalibration below.
	if (!obj["rotation_quat"].is<picojson::array>() || !obj["translation_meters"].is<picojson::array>())
		throw std::runtime_error("profile is missing rotation_quat/translation_meters");

	auto &quatArr = obj["rotation_quat"].get<picojson::array>();
	auto &transArr = obj["translation_meters"].get<picojson::array>();
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

	double scale = HasTypedValue<double>(obj, "scale") ? GetDouble(obj["scale"]) : 1.0;
	if (!questcal::IsValidCalibrationTransform(rotation, translationMeters, scale))
		throw std::runtime_error("invalid calibration transform");
	ctx.SetCalibration(rotation, translationMeters, scale);

	if (HasTypedValue<double>(obj, "time_offset"))
		ctx.calibratedTimeOffset = GetDouble(obj["time_offset"]);

	// Unix seconds of the last successful solve; 0 = unknown (older profile).
	if (HasTypedValue<double>(obj, "calibration_time"))
	{
		ctx.calibrationUnixTime = GetDouble(obj["calibration_time"]);
		if (ctx.calibrationUnixTime < 0.0)
			throw std::runtime_error("invalid calibration_time");
	}

	if (HasTypedValue<bool>(obj, "apply_time_offset"))
		ctx.applyTimeOffset = obj["apply_time_offset"].get<bool>();

	// One-time migration (settings_version < 2): scale solving used to default
	// on, but streamed reference poses are motion-smoothed and the solved
	// scale absorbs the attenuation (several percent, varying with motion
	// speed) — so it is opt-in now, including for profiles saved before the
	// change. The already-applied scale is deliberately kept: it was solved
	// jointly with the translation, and clearing it without re-solving would
	// visibly misalign the space. The next recalibration replaces it.
	double settingsVersionValue = HasTypedValue<double>(obj, "settings_version")
		? GetDouble(obj["settings_version"]) : 1.0;
	if (settingsVersionValue < 1.0 || settingsVersionValue > 100.0 ||
		std::floor(settingsVersionValue) != settingsVersionValue)
		throw std::runtime_error("invalid settings_version");
	int settingsVersion = static_cast<int>(settingsVersionValue);
	bool hasSolveScale = HasTypedValue<bool>(obj, "solve_scale");
	if (settingsVersion >= 2 && hasSolveScale)
		ctx.solveScale = obj["solve_scale"].get<bool>();

	if (HasTypedValue<bool>(obj, "ui_advanced"))
		ctx.uiAdvanced = obj["ui_advanced"].get<bool>();

	if (HasTypedValue<bool>(obj, "chaperone_warning_ack"))
		ctx.chaperoneWarningAck = obj["chaperone_warning_ack"].get<bool>();

	if (HasTypedValue<double>(obj, "calibration_speed"))
	{
		double speed = GetDouble(obj["calibration_speed"]);
		if (speed < CalibrationContext::FAST || speed > CalibrationContext::VERY_SLOW ||
			std::floor(speed) != speed)
			throw std::runtime_error("invalid calibration_speed");
		ctx.calibrationSpeed = static_cast<CalibrationContext::Speed>(static_cast<int>(speed));
	}

	if (HasTypedValue<bool>(obj, "field_enabled"))
		ctx.fieldEnabled = obj["field_enabled"].get<bool>();

	// Continuous calibration (all optional: older profiles load unchanged).
	if (HasTypedValue<bool>(obj, "continuous_enabled"))
		ctx.continuousEnabled = obj["continuous_enabled"].get<bool>();

	if (HasTypedValue<std::string>(obj, "continuous_tracker_serial"))
		ctx.continuousTrackerSerial = obj["continuous_tracker_serial"].get<std::string>();

	if (HasTypedValue<bool>(obj, "continuous_latency_reestimation"))
		ctx.continuousLatencyReestimation = obj["continuous_latency_reestimation"].get<bool>();

	if (HasTypedValue<bool>(obj, "hide_mounted_tracker"))
		ctx.hideMountedTracker = obj["hide_mounted_tracker"].get<bool>();

	// Presence makes the mount extrinsic part of the profile contract. Reject
	// malformed data instead of normalizing a degenerate quaternion and
	// silently arming continuous calibration with NaNs.
	ctx.mountExtrinsic = questcal::MountExtrinsic();
	if (HasTypedValue<picojson::object>(obj, "mount_extrinsic"))
	{
		auto extrinsic = obj["mount_extrinsic"].get<picojson::object>();
		if (!extrinsic["rotation_quat"].is<picojson::array>() ||
			!extrinsic["translation_meters"].is<picojson::array>())
			throw std::runtime_error("malformed mount_extrinsic");
		auto &rotArr = extrinsic["rotation_quat"].get<picojson::array>();
		auto &traArr = extrinsic["translation_meters"].get<picojson::array>();
		if (rotArr.size() != 4 || traArr.size() != 3)
			throw std::runtime_error("malformed mount_extrinsic");

		Eigen::Quaterniond mountRotation(
			GetDouble(rotArr[0]), GetDouble(rotArr[1]),
			GetDouble(rotArr[2]), GetDouble(rotArr[3]));
		Eigen::Vector3d mountPosition(
			GetDouble(traArr[0]), GetDouble(traArr[1]), GetDouble(traArr[2]));
		if (!questcal::IsValidRotation(mountRotation) ||
			!questcal::IsFinite(mountPosition))
			throw std::runtime_error("invalid mount_extrinsic");

		ctx.mountExtrinsic.rot = mountRotation.normalized();
		ctx.mountExtrinsic.pos = mountPosition;
		if (HasTypedValue<double>(extrinsic, "rot_rms_deg"))
			ctx.mountExtrinsic.rotRmsDeg = GetDouble(extrinsic["rot_rms_deg"]);
		if (HasTypedValue<double>(extrinsic, "pos_rms_m"))
			ctx.mountExtrinsic.posRmsM = GetDouble(extrinsic["pos_rms_m"]);
		if (!questcal::IsValidResidual(ctx.mountExtrinsic.rotRmsDeg) ||
			!questcal::IsValidResidual(ctx.mountExtrinsic.posRmsM))
			throw std::runtime_error("invalid mount_extrinsic residual");
		ctx.mountExtrinsic.valid = true;
	}

	ctx.fieldAnchors.clear();
	if (HasTypedValue<picojson::array>(obj, "field_anchors"))
	{
		for (auto &anchorV : obj["field_anchors"].get<picojson::array>())
		{
			if (!anchorV.is<picojson::object>())
				throw std::runtime_error("malformed field anchor");
			auto anchorObj = anchorV.get<picojson::object>();

			if (!anchorObj["position"].is<picojson::array>() ||
				!anchorObj["rotation_quat"].is<picojson::array>() ||
				!anchorObj["translation_meters"].is<picojson::array>())
				throw std::runtime_error("malformed field anchor");

			auto &posArr = anchorObj["position"].get<picojson::array>();
			auto &rotArr = anchorObj["rotation_quat"].get<picojson::array>();
			auto &traArr = anchorObj["translation_meters"].get<picojson::array>();
			if (posArr.size() != 3 || rotArr.size() != 4 || traArr.size() != 3)
				throw std::runtime_error("malformed field anchor");

			CalibrationContext::FieldAnchor anchor;
			anchor.position = Eigen::Vector3d(GetDouble(posArr[0]), GetDouble(posArr[1]), GetDouble(posArr[2]));
			Eigen::Quaterniond anchorRotation(GetDouble(rotArr[0]), GetDouble(rotArr[1]),
				GetDouble(rotArr[2]), GetDouble(rotArr[3]));
			anchor.translationMeters = Eigen::Vector3d(GetDouble(traArr[0]), GetDouble(traArr[1]), GetDouble(traArr[2]));
			if (!questcal::IsFinite(anchor.position) ||
				!questcal::IsValidRotation(anchorRotation) ||
				!questcal::IsFinite(anchor.translationMeters))
				throw std::runtime_error("invalid field anchor");
			anchor.rotation = anchorRotation.normalized();

			if (ctx.fieldAnchors.size() < protocol::SetAlignmentField::MaxAnchors)
				ctx.fieldAnchors.push_back(anchor);
		}
	}

	if (HasTypedValue<picojson::object>(obj, "chaperone"))
	{
		auto chaperone = obj["chaperone"].get<picojson::object>();
		if (HasTypedValue<bool>(chaperone, "auto_apply"))
			ctx.chaperone.autoApply = chaperone["auto_apply"].get<bool>();

		LoadFloatArray(chaperone["play_space_size"], ctx.chaperone.playSpaceSize.v, 2);

		LoadFloatArray(
			chaperone["standing_center"],
			(float *) ctx.chaperone.standingCenter.m,
			sizeof(ctx.chaperone.standingCenter.m) / sizeof(float)
		);

		if (!chaperone["geometry"].is<picojson::array>())
			throw std::runtime_error("chaperone geometry is not an array");

		auto &geometry = chaperone["geometry"].get<picojson::array>();

		// The array is a flat float dump of HmdQuad_t's (12 floats per quad).
		// A length that isn't a whole number of quads would make the resize
		// below round down and the fill overrun the buffer — reject it, along
		// with absurd lengths that would turn into a giant allocation.
		constexpr size_t floatsPerQuad = sizeof(vr::HmdQuad_t) / sizeof(float);
		if (geometry.size() % floatsPerQuad != 0 || geometry.size() > 16384 * floatsPerQuad)
			throw std::runtime_error("chaperone geometry has invalid length");

		ctx.chaperone.geometry.resize(geometry.size() / floatsPerQuad);
		if (!geometry.empty())
			LoadFloatArray(chaperone["geometry"], (float *) ctx.chaperone.geometry.data(), geometry.size());

		if (HasTypedValue<double>(chaperone, "copy_time"))
		{
			ctx.chaperone.copyUnixTime = GetDouble(chaperone["copy_time"]);
			if (ctx.chaperone.copyUnixTime < 0.0)
				throw std::runtime_error("invalid chaperone copy_time");
		}

		// A snapshot with no walls is still a snapshot (standing center + play
		// area size): it just never auto-restores.
		ctx.chaperone.valid = true;
	}

	if (settingsVersion < 2)
	{
		ctx.Log("Playspace scale solving is now opt-in and has been turned off for this profile (re-enable it in settings if you need it)\n");
		if (ctx.calibratedScale < 0.98 || ctx.calibratedScale > 1.02)
			ctx.Log("The stored playspace scale (" +
				std::to_string(ctx.calibratedScale).substr(0, 5) +
				"x) likely came from streamed-pose smoothing -- recalibrate to clear it\n");
	}

	ctx.validProfile = true;
}

static void WriteProfile(CalibrationContext &ctx, std::ostream &out)
{
	if (!ctx.validProfile)
		return;

	picojson::object profile;
	profile["reference_tracking_system"].set<std::string>(ctx.referenceTrackingSystem);
	profile["target_tracking_system"].set<std::string>(ctx.targetTrackingSystem);

	picojson::array quat;
	quat.push_back(picojson::value(ctx.calibratedRotationQ.w()));
	quat.push_back(picojson::value(ctx.calibratedRotationQ.x()));
	quat.push_back(picojson::value(ctx.calibratedRotationQ.y()));
	quat.push_back(picojson::value(ctx.calibratedRotationQ.z()));
	profile["rotation_quat"].set<picojson::array>(quat);

	Eigen::Vector3d translationMeters = ctx.TranslationMeters();
	picojson::array trans;
	trans.push_back(picojson::value(translationMeters(0)));
	trans.push_back(picojson::value(translationMeters(1)));
	trans.push_back(picojson::value(translationMeters(2)));
	profile["translation_meters"].set<picojson::array>(trans);

	profile["scale"].set<double>(ctx.calibratedScale);
	profile["time_offset"].set<double>(ctx.calibratedTimeOffset);
	profile["calibration_time"].set<double>(ctx.calibrationUnixTime);
	profile["apply_time_offset"].set<bool>(ctx.applyTimeOffset);
	profile["solve_scale"].set<bool>(ctx.solveScale);
	// Bumped when a load-time migration must not re-run (see ParseProfile).
	double settingsVersion = 2.0;
	profile["settings_version"].set<double>(settingsVersion);
	profile["ui_advanced"].set<bool>(ctx.uiAdvanced);
	profile["chaperone_warning_ack"].set<bool>(ctx.chaperoneWarningAck);

	double speed = (int) ctx.calibrationSpeed;
	profile["calibration_speed"].set<double>(speed);

	profile["continuous_enabled"].set<bool>(ctx.continuousEnabled);
	if (!ctx.continuousTrackerSerial.empty())
		profile["continuous_tracker_serial"].set<std::string>(ctx.continuousTrackerSerial);
	profile["continuous_latency_reestimation"].set<bool>(ctx.continuousLatencyReestimation);
	profile["hide_mounted_tracker"].set<bool>(ctx.hideMountedTracker);

	if (ctx.mountExtrinsic.valid)
	{
		picojson::object extrinsic;

		picojson::array rot, tra;
		rot.push_back(picojson::value(ctx.mountExtrinsic.rot.w()));
		rot.push_back(picojson::value(ctx.mountExtrinsic.rot.x()));
		rot.push_back(picojson::value(ctx.mountExtrinsic.rot.y()));
		rot.push_back(picojson::value(ctx.mountExtrinsic.rot.z()));
		for (int k = 0; k < 3; ++k)
			tra.push_back(picojson::value(ctx.mountExtrinsic.pos(k)));

		extrinsic["rotation_quat"].set<picojson::array>(rot);
		extrinsic["translation_meters"].set<picojson::array>(tra);
		extrinsic["rot_rms_deg"].set<double>(ctx.mountExtrinsic.rotRmsDeg);
		extrinsic["pos_rms_m"].set<double>(ctx.mountExtrinsic.posRmsM);

		profile["mount_extrinsic"].set<picojson::object>(extrinsic);
	}

	profile["field_enabled"].set<bool>(ctx.fieldEnabled);
	if (!ctx.fieldAnchors.empty())
	{
		picojson::array anchors;
		for (const auto &a : ctx.fieldAnchors)
		{
			picojson::object anchorObj;

			picojson::array pos, rot, tra;
			for (int k = 0; k < 3; ++k)
			{
				pos.push_back(picojson::value(a.position(k)));
				tra.push_back(picojson::value(a.translationMeters(k)));
			}
			rot.push_back(picojson::value(a.rotation.w()));
			rot.push_back(picojson::value(a.rotation.x()));
			rot.push_back(picojson::value(a.rotation.y()));
			rot.push_back(picojson::value(a.rotation.z()));

			anchorObj["position"].set<picojson::array>(pos);
			anchorObj["rotation_quat"].set<picojson::array>(rot);
			anchorObj["translation_meters"].set<picojson::array>(tra);

			picojson::value anchorV;
			anchorV.set<picojson::object>(anchorObj);
			anchors.push_back(anchorV);
		}
		profile["field_anchors"].set<picojson::array>(anchors);
	}

	if (ctx.chaperone.valid)
	{
		picojson::object chaperone;
		chaperone["auto_apply"].set<bool>(ctx.chaperone.autoApply);
		chaperone["play_space_size"].set<picojson::array>(FloatArray(ctx.chaperone.playSpaceSize.v, 2));

		chaperone["standing_center"].set<picojson::array>(FloatArray(
			(float *) ctx.chaperone.standingCenter.m,
			sizeof(ctx.chaperone.standingCenter.m) / sizeof(float)
		));

		chaperone["geometry"].set<picojson::array>(FloatArray(
			(float *) ctx.chaperone.geometry.data(),
			sizeof(ctx.chaperone.geometry[0]) / sizeof(float) * ctx.chaperone.geometry.size()
		));

		chaperone["copy_time"].set<double>(ctx.chaperone.copyUnixTime);

		profile["chaperone"].set<picojson::object>(chaperone);
	}

	picojson::value profileV;
	profileV.set<picojson::object>(profile);

	picojson::array profiles;
	profiles.push_back(profileV);

	picojson::value profilesV;
	profilesV.set<picojson::array>(profiles);

	out << profilesV.serialize(true);
}

static void LogRegistryResult(LSTATUS result)
{
	char *message;
	FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_ALLOCATE_BUFFER, 0, result, LANG_USER_DEFAULT, (LPSTR)&message, 0, NULL);
	std::cerr << "Opening registry key: " << message << std::endl;
}

static const char *RegistryKey = "Software\\QuestCalibrator";

static std::string ReadRegistryKey()
{
	DWORD size = 0;
	auto result = RegGetValueA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, "Config", RRF_RT_REG_SZ, 0, 0, &size);
	if (result != ERROR_SUCCESS)
	{
		LogRegistryResult(result);
		return "";
	}

	// size counts the trailing NUL; zero would underflow the resize below.
	if (size == 0)
		return "";

	std::string str;
	str.resize(size);

	result = RegGetValueA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, "Config", RRF_RT_REG_SZ, 0, &str[0], &size);
	if (result != ERROR_SUCCESS)
	{
		LogRegistryResult(result);
		return "";
	}

	if (size == 0)
		return "";

	str.resize(size - 1);
	return str;
}

static void WriteRegistryKey(std::string str)
{
	if (str.size() >= std::numeric_limits<DWORD>::max())
	{
		std::cerr << "Profile is too large to write to the registry" << std::endl;
		return;
	}

	HKEY hkey;
	auto result = RegCreateKeyExA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, 0, REG_NONE, 0, KEY_ALL_ACCESS, 0, &hkey, 0);
	if (result != ERROR_SUCCESS)
	{
		LogRegistryResult(result);
		return;
	}

	DWORD size = static_cast<DWORD>(str.size() + 1);

	result = RegSetValueExA(hkey, "Config", 0, REG_SZ, reinterpret_cast<const BYTE*>(str.c_str()), size);
	if (result != ERROR_SUCCESS)
		LogRegistryResult(result);

	RegCloseKey(hkey);
}

void LoadProfile(CalibrationContext &ctx)
{
	ctx.validProfile = false;

	auto str = ReadRegistryKey();
	if (str == "")
	{
		std::cout << "Profile is empty" << std::endl;
		ctx.Clear();
		return;
	}

	try
	{
		std::stringstream io(str);
		// Parse transactionally. A malformed late field must not leave an
		// earlier transform active after the overall profile load failed.
		CalibrationContext parsed = ctx;
		parsed.validProfile = false;
		ParseProfile(parsed, io);
		ctx = std::move(parsed);
		std::cout << "Loaded profile" << std::endl;
	}
	catch (const std::exception &e)
	{
		std::cerr << "Error loading profile: " << e.what() << std::endl;
	}
}

void SaveProfile(CalibrationContext &ctx)
{
	// UI preview runs on fake state; never let it clobber the real profile.
	if (g_uiPreviewMode)
		return;

	std::cout << "Saving profile to registry" << std::endl;

	std::stringstream io;
	WriteProfile(ctx, io);
	WriteRegistryKey(io.str());
}
