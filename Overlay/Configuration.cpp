#include "stdafx.h"
#include "Configuration.h"
#include "UserInterface.h"
#include "../common/Protocol.h"

#include <picojson.h>

#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <limits>

static picojson::array FloatArray(const float *buf, int numFloats)
{
	picojson::array arr;

	for (int i = 0; i < numFloats; i++)
		arr.push_back(picojson::value(double(buf[i])));

	return arr;
}

static void LoadFloatArray(const picojson::value &obj, float *buf, int numFloats)
{
	if (!obj.is<picojson::array>())
		throw std::runtime_error("expected array, got " + obj.to_str());

	auto &arr = obj.get<picojson::array>();
	if (arr.size() != numFloats)
		throw std::runtime_error("wrong buffer size");

	for (int i = 0; i < numFloats; i++)
		buf[i] = (float) arr[i].get<double>();
}

static void ParseProfile(CalibrationContext &ctx, std::istream &stream)
{
	picojson::value v;
	std::string err = picojson::parse(v, stream);
	if (!err.empty())
		throw std::runtime_error(err);

	auto arr = v.get<picojson::array>();
	if (arr.size() < 1)
		throw std::runtime_error("no profiles in file");

	auto obj = arr[0].get<picojson::object>();

	ctx.referenceTrackingSystem = obj["reference_tracking_system"].get<std::string>();
	ctx.targetTrackingSystem = obj["target_tracking_system"].get<std::string>();

	// The quaternion is the stored truth; Euler display values are derived
	// from it by SetCalibration below.
	if (!obj["rotation_quat"].is<picojson::array>() || !obj["translation_meters"].is<picojson::array>())
		throw std::runtime_error("profile is missing rotation_quat/translation_meters");

	auto &quatArr = obj["rotation_quat"].get<picojson::array>();
	auto &transArr = obj["translation_meters"].get<picojson::array>();
	if (quatArr.size() != 4 || transArr.size() != 3)
		throw std::runtime_error("malformed rotation_quat/translation_meters");

	Eigen::Quaterniond rotation(
		quatArr[0].get<double>(),   // w
		quatArr[1].get<double>(),   // x
		quatArr[2].get<double>(),   // y
		quatArr[3].get<double>());  // z
	Eigen::Vector3d translationMeters(
		transArr[0].get<double>(),
		transArr[1].get<double>(),
		transArr[2].get<double>());

	double scale = obj["scale"].is<double>() ? obj["scale"].get<double>() : 1.0;
	ctx.SetCalibration(rotation, translationMeters, scale);

	if (obj["time_offset"].is<double>())
		ctx.calibratedTimeOffset = obj["time_offset"].get<double>();

	// Unix seconds of the last successful solve; 0 = unknown (older profile).
	if (obj["calibration_time"].is<double>())
		ctx.calibrationUnixTime = obj["calibration_time"].get<double>();

	if (obj["apply_time_offset"].is<bool>())
		ctx.applyTimeOffset = obj["apply_time_offset"].get<bool>();

	if (obj["solve_scale"].is<bool>())
		ctx.solveScale = obj["solve_scale"].get<bool>();

	if (obj["ui_advanced"].is<bool>())
		ctx.uiAdvanced = obj["ui_advanced"].get<bool>();

	if (obj["chaperone_warning_ack"].is<bool>())
		ctx.chaperoneWarningAck = obj["chaperone_warning_ack"].get<bool>();

	if (obj["calibration_speed"].is<double>())
		ctx.calibrationSpeed = (CalibrationContext::Speed)(int) obj["calibration_speed"].get<double>();

	if (obj["field_enabled"].is<bool>())
		ctx.fieldEnabled = obj["field_enabled"].get<bool>();

	// Continuous calibration (all optional: older profiles load unchanged).
	if (obj["continuous_enabled"].is<bool>())
		ctx.continuousEnabled = obj["continuous_enabled"].get<bool>();

	if (obj["continuous_tracker_serial"].is<std::string>())
		ctx.continuousTrackerSerial = obj["continuous_tracker_serial"].get<std::string>();

	if (obj["continuous_latency_reestimation"].is<bool>())
		ctx.continuousLatencyReestimation = obj["continuous_latency_reestimation"].get<bool>();

	if (obj["hide_mounted_tracker"].is<bool>())
		ctx.hideMountedTracker = obj["hide_mounted_tracker"].get<bool>();

	// Presence of a well-formed mount_extrinsic implies validity; absent or
	// malformed leaves it invalid (continuous mode stays disarmed).
	ctx.mountExtrinsic = questcal::MountExtrinsic();
	if (obj["mount_extrinsic"].is<picojson::object>())
	{
		auto extrinsic = obj["mount_extrinsic"].get<picojson::object>();
		if (extrinsic["rotation_quat"].is<picojson::array>() &&
			extrinsic["translation_meters"].is<picojson::array>())
		{
			auto &rotArr = extrinsic["rotation_quat"].get<picojson::array>();
			auto &traArr = extrinsic["translation_meters"].get<picojson::array>();
			if (rotArr.size() == 4 && traArr.size() == 3)
			{
				ctx.mountExtrinsic.rot = Eigen::Quaterniond(
					rotArr[0].get<double>(), rotArr[1].get<double>(),
					rotArr[2].get<double>(), rotArr[3].get<double>()).normalized();
				ctx.mountExtrinsic.pos = Eigen::Vector3d(
					traArr[0].get<double>(), traArr[1].get<double>(), traArr[2].get<double>());
				if (extrinsic["rot_rms_deg"].is<double>())
					ctx.mountExtrinsic.rotRmsDeg = extrinsic["rot_rms_deg"].get<double>();
				if (extrinsic["pos_rms_m"].is<double>())
					ctx.mountExtrinsic.posRmsM = extrinsic["pos_rms_m"].get<double>();
				ctx.mountExtrinsic.valid = true;
			}
		}
	}

	ctx.fieldAnchors.clear();
	if (obj["field_anchors"].is<picojson::array>())
	{
		for (auto &anchorV : obj["field_anchors"].get<picojson::array>())
		{
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
			anchor.position = Eigen::Vector3d(posArr[0].get<double>(), posArr[1].get<double>(), posArr[2].get<double>());
			anchor.rotation = Eigen::Quaterniond(rotArr[0].get<double>(), rotArr[1].get<double>(),
				rotArr[2].get<double>(), rotArr[3].get<double>()).normalized();
			anchor.translationMeters = Eigen::Vector3d(traArr[0].get<double>(), traArr[1].get<double>(), traArr[2].get<double>());

			if (ctx.fieldAnchors.size() < protocol::SetAlignmentField::MaxAnchors)
				ctx.fieldAnchors.push_back(anchor);
		}
	}

	if (obj["chaperone"].is<picojson::object>())
	{
		auto chaperone = obj["chaperone"].get<picojson::object>();
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
			LoadFloatArray(chaperone["geometry"], (float *) ctx.chaperone.geometry.data(), (int) geometry.size());

		if (chaperone["copy_time"].is<double>())
			ctx.chaperone.copyUnixTime = chaperone["copy_time"].get<double>();

		// A snapshot with no walls is still a snapshot (standing center + play
		// area size): it just never auto-restores.
		ctx.chaperone.valid = true;
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
	HKEY hkey;
	auto result = RegCreateKeyExA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, 0, REG_NONE, 0, KEY_ALL_ACCESS, 0, &hkey, 0);
	if (result != ERROR_SUCCESS)
	{
		LogRegistryResult(result);
		return;
	}

	DWORD size = str.size() + 1;

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
		ParseProfile(ctx, io);
		std::cout << "Loaded profile" << std::endl;
	}
	catch (const std::runtime_error &e)
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
