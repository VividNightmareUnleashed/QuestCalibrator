#include "stdafx.h"
#include "Diagnostics.h"
#include "Calibration.h"
#include "../common/Version.h"

#include <shellapi.h>

#include <algorithm>
#include <cctype>
#include <ctime>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::string Utf8(const std::wstring &wide)
{
	if (wide.empty())
		return std::string();
	int bytes = WideCharToMultiByte(CP_UTF8, 0, wide.c_str(), -1, nullptr, 0, nullptr, nullptr);
	if (bytes <= 1)
		return std::string();
	std::string out(static_cast<size_t>(bytes), '\0');
	WideCharToMultiByte(CP_UTF8, 0, wide.c_str(), -1, &out[0], bytes, nullptr, nullptr);
	out.resize(static_cast<size_t>(bytes) - 1);
	return out;
}

std::wstring Wide(const std::string &utf8)
{
	if (utf8.empty())
		return std::wstring();
	int chars = MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, nullptr, 0);
	if (chars <= 1)
		return std::wstring();
	std::wstring out(static_cast<size_t>(chars), L'\0');
	MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, &out[0], chars);
	out.resize(static_cast<size_t>(chars) - 1);
	return out;
}

std::wstring EnvW(const wchar_t *name)
{
	wchar_t buf[MAX_PATH] = {};
	DWORD len = GetEnvironmentVariableW(name, buf, MAX_PATH);
	if (len == 0 || len >= MAX_PATH)
		return std::wstring();
	return std::wstring(buf, len);
}

// Case-insensitive replace of every occurrence: Windows paths arrive in
// whatever case the writer used.
void ReplaceAllNoCase(std::string &text, const std::string &needle, const std::string &with)
{
	if (needle.empty())
		return;
	std::string lowerText = text, lowerNeedle = needle;
	std::transform(lowerText.begin(), lowerText.end(), lowerText.begin(),
		[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	std::transform(lowerNeedle.begin(), lowerNeedle.end(), lowerNeedle.begin(),
		[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	std::string out;
	size_t pos = 0;
	for (;;)
	{
		size_t hit = lowerText.find(lowerNeedle, pos);
		if (hit == std::string::npos)
		{
			out.append(text, pos, std::string::npos);
			break;
		}
		out.append(text, pos, hit - pos);
		out += with;
		pos = hit + needle.size();
	}
	text.swap(out);
}

std::string ReadWholeFile(const std::wstring &path)
{
	std::ifstream in(path.c_str(), std::ios::in | std::ios::binary);
	if (!in.is_open())
		return std::string();
	std::ostringstream ss;
	ss << in.rdbuf();
	return ss.str();
}

std::string Stamp(const char *format)
{
	char buf[64] = {};
	std::time_t now = std::time(nullptr);
	std::tm tm;
	if (localtime_s(&tm, &now) == 0)
		std::strftime(buf, sizeof buf, format, &tm);
	return buf;
}

std::string Clock(double unixTime)
{
	char buf[32] = {};
	std::time_t t = static_cast<std::time_t>(unixTime);
	std::tm tm;
	if (localtime_s(&tm, &t) == 0)
		std::strftime(buf, sizeof buf, "%H:%M:%S", &tm);
	return buf;
}

const char *OnOff(bool v) { return v ? "on" : "off"; }

} // namespace

std::string AnonymiseDiagnosticsText(const std::string &text,
	const std::string &userProfileDir, const std::string &userName, const std::string &computerName)
{
	std::string out = text;
	// Longest first, so the directory goes before the bare name inside it.
	if (!userProfileDir.empty())
		ReplaceAllNoCase(out, userProfileDir, "<user>");
	if (userName.size() >= 2)
		ReplaceAllNoCase(out, userName, "<user>");
	if (computerName.size() >= 2)
		ReplaceAllNoCase(out, computerName, "<pc>");
	return out;
}

bool WriteDiagnosticsFile(const CalibrationContext &ctx, std::string &pathOut, std::string &error)
{
	std::wstring localAppData = EnvW(L"LOCALAPPDATA");
	if (localAppData.empty())
	{
		error = "Couldn't find the local application data folder.";
		return false;
	}
	std::wstring appDir = localAppData + L"\\QuestCalibrator";
	std::wstring dir = appDir + L"\\diagnostics";
	CreateDirectoryW(appDir.c_str(), nullptr);
	CreateDirectoryW(dir.c_str(), nullptr);

	std::ostringstream out;
	out << "QuestCalibrator " << QUESTCAL_VERSION_STRING << " diagnostics, written " << Stamp("%Y-%m-%d %H:%M:%S") << "\n";
	out << "Personal folders and the account name are shown as <user>, the computer name as <pc>.\n";
	out << "Device serial numbers are kept: they identify hardware, not people.\n\n";

	out << "[settings]\n";
	out << "calibration duration: " << ctx.CollectionSeconds() << " s\n";
	out << "advanced mode: " << OnOff(ctx.uiAdvanced) << "\n";
	out << "notifications in VR: " << OnOff(ctx.notifyPoorCalibration) << "\n";
	out << "solve scale: " << OnOff(ctx.solveScale) << "\n";
	out << "apply time offset: " << OnOff(ctx.applyTimeOffset) << "\n";
	out << "detailed logging: " << OnOff(ctx.detailedLogging) << "\n\n";

	out << "[profile]\n";
	out << "valid: " << OnOff(ctx.validProfile) << ", enabled: " << OnOff(ctx.enabled) << "\n";
	out << "reference: " << ctx.referenceTrackingSystem << ", target: " << ctx.targetTrackingSystem << "\n";
	const auto &q = ctx.transform.rotation;
	out << "rotation (w x y z): " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << "\n";
	const auto &t = ctx.transform.translationMeters;
	out << "translation (m): " << t.x() << " " << t.y() << " " << t.z() << "\n";
	out << "scale: " << ctx.transform.scale << ", time offset: " << ctx.transform.timeOffset * 1000.0 << " ms\n";
	if (ctx.lastResult.valid)
		out << "last solve: rotation RMS " << ctx.lastResult.rotationRmsDeg << " deg, position RMS "
			<< ctx.lastResult.translationRmsMeters * 100.0 << " cm, time offset "
			<< ctx.lastResult.timeOffset * 1000.0 << " ms\n";
	out << "field anchors: " << ctx.fieldAnchors.size() << " (" << OnOff(ctx.fieldEnabled) << ")\n";
	out << "chaperone protected: " << OnOff(ctx.chaperone.valid);
	if (ctx.chaperone.valid)
		out << ", " << ctx.chaperone.geometry.size() << " walls, "
			<< ctx.chaperone.playSpaceSize.v[0] << " x " << ctx.chaperone.playSpaceSize.v[1] << " m";
	out << "\n\n";

	out << "[continuous calibration]\n";
	out << "enabled: " << OnOff(ctx.continuousEnabled)
		<< ", method: " << (ctx.continuousMode == ContinuousMode::Legacy ? "legacy" : "questcalibrator") << "\n";
	out << "headset tracker: " << (ctx.continuousTrackerSerial.empty() ? "(none)" : ctx.continuousTrackerSerial) << "\n";
	out << "mount measured: " << OnOff(ctx.mountExtrinsic.valid);
	if (ctx.mountExtrinsic.valid)
		out << " (" << ctx.mountExtrinsic.rotRmsDeg << " deg / " << ctx.mountExtrinsic.posRmsM * 1000.0
			<< " mm spread, " << ctx.mountExtrinsic.pairs << " pairs)";
	out << "\n";
	out << "state: " << static_cast<int>(ctx.continuousState) << ", corrections applied: " << ctx.autoCorrectionsApplied << "\n";
	if (ctx.continuousDeviation.valid)
		out << "deviation: yaw " << ctx.continuousDeviation.yawDeg << " deg, tilt " << ctx.continuousDeviation.tiltDeg
			<< " deg, position " << ctx.continuousDeviation.posM * 100.0 << " cm; scatter "
			<< ctx.continuousScatterRotDeg << " deg / " << ctx.continuousScatterPosM * 100.0 << " cm\n";
	out << "drift: " << ctx.driftSlideEvents << " slips up to " << ctx.driftMaxSlideM * 100.0 << " cm, "
		<< ctx.discontinuousLossEvents << " tracking dropouts with a position change, "
		<< ctx.jumpsCompensated << " universe jumps compensated\n";
	out << "pose channel: " << (ctx.poseRingOpen ? "open" : "closed") << "\n\n";

	out << "[recent]\n";
	for (const auto &entry : ctx.activity)
		out << Clock(entry.unixTime) << "  " << entry.text << "\n";
	out << "\n";

	out << "[session log]\n" << ReadWholeFile(appDir + L"\\QuestCalibrator.log") << "\n";
	out << "[previous session log]\n" << ReadWholeFile(appDir + L"\\QuestCalibrator.prev.log") << "\n";

	std::string text = AnonymiseDiagnosticsText(out.str(),
		Utf8(EnvW(L"USERPROFILE")), Utf8(EnvW(L"USERNAME")), Utf8(EnvW(L"COMPUTERNAME")));

	std::wstring path = dir + L"\\QuestCalibrator-diagnostics-" + Wide(Stamp("%Y%m%d-%H%M%S")) + L".txt";
	std::ofstream file(path.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
	if (!file.is_open())
	{
		error = "Couldn't create the diagnostics file in " + Utf8(dir) + ".";
		return false;
	}
	file.write(text.data(), static_cast<std::streamsize>(text.size()));
	if (!file.good())
	{
		error = "Couldn't finish writing the diagnostics file.";
		return false;
	}
	pathOut = Utf8(path);
	return true;
}

void RevealInExplorer(const std::string &utf8Path)
{
	std::wstring args = L"/select,\"" + Wide(utf8Path) + L"\"";
	ShellExecuteW(nullptr, L"open", L"explorer.exe", args.c_str(), nullptr, SW_SHOWNORMAL);
}
