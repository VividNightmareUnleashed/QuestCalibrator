#include "stdafx.h"
#include "Diagnostics.h"
#include "Calibration.h"
#include "Updater.h"
#include "../common/Version.h"

#include <shellapi.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
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

void DescribeModule(std::ostream &out, const char *name, HMODULE module)
{
	std::wstring path(32768, L'\0');
	const DWORD length = GetModuleFileNameW(module, path.data(), static_cast<DWORD>(path.size()));
	if (!length || length >= path.size())
	{
		out << name << ": path unavailable\n";
		return;
	}
	path.resize(length);
	out << name << ": " << Utf8(path) << "\nSHA-256: ";
	std::array<unsigned char, 32> digest{};
	if (questcal::update::HashFileSha256(path, digest))
	{
		for (unsigned char value : digest)
			out << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(value);
		out << std::dec << std::setfill(' ');
	}
	else
		out << "unavailable";
	out << "\n";
}

void DescribeRawPoses(std::ostream &out, const PoseStreamHub::Diagnostics &stream, double qpcNow,
	double qpcToSeconds)
{
	out << "[raw driver poses at export]\n";
	out << "channel open: " << OnOff(stream.open) << ", stream boundaries " << stream.streamBoundaries
		<< ", gap markers " << stream.gapMarkers << ", reported loss " << stream.reportedLoss
		<< " (includes one marker per boundary)\n";
	out << "Latest pre-calibration samples; device counts include all input, even while continuous mode is off.\n";
	for (uint32_t id = 0; id < stream.devices.size(); ++id)
	{
		const auto &device = stream.devices[id];
		if (!device.received)
			continue;
		const auto &s = device.latest;
		out << "device " << id << ": received " << device.received << ", stream boundary " << device.streamBoundary
			<< ", capture age (ms) " << (qpcNow > 0.0 ? (qpcNow - RingCaptureTime(s, qpcToSeconds)) * 1000.0 : -1.0)
			<< ", connected " << OnOff(s.deviceIsConnected) << ", valid " << OnOff(s.poseIsValid)
			<< ", tracking result " << s.trackingResult << ", pose time offset (ms) " << s.poseTimeOffset * 1000.0 << "\n";
		auto vector = [&](const char *label, const double (&value)[3]) {
			out << label << value[0] << " " << value[1] << " " << value[2];
		};
		auto quaternion = [&](const char *label, const vr::HmdQuaternion_t &q) {
			out << label << q.w << " " << q.x << " " << q.y << " " << q.z;
		};
		vector("  local position (m) ", s.position);
		quaternion(", rotation (w x y z) ", s.rotation);
		vector(", velocity (m/s) ", s.velocity);
		vector(", angular velocity (rad/s) ", s.angularVelocity);
		out << "\n";
		vector("  world-from-driver translation (m) ", s.worldFromDriverTranslation);
		quaternion(", rotation (w x y z) ", s.worldFromDriverRotation);
		questcal::PoseSample composed;
		const bool trusted = TryComposeRingSample(s, qpcToSeconds, composed);
		out << ", trusted " << OnOff(trusted);
		if (trusted)
			out << ", composed position (m) " << composed.pos.transpose();
		out << "\n";
	}
	out << "\n";
}

void DescribeRuntimePoses(std::ostream &out, const CalibrationContext &ctx, vr::IVRSystem *system)
{
	out << "[runtime poses at export]\n";
	if (!system)
	{
		out << "SteamVR unavailable\n\n";
		return;
	}
	vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount]{};
	system->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0.0f,
		poses, vr::k_unMaxTrackedDeviceCount);
	const auto floor = system->GetRawZeroPoseToStandingAbsoluteTrackingPose();
	out << "raw-to-standing transform (3 rows; translation in meters):\n";
	for (const auto &row : floor.m)
		out << row[0] << " " << row[1] << " " << row[2] << " " << row[3] << "\n";
	out << "Runtime poses include the active driver calibration. Standing Y is height above SteamVR's floor.\n";
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		const auto &pose = poses[id];
		if (!pose.bDeviceIsConnected)
			continue;
		auto property = [&](vr::ETrackedDeviceProperty key) {
			char value[256]{};
			vr::ETrackedPropertyError error = vr::TrackedProp_Success;
			system->GetStringTrackedDeviceProperty(id, key, value, sizeof value, &error);
			return error == vr::TrackedProp_Success ? std::string(value) : std::string("(unavailable)");
		};
		out << "device " << id << ": serial " << property(vr::Prop_SerialNumber_String)
			<< ", model " << property(vr::Prop_ModelNumber_String)
			<< ", manufacturer " << property(vr::Prop_ManufacturerName_String)
			<< ", system " << property(vr::Prop_TrackingSystemName_String)
			<< ", class " << static_cast<int>(system->GetTrackedDeviceClass(id))
			<< ", target assignment " << OnOff(ctx.targetDeviceMask[id])
			<< ", valid " << OnOff(pose.bPoseIsValid)
			<< ", tracking result " << static_cast<int>(pose.eTrackingResult);
		if (pose.bPoseIsValid)
		{
			const auto &m = pose.mDeviceToAbsoluteTracking.m;
			out << ", raw position (m) " << m[0][3] << " " << m[1][3] << " " << m[2][3]
				<< ", standing position (m)";
			for (const auto &row : floor.m)
				out << " " << row[0] * m[0][3] + row[1] * m[1][3] + row[2] * m[2][3] + row[3];
		}
		out << "\n";
	}
	out << "\n";
}

} // namespace

std::string DescribeContinuousDiagnostics(const CalibrationContext &ctx, double qpcNow)
{
	const auto &input = ctx.continuousDiagnostics;
	const auto &engine = input.engine;
	std::ostringstream out;
	out << std::fixed << std::setprecision(1);
	auto age = [&](double timestamp) {
		if (qpcNow > 0.0 && timestamp > 0.0 && std::isfinite(timestamp))
			out << (qpcNow - timestamp) * 1000.0 << " ms";
		else
			out << "unavailable";
	};
	out << "continuous input diagnostics v1 (session totals across both methods; device counts cover the selected pair)\n";
	out << "loop eligible: " << OnOff(ctx.ContinuousShouldRun())
		<< ", method: " << (ctx.continuousMode == ContinuousMode::Legacy ? "legacy" : "questcalibrator")
		<< ", calibration state: " << static_cast<int>(ctx.state)
		<< ", HMD is reference: " << OnOff(ctx.referenceDeviceMask[vr::k_unTrackedDeviceIndex_Hmd])
		<< ", tracker slot: " << ctx.continuousTrackerId
		<< ", pose hook mask: " << ctx.driverPoseHookMask << "\n";
	out << "Quest last engine update age: ";
	age(input.lastUpdateTime);
	out << "; last observation age: ";
	age(engine.lastObservationTime);
	out << "\n";
	for (uint32_t id = 0; id < input.devices.size(); ++id)
	{
		const auto &device = input.devices[id];
		if (device.received == 0 && id != vr::k_unTrackedDeviceIndex_Hmd && id != ctx.continuousTrackerId)
			continue;
		out << "device " << id << ": received " << device.received
			<< ", accepted " << device.accepted << ", rejected tracking " << device.trackingRejected
			<< " / numeric " << device.numericRejected << ", capture age ";
		age(device.lastCaptureTime);
		out << ", accepted capture age ";
		age(device.lastAcceptedCaptureTime);
		out << "\n";
	}
	out << "stream: batches " << input.batches << ", samples " << input.samples
		<< ", gap events " << input.gapEvents << ", reported loss " << input.reportedLoss
		<< " (all devices; session boundaries count as one, not a loss rate)\n";
	out << "Quest window: reference " << engine.referenceSamples << ", target " << engine.targetSamples
		<< ", pending targets " << engine.pendingTargets << ", observations " << engine.observations
		<< "/" << engine.requiredObservations << " required\n";
	out << "Quest pairing totals: out of order reference " << engine.referenceOutOfOrder
		<< " / target " << engine.targetOutOfOrder << ", speed rejected reference " << engine.referenceSpeedRejected
		<< " / target " << engine.targetSpeedRejected << ", reference too old " << engine.referenceTooOld
		<< ", interpolation rejected " << engine.interpolationRejected
		<< ", waiting for reference (update attempts) " << engine.referenceWaitUpdates << "\n";
	out << "Quest observation totals: formed " << engine.observationsFormed << ", kept after thinning " << engine.observationsKept
		<< ", jump guard rejected " << engine.jumpGuardRejected << "\n";
	using Reason = questcal::ContinuousAlignment::ResetReason;
	auto resets = [&](Reason reason) { return engine.resets[static_cast<size_t>(reason)]; };
	out << "Quest window resets: stream gap " << resets(Reason::StreamGap)
		<< ", universe jump " << resets(Reason::UniverseJump) << ", suspended " << resets(Reason::Suspended)
		<< ", mode changed " << resets(Reason::ModeChanged) << ", requested " << resets(Reason::Requested)
		<< ", observation discontinuity " << engine.jumpGuardResets << "\n";
	const auto &legacy = input.legacy;
	out << "Legacy window: samples " << legacy.samples << ", valid solve " << OnOff(legacy.valid)
		<< "; totals: solve attempts " << legacy.solveAttempts << ", accepted " << legacy.solvesAccepted
		<< ", pair skew rejected " << legacy.pairSkewRejected << "\n";
	out << "Legacy resets: total " << legacy.resets << ", stream gap " << legacy.gapResets
		<< ", binding changed " << legacy.bindingResets << ", stale input " << legacy.staleResets << "\n";
	return out.str();
}

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

bool WriteDiagnosticsFile(const CalibrationContext &ctx, std::string &pathOut, std::string &error,
	vr::IVRSystem *system, const DiagnosticCapture &capture)
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

	// Acquire runtime positions before executable hashing can delay the export.
	std::ostringstream runtimePoses;
	runtimePoses << std::setprecision(10);
	DescribeRuntimePoses(runtimePoses, ctx, system);
	std::ostringstream out;
	out << "QuestCalibrator " << QUESTCAL_VERSION_STRING << " diagnostics, written " << Stamp("%Y-%m-%d %H:%M:%S") << "\n";
	out << "Personal folders and the account name are shown as <user>, the computer name as <pc>.\n";
	out << "Device serial numbers are kept: they identify hardware, not people.\n\n";
	out << std::setprecision(10);
	out << "[build]\n";
	DescribeModule(out, "overlay executable", nullptr);
	if (const auto module = GetModuleHandleW(L"openvr_api.dll"))
		DescribeModule(out, "loaded OpenVR API", module);
	out << "expected IPC protocol: " << protocol::Version << "\n\n";
	out << "SteamVR worldScale setting: ";
	if (capture.worldScaleError == vr::VRSettingsError_None)
		out << capture.steamVrWorldScale;
	else
		out << "unavailable (read error " << static_cast<int>(capture.worldScaleError) << ")";
	out << "\n\n";

	out << "[settings]\n";
	out << "calibration duration: " << ctx.CollectionSeconds() << " s\n";
	out << "advanced mode: " << OnOff(ctx.uiAdvanced) << "\n";
	out << "notifications in VR: " << OnOff(ctx.notifyPoorCalibration) << "\n";
	out << "solve scale: " << OnOff(ctx.solveScale) << "\n";
	out << "apply time offset: " << OnOff(ctx.applyTimeOffset) << "\n";
	out << "manual time override: " << OnOff(ctx.useManualTimeOffset) << ", value " << ctx.manualTimeOffsetMs << " ms\n";
	out << "requested driver time shift: " << ctx.appliedTimeOffset * 1000.0 << " ms\n";
	out << "latency reestimation: " << OnOff(ctx.continuousLatencyReestimation)
		<< ", require trigger: " << OnOff(ctx.continuousRequireTrigger)
		<< ", hide headset tracker: " << OnOff(ctx.hideMountedTracker) << "\n";
	out << "detailed logging: " << OnOff(ctx.detailedLogging) << "\n\n";

	out << "[profile]\n";
	out << "valid: " << OnOff(ctx.validProfile) << ", enabled: " << OnOff(ctx.enabled) << "\n";
	out << "disable reason: " << static_cast<int>(ctx.disableReason)
		<< ", universe unsafe: " << OnOff(ctx.profileUniverseUnsafe)
		<< ", owner HMD: " << ctx.profileHmdSerial << "\n";
	out << "base generation: " << ctx.baseGeneration << ", field generation: " << ctx.fieldGeneration << "\n";
	out << "reference: " << ctx.referenceTrackingSystem << ", target: " << ctx.targetTrackingSystem << "\n";
	const auto &q = ctx.transform.rotation;
	out << "rotation (w x y z): " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << "\n";
	const auto &t = ctx.transform.translationMeters;
	out << "translation (m): " << t.x() << " " << t.y() << " " << t.z() << "\n";
	out << "scale: " << ctx.transform.scale << ", time offset: " << ctx.transform.timeOffset * 1000.0 << " ms\n";
	if (ctx.lastResult.valid)
	{
		out << "last solve: rotation RMS " << ctx.lastResult.rotationRmsDeg << " deg, position RMS "
			<< ctx.lastResult.translationRmsMeters * 100.0 << " cm, time offset "
			<< ctx.lastResult.timeOffset * 1000.0 << " ms\n";
		const auto &result = ctx.lastResult;
		out << "scale identifiable: " << OnOff(result.scaleIdentifiable) << ", condition " << result.scaleCondition
			<< ", one-sigma " << result.scaleStdDev << ", guard " << static_cast<int>(result.scaleGuard) << "\n";
		out << "motion gain valid: " << OnOff(result.motionGainValid) << ", gross " << result.motionGainLow
			<< ", fine " << result.motionGainHigh << ", smoothing " << OnOff(result.motionSmoothingDetected)
			<< ", inconsistent " << OnOff(result.motionGainInconsistent) << "\n";
		out << "time estimate valid: " << OnOff(result.timeOffsetValid) << ", score " << result.timeOffsetScore
			<< ", peak margin " << result.timeOffsetPeakMargin << ", axis spread " << result.axisSpread
			<< ", translation conditioning " << result.transEigRatio << ", tilt " << result.tiltDeg << " deg\n";
		out << "samples used/gated: " << result.samplesUsed << "/" << result.samplesGated
			<< ", pairs used/rejected: " << result.pairsUsed << "/" << result.pairsRejected << "\n";
	}
	out << "field anchors: " << ctx.fieldAnchors.size() << " (" << OnOff(ctx.fieldEnabled) << ")\n";
	for (size_t i = 0; i < ctx.fieldAnchors.size(); ++i)
	{
		const auto &anchor = ctx.fieldAnchors[i];
		out << "anchor " << i << ": position (m) " << anchor.position.transpose()
			<< ", rotation (w x y z) " << anchor.rotation.w() << " " << anchor.rotation.x()
			<< " " << anchor.rotation.y() << " " << anchor.rotation.z()
			<< ", translation (m) " << anchor.translationMeters.transpose() << "\n";
	}
	out << "chaperone protected: " << OnOff(ctx.chaperone.valid);
	if (ctx.chaperone.valid)
		out << ", " << ctx.chaperone.geometry.size() << " walls, "
			<< ctx.chaperone.playSpaceSize.v[0] << " x " << ctx.chaperone.playSpaceSize.v[1] << " m";
	out << "\n\n";

	out << "[driver synchronization]\n";
	const auto &sync = capture.driverSync;
	out << "submitted sequence: " << sync.latestSequence << ", state change sequence: " << sync.latestStateChangeSequence
		<< ", last verdict sequence: " << sync.lastAcceptedVerdictSequence << ", refused " << OnOff(sync.lastVerdictRefused)
		<< ", changed since verdict " << OnOff(sync.stateChangedSinceVerdict) << "\n";
	out << "overlay error source: " << static_cast<int>(ctx.uiErrorSource) << ", error: " << ctx.uiError << "\n\n";

	out << "[continuous calibration]\n";
	out << "enabled: " << OnOff(ctx.continuousEnabled)
		<< ", method: " << (ctx.continuousMode == ContinuousMode::Legacy ? "legacy" : "questcalibrator") << "\n";
	out << "headset tracker: " << (ctx.continuousTrackerSerial.empty() ? "(none)" : ctx.continuousTrackerSerial) << "\n";
	out << "mount measured: " << OnOff(ctx.mountExtrinsic.valid);
	if (ctx.mountExtrinsic.valid)
	{
		out << " (" << ctx.mountExtrinsic.rotRmsDeg << " deg / " << ctx.mountExtrinsic.posRmsM * 1000.0
			<< " mm spread, " << ctx.mountExtrinsic.pairs << " pairs)";
		const auto &mount = ctx.mountExtrinsic;
		out << "\nmount rotation (w x y z): " << mount.rot.w() << " " << mount.rot.x()
			<< " " << mount.rot.y() << " " << mount.rot.z() << "; position (m): " << mount.pos.transpose();
	}
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
	LARGE_INTEGER frequency{};
	QueryPerformanceFrequency(&frequency);
	const double qpcNow = capture.sampleClock;
	out << DescribeContinuousDiagnostics(ctx, qpcNow) << "\n";
	DescribeRawPoses(out, capture.poseStream, qpcNow,
		frequency.QuadPart > 0 ? 1.0 / static_cast<double>(frequency.QuadPart) : 0.0);
	out << runtimePoses.str();

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
