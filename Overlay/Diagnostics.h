#pragma once

// The diagnostics file a player attaches to a bug report, written on request
// from Settings: version, settings and profile summary, recent activity and the
// session logs, with the account name, profile directories and computer name
// replaced. Device serials stay: they identify hardware, not people.

#include <openvr.h>
#include <string>
#include "PoseStreamHub.h"
#include "DriverSyncTracker.h"

struct CalibrationContext;
namespace vr { class IVRSystem; }

// The same snapshot goes to the export and the throttled detailed log.
// qpcNow uses the raw pose clock, or zero when that clock is unavailable.
std::string DescribeContinuousDiagnostics(const CalibrationContext &ctx, double qpcNow);
struct DiagnosticCapture
{
	PoseStreamHub::Diagnostics poseStream;
	questcal::DriverSyncTracker driverSync;
	double sampleClock = 0.0;
	float steamVrWorldScale = 0.0f;
	vr::EVRSettingsError worldScaleError = vr::VRSettingsError_ReadFailed;
};
DiagnosticCapture CaptureCalibrationDiagnostics();

// Writes the file under %LOCALAPPDATA%\QuestCalibrator\diagnostics. On
// success pathOut is the file's UTF-8 path; on failure error says why.
bool WriteDiagnosticsFile(const CalibrationContext &ctx, std::string &pathOut, std::string &error,
	vr::IVRSystem *system, const DiagnosticCapture &capture);

// Opens Explorer with the file selected, so "send this file" needs no search.
void RevealInExplorer(const std::string &utf8Path);

// The substitution the file applies, exposed for the test harness.
std::string AnonymiseDiagnosticsText(const std::string &text,
	const std::string &userProfileDir, const std::string &userName, const std::string &computerName);

// A path for the session log and the screen: under %LOCALAPPDATA% or
// %USERPROFILE% it is written with the variable, which still pastes into
// Explorer but carries no account name. The raw log gets shared as often as
// the diagnostics file.
std::string PathForLog(const std::string &utf8Path);
std::string ShortenUserPath(const std::string &path,
	const std::string &localAppData, const std::string &userProfileDir);
